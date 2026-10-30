#include "swarmtraj/planning.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace swarmtraj {
namespace {
using Clock=std::chrono::steady_clock;
double elapsed(Clock::time_point start){return std::chrono::duration<double>(Clock::now()-start).count();}
struct Family {std::string name;std::size_t levels;bool speed;};
std::vector<Family> families(const PlanningSettings& c) {
    std::vector<Family> result{{"baseline",1,false}};
    for(std::size_t n=2;n<=c.cruise_levels;++n)result.push_back({"levels_"+std::to_string(n),n,false});
    if(c.smooth_speed){result.push_back({"speed",1,true});for(std::size_t n=2;n<=c.cruise_levels;++n)result.push_back({"levels_"+std::to_string(n)+"_speed",n,true});}
    return result;
}
std::size_t count(const std::map<std::string,std::size_t>& m,const std::string& key) {auto it=m.find(key);return it==m.end()?0:it->second;}
Value counts(const std::map<std::string,std::size_t>& m){Value::Object result;for(const auto& [k,v]:m)result[k]=v;return result;}
}
Value witness_record(const Witness& w) {
    return object({{"drone_ids",array({w.drones[0],w.drones[1]})},{"candidate_ids",array({w.candidates[0],w.candidates[1]})},
        {"sample_time_s",w.time},{"positions",array({array({w.positions[0][0],w.positions[0][1],w.positions[0][2]}),array({w.positions[1][0],w.positions[1][1],w.positions[1][2]})})},
        {"lower_bound_m",w.lower},{"required_m",w.required}});
}
Value window_record(const DepartureWindows& w) {
    Value::Array intervals;for(auto x:w.intervals)intervals.push_back(array({x[0],x[1]}));
    return object({{"parent_candidate_id",w.parent},{"reference_drone_ids",values(w.references)},
        {"intervals_s",intervals},{"constraints",w.constraints},{"budget_exhausted",w.exhausted}});
}
ValidationResult validate_individual(const Candidate& path,const Designation& snapshot,const PlanningSettings& config) {
    if(path.drone_id!=snapshot.drone_id)return {"validation_drone_id",{}};
    auto resolved=config;
    if(config.mixed) {
        const auto menu=families(config);const auto found=std::find_if(menu.begin(),menu.end(),[&](const auto& f){return f.name==path.family;});
        if(found==menu.end())return {"validation_profile_family",{}};
        resolved.mixed=false;resolved.cruise_levels=found->levels;resolved.smooth_speed=found->speed;
    } else if(!path.family.empty())return {"validation_profile_family",{}};
    if(!std::isfinite(path.arrival)||path.arrival<path.release||path.arrival>resolved.latest_arrival+1e-9)
        return {"arrival_timing",{},config.deadline&&path.arrival+config.clock_error>*config.deadline?"arrival_deadline_exceeded":"post_arrival_window_exceeds_horizon"};
    if(path.dt!=resolved.physics.dt||path.projected.size()!=resolved.steps+1||path.acceleration.size()!=resolved.steps)return {"plan_horizon",{}};
    return validate_candidate(path,snapshot.initial,snapshot.yaw_rate,snapshot.cruise,resolved);
}
void extend_catalog(Catalog& catalog,const Designation& snapshot,const PlanningSettings& c,std::size_t goal,
    std::optional<double> approach,const Repair* repair,const DepartureWindows* windows,const std::string& expansion) {
    const auto menu=families(c);
    if(catalog.profile_stage>=menu.size()||catalog.attempted>c.attempts||catalog.plans.size()>c.candidates)
        throw std::invalid_argument("invalid resumed catalog state");
    auto unlock=[&](const std::string& reason) {
        if(c.mixed&&catalog.profile_stage+1<menu.size()) {
            ++catalog.profile_stage;catalog.profile_cursor=0;catalog.profile_failures=0;
            catalog.profile_events.push_back(object({{"before_candidate_id",catalog.attempted},{"reason",reason},{"unlocked_family",menu[catalog.profile_stage].name}}));
        }
    };
    if(expansion=="conflict"||expansion=="margin")unlock(expansion);
    if(windows&&c.windows)catalog.interval_contexts.push_back(window_record(*windows));else windows=nullptr;
    std::vector<std::string> modes;
    if(repair&&c.witnesses) {
        if(c.cruise_range)modes.push_back("altitude");
        if(c.detour>0)modes.push_back("lateral");
        if(c.max_delay>c.min_delay)modes.push_back("departure");
    }
    auto entropy=c.seed;append_entropy(entropy,snapshot.drone_id);
    Pcg64 rng=catalog.rng?Pcg64(*catalog.rng):Pcg64(entropy);
    std::size_t batch_attempt=0;
    while(catalog.attempted<c.attempts&&catalog.plans.size()<std::min(goal,c.candidates)) {
        auto proposal_config=c;Value *profile=nullptr,*guided=nullptr,*timed=nullptr,*adjustment=nullptr;
        std::string family_name;
        if(c.mixed) {
            const auto& family=menu[(catalog.profile_stage+catalog.profile_stage+1-catalog.profile_cursor%(catalog.profile_stage+1))%(catalog.profile_stage+1)];
            ++catalog.profile_cursor;proposal_config.mixed=false;proposal_config.cruise_levels=family.levels;proposal_config.smooth_speed=family.speed;
            family_name=family.name;
            catalog.profile_proposals.push_back(object({{"candidate_id",catalog.attempted},{"family",family.name},{"accepted",false}}));profile=&catalog.profile_proposals.back();
        }
        std::string mode;
        if(!modes.empty()&&batch_attempt%2==0) {
            mode=modes[catalog.guided.size()%modes.size()];
            catalog.guided.push_back(object({{"candidate_id",catalog.attempted},{"mode",mode},{"witness",witness_record(repair->conflict)},{"accepted",false}}));
            guided=&catalog.guided.back();
        }
        std::optional<double> departure;
        if(windows&&!windows->intervals.empty()&&batch_attempt%2==0) {
            const auto window=windows->intervals[rng.integer(windows->intervals.size())];departure=rng.uniform(window[0],window[1]);
            catalog.interval_proposals.push_back(object({{"candidate_id",catalog.attempted},{"departure_interval_s",array({window[0],window[1]})},{"requested_departure_s",*departure},{"accepted",false}}));
            timed=&catalog.interval_proposals.back();
        }
        const auto generation_start=Clock::now();std::optional<double> hold;double extra=0.;
        if(c.rejection_feedback&&batch_attempt%2==0) {
            std::vector<std::pair<std::string,std::string>> available;
            if(proposal_config.cruise_levels>1&&count(catalog.rejections,"validation_cruise_level_not_reached")>=2)available.emplace_back("settle_levels","validation_cruise_level_not_reached");
            if(!c.obstacles.empty()&&count(catalog.rejections,"obstacle_clearance")>=2)available.emplace_back("wider_route","obstacle_clearance");
            if(!available.empty()) {
                const auto trial=catalog.adjustments.size(),cycle=trial/available.size();const auto [recipe,trigger]=available[trial%available.size()];
                if(recipe=="settle_levels")hold=std::max(5.,c.level_hold+2*(1+cycle%2));else extra=5.*(1+cycle%3);
                catalog.adjustments.push_back(object({{"candidate_id",catalog.attempted},{"mode",recipe},{"trigger",trigger},{"trigger_count",count(catalog.rejections,trigger)},
                    {"level_hold_s",hold?Value(*hold):Value()},{"routing_extra_m",extra},{"accepted",false}}));adjustment=&catalog.adjustments.back();
            }
        }
        auto proposal=physical_proposal(snapshot,proposal_config,catalog.attempted,rng,approach,
            guided?repair:nullptr,mode,departure,hold,extra);
        if(proposal.plan&&c.mixed)proposal.plan->family=family_name;
        catalog.generation_s+=elapsed(generation_start);++catalog.attempted;++batch_attempt;
        std::string reason=proposal.rejection,detail;
        if(proposal.plan) {
            const auto validation_start=Clock::now();const auto validation=validate_individual(*proposal.plan,snapshot,c);
            catalog.validation_s+=elapsed(validation_start);reason=validation.rejection;detail=validation.detail;
            if(validation.accepted()) {
                catalog.profile_failures=0;
                for(auto* item:{profile,guided,timed,adjustment})if(item)(*item)["accepted"]=true;
                catalog.plans.push_back(std::move(*proposal.plan));continue;
            }
        }
        if(reason.empty())reason=proposal.plan?"individual_validation":"proposal_rejected";
        if(adjustment)(*adjustment)["rejection"]=reason;
        if(c.mixed){++catalog.profile_failures;if(catalog.profile_failures>=std::max<std::size_t>(2,c.initial_candidates))unlock("consecutive_rejections");}
        ++catalog.rejections[reason];++catalog.details[detail.empty()?reason:detail];
    }
    catalog.rng=rng.state();
}
Value catalog_diagnostics(const Catalog& c,const Designation& s,const PlanningSettings& config) {
    std::map<std::string,std::size_t> routing;for(const auto& p:c.plans)++routing[p.routing_mode];
    return object({{"drone_id",s.drone_id},{"attempted",c.attempted},{"accepted",c.plans.size()},
        {"rejections",counts(c.rejections)},{"rejection_details",counts(c.details)},{"guided_proposals",c.guided},
        {"interval_contexts",c.interval_contexts},{"interval_proposals",c.interval_proposals},
        {"profile_events",c.profile_events},{"profile_proposals",c.profile_proposals},
        {"passage_contexts",Value::Array{}},{"passage_proposals",Value::Array{}},{"candidate_proposals",c.adjustments},
        {"accepted_routing_modes",counts(routing)},{"attempt_budget_exhausted",c.attempted==config.attempts},{"candidate_limit_reached",c.plans.size()==config.candidates}});
}
}
