#include "swarmtraj/mission.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <locale>
#include <numeric>
#include <sstream>

namespace swarmtraj {
namespace {
using Clock=std::chrono::steady_clock;
double elapsed(Clock::time_point start){return std::chrono::duration<double>(Clock::now()-start).count();}
Control trim(const State& s,const Parameters& p) {
    const double load=std::cos(s[5])/std::cos(s[6]);
    const double axial=p.drag0*s[3]*s[3]+p.drag_lift*load*load/(s[3]*s[3])+gravity*std::sin(s[5]);
    return {std::max(0.,axial),std::max(0.,-axial),load,0.};
}
std::string general(double value){std::ostringstream stream;stream.imbue(std::locale::classic());stream<<std::setprecision(6)<<std::defaultfloat<<value;return stream.str();}
std::string one_decimal(double value){std::ostringstream stream;stream.imbue(std::locale::classic());stream<<std::fixed<<std::setprecision(1)<<value;return stream.str();}
double norm(double x,double y,double z=0.){return std::sqrt(x*x+y*y+z*z);}
std::optional<MissionResult> preflight(const std::vector<Designation>& snapshots,const std::vector<PlanningSettings>& settings) {
    const auto& shared=settings.front();const double horizon=shared.horizon;
    auto fail=[](const char* status,std::string reason){MissionResult result;result.status=status;result.reason=std::move(reason);return std::optional<MissionResult>{std::move(result)};};
    for(std::size_t i=0;i<snapshots.size();++i) {
        const auto& s=snapshots[i];const auto& c=settings[i];const auto& p=c.physics;const double speed=s.initial[3];
        if(s.drone_id<0||!std::isfinite(s.initial[0])||!std::isfinite(s.initial[1])||!std::isfinite(s.initial[4])||!std::isfinite(s.yaw_rate))throw std::invalid_argument("invalid snapshot");
        if(!(p.min_speed<=speed&&speed<=p.max_speed))return fail("invalid_initial_state","Initial airspeed violates the model limits.");
        if(c.smooth_speed&&!(c.speed_range[0]<=speed&&speed<=c.speed_range[1]))return fail("invalid_initial_state","Initial airspeed must lie within the requested speed profile range.");
        if(std::abs(s.yaw_rate)>gravity*std::tan(p.max_bank)/speed)return fail("invalid_initial_state","Initial turn rate violates the model limits.");
        const double cruise=s.cruise.value_or(s.initial[2]);
        if(!std::isfinite(s.initial[2])||s.initial[2]<p.min_altitude||s.initial[2]>p.max_altitude
            ||(!c.cruise_range&&(!std::isfinite(cruise)||cruise<p.min_altitude||cruise>p.max_altitude)))return fail("invalid_initial_state","Initial/cruise altitude violates the altitude limits.");
        if(c.descent_degrees>0) {
            const double angle=c.descent_degrees*(pi/180),rate=speed*std::sin(angle);
            const double ramp=std::max(1.,std::ceil(rate/(p.max_vertical_acceleration*p.dt)));
            const double level=std::ceil(std::max(1.,c.target_radius/speed+c.clock_error+p.dt)/p.dt);
            const double drop=rate*(.5*ramp+level)*p.dt,recovery=rate*ramp*p.dt/2;
            const std::string prefix="Drone "+std::to_string(s.drone_id)+": ";
            if(rate>p.max_descent+1e-9)return fail("descent_not_feasible",prefix+"descent angle limited to "+one_decimal(std::asin(std::min(1.,p.max_descent/speed))*(180/pi))+"° at "+general(speed)+" m/s.");
            if(c.descent_degrees>c.max_path_degrees)return fail("descent_not_feasible",prefix+"flight-path angle exceeds the flight-model limit.");
            const double load=std::cos(angle),axial=p.drag0*speed*speed+p.drag_lift*load*load/(speed*speed)-gravity*std::sin(angle);
            if(axial<-p.max_brake-1e-9)return fail("descent_not_feasible",prefix+"Insufficient drag/airbrake authority to maintain airspeed on this slope.");
            if(*c.target_altitude-recovery<p.min_altitude-1e-8)return fail("descent_not_feasible",prefix+"target altitude too low for the post-passage pull-out.");
            const double ceiling=c.cruise_range?c.cruise_range->at(1):cruise;
            if(ceiling<*c.target_altitude+drop-1e-8)return fail("descent_not_feasible",prefix+"cruise altitude too low to establish the requested descent angle.");
        }
        if(s.initial[5]!=0||std::abs(s.initial[6]-std::atan(speed*s.yaw_rate/gravity))>1e-12)throw std::invalid_argument("snapshot bank/slope differs from the designated holding state");
        if(!envelope(s.initial,trim(s.initial,p),p))return fail("invalid_initial_state","Initial loiter violates the coupled flight/stall/propulsion envelope.");
    }
    for(std::size_t i=0;i<snapshots.size();++i) {
        const auto& c=settings[i];const auto& s=snapshots[i];
        for(std::size_t j=0;j<shared.obstacles.size();++j) {
            const double d=solid_distance(shared.obstacles[j],s.initial);
            if(d<shared.obstacle_clearance+c.aircraft_radius)return fail("initial_obstacle_violation","Drone "+std::to_string(s.drone_id)+" starts inside obstacle "+std::to_string(j)+"'s clearance zone.");
            if(d<c.aircraft_radius+shared.obstacle_clearance+shared.position_error+c.physics.max_speed*shared.clock_error)return fail("initial_obstacle_margin_not_certified","Drone "+std::to_string(s.drone_id)+": initial robust margin to obstacle "+std::to_string(j)+" is insufficient.");
        }
    }
    if(shared.target_altitude) {
        double radius=0.;for(const auto& c:settings)radius=std::max(radius,c.aircraft_radius);
        const State target{shared.target[0],shared.target[1],*shared.target_altitude,0,0,0,0};
        for(std::size_t j=0;j<shared.obstacles.size();++j)
            if(solid_distance(shared.obstacles[j],target)+shared.target_radius-shared.position_error<shared.obstacle_clearance+radius)
                return fail("target_region_obstructed","The entire nominal arrival region violates clearance to obstacle "+std::to_string(j)+".");
    }
    for(std::size_t a=0;a<snapshots.size();++a)for(std::size_t b=a+1;b<snapshots.size();++b) {
        const auto& x=snapshots[a].initial;const auto& y=snapshots[b].initial;
        const double distance=norm(x[0]-y[0],x[1]-y[1],x[2]-y[2]);
        const double required=shared.separation+settings[a].aircraft_radius+settings[b].aircraft_radius;
        if(distance<required)return fail("initial_separation_violation","Separation is already violated at t=0.");
        const double robust=required+2*shared.position_error+(settings[a].physics.max_speed+settings[b].physics.max_speed)*shared.clock_error;
        if(distance<robust)return fail("initial_margin_not_certified","Initial separation cannot satisfy the chosen tube criterion.");
    }
    for(std::size_t i=0;i<snapshots.size();++i) {
        const auto& s=snapshots[i].initial;
        const double distance=norm(s[0]-shared.target[0],s[1]-shared.target[1],shared.target_altitude?s[2]-*shared.target_altitude:0.);
        if(distance>settings[i].physics.max_speed*horizon+shared.target_radius)return fail("destination_unreachable_within_horizon","Even a straight path at the model's allowed speed bound cannot reach the target before the horizon.");
    }
    return {};
}
Value timing_diagnostics(const PlanningSettings& c,const std::vector<Catalog>& catalogs,
    const std::vector<Designation>& snapshots,const std::vector<PlanningSettings>& own,const Domain& plans) {
    std::map<std::string,std::size_t> details;for(const auto& catalog:catalogs)for(auto [reason,n]:catalog.details)details[reason]+=n;
    auto count=[&](const std::string& key){auto found=details.find(key);return found==details.end()?std::size_t(0):found->second;};
    bool temporal=false;for(const char* key:{"target_not_reached_within_horizon","post_arrival_window_exceeds_horizon","recovery_exceeds_horizon","validation_arrival_horizon","validation_cruise_recovery"})temporal|=count(key)!=0;
    Value::Array advice,templates,per_drone,schedule;Value suggested;
    if(c.descent_degrees>0)for(std::size_t k=0;k<snapshots.size();++k) {
        const auto& s=snapshots[k];const auto& p=own[k].physics;const double theta=c.descent_degrees*(pi/180),speed=s.initial[3];
        const double rate=speed*std::tan(theta),steps=std::max(1.,std::ceil(rate/(.9*p.max_vertical_acceleration*p.dt)));
        const double geometric=rate*steps*p.dt/2,vertical=speed*std::sin(theta);
        const double physical=vertical*std::max(1.,std::ceil(vertical/(p.max_vertical_acceleration*p.dt)))*p.dt/2;
        const double available=*c.target_altitude-p.min_altitude;const bool limitation=geometric>available+1e-8&&physical<=available+1e-8;
        templates.push_back(object({{"drone_id",s.drone_id},{"geometric_recovery_drop_m",geometric},{"coupled_preflight_recovery_drop_m",physical},{"available_height_m",available},{"conservative_template_limitation",limitation}}));
        if(limitation)advice.emplace_back("Drone "+std::to_string(s.drone_id)+": the conservative geometric recovery template exceeds available height although the coupled preflight check does not. This is a generator limitation, not physical infeasibility; review explicit speed, slope or target-height settings.");
    }
    if(plans.empty()&&temporal) {
        double value=1.5*c.horizon;for(const auto& catalog:catalogs)if(catalog.minimum_horizon)value=std::max(value,*catalog.minimum_horizon);
        suggested=std::ceil(std::ceil(value/10)*10/c.physics.dt)*c.physics.dt;
        advice.emplace_back("Try a longer horizon to cover routing, target passage and recovery; this does not guarantee a solution.");
    }
    if(plans.empty()&&(count("arrival_deadline_exceeded")||c.deadline))advice.emplace_back("An explicit arrival deadline remains binding: increasing horizon alone cannot relax it.");
    if(plans.empty()&&(count("climb_cannot_finish_before_arrival")||count("insufficient_time_before_descent")||count("insufficient_time_before_target_altitude")))
        advice.emplace_back("Some proposals leave insufficient time for altitude transitions before arrival; review cruise range or route geometry.");
    if(plans.empty()&&count("obstacle_clearance"))advice.emplace_back("Integrated paths violate robust obstacle clearance; review routing alternatives, not safety margins.");
    Value::Object reasons;for(auto [reason,n]:details)reasons[reason]=n;
    for(std::size_t k=0;k<snapshots.size();++k)per_drone.push_back(object({{"drone_id",snapshots[k].drone_id},{"minimum_observed_required_horizon_s",catalogs[k].minimum_horizon?Value(*catalogs[k].minimum_horizon):Value()}}));
    for(auto* p:plans) {
        std::optional<double> recovery;bool settled=true;
        for(std::size_t k=p->projected.size();k-->0;) {
            settled=settled&&p->cruise&&std::abs(p->projected[k][4]-*p->cruise)<=.2&&std::abs(p->projected[k][5])<=.1;
            if(settled&&k*p->dt>=p->arrival)recovery=k*p->dt;
        }
        schedule.push_back(object({{"drone_id",p->drone_id},{"departure_s",p->release},{"arrival_s",p->arrival},{"recovery_completed_s",recovery?Value(*recovery):Value()},
            {"post_arrival_available_s",c.horizon-p->arrival},{"routing_mode",p->routing_mode},{"cruise_altitude_m",p->cruise?Value(*p->cruise):Value()}}));
    }
    return object({{"horizon_s",c.horizon},{"arrival_deadline_s",c.deadline?Value(*c.deadline):Value()},{"minimum_post_arrival_s",c.post_arrival},{"latest_nominal_arrival_s",c.latest_arrival},
        {"suggested_horizon_s",suggested},{"suggestion_is_feasibility_guarantee",false},{"rejection_details",reasons},{"advice",advice},{"geometric_template",templates},{"per_drone",per_drone},{"schedule",schedule}});
}
std::vector<std::size_t> counts(const std::vector<Catalog>& catalogs){std::vector<std::size_t> result;for(const auto& c:catalogs)result.push_back(c.plans.size());return result;}
std::vector<std::size_t> attempts(const std::vector<Catalog>& catalogs){std::vector<std::size_t> result;for(const auto& c:catalogs)result.push_back(c.attempted);return result;}
Domains domains(const std::vector<Catalog>& catalogs){Domains result;for(const auto& c:catalogs){Domain d;for(const auto& p:c.plans)d.push_back(&p);result.push_back(std::move(d));}return result;}
std::vector<long long> candidate_ids(const Domain& plans){std::vector<long long> result;for(auto* p:plans)result.push_back(p->candidate_id);return result;}

MissionResult run(std::vector<Designation> snapshots,std::vector<PlanningSettings> settings) {
    if(snapshots.empty())throw std::invalid_argument("at least one snapshot is required");
    if(snapshots.size()!=settings.size())throw std::invalid_argument("snapshot/own configuration mismatch");
    const auto& common=settings.front();
    for(const auto& c:settings) {
        validate(c.physics);
        if(c.steps!=common.steps||c.physics.dt!=common.physics.dt||c.horizon!=common.horizon||c.activation!=common.activation
            ||c.min_delay!=common.min_delay||c.max_delay!=common.max_delay||c.latest_arrival!=common.latest_arrival
            ||c.target!=common.target||c.target_altitude!=common.target_altitude||c.target_radius!=common.target_radius
            ||c.position_error!=common.position_error||c.descent_degrees!=common.descent_degrees||c.spread_degrees!=common.spread_degrees
            ||c.cruise_range!=common.cruise_range||c.cruise_levels!=common.cruise_levels||c.smooth_speed!=common.smooth_speed
            ||c.speed_range!=common.speed_range||c.speed_acceleration!=common.speed_acceleration||c.clock_error!=common.clock_error
            ||c.obstacle_clearance!=common.obstacle_clearance||c.obstacles!=common.obstacles||c.seed!=common.seed
            ||c.candidates!=common.candidates||c.attempts!=common.attempts||c.initial_candidates!=common.initial_candidates
            ||c.max_nodes!=common.max_nodes||c.margin_nodes!=common.margin_nodes||c.margin_rounds!=common.margin_rounds
            ||c.max_entries!=common.max_entries||c.max_constraints!=common.max_constraints||c.separation!=common.separation
            ||c.detour!=common.detour||c.level_hold!=common.level_hold||c.cell_size!=common.cell_size||c.post_arrival!=common.post_arrival
            ||c.deadline!=common.deadline||c.spatial!=common.spatial||c.adaptive!=common.adaptive||c.mrv!=common.mrv
            ||c.max_margin!=common.max_margin||c.witnesses!=common.witnesses||c.reservations!=common.reservations
            ||c.tangent_approach!=common.tangent_approach||c.efficient!=common.efficient||c.efficiency_slack_cap!=common.efficiency_slack_cap
            ||c.windows!=common.windows||c.mixed!=common.mixed||c.rejection_feedback!=common.rejection_feedback||c.id_approach!=common.id_approach)
            throw std::invalid_argument("shared mission settings must match across aircraft");
    }
    std::vector<std::size_t> order(snapshots.size());std::iota(order.begin(),order.end(),0);std::sort(order.begin(),order.end(),[&](auto a,auto b){return snapshots[a].drone_id<snapshots[b].drone_id;});
    auto old_snapshots=std::move(snapshots);auto old_settings=std::move(settings);snapshots.clear();settings.clear();
    for(auto i:order){snapshots.push_back(std::move(old_snapshots[i]));settings.push_back(std::move(old_settings[i]));}
    for(std::size_t k=1;k<snapshots.size();++k)if(snapshots[k-1].drone_id==snapshots[k].drone_id)throw std::invalid_argument("drone IDs must be unique");
    if(auto failure=preflight(snapshots,settings))return std::move(*failure);
    const auto& c=settings.front();const auto assignment=approach_angles(snapshots,settings);
    std::vector<Catalog> catalogs(snapshots.size());std::map<long long,PlanningSettings> own;
    for(std::size_t k=0;k<snapshots.size();++k)own.emplace(snapshots[k].drone_id,settings[k]);
    JointSearch search(c,own);Value::Array rounds;double generation_s=0.,search_s=0.,optimization_s=0.;
    std::vector<std::size_t> indices(snapshots.size());std::iota(indices.begin(),indices.end(),0);
    const auto goal=c.adaptive?c.initial_candidates:c.candidates;std::optional<Domain> plans;
    auto drone_ids=[&](const std::vector<std::size_t>& selected){std::vector<long long> result;for(auto i:selected)result.push_back(snapshots[i].drone_id);return result;};
    auto generate=[&](const std::vector<std::size_t>& selected,const Domain* incumbent,const std::string& reason) {
        auto start=Clock::now();auto hints=search.repair_hints(drone_ids(selected),incumbent);auto temporal=search.interval_hints(catalogs,selected,incumbent);
        for(auto i:selected) {
            const auto id=snapshots[i].drone_id;auto hint=hints.find(id);auto window=temporal.find(id);
            const auto limit=std::min(c.candidates,reason=="initial"?goal:catalogs[i].plans.size()+c.initial_candidates);
            extend_catalog(catalogs[i],snapshots[i],settings[i],limit,assignment.at(id),
                hint==hints.end()?nullptr:&hint->second,window==temporal.end()?nullptr:&window->second,reason);
        }
        generation_s+=elapsed(start);
    };
    while(true) {
        generate(indices,nullptr,rounds.empty()?"initial":"conflict");auto start=Clock::now();plans=search.solve(domains(catalogs));search_s+=elapsed(start);
        if(!plans&&search.nodes>=c.max_nodes)search.exhausted=true;
        auto current=counts(catalogs);
        rounds.push_back(object({{"expanded_drone_ids",values(drone_ids(indices))},{"counts",values(current)},{"attempts",values(attempts(catalogs))},{"search_nodes",search.nodes},{"solved",plans.has_value()}}));
        if(plans||search.exhausted||!c.adaptive||std::any_of(current.begin(),current.end(),[](auto n){return n==0;}))break;
        indices=search.expansion_indices(catalogs,snapshots);if(indices.empty())break;
    }
    Value selection=object({{"policy",c.efficient?"efficient":c.max_margin?"max_margin":"first"}});
    if(plans&&(c.max_margin||c.efficient)) {
        const auto initial_plans=*plans;const auto initial_score=search.score(*plans);Value::Array steps;std::vector<long long> expanded;
        for(std::size_t round=0;round<=c.margin_rounds;++round) {
            auto start=Clock::now();auto [chosen,quality]=search.refine(domains(catalogs),*plans);const double seconds=elapsed(start);
            search_s+=seconds;optimization_s+=seconds;plans=std::move(chosen);
            quality["expanded_drone_ids"]=values(expanded);quality["counts"]=values(counts(catalogs));quality["attempts"]=values(attempts(catalogs));steps.push_back(quality);
            if(round==c.margin_rounds||!std::get<bool>(quality.at("catalog_optimal").data)||!search.optimization_available()||plans->size()<2)break;
            indices=search.margin_expansion_indices(catalogs,snapshots,*plans);if(indices.empty())break;
            generate(indices,&*plans,"margin");expanded=drone_ids(indices);
        }
        const auto final_score=search.score(*plans);
        selection["objective"]=c.efficient?"capped_slack_then_duration_distance":"leximin_pairwise_robust_slack";selection["steps"]=steps;
        selection["initial_sorted_slack_m"]=values(initial_score);selection["selected_sorted_slack_m"]=values(final_score);
        selection["improved"]=c.efficient?search.efficiency_score(*plans)>search.efficiency_score(initial_plans):final_score>initial_score;
        if(c.efficient){selection["initial_efficiency_score"]=values(search.efficiency_score(initial_plans));selection["selected_efficiency_score"]=values(search.efficiency_score(*plans));selection["slack_cap_m"]=c.efficiency_slack_cap;}
        selection["initial_candidate_ids"]=values(candidate_ids(initial_plans));selection["selected_candidate_ids"]=values(candidate_ids(*plans));
        selection["initial_pairs"]=search.pair_scores(initial_plans);selection["selected_pairs"]=search.pair_scores(*plans);
        selection["expansion_rounds"]=steps.size()-1;selection["optimization_nodes"]=search.optimization_nodes;
        selection["optimization_budget_exhausted"]=search.optimization_budget_exhausted;selection["catalog_optimal"]=steps.back().at("catalog_optimal");
    }
    MissionResult result;result.counts=counts(catalogs);result.search_nodes=search.nodes;
    Value::Array catalog_records,per_drone;
    for(std::size_t k=0;k<catalogs.size();++k) {
        catalog_records.push_back(catalog_diagnostics(catalogs[k],snapshots[k],settings[k]));
        per_drone.push_back(object({{"drone_id",snapshots[k].drone_id},{"generation_s",catalogs[k].generation_s},{"validation_s",catalogs[k].validation_s}}));
    }
    result.diagnostics=object({{"strategy",c.adaptive?"adaptive":"fixed"},{"search_order",c.mrv?"mrv":"id"},{"conflict_guidance",c.witnesses?"witness":"none"},
        {"interval_guidance",c.windows?"safe_windows":"none"},{"profile_policy",c.mixed?"adaptive_mixed":"strict"},{"passage_guidance","none"},{"candidate_guidance",c.rejection_feedback?"rejections":"none"},
        {"rounds",rounds},{"search",search.diagnostics()},{"selection",selection},{"catalogs",catalog_records},
        {"timing",timing_diagnostics(c,catalogs,snapshots,settings,plans?*plans:Domain{})}});
    result.timings=object({{"catalog_wall_s",generation_s},{"search_wall_s",search_s},{"optimization_wall_s",optimization_s},{"per_drone",per_drone}});
    if(std::any_of(result.counts.begin(),result.counts.end(),[](auto n){return n==0;})) {
        result.status="no_candidate";result.reason="No complete candidate for at least one drone; this is not an impossibility proof.";result.search_nodes=0;return result;
    }
    if(!plans){result.status=search.exhausted?"search_budget_exhausted":"catalog_exhausted";result.reason="No validated combination found; no claim about continuous-space infeasibility.";return result;}
    const auto start=Clock::now();auto verified=verify_joint(*plans,c,own);
    if(auto value=std::get_if<double>(&verified.at("minimum_distance_lower_bound").data))result.minimum_distance=*value;
    if(auto value=std::get_if<double>(&verified.at("required_distance").data))result.required_distance=*value;
    auto& fields=std::get<Value::Object>(verified.data);if(fields.count("reservations"))result.diagnostics["reservations"]=fields.at("reservations");
    result.diagnostics["obstacle_checks"]=verified.at("obstacle_checks");result.timings["joint_verification_wall_s"]=elapsed(start);
    for(auto* plan:*plans)result.plans.push_back(*plan);
    result.status="validated";result.reason="Numerically validated coupled point-mass flight on [0, horizon]; separation bound is conditional on integration accuracy and tracking tubes.";return result;
}
}
MissionResult plan_mission(std::vector<Designation> snapshots,std::vector<PlanningSettings> settings) {
    try{return run(std::move(snapshots),std::move(settings));}
    catch(const ReservationBudgetError& error){MissionResult result;result.status="reservation_budget_exhausted";result.reason=error.what();return result;}
}
}
