#include "swarmtraj/search.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <set>

namespace swarmtraj {
namespace {
double separation_at(const Candidate& a,const Candidate& b,std::size_t k) {
    const auto& x=a.projected[k];const auto& y=b.projected[k];
    const double dx=x[0]-y[0],dy=x[1]-y[1],dz=x[4]-y[4];return std::sqrt(dx*dx+dy*dy+dz*dz);
}
std::pair<double,std::size_t> nearest(const Candidate& a,const Candidate& b) {
    if(a.dt!=b.dt||a.projected.size()!=b.projected.size())throw std::invalid_argument("certification requires a common time grid");
    if(a.projected.empty())throw std::invalid_argument("empty joint trajectory");
    double minimum=std::numeric_limits<double>::infinity();std::size_t index=0;
    for(std::size_t k=0;k<a.projected.size();++k){const double d=separation_at(a,b,k);if(!std::isfinite(d))throw std::invalid_argument("nonfinite joint geometry");if(d<minimum){minimum=d;index=k;}}
    return {minimum,index};
}
Witness witness(const Candidate& a,const Candidate& b,double lower,double required) {
    const auto k=nearest(a,b).second;const auto& x=a.projected[k];const auto& y=b.projected[k];
    return {{a.drone_id,b.drone_id},{a.candidate_id,b.candidate_id},k*a.dt,lower,required,{{{x[0],x[1],x[4]},{y[0],y[1],y[4]}}}};
}
std::size_t choose(const Domains& domains,bool mrv) {
    std::size_t best=0;if(mrv)for(std::size_t k=1;k<domains.size();++k)
        if(domains[k].size()<domains[best].size()||(domains[k].size()==domains[best].size()&&domains[k][0]->drone_id<domains[best][0]->drone_id))best=k;
    return best;
}
Domain ordered(Domain plans){std::sort(plans.begin(),plans.end(),[](auto a,auto b){return a->drone_id<b->drone_id;});return plans;}
std::vector<double> optimistic(std::vector<double> margins,std::size_t pairs) {
    std::sort(margins.begin(),margins.end());margins.resize(pairs,std::numeric_limits<double>::infinity());return margins;
}
}
double continuous_distance(const Candidate& a,const Candidate& b,double va,double vb) {
    return nearest(a,b).first-(va+vb)*a.dt/2;
}
const PlanningSettings& JointSearch::own(long long id) const {
    if(own_.empty())return config_;
    auto found=own_.find(id);if(found==own_.end())throw std::invalid_argument("missing own aircraft configuration");return found->second;
}
double JointSearch::required(const Candidate& a,const Candidate& b) const {
    return (config_.separation+own(a.drone_id).aircraft_radius+own(b.drone_id).aircraft_radius)
        +2*config_.position_error+(a.max_speed+b.max_speed)*config_.clock_error;
}
const Occupancy& JointSearch::occupied(const Candidate& p) {
    const std::array<long long,2> key{p.drone_id,p.candidate_id};auto found=occupancies_.find(key);
    if(found==occupancies_.end())found=occupancies_.emplace(key,occupancy(p,own(p.drone_id))).first;
    return found->second;
}
bool JointSearch::compatible(const Candidate& first,const Candidate& second) {
    const auto& a=first.drone_id<second.drone_id?first:second;const auto& b=first.drone_id<second.drone_id?second:first;
    const Key key{a.drone_id,a.candidate_id,b.drone_id,b.candidate_id};auto cached=cache_.find(key);
    if(cached!=cache_.end()){++cache_hits_;return cached->second;}
    const double lower=continuous_distance(a,b,a.max_speed,b.max_speed);
    const double needed=required(a,b);bool valid=lower>=needed+1e-7;
    if(valid&&config_.reservations) {
        auto conflict=first_conflict(occupied(a),occupied(b));
        if(conflict){valid=false;++reservation_conflicts_;if(!first_reservation_)first_reservation_=conflict;}
    }
    cache_[key]=valid;margins_[key]=lower-needed;
    const std::array<long long,2> pair_key{a.drone_id,b.drone_id};
    if(!valid&&config_.witnesses) {
        auto previous=witnesses_.find(pair_key);
        if(previous==witnesses_.end()||lower-needed<previous->second.slack())witnesses_[pair_key]=witness(a,b,lower,needed);
    }
    auto found=pairs_.find(pair_key);
    if(found==pairs_.end())found=pairs_.emplace(pair_key,PairMetrics{pair_key,0,0,lower,needed,{}}).first;
    auto& metric=found->second;++metric.checks;if(!valid)++metric.conflicts;
    if(!metric.closest||lower<metric.worst){metric.worst=lower;metric.required=needed;metric.closest=nearest(a,b).second*a.dt;}
    return valid;
}
double JointSearch::margin(const Candidate& first,const Candidate& second) {
    const auto& a=first.drone_id<second.drone_id?first:second;const auto& b=first.drone_id<second.drone_id?second:first;
    compatible(a,b);return margins_.at({a.drone_id,a.candidate_id,b.drone_id,b.candidate_id});
}
std::vector<double> JointSearch::score(const Domain& plans) {
    std::vector<double> result;for(std::size_t a=0;a<plans.size();++a)for(std::size_t b=a+1;b<plans.size();++b)result.push_back(margin(*plans[a],*plans[b]));
    std::sort(result.begin(),result.end());return result;
}
Value JointSearch::pair_scores(const Domain& plans) {
    Value::Array result;for(std::size_t a=0;a<plans.size();++a)for(std::size_t b=a+1;b<plans.size();++b)
        result.push_back(object({{"drone_ids",array({plans[a]->drone_id,plans[b]->drone_id})},{"extra_margin_m",margin(*plans[a],*plans[b])}}));
    return result;
}
std::optional<Domain> JointSearch::solve(const Domains& catalogs) {
    if(std::any_of(catalogs.begin(),catalogs.end(),[](const auto& d){return d.empty();}))return {};
    std::function<std::optional<Domain>(Domain,const Domains&)> visit;
    visit=[&](Domain selected,const Domains& domains)->std::optional<Domain> {
        if(domains.empty())return ordered(std::move(selected));
        const auto index=choose(domains,config_.mrv);Domains others=domains;others.erase(others.begin()+index);
        for(auto* candidate:domains[index]) {
            if(nodes>=config_.max_nodes){exhausted=true;return {};}
            ++nodes;Domains remaining;bool viable=true;
            for(const auto& future:others) {
                Domain filtered;for(auto* other:future)if(compatible(*candidate,*other))filtered.push_back(other);
                if(filtered.empty()){++wipeouts_[candidate->drone_id];++wipeouts_[future[0]->drone_id];viable=false;break;}
                remaining.push_back(std::move(filtered));
            }
            if(viable){auto next=selected;next.push_back(candidate);auto solution=visit(std::move(next),remaining);if(solution)return solution;if(exhausted)return {};}
        }
        return {};
    };
    return visit({},catalogs);
}
bool JointSearch::optimization_available() const {return nodes<config_.max_nodes&&optimization_nodes<config_.margin_nodes;}
std::pair<Domain,Value> JointSearch::refine(const Domains& catalogs,const Domain& incumbent) {
    Domain best=incumbent;auto best_score=score(incumbent);const auto pair_count=best_score.size();bool complete=true;std::size_t improvements=0;
    std::function<void(Domain,const Domains&,std::vector<double>)> visit;
    visit=[&](Domain selected,const Domains& domains,std::vector<double> margins) {
        auto upper=optimistic(margins,pair_count);if(upper<=best_score)return;
        if(domains.empty()){best=ordered(std::move(selected));best_score=std::move(upper);++improvements;return;}
        const auto index=choose(domains,config_.mrv);Domains others=domains;others.erase(others.begin()+index);
        struct Ranked{const Candidate* candidate;std::vector<double> key;};std::vector<Ranked> ranked;
        for(auto* candidate:domains[index]) {
            std::vector<double> key;for(auto* previous:selected)key.push_back(margin(*candidate,*previous));std::sort(key.begin(),key.end());ranked.push_back({candidate,std::move(key)});
        }
        std::stable_sort(ranked.begin(),ranked.end(),[](const auto& a,const auto& b){return a.key>b.key;});
        for(const auto& item:ranked) {
            auto* candidate=item.candidate;std::vector<double> added;for(auto* previous:selected)added.push_back(margin(*candidate,*previous));
            bool acceptable=true;for(auto* previous:selected)if(!compatible(*candidate,*previous)){acceptable=false;break;}if(!acceptable)continue;
            auto next_margins=margins;next_margins.insert(next_margins.end(),added.begin(),added.end());if(optimistic(next_margins,pair_count)<=best_score)continue;
            if(!optimization_available()){complete=false;optimization_budget_exhausted=true;return;}
            ++nodes;++optimization_nodes;Domains remaining;bool viable=true;
            for(const auto& future:others) {
                Domain filtered;for(auto* other:future)if(compatible(*candidate,*other))filtered.push_back(other);
                if(filtered.empty()){viable=false;break;}remaining.push_back(std::move(filtered));
            }
            if(viable){auto next=selected;next.push_back(candidate);visit(std::move(next),remaining,std::move(next_margins));if(!complete)return;}
        }
    };
    if(pair_count)visit({},catalogs,{});
    return {best,object({{"catalog_optimal",complete},{"improvements",improvements},{"optimization_nodes",optimization_nodes},{"sorted_slack_m",values(best_score)}})};
}
std::vector<std::size_t> JointSearch::expansion_indices(const std::vector<Catalog>& catalogs,const std::vector<Designation>& snapshots) const {
    std::map<long long,std::size_t> scores;
    for(const auto& [key,pair]:pairs_)for(auto id:pair.drones)scores[id]+=pair.conflicts;
    auto count=[](const auto& map,long long id){auto found=map.find(id);return found==map.end()?std::size_t(0):found->second;};
    std::vector<std::size_t> available;
    for(std::size_t k=0;k<catalogs.size();++k)if(catalogs[k].attempted<config_.attempts&&catalogs[k].plans.size()<config_.candidates)available.push_back(k);
    std::stable_sort(available.begin(),available.end(),[&](auto a,auto b) {
        const auto x=snapshots[a].drone_id,y=snapshots[b].drone_id;
        if(count(wipeouts_,x)!=count(wipeouts_,y))return count(wipeouts_,x)>count(wipeouts_,y);
        if(scores[x]!=scores[y])return scores[x]>scores[y];
        if(catalogs[a].plans.size()!=catalogs[b].plans.size())return catalogs[a].plans.size()<catalogs[b].plans.size();
        return x<y;
    });
    if(available.size()>2)available.resize(2);
    return available;
}
std::vector<std::size_t> JointSearch::margin_expansion_indices(const std::vector<Catalog>& catalogs,const std::vector<Designation>& snapshots,const Domain& plans) {
    std::map<long long,std::size_t> indices;for(std::size_t k=0;k<snapshots.size();++k)indices[snapshots[k].drone_id]=k;
    struct Ranked{const Candidate* a;const Candidate* b;double slack;};std::vector<Ranked> ranked;
    for(std::size_t a=0;a<plans.size();++a)for(std::size_t b=a+1;b<plans.size();++b)ranked.push_back({plans[a],plans[b],margin(*plans[a],*plans[b])});
    std::stable_sort(ranked.begin(),ranked.end(),[](const auto& a,const auto& b){if(a.slack!=b.slack)return a.slack<b.slack;if(a.a->drone_id!=b.a->drone_id)return a.a->drone_id<b.a->drone_id;return a.b->drone_id<b.b->drone_id;});
    std::vector<std::size_t> selected;
    for(const auto& pair:ranked)for(auto* p:{pair.a,pair.b}) {
        const auto i=indices.at(p->drone_id);const auto& c=catalogs[i];
        if(std::find(selected.begin(),selected.end(),i)==selected.end()&&c.attempted<config_.attempts&&c.plans.size()<config_.candidates){selected.push_back(i);if(selected.size()==2)return selected;}
    }
    return selected;
}
std::map<long long,Repair> JointSearch::repair_hints(const std::vector<long long>& drones,const Domain* plans) {
    std::map<long long,Repair> result;if(!config_.witnesses)return result;std::vector<Witness> candidates;
    if(plans)for(std::size_t a=0;a<plans->size();++a)for(std::size_t b=a+1;b<plans->size();++b) {
        const auto& first=*plans->at(a);const auto& second=*plans->at(b);const double needed=required(first,second);
        candidates.push_back(witness(first,second,margin(first,second)+needed,needed));
    } else for(const auto& [key,w]:witnesses_)candidates.push_back(w);
    std::stable_sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){if(a.slack()!=b.slack())return a.slack()<b.slack();if(a.drones!=b.drones)return a.drones<b.drones;return a.candidates<b.candidates;});
    for(const auto& w:candidates)for(auto id:w.drones)if(std::find(drones.begin(),drones.end(),id)!=drones.end()&&!result.count(id))result.emplace(id,Repair{w,id});
    return result;
}
std::map<long long,DepartureWindows> JointSearch::interval_hints(const std::vector<Catalog>& catalogs,const std::vector<std::size_t>& indices,const Domain* plans) {
    std::map<long long,DepartureWindows> result;
    if(!config_.windows||std::any_of(catalogs.begin(),catalogs.end(),[](const auto& c){return c.plans.empty();}))return result;
    std::map<long long,const Candidate*> references;
    if(plans)for(auto* p:*plans)references[p->drone_id]=p;else for(const auto& c:catalogs)references[c.plans[0].drone_id]=&c.plans[0];
    for(auto i:indices) {
        const auto drone=catalogs.at(i).plans[0].drone_id;const auto& parent=*references.at(drone);std::vector<const Occupancy*> blockers;
        for(auto [id,p]:references)if(id!=drone)blockers.push_back(&occupied(*p));
        result.emplace(drone,departure_windows(parent,occupied(parent),blockers,config_));
    }
    return result;
}
Value JointSearch::diagnostics() const {
    Value::Object wipeouts;for(auto [id,n]:wipeouts_)wipeouts[std::to_string(id)]=n;Value::Array pairs;
    for(const auto& [key,p]:pairs_)pairs.push_back(object({{"drone_ids",array({p.drones[0],p.drones[1]})},{"checks",p.checks},{"conflicts",p.conflicts},
        {"worst_lower_bound_m",p.worst},{"required_at_worst_m",p.required},{"closest_sample_time_s",p.closest?Value(*p.closest):Value()}}));
    return object({{"pair_checks",cache_.size()},{"pair_cache_hits",cache_hits_},{"reservation_conflicts",reservation_conflicts_},
        {"first_reservation_conflict",first_reservation_?conflict_record(*first_reservation_):Value()},
        {"domain_wipeouts",wipeouts},{"pairs",pairs}});
}
}
