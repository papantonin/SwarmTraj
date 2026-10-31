#pragma once
#include "swarmtraj/planning.hpp"
#include <stdexcept>

namespace swarmtraj {
using Cell=std::array<long long,3>;
struct Occupancy {
    long long drone_id=0,candidate_id=0;
    double inflation=0.;
    std::map<Cell,std::vector<Point>> cells;
};
struct ReservationBudgetError:std::runtime_error {using std::runtime_error::runtime_error;};
struct ReservationConflict {Cell cell;double start,end;std::array<long long,2> drones;};
Occupancy occupancy(const Candidate&,const PlanningSettings&);
std::optional<ReservationConflict> first_conflict(const Occupancy&,const Occupancy&);
Value conflict_record(const ReservationConflict&);
DepartureWindows departure_windows(const Candidate&,const Occupancy&,const std::vector<const Occupancy*>&,
    const PlanningSettings&);
double continuous_distance(const Candidate&,const Candidate&,double first_speed,double second_speed);
using Domain=std::vector<const Candidate*>;
using Domains=std::vector<Domain>;
Value verify_joint(const Domain&,const PlanningSettings&,const std::map<long long,PlanningSettings>& own={});
struct PairMetrics {std::array<long long,2> drones;std::size_t checks=0,conflicts=0;double worst=0.,required=0.;std::optional<double> closest;};
class JointSearch {
    using Key=std::array<long long,4>;
    PlanningSettings config_;
    std::map<long long,PlanningSettings> own_;
    std::map<Key,bool> cache_;
    std::map<Key,double> margins_;
    std::map<long long,std::size_t> wipeouts_;
    std::map<std::array<long long,2>,PairMetrics> pairs_;
    std::map<std::array<long long,2>,Witness> witnesses_;
    std::map<std::array<long long,2>,Occupancy> occupancies_;
    std::map<std::array<long long,2>,std::array<double,2>> costs_;
    std::optional<ReservationConflict> first_reservation_;
    std::size_t cache_hits_=0,reservation_conflicts_=0;
public:
    std::size_t nodes=0,optimization_nodes=0;
    bool exhausted=false,optimization_budget_exhausted=false;
    explicit JointSearch(PlanningSettings config,std::map<long long,PlanningSettings> own={}):config_(std::move(config)),own_(std::move(own)){}
    const PlanningSettings& own(long long id) const;
    double required(const Candidate&,const Candidate&) const;
    const Occupancy& occupied(const Candidate&);
    bool compatible(const Candidate&,const Candidate&);
    double margin(const Candidate&,const Candidate&);
    std::vector<double> score(const Domain&);
    std::array<double,2> cost(const Candidate&);
    std::vector<double> efficiency_score(const Domain&);
    Value pair_scores(const Domain&);
    std::optional<Domain> solve(const Domains&);
    bool optimization_available() const;
    std::pair<Domain,Value> refine(const Domains&,const Domain& incumbent);
    std::vector<std::size_t> expansion_indices(const std::vector<Catalog>&,const std::vector<Designation>&) const;
    std::vector<std::size_t> margin_expansion_indices(const std::vector<Catalog>&,const std::vector<Designation>&,const Domain&);
    std::map<long long,Repair> repair_hints(const std::vector<long long>&,const Domain* plans=nullptr);
    std::map<long long,DepartureWindows> interval_hints(const std::vector<Catalog>&,const std::vector<std::size_t>&,const Domain* plans=nullptr);
    Value diagnostics() const;
    Value verify(const Domain& plans) const{return verify_joint(plans,config_,own_);}
};
}
