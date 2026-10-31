#pragma once
#include "swarmtraj/random.hpp"
#include "swarmtraj/validation.hpp"
#include <map>
#include <deque>
#include <memory>
#include <type_traits>
#include <variant>

namespace swarmtraj {
struct Value {
    using Array=std::vector<Value>;
    using Object=std::map<std::string,Value>;
    std::variant<std::monostate,bool,long long,double,std::string,Array,Object> data;
    Value()=default;
    Value(bool v):data(v){}
    template<class T,std::enable_if_t<std::is_integral_v<T>&&!std::is_same_v<T,bool>,int> =0>
    Value(T v):data(static_cast<long long>(v)){}
    Value(double v):data(v){}
    Value(const char* v):data(std::string(v)){}
    Value(std::string v):data(std::move(v)){}
    Value(Array v):data(std::move(v)){}
    Value(Object v):data(std::move(v)){}
    Value& operator[](const std::string& key){if(std::holds_alternative<std::monostate>(data))data=Object{};return std::get<Object>(data)[key];}
    const Value& at(const std::string& key) const{return std::get<Object>(data).at(key);}
};
inline Value object(std::initializer_list<std::pair<const std::string,Value>> values){return Value::Object(values);}
inline Value array(std::initializer_list<Value> values){return Value::Array(values);}
template<class T> Value values(const std::vector<T>& items){Value::Array result;for(const auto& v:items)result.emplace_back(v);return result;}
struct Designation {
    long long drone_id=0;
    State initial{};
    double yaw_rate=0.;
    std::optional<double> cruise;
};
struct PlanningSettings:ValidationSettings {
    std::vector<std::uint32_t> seed{7};
    std::size_t candidates=28, attempts=90, initial_candidates=4, max_nodes=30000;
    std::size_t margin_nodes=2000, margin_rounds=1, max_entries=1000000, max_constraints=50000;
    double separation=8.,detour=45.,level_hold=3.,cell_size=20.,post_arrival=5.,horizon=90.,max_path_degrees=45.;
    // Proposal-only extension after own alignment rejection; never a timing allowance.
    double approach_lead_extra_s=0.;
    bool tangent_approach=false;
    std::optional<double> deadline;
    bool spatial=false,adaptive=false,mrv=false,max_margin=false,witnesses=false;
    bool reservations=false,windows=false,mixed=false,rejection_feedback=false,id_approach=false;
};
struct Witness {
    std::array<long long,2> drones{},candidates{};
    double time=0.,lower=0.,required=0.;
    std::array<std::array<double,3>,2> positions{};
    double slack() const{return lower-required;}
};
struct Repair {
    Witness conflict;
    long long drone_id=0;
    std::size_t side() const{return conflict.drones[0]==drone_id?0:1;}
};
struct DepartureWindows {
    long long parent=0;
    std::vector<long long> references;
    std::vector<Point> intervals;
    std::size_t constraints=0;
    bool exhausted=false;
};
struct Catalog {
    std::deque<Candidate> plans; // Appending never invalidates selected plan references.
    std::size_t attempted=0,profile_stage=0,profile_cursor=0,profile_failures=0;
    std::optional<RandomState> rng;
    std::map<std::string,std::size_t> rejections,details;
    double generation_s=0.,validation_s=0.;
    std::optional<double> minimum_horizon;
    Value::Array guided,interval_contexts,interval_proposals,profile_events,profile_proposals,adjustments;
};
struct ProposalResult {
    std::optional<Candidate> plan;
    std::string rejection;
};
Value witness_record(const Witness&);
Value window_record(const DepartureWindows&);
ValidationResult validate_individual(const Candidate&,const Designation&,const PlanningSettings&);
std::vector<Point> avoidance_route(Point start,const std::vector<Point>& waypoints,
    const PlanningSettings&,double speed,long long candidate_id);
ProposalResult physical_proposal(const Designation&,const PlanningSettings&,long long candidate_id,
    Pcg64&,std::optional<double> approach,const Repair* repair=nullptr,const std::string& mode="",
    std::optional<double> departure={},std::optional<double> level_hold={},double extra_clearance=0.);
void extend_catalog(Catalog&,const Designation&,const PlanningSettings&,std::size_t goal,
    std::optional<double> approach,const Repair* repair=nullptr,const DepartureWindows* windows=nullptr,
    const std::string& expansion="initial");
Value catalog_diagnostics(const Catalog&,const Designation&,const PlanningSettings&);
}
