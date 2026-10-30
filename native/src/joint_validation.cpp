#include "swarmtraj/search.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace swarmtraj {
Value verify_joint(const Domain& plans,const PlanningSettings& config,const std::map<long long,PlanningSettings>& aircraft) {
    auto own=[&](long long id)->const PlanningSettings& {
        if(aircraft.empty())return config;
        auto found=aircraft.find(id);if(found==aircraft.end())throw std::invalid_argument("missing own aircraft configuration");return found->second;
    };
    Value::Array checks,obstacles;std::optional<double> minimum,maximum;
    for(std::size_t a=0;a<plans.size();++a)for(std::size_t b=a+1;b<plans.size();++b) {
        const auto& first=*plans[a];const auto& second=*plans[b];const auto& ca=own(first.drone_id);const auto& cb=own(second.drone_id);
        const double lower=continuous_distance(first,second,first.max_speed,second.max_speed);
        const double required=(config.separation+ca.aircraft_radius+cb.aircraft_radius)+2*config.position_error+(first.max_speed+second.max_speed)*config.clock_error;
        if(lower<required)throw std::runtime_error("joint verification failed");
        minimum=minimum?std::min(*minimum,lower):lower;maximum=maximum?std::max(*maximum,required):required;
        checks.push_back(array({lower,required}));
    }
    Value result=object({{"checks",checks},{"minimum_distance_lower_bound",minimum?Value(*minimum):Value()},
        {"required_distance",maximum?Value(*maximum):Value()}});
    if(config.reservations) {
        std::vector<Occupancy> allocated;for(auto* p:plans)allocated.push_back(occupancy(*p,own(p->drone_id)));
        for(std::size_t a=0;a<allocated.size();++a)for(std::size_t b=a+1;b<allocated.size();++b)
            if(first_conflict(allocated[a],allocated[b]))throw std::runtime_error("independent reservation verification failed");
        Value::Array allocations;
        for(const auto& p:allocated) {
            std::size_t intervals=0;for(const auto& [cell,times]:p.cells)intervals+=times.size();
            allocations.push_back(object({{"drone_id",p.drone_id},{"candidate_id",p.candidate_id},{"cells",p.cells.size()},
                {"intervals",intervals},{"inflation_m",p.inflation}}));
        }
        result["reservations"]=object({{"policy","grid"},{"cell_size_m",config.cell_size},{"full_horizon_verified",true},{"allocated",allocations}});
    }
    for(auto* p:plans)if(!config.obstacles.empty()) {
        const auto& c=own(p->drone_id);const double lower=clearance(*p,config.obstacles)-p->max_speed*p->dt/2;
        const double required=c.aircraft_radius+config.obstacle_clearance+config.position_error+p->max_speed*config.clock_error;
        if(lower<required)throw std::runtime_error("independent obstacle verification failed");
        obstacles.push_back(object({{"drone_id",p->drone_id},{"lower_bound_m",lower},{"required_m",required}}));
    }
    result["obstacle_checks"]=obstacles;return result;
}
}
