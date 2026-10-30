#pragma once
#include "swarmtraj/search.hpp"

namespace swarmtraj {
struct MissionResult {
    std::string status,reason;
    std::vector<Candidate> plans;
    std::optional<double> minimum_distance,required_distance;
    std::vector<std::size_t> counts;
    std::size_t search_nodes=0;
    Value diagnostics=Value::Object{},timings=Value::Object{};
};
// Inputs are frozen designation snapshots and complete resolved own settings.
// No callbacks, telemetry, Python objects or construction-acceptance shortcut.
MissionResult plan_mission(std::vector<Designation>,std::vector<PlanningSettings>);
std::map<long long,std::optional<double>> approach_angles(const std::vector<Designation>&,
    const std::vector<PlanningSettings>&);
}
