#pragma once
#include "swarmtraj/flight.hpp"
#include <optional>

namespace swarmtraj {
using Projection = std::array<double,6>;
struct Solid {
    bool cylinder = false;
    std::array<double,3> center{}, half{};
};
struct Candidate {
    long long drone_id=0, candidate_id=0;
    double speed=12., dt=.2, release=0., arrival=0.;
    std::optional<double> cruise, approach;
    std::vector<State> states;
    std::vector<Control> controls;
    std::vector<Projection> projected;
    std::vector<double> acceleration, vertical_acceleration, levels, speed_reference;
    bool has_speed_reference=false;
    std::string family, routing_mode="lateral";
    std::vector<Phase> phases;
};
struct ValidationSettings {
    Parameters physics{};
    std::size_t steps=0;
    double activation=3., min_delay=0., max_delay=20., latest_arrival=0.;
    Point target{};
    std::optional<double> target_altitude;
    double target_radius=6., position_error=2., descent_degrees=30., spread_degrees=0.;
    std::optional<Point> cruise_range;
    std::size_t cruise_levels=1;
    bool smooth_speed=false;
    Point speed_range{};
    double speed_acceleration=.5, clock_error=.15, aircraft_radius=0., obstacle_clearance=5.;
    std::vector<Solid> obstacles;
};
struct ValidationResult {
    std::string rejection;
    std::optional<double> contact_time;
    bool accepted() const { return rejection.empty(); }
};
Projection project_state(const State&, const Control&, const Parameters&);
State sample_candidate(const Candidate&, double time, const Parameters&);
double solid_distance(const Solid&, const State&);
double clearance(const Candidate&, const std::vector<Solid>&);
// Independent full-horizon replay: never relies on construction acceptance.
ValidationResult validate_candidate(const Candidate&, const State& snapshot, double yaw_rate,
    std::optional<double> requested_cruise, const ValidationSettings&);
}
