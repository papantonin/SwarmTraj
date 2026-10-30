#pragma once
#include "swarmtraj/flight.hpp"

namespace swarmtraj {
struct OwnReference {
    std::vector<State> states;
    std::vector<Control> controls;
    Parameters physics;
};
struct ReferenceWindow {
    State state;
    Control feedforward;
};
// Immutable own plan only. No fleet, disturbance or observer input is accepted.
ReferenceWindow sample_reference(const OwnReference&, double local_time, double local_duration);

struct OwnHistory {
    State measured;
    Control applied;
};
struct ProtectionResult {
    Control control;
    bool intervened;
    bool feasible;
};
struct LocalControlResult {
    Control control;
    std::array<double,3> estimated_wind;
    bool intervened;
    bool feasible;
};

Control clip_execution_control(const State&, const Control&, const Parameters&, double hold_dt);
bool held_control_ok(const State&, const Control&, const Parameters&, double hold_dt);
ProtectionResult protect_control(const State&, const Control&, const Parameters&, double hold_dt);

// Own measured state, own reference and own estimator history only. No true
// simulator disturbance, peer state, observer state or fleet configuration.
LocalControlResult local_control_update(const State& measured, const State& reference,
    const Control& base, const OwnHistory* previous,
    std::array<double,3> estimated_wind, const Parameters&,
    double measurement_dt, double hold_dt);

// Simulator physics, deliberately separate from the controller interface.
// Every duration is independently integrated from the supplied initial state.
std::vector<State> propagate_samples(const State&, const Control&, const std::vector<double>& durations,
    const Parameters&, unsigned subdivisions, const std::array<double,3>& disturbance);
}
