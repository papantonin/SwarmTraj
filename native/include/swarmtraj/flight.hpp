#pragma once
#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace swarmtraj {
using State = std::array<double, 7>;
using Control = std::array<double, 4>;
using Point = std::array<double, 2>;
inline constexpr double gravity = 9.81;
inline constexpr double pi = 3.141592653589793238462643383279502884;
struct Parameters {
    double stall_speed, stall_margin, max_load, min_load, max_path, max_path_rate;
    double max_thrust, max_brake, drag0, drag_lift;
    double min_speed, max_speed, max_bank, max_roll;
    double min_altitude, max_altitude, max_climb, max_descent, max_vertical_acceleration, dt;
};
struct Phase { std::string name; std::size_t start, end; bool operator==(const Phase&) const = default; };
struct Path {
    std::vector<State> states;
    std::vector<Control> controls;
    std::vector<Phase> phases;
    std::string rejection;
    double arrival = 0.;
    std::vector<double> speed_reference;
    bool accepted() const { return rejection.empty(); }
};
struct Recipe {
    std::size_t steps;
    double speed, yaw_rate, cruise, release, minimum_hold, speed_end, heading, angle;
    double target_z, target_radius, position_error, latest_arrival;
    bool rich;
    bool tangent_approach=false;
    bool operating_schedule=false;
    std::array<double,5> phase_speeds{};
    double speed_acceleration=.3;
    Point gate, direction, normal, target;
    std::vector<double> levels, speed_reference, speed_derivative;
    std::vector<Point> waypoints;
};
double wrap(double angle);
void validate(const Parameters& p);
State derivative(const State& s, const Control& u, const Parameters& p,
                 const std::array<double,3>& wind = {});
State propagate(State state, const Control& u, double duration, const Parameters& p,
                unsigned subdivisions = 2, const std::array<double,3>& wind = {});
bool envelope(const State& s, const Control& u, const Parameters& p, double tolerance=1e-7);
bool checked_step(const State& s, const Control& u, const Parameters& p, State& following);
Control guidance(const State& s, const State& reference, double yaw, double speed_ff, const Parameters& p);
Path rollout(const State& initial, const std::vector<Control>& commands, const Parameters& p,
             bool check, unsigned subdivisions=2, const std::array<double,3>& wind={});
Path pullout(const State& initial, const Parameters& p, std::size_t max_steps);
Path prefix(const State& initial, const Parameters& p, const Recipe& r);
Path onward(const State& initial, const Parameters& p, const Recipe& r,
            std::size_t first_step, const std::vector<Point>& waypoints);
bool terminal_distance(double speed, double cruise, const Parameters& p, const Recipe& r, double& distance);
bool headroom(double speed, const Parameters& p, const Recipe& r, double& loss);
}
