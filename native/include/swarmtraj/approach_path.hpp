#pragma once
#include "swarmtraj/flight.hpp"

namespace swarmtraj {
struct RouteSample {double x,y,heading,curvature,distance;};
// Shortest of four forward circle-straight-circle reference connections.
// This geometry is a proposal only: it is not an integrated aircraft plan.
std::vector<RouteSample> tangent_connection(const std::array<double,3>& initial,
    const std::array<double,3>& final,double radius,double spacing,double maximum_length);
}
