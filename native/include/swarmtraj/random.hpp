#pragma once
#include <array>
#include <cstdint>
#include <vector>

namespace swarmtraj {
// Independent implementation of NumPy SeedSequence/PCG64 compatibility.
// See docs/native_planner.md for the pinned algorithm sources and contract.
struct UInt128 {std::uint64_t high=0,low=0;};
struct RandomState {
    UInt128 state{},increment{};
    bool has_uint32=false;
    std::uint32_t cached_uint32=0;
};
std::array<std::uint64_t,4> seed_sequence(const std::vector<std::uint32_t>& entropy);
void append_entropy(std::vector<std::uint32_t>&,std::uint64_t);
class Pcg64 {
    RandomState value_;
public:
    explicit Pcg64(const std::vector<std::uint32_t>& entropy);
    explicit Pcg64(RandomState state);
    const RandomState& state() const {return value_;}
    std::uint64_t raw();
    std::uint32_t uint32();
    double uniform(double low,double high);
    std::uint64_t integer(std::uint64_t exclusive_high);
};
}
