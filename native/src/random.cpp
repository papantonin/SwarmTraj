#include "swarmtraj/random.hpp"
#include <bit>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace swarmtraj {
namespace {
UInt128 add(UInt128 a,UInt128 b) {
    const auto low=a.low+b.low;
    return {a.high+b.high+static_cast<std::uint64_t>(low<a.low),low};
}
UInt128 product64(std::uint64_t a,std::uint64_t b) {
    const std::uint64_t mask=0xffffffffULL;
    const auto a0=a&mask,a1=a>>32,b0=b&mask,b1=b>>32;
    const auto w0=a0*b0;
    const auto t=a1*b0+(w0>>32);
    const auto w1=(t&mask)+a0*b1;
    return {a1*b1+(t>>32)+(w1>>32),(w1<<32)+(w0&mask)};
}
UInt128 multiply(UInt128 a,UInt128 b) {
    auto value=product64(a.low,b.low);value.high+=a.high*b.low+a.low*b.high;return value;
}
UInt128 step(UInt128 state,UInt128 increment) {
    return add(multiply(state,{0x2360ed051fc65da4ULL,0x4385df649fccf645ULL}),increment);
}
}
void append_entropy(std::vector<std::uint32_t>& words,std::uint64_t value) {
    do{words.push_back(static_cast<std::uint32_t>(value));value>>=32;}while(value);
}
std::array<std::uint64_t,4> seed_sequence(const std::vector<std::uint32_t>& entropy) {
    std::array<std::uint32_t,4> pool{};
    std::uint32_t hash=0x43b0d7e5;
    auto mixhash=[&](std::uint32_t v){v^=hash;hash*=0x931e8875;v*=hash;return v^(v>>16);};
    auto mix=[](std::uint32_t a,std::uint32_t b){std::uint32_t v=0xca01f9dd*a-0x4973f715*b;return v^(v>>16);};
    for(std::size_t k=0;k<4;++k)pool[k]=mixhash(k<entropy.size()?entropy[k]:0);
    for(std::size_t src=0;src<4;++src)for(std::size_t dst=0;dst<4;++dst)
        if(src!=dst)pool[dst]=mix(pool[dst],mixhash(pool[src]));
    for(std::size_t src=4;src<entropy.size();++src)for(auto& v:pool)v=mix(v,mixhash(entropy[src]));
    std::array<std::uint32_t,8> words{};hash=0x8b51f9dd;
    for(std::size_t k=0;k<8;++k){auto v=pool[k%4]^hash;hash*=0x58f38ded;v*=hash;words[k]=v^(v>>16);}
    std::array<std::uint64_t,4> out;
    for(std::size_t k=0;k<4;++k)out[k]=static_cast<std::uint64_t>(words[2*k])|(static_cast<std::uint64_t>(words[2*k+1])<<32);
    return out;
}
Pcg64::Pcg64(const std::vector<std::uint32_t>& entropy) {
    const auto words=seed_sequence(entropy);
    value_.increment={(words[2]<<1)|(words[3]>>63),(words[3]<<1)|1};
    value_.state=step({},value_.increment);
    value_.state=step(add(value_.state,{words[0],words[1]}),value_.increment);
}
Pcg64::Pcg64(RandomState state):value_(state) {
    if(!(state.increment.low&1))throw std::invalid_argument("PCG64 increment must be odd");
}
std::uint64_t Pcg64::raw() {
    value_.state=step(value_.state,value_.increment);
    return std::rotr(value_.state.high^value_.state.low,static_cast<int>(value_.state.high>>58));
}
std::uint32_t Pcg64::uint32() {
    if(value_.has_uint32){value_.has_uint32=false;return value_.cached_uint32;}
    const auto value=raw();value_.has_uint32=true;value_.cached_uint32=static_cast<std::uint32_t>(value>>32);
    return static_cast<std::uint32_t>(value);
}
double Pcg64::uniform(double low,double high) {
    const double unit=static_cast<double>(raw()>>11)*(1./9007199254740992.);
    return low+(high-low)*unit;
}
std::uint64_t Pcg64::integer(std::uint64_t high) {
    if(!high)throw std::invalid_argument("empty bounded integer range");
    if(high==1)return 0;
    if(high==(1ULL<<32))return uint32();
    if(high<(1ULL<<32)) {
        const auto bound=static_cast<std::uint32_t>(high);
        auto product=static_cast<std::uint64_t>(uint32())*bound;
        auto leftover=static_cast<std::uint32_t>(product);
        if(leftover<bound) {
            const auto threshold=static_cast<std::uint32_t>(-bound)%bound;
            while(leftover<threshold){product=static_cast<std::uint64_t>(uint32())*bound;leftover=static_cast<std::uint32_t>(product);}
        }
        return product>>32;
    }
    auto product=product64(raw(),high);
    if(product.low<high) {
        const auto threshold=-high%high;
        while(product.low<threshold)product=product64(raw(),high);
    }
    return product.high;
}
}
