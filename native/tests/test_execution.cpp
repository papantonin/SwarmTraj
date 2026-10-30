#include "swarmtraj/execution.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace swarmtraj;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main() {
    Parameters p{6,1.15,2.5,.2,45*pi/180,8*pi/180,5,5,.003,20,8,16,50*pi/180,60*pi/180,0,200,3,10,1,.2};
    State s{0,0,80,12,0,0,0};Control base{.003*144+20./144,0,1,0};
    const auto nominal=local_control_update(s,s,base,nullptr,{},p,.2,.2);
    require(nominal.feasible&&!nominal.intervened,"nominal local controller rejected trim");
    for(std::size_t j=0;j<4;++j)require(std::abs(nominal.control[j]-base[j])<1e-12,"nominal command changed");
    auto sampled=propagate_samples(s,base,{0.,.1,.2},p,4,{.2,-.1,0.});
    require(std::abs(sampled.back()[0]-2.44)<1e-12,"independent simulator samples / wind");
    auto measured=s;measured[0]+=.03;measured[1]+=.02;
    OwnHistory prior{measured,base};measured=sampled.back();measured[0]+=.03;measured[1]+=.02;
    const auto observed=local_control_update(measured,propagate(s,base,.2,p,4),base,&prior,{},p,.2,.2);
    const double alpha=1-std::exp(-.2/1.5);
    require(std::abs(observed.estimated_wind[0]-.2*alpha)<1e-12,"own-motion estimate / fixed bias");
    require(std::abs(observed.estimated_wind[1]+.1*alpha)<1e-12,"horizontal drift estimate");
    auto unsafe=base;unsafe[2]=2.5;
    require(!held_control_ok(s,unsafe,p,.2),"unsafe held load accepted");
    auto protected_request=protect_control(s,unsafe,p,.2);
    require(protected_request.intervened&&protected_request.feasible,"safe fallback not found");
    require(held_control_ok(s,protected_request.control,p,.2),"unverified fallback retained");
    s[2]=-1.;auto failed=protect_control(s,unsafe,p,.2);
    require(failed.intervened&&!failed.feasible,"failed fallback marked feasible");
    require(s[2]==-1.,"physical state clipped");
    std::cout<<"native own-state execution tests passed\n";
}
