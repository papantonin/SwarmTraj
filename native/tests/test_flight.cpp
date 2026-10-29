#include "swarmtraj/flight.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace swarmtraj;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main() {
    Parameters p{6,1.15,2.5,.2,45*pi/180,8*pi/180,5,5,.003,20,8,16,50*pi/180,60*pi/180,0,200,3,10,1,.2};
    validate(p);State s{0,0,80,12,0,0,0};Control u{.003*144+20./144,0,1,0};
    const auto d=derivative(s,u,p);require(std::abs(d[0]-12)<1e-14,"trim forward velocity");require(std::abs(d[3])<1e-14,"trim acceleration");
    auto path=rollout(s,std::vector<Control>(500,u),p,true);require(path.accepted(),"steady rollout rejected");
    require(std::abs(path.states.back()[0]-1200)<1e-8,"steady manufactured displacement");
    require(std::abs(path.states.back()[2]-80)<1e-12,"steady altitude");
    u[2]=100;State end;require(!checked_step(s,u,p,end),"unsafe held load accepted");
    u={.003*144+20./144,0,1,0};s[3]=7;require(!envelope(s,u,p),"underspeed accepted");
    require(wrap(pi)==-pi && wrap(-pi)==-pi,"heading wrap boundary");
    s={0,0,80,13,0,-pi/6,0};auto recovered=pullout(s,p,300);
    require(recovered.accepted(),"pullout fixture rejected");require(std::abs(recovered.states.back()[5])<=1e-4,"unfinished pullout");
    s[6]=.01;require(!pullout(s,p,300).accepted(),"banked pullout accepted");
    std::cout<<"native numerical tests passed\n";
}
