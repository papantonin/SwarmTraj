#include "swarmtraj/validation.hpp"
#include <iostream>
#include <stdexcept>
using namespace swarmtraj;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main() {
    Parameters p{6,1.15,2.5,.2,45*pi/180,8*pi/180,5,5,.003,20,8,16,50*pi/180,60*pi/180,0,200,3,10,1,.2};
    State initial{0,0,80,12,0,0,0};Control trim{.003*144+20./144,0,1,0};
    Candidate path;path.speed=12;path.dt=.2;path.cruise=80;path.arrival=2;path.states.push_back(initial);
    for(int k=0;k<20;++k){path.controls.push_back(trim);path.states.push_back(propagate(path.states.back(),trim,.2,p));}
    for(const auto& state:path.states)path.projected.push_back(project_state(state,trim,p));
    path.acceleration.resize(20);path.vertical_acceleration.resize(20);
    ValidationSettings c;c.physics=p;c.steps=20;c.activation=0;c.max_delay=0;c.latest_arrival=3.85;
    c.target={24,0};c.target_altitude=80;c.descent_degrees=0;
    auto result=validate_candidate(path,initial,0,{},c);
    require(result.accepted()&&result.contact_time&&std::abs(*result.contact_time-1.5)<1e-6,"valid trajectory rejected");
    auto corrupted=path;corrupted.states.back()[0]+=.01;
    require(validate_candidate(corrupted,initial,0,{},c).rejection=="validation_integration","corrupted history accepted");
    corrupted=path;corrupted.projected[2][0]+=.01;
    require(validate_candidate(corrupted,initial,0,{},c).rejection=="validation_projection","corrupted projection accepted");
    c.obstacles.push_back({false,{48,0,80},{2,2,2}});
    require(validate_candidate(path,initial,0,{},c).rejection=="obstacle_clearance","post-arrival obstacle accepted");
    std::cout<<"independent native validation tests passed\n";
}
