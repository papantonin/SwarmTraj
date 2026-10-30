#include "swarmtraj/mission.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace swarmtraj;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main() {
    PlanningSettings c;c.physics={6,1.15,2.5,.2,45*pi/180,8*pi/180,5,5,.003,20,8,16,50*pi/180,60*pi/180,0,200,3,8,1,.2};
    c.steps=600;c.horizon=120;c.latest_arrival=114.85;c.target={500,0};c.target_altitude=30;
    c.cruise_range=Point{70,100};c.max_delay=0;c.attempts=3;c.candidates=1;
    Designation s;s.initial={25,0,80,13,pi/2,0,std::atan(13*.52/gravity)};s.yaw_rate=.52;
    auto mission=plan_mission({s},{c});
    require(mission.status=="validated"&&mission.plans.size()==1,"complete native mission failed");
    require(mission.plans[0].states.size()==601&&mission.search_nodes==1,"mission horizon/search changed");
    require(validate_candidate(mission.plans[0],s.initial,s.yaw_rate,{},c).accepted(),"native mission independent replay failed");
    auto bad_family=mission.plans[0];bad_family.family="unrequested";
    require(validate_individual(bad_family,s,c).rejection=="validation_profile_family","native validation accepted undeclared profile family");
    auto bad_arrival=mission.plans[0];bad_arrival.arrival=c.horizon;
    require(validate_individual(bad_arrival,s,c).detail=="post_arrival_window_exceeds_horizon","arrival timing detail lost");
    auto deadline=c;deadline.deadline=20;
    require(validate_individual(bad_arrival,s,deadline).detail=="arrival_deadline_exceeded","absolute deadline detail lost");
    auto rejected=c;rejected.physics.max_brake=0;
    require(plan_mission({s},{rejected}).status=="descent_not_feasible","native preflight softened insufficient airbrake");
    bool duplicate=false;try{plan_mission({s,s},{c,c});}catch(const std::invalid_argument&){duplicate=true;}
    require(duplicate,"duplicate drone IDs accepted");
    std::cout<<"complete standalone native mission tests passed\n";
}
