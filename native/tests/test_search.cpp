#include "swarmtraj/search.hpp"
#include <iostream>
#include <stdexcept>
using namespace swarmtraj;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main() {
    PlanningSettings c;c.physics={6,1.15,2.5,.2,45*pi/180,8*pi/180,5,5,.003,20,8,16,50*pi/180,60*pi/180,0,200,3,10,1,.2};
    c.steps=20;c.activation=0;c.max_delay=0;c.latest_arrival=3.;c.descent_degrees=0;c.target={24,0};c.target_altitude=80;c.margin_nodes=100;
    const Control trim{.003*144+20./144,0,1,0};
    auto make=[&](long long drone,long long candidate,double y) {
        Candidate p;p.drone_id=drone;p.candidate_id=candidate;p.cruise=80;p.arrival=2;
        p.states.push_back({0,y,80,12,0,0,0});
        for(int k=0;k<20;++k){p.controls.push_back(trim);p.states.push_back(propagate(p.states.back(),trim,.2,c.physics));}
        for(const auto& state:p.states)p.projected.push_back(project_state(state,trim,c.physics));
        p.acceleration.resize(20);p.vertical_acceleration.resize(20);return p;
    };
    const auto a=make(0,0,0),b=make(0,1,30),d=make(1,0,5),e=make(1,1,60),f=make(2,0,10),g=make(2,1,90);
    Domains catalogs{{&a,&b},{&d,&e},{&f,&g}};JointSearch search(c);
    auto selected=search.solve(catalogs);require(selected&&selected->size()==3,"native joint search failed");
    auto [best,quality]=search.refine(catalogs,*selected);require(best.size()==3,"margin refinement lost incumbent");
    search.verify(best);
    c.max_nodes=1;JointSearch bounded(c);require(!bounded.solve(catalogs)&&bounded.exhausted&&bounded.nodes==1,"search budget changed");
    c.max_nodes=100;c.reservations=true;JointSearch grid(c);const auto overlap=make(1,2,25);
    require(!grid.compatible(a,overlap),"closed grid overlap accepted");
    bool rejected=false;try{grid.verify({&a,&overlap});}catch(const std::runtime_error&){rejected=true;}
    require(rejected,"independent reservation check trusted cached selection");
    std::cout<<"native joint search and independent verification tests passed\n";
}
