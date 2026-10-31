#include "swarmtraj/approach_path.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace swarmtraj;
void require(bool x,const char* message){if(!x)throw std::runtime_error(message);}
int main(){
    for(const std::array<double,3> goal: {std::array<double,3>{1000,0,0}, {1000,1000,pi/2},
        {1000,-1000,-pi/2},{-500,100,pi},{0,0,0},{200,200,pi/2}}) {
        auto path=tangent_connection({0,0,0},goal,200,5,10000);
        require(!path.empty(),"CSC path missing");
        require(std::hypot(path.back().x-goal[0],path.back().y-goal[1])<1e-6,"wrong endpoint");
        require(std::abs(wrap(path.back().heading-goal[2]))<1e-9,"wrong terminal heading");
        for(std::size_t k=1;k<path.size();++k){
            const auto& a=path[k-1];const auto& b=path[k];
            require(std::abs(b.curvature)<=1./200+1e-12,"curvature exceeds bound");
            require(std::hypot(b.x-a.x,b.y-a.y)<=5+1e-8,"sampling gap exceeds bound");
            require(b.distance>=a.distance,"non-monotone path progress");
        }
    }
    require(tangent_connection({0,0,0},{1000,0,0},200,5,100).empty(),"length budget ignored");
    bool invalid=false;try{tangent_connection({0,0,0},{1000,0,0},0,5,1000);}catch(const std::invalid_argument&){invalid=true;}
    require(invalid,"invalid radius accepted");
    std::cout<<"bounded tangent geometry tests passed\n";
}
