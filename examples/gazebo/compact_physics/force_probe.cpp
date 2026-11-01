#include "aerodynamics.hpp"
#include <iostream>
#include <iomanip>
int main(int argc,char** argv) {
 try {
    if(argc!=12&&argc!=14)throw std::invalid_argument("density area cd0 k clmax brake_max vx vy vz elevator brake [lift_slope zero_lift_angle]");
    compact::Polar p{std::stod(argv[1]),std::stod(argv[2]),std::stod(argv[3]),std::stod(argv[4]),std::stod(argv[5]),std::stod(argv[6])};
    if(argc==14){p.lift_slope=std::stod(argv[12]);p.zero_lift_angle=std::stod(argv[13]);}
    auto f=compact::evaluate(p,{std::stod(argv[7]),std::stod(argv[8]),std::stod(argv[9])},std::stod(argv[10]),std::stod(argv[11]));
    std::cout<<std::setprecision(17)<<f.force[0]<<' '<<f.force[1]<<' '<<f.force[2]<<' '<<f.cl<<' '<<f.lift<<' '<<f.drag<<' '<<f.brake<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
