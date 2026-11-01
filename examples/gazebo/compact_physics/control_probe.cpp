#include "control_dynamics.hpp"
#include <iostream>
#include <iomanip>
int main(int argc,char** argv) {
 try {
    // fixed ordered SI/scalar interface; Python owns strict versioned JSON binding.
    if(argc!=36)throw std::invalid_argument("expected 35 force/stability/state/control scalars");
    double a[35];for(int k=0;k<35;++k)a[k]=std::stod(argv[k+1]);
    compact::Polar p{a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7]};
    compact::Stability s{a[8],a[9],a[10],a[11],a[12],a[13],a[14],a[15],a[16],a[17],a[18],a[19],a[20]};
    auto u=compact::positions({a[27],a[28],a[29],a[30],a[31]},a[32],a[33]);
    // Last scalar is reserved for explicit interface version, never ignored.
    if(a[34]!=1)throw std::invalid_argument("unsupported control probe interface");
    auto f=compact::wrench(p,s,{a[21],a[22],a[23]},{a[24],a[25],a[26]},u);
    std::cout<<std::setprecision(17);
    for(double x:f.aerodynamic.force)std::cout<<x<<' ';
    for(double x:f.torque)std::cout<<x<<' ';
    std::cout<<u.aileron<<' '<<u.elevator<<' '<<u.rudder<<' '<<u.brake<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
