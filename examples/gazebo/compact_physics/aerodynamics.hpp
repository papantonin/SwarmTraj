#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
namespace compact {
using Vec=std::array<double,3>;
struct Polar {double density,area,cd0,k,clmax,max_brake,lift_slope=4.75,zero_lift_angle=-.06;};
struct Forces {Vec force{};double alpha=0,cl=0,lift=0,drag=0,brake=0;};
// Body axes: forward X, left Y, up Z. Air-relative velocity, SI units.
// Ideal constant-force brake matches the planner assumption, not a hardware model.
inline Forces evaluate(const Polar& p,Vec velocity,double elevator,double brake) {
    for(double v:velocity)if(!std::isfinite(v))throw std::invalid_argument("nonfinite air velocity");
    if(!std::isfinite(elevator)||!std::isfinite(brake)||brake<0||brake>1||std::abs(elevator)>1)
        throw std::invalid_argument("invalid actuator fraction");
    for(double v:{p.density,p.area,p.cd0,p.k,p.clmax,p.max_brake,p.lift_slope,p.zero_lift_angle})
        if(!std::isfinite(v))throw std::invalid_argument("nonfinite polar");
    if(p.density<=0||p.area<=0||p.cd0<0||p.k<0||p.clmax<=0||p.max_brake<0||p.lift_slope<=0)
        throw std::invalid_argument("invalid polar");
    Forces f;double v2=0;for(double x:velocity)v2+=x*x;
    if(!std::isfinite(v2))throw std::invalid_argument("air velocity out of numerical range");
    if(v2<1e-12)return f;
    const double speed=std::sqrt(v2),q=.5*p.density*v2;
    f.alpha=std::atan2(-velocity[2],velocity[0]);
    f.cl=std::clamp(p.lift_slope*(f.alpha-p.zero_lift_angle)+elevator,-p.clmax,p.clmax);
    f.lift=q*p.area*f.cl;f.drag=q*p.area*(p.cd0+p.k*f.cl*f.cl);f.brake=p.max_brake*brake;
    Vec direction{};for(int j=0;j<3;++j)direction[j]=velocity[j]/speed;
    Vec up{-direction[2]*direction[0],-direction[2]*direction[1],1-direction[2]*direction[2]};
    double norm=0;for(double x:up)norm+=x*x;norm=std::sqrt(norm);
    // Lift vanishes for flow parallel to body Z; this bench is not post-stall physics.
    for(int j=0;j<3;++j)f.force[j]=-(f.drag+f.brake)*direction[j]+(norm>1e-9?f.lift*up[j]/norm:0.);
    for(double x:f.force)if(!std::isfinite(x))throw std::invalid_argument("force out of numerical range");
    return f;
}
}
