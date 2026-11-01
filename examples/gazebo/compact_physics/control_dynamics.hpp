#pragma once
#include "aerodynamics.hpp"
namespace compact {
struct Controls {double aileron=0,elevator=0,rudder=0,brake=0;};
struct Stability {
    double span,chord,alpha_trim;
    double roll_beta,roll_damping,roll_control;
    double pitch_stiffness,pitch_damping,pitch_control;
    double yaw_stiffness,yaw_damping,yaw_control,sideforce;
};
inline void validate(const Stability& s) {
    for(double x:{s.span,s.chord,s.alpha_trim,s.roll_beta,s.roll_damping,s.roll_control,
        s.pitch_stiffness,s.pitch_damping,s.pitch_control,s.yaw_stiffness,s.yaw_damping,s.yaw_control,s.sideforce})
        if(!std::isfinite(x))throw std::invalid_argument("nonfinite stability coefficient");
    if(s.span<=0||s.chord<=0||s.roll_damping<0||s.pitch_damping<0||s.yaw_damping<0||s.sideforce<0)
        throw std::invalid_argument("invalid stability geometry/damping");
}
// Pinned Cessna allocation: positive roll -> left negative / right positive;
// positive elevator/rudder -> FRD pitch-up/yaw-right, i.e. negative FLU Y/Z.
inline Controls positions(const std::array<double,5>& q,double surface_limit,double brake_limit) {
    if(!std::isfinite(surface_limit)||!std::isfinite(brake_limit)||surface_limit<=0||brake_limit<=0)
        throw std::invalid_argument("invalid actuator travel");
    for(double x:q)if(!std::isfinite(x))throw std::invalid_argument("nonfinite own joint position");
    const auto scaled=[&](double x){return std::clamp(x/surface_limit,-1.,1.);};
    return {(scaled(q[1])-scaled(q[0]))/2,scaled(q[2]),scaled(q[3]),std::clamp(q[4]/brake_limit,0.,1.)};
}
struct Wrench {Forces aerodynamic;Vec torque{};double beta=0;};
inline Wrench wrench(const Polar& p,const Stability& s,Vec velocity,Vec rates,Controls u) {
    validate(s);
    for(double x:rates)if(!std::isfinite(x))throw std::invalid_argument("nonfinite own angular rate");
    for(double x:{u.aileron,u.elevator,u.rudder})if(!std::isfinite(x)||std::abs(x)>1)
        throw std::invalid_argument("invalid control fraction");
    Wrench out;out.aerodynamic=evaluate(p,velocity,u.elevator,u.brake);
    double v2=0;for(double x:velocity)v2+=x*x;
    if(v2<1e-12)return out;
    const double v=std::sqrt(v2),qS=.5*p.density*v2*p.area;
    out.beta=std::atan2(velocity[1],std::hypot(velocity[0],velocity[2]));
    out.aerodynamic.force[1]-=qS*s.sideforce*out.beta;
    out.torque={qS*s.span*(s.roll_beta*out.beta-s.roll_damping*rates[0]*s.span/(2*v)+s.roll_control*u.aileron),
        qS*s.chord*(s.pitch_stiffness*(out.aerodynamic.alpha-s.alpha_trim)-s.pitch_damping*rates[1]*s.chord/(2*v)-s.pitch_control*u.elevator),
        qS*s.span*(s.yaw_stiffness*out.beta-s.yaw_damping*rates[2]*s.span/(2*v)-s.yaw_control*u.rudder)};
    for(double x:out.torque)if(!std::isfinite(x))throw std::invalid_argument("torque out of numerical range");
    for(double x:out.aerodynamic.force)if(!std::isfinite(x))throw std::invalid_argument("force out of numerical range");
    return out;
}
}
