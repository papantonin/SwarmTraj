#include "swarmtraj/execution.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace swarmtraj {
namespace {
double clip(double value, double low, double high) {
    // NumPy clip semantics also when a failed state yields an empty interval.
    return std::min(std::max(value,low),high);
}
double ceiling(double speed, const Parameters& p) {
    return std::min(p.max_load,std::pow(speed/(p.stall_speed*p.stall_margin),2));
}
double drag(double speed, double load, const Parameters& p) {
    return p.drag0*speed*speed+p.drag_lift*load*load/(speed*speed);
}
Control trim(const State& s, const Parameters& p) {
    const double load=std::cos(s[5])/std::cos(s[6]);
    const double axial=drag(s[3],load,p)+gravity*std::sin(s[5]);
    return {std::max(0.,axial),std::max(0.,-axial),load,0.};
}
void check_duration(double dt) {
    if(!std::isfinite(dt)||dt<=0)throw std::invalid_argument("execution interval must be finite and positive");
}
Control tracking_guidance(const State& s, const State& r, const State& ff,
    const Parameters& p, double dt, const std::array<double,3>* estimated_wind) {
    const double speed=s[3],gamma=s[5],bank=s[6];
    const double horizontal=r[3]*std::cos(r[5]);
    std::array<double,3> velocity{horizontal*std::cos(r[4])+.8*(r[0]-s[0]),
        horizontal*std::sin(r[4])+.8*(r[1]-s[1]),r[3]*std::sin(r[5])+.8*(r[2]-s[2])};
    const double air_path=std::atan2(velocity[2],std::sqrt(velocity[0]*velocity[0]+velocity[1]*velocity[1]));
    if(estimated_wind)for(std::size_t j=0;j<3;++j)velocity[j]-=(*estimated_wind)[j];
    const double heading=std::atan2(velocity[1],velocity[0]);
    const double yaw=ff[4]+1.5*wrap(heading-s[4]);
    const double desired_gamma=estimated_wind?air_path:std::atan2(velocity[2],std::sqrt(velocity[0]*velocity[0]+velocity[1]*velocity[1]));
    double gamma_rate=ff[5]+1.5*(desired_gamma-gamma);
    const double up=std::min(p.max_path,std::asin(std::min(1.,.98*p.max_climb/speed)));
    const double down=std::min(p.max_path,std::asin(std::min(1.,.98*p.max_descent/speed)));
    const double max_pitch=std::min(p.max_path_rate,.95*p.max_vertical_acceleration/std::max(speed,1e-6));
    gamma_rate=clip(gamma_rate,std::max(-max_pitch,(-down-gamma)/dt),std::min(max_pitch,(up-gamma)/dt));
    const double lift_vertical=gravity*std::cos(gamma)+speed*gamma_rate;
    double desired_bank=std::atan2(speed*std::cos(gamma)*yaw,lift_vertical);
    const double cap=ceiling(speed,p);
    const double max_bank=std::min(p.max_bank,std::acos(clip(lift_vertical/(gravity*cap),0.,1.)));
    desired_bank=clip(desired_bank,-max_bank,max_bank);
    double roll=clip((desired_bank-bank)/dt,-p.max_roll,p.max_roll);
    const double budget=.85*p.max_vertical_acceleration/(gravity*std::cos(gamma));
    double lower=0.,upper=0.;
    for(int k=0;k<8;++k) {
        const double end=bank+roll*dt;
        const double min_cos=std::min(std::cos(bank),std::cos(end));
        const double max_cos=bank*end<=0?1.:std::max(std::cos(bank),std::cos(end));
        lower=std::max(p.min_load,(std::cos(gamma)-budget)/min_cos);
        upper=std::min(cap,(std::cos(gamma)+budget)/max_cos);
        if(lower<=upper)break;
        roll*=.5;
    }
    const double mid=bank+roll*dt/2;
    const double load=clip(lift_vertical/(gravity*std::cos(mid)),lower,std::max(lower,upper));
    const double desired_speed=clip(std::sqrt(velocity[0]*velocity[0]+velocity[1]*velocity[1]+velocity[2]*velocity[2]),p.min_speed,p.max_speed);
    const double acceleration=1.5*(desired_speed-speed);
    const double axial=acceleration+drag(speed,load,p)+gravity*std::sin(gamma);
    return {clip(axial,0.,p.max_thrust),clip(-axial,0.,p.max_brake),load,roll};
}
}

Control clip_execution_control(const State& s, const Control& u, const Parameters& p, double dt) {
    check_duration(dt);
    Control out=u;
    out[0]=clip(out[0],0.,p.max_thrust);
    out[1]=clip(out[1],0.,p.max_brake);
    out[2]=clip(out[2],p.min_load,std::max(p.min_load,ceiling(s[3],p)));
    const double low=std::max(-p.max_roll,(-p.max_bank-s[6])/dt);
    const double high=std::min(p.max_roll,(p.max_bank-s[6])/dt);
    out[3]=low<=high?clip(out[3],low,high):clip(-s[6]/dt,-p.max_roll,p.max_roll);
    return out;
}

bool held_control_ok(const State& s, const Control& u, const Parameters& p, double dt) {
    check_duration(dt);
    const double step=dt/20.;
    for(int k=0;k<=20;++k) {
        const double time=k==20?dt:k*step;
        if(!envelope(propagate(s,u,time,p,4),u,p,1e-8))return false;
    }
    return true;
}

ProtectionResult protect_control(const State& s, const Control& request, const Parameters& p, double dt) {
    validate(p);
    const auto requested=clip_execution_control(s,request,p,dt);
    if(held_control_ok(s,requested,p,dt))return {requested,false,true};
    const auto fallback=clip_execution_control(s,trim(s,p),p,dt);
    if(!held_control_ok(s,fallback,p,dt))return {requested,true,false};
    Control best=fallback;
    double low=0.,high=1.;
    for(int k=0;k<10;++k) {
        const double fraction=(low+high)/2;
        Control candidate;
        for(std::size_t j=0;j<4;++j)candidate[j]=fallback[j]+fraction*(requested[j]-fallback[j]);
        if(held_control_ok(s,candidate,p,dt)){best=candidate;low=fraction;}
        else high=fraction;
    }
    return {best,true,true};
}

LocalControlResult local_control_update(const State& measured, const State& reference,
    const Control& base, const OwnHistory* previous,
    std::array<double,3> estimate, const Parameters& p, double measurement_dt, double hold_dt) {
    validate(p);check_duration(measurement_dt);check_duration(hold_dt);
    if(previous) {
        const auto predicted=propagate(previous->measured,previous->applied,measurement_dt,p,4);
        const double alpha=1-std::exp(-measurement_dt/1.5);
        for(std::size_t j=0;j<3;++j) {
            const double residual=j==2?0.:(measured[j]-predicted[j])/measurement_dt;
            estimate[j]+=alpha*(residual-estimate[j]);
        }
    }
    const auto ff=derivative(reference,base,p);
    const auto nominal=tracking_guidance(reference,reference,ff,p,hold_dt,nullptr);
    const auto actual=tracking_guidance(measured,reference,ff,p,hold_dt,&estimate);
    Control request;
    for(std::size_t j=0;j<4;++j)request[j]=base[j]+actual[j]-nominal[j];
    const auto protected_control=protect_control(measured,request,p,hold_dt);
    return {protected_control.control,estimate,protected_control.intervened,protected_control.feasible};
}

std::vector<State> propagate_samples(const State& s, const Control& u,
    const std::vector<double>& durations, const Parameters& p, unsigned subdivisions,
    const std::array<double,3>& disturbance) {
    validate(p);
    std::vector<State> out;out.reserve(durations.size());
    for(double duration:durations)out.push_back(propagate(s,u,duration,p,subdivisions,disturbance));
    return out;
}
}
