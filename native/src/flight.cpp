#include "swarmtraj/flight.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace swarmtraj {
namespace {
double clip(double x, double low, double high) { return std::min(std::max(x,low),high); }
double ceiling(double speed, const Parameters& p) { return std::min(p.max_load, std::pow(speed/(p.stall_speed*p.stall_margin),2)); }
double drag(double v, double n, const Parameters& p) { return p.drag0*v*v + p.drag_lift*n*n/(v*v); }
std::array<double,3> velocity(const State& s) {
    const double horizontal=s[3]*std::cos(s[5]);
    return {horizontal*std::cos(s[4]),horizontal*std::sin(s[4]),s[3]*std::sin(s[5])};
}
Control trim(const State& s,const Parameters& p) {
    const double load=std::cos(s[5])/std::cos(s[6]);
    const double axial=drag(s[3],load,p)+gravity*std::sin(s[5]);
    return {std::max(0.,axial),std::max(0.,-axial),load,0.};
}
Control control(const State& s,double speed,double rate,const Parameters& p,bool zero_bank=false) {
    const double max_rate=std::min(p.max_path_rate,.9*p.max_vertical_acceleration/s[3]);
    rate=clip(rate,-max_rate,max_rate);
    const double roll=zero_bank?clip(-s[6]/p.dt,-p.max_roll,p.max_roll):0.;
    const double mid=s[6]+roll*p.dt/2;
    const double load=clip((std::cos(s[5])+s[3]*rate/gravity)/std::cos(mid),p.min_load,ceiling(s[3],p));
    const double axial=drag(s[3],load,p)+gravity*std::sin(s[5])+1.5*(speed-s[3]);
    return {clip(axial,0,p.max_thrust),clip(-axial,0,p.max_brake),load,roll};
}
double requested_vz(const State& s,double altitude,const Parameters& p) {
    const double error=altitude-s[2],rate=error>=0?p.max_climb:p.max_descent;
    const double v=std::min({.6*std::abs(error),std::sqrt(1.2*p.max_vertical_acceleration*std::abs(error)),.85*rate,s[3]*std::sin(.95*p.max_path)});
    return std::copysign(v,error);
}
State reference(const State& s,double speed,double altitude,double heading,const Parameters& p) {
    State r=s;r[3]=speed;r[4]=heading;r[5]=std::asin(clip(requested_vz(s,altitude,p)/speed,-.95,.95));r[6]=0.;return r;
}
Control altitude_control(const State& s,double speed,double altitude,const Parameters& p) {
    const double gamma=std::asin(clip(requested_vz(s,altitude,p)/s[3],-.95,.95));
    return control(s,speed,1.5*(gamma-s[5]),p);
}
bool settled(const State& s,double z) { return std::abs(s[2]-z)<=.02 && std::abs(s[3]*std::sin(s[5]))<=.01; }
struct Builder {
    const Parameters& p; const Recipe& r; Path path; std::size_t offset=0;
    Builder(const State& s,const Parameters& p_,const Recipe& r_,std::size_t offset_=0):p(p_),r(r_),offset(offset_) {
        path.states.reserve(r.steps-offset+1);path.controls.reserve(r.steps-offset);path.states.push_back(s);
    }
    std::size_t step() const { return offset+path.controls.size(); }
    const State& state() const { return path.states.back(); }
    bool append(const Control& u) {
        if(step()>=r.steps)return false;
        State next;
        if(!checked_step(state(),u,p,next))return false;
        path.controls.push_back(u);path.states.push_back(next);return true;
    }
    void phase(const std::string& name,std::size_t first) { if(step()>first)path.phases.push_back({name,first,step()}); }
    Path fail(const char* reason) { path.rejection=reason;return std::move(path); }
};
bool pitch(Builder& b,double speed,double angle,std::size_t budget) {
    for(std::size_t k=0;k<budget;++k) {
        if(std::abs(b.state()[5]-angle)<=1e-4)return true;
        if(!b.append(control(b.state(),speed,(angle-b.state()[5])/b.p.dt,b.p)))return false;
    }
    return std::abs(b.state()[5]-angle)<=1e-4;
}
double along(const State& s,const Point& gate,const Point& v) { return (s[0]-gate[0])*v[0]+(s[1]-gate[1])*v[1]; }
double distance_xy(const State& s,const Point& q) { const double x=s[0]-q[0],y=s[1]-q[1];return std::sqrt(x*x+y*y); }
}

double wrap(double a) {
    double v=std::fmod(a+pi,2*pi);if(v<0)v+=2*pi;return v-pi;
}
void validate(const Parameters& p) {
    const double fields[]={p.stall_speed,p.stall_margin,p.max_load,p.min_load,p.max_path,p.max_path_rate,p.max_thrust,p.max_brake,p.drag0,p.drag_lift,p.min_speed,p.max_speed,p.max_bank,p.max_roll,p.min_altitude,p.max_altitude,p.max_climb,p.max_descent,p.max_vertical_acceleration,p.dt};
    for(double x:fields)if(!std::isfinite(x))throw std::invalid_argument("nonfinite native parameters");
    if(p.stall_speed<=0||p.stall_margin<1||p.min_load<=0||p.max_load<=p.min_load||p.max_path<=0||p.max_path>=80*pi/180||p.max_path_rate<=0||p.max_thrust<=0||p.max_brake<0||p.drag0<0||p.drag_lift<0||p.min_speed<=0||p.max_speed<p.min_speed||p.max_bank<=0||p.max_bank>=pi/2||p.max_roll<=0||p.max_altitude<p.min_altitude||p.max_climb<=0||p.max_descent<=0||p.max_vertical_acceleration<=0||p.dt<=0)
        throw std::invalid_argument("invalid native physical parameters");
}
State derivative(const State& s,const Control& u,const Parameters& p,const std::array<double,3>& wind) {
    const double v=s[3],cg=std::cos(s[5]),sg=std::sin(s[5]);
    return {v*cg*std::cos(s[4])+wind[0],v*cg*std::sin(s[4])+wind[1],v*sg+wind[2],u[0]-u[1]-drag(v,u[2],p)-gravity*sg,gravity*u[2]*std::sin(s[6])/(v*cg),gravity*(u[2]*std::cos(s[6])-cg)/v,u[3]};
}
State propagate(State s,const Control& u,double duration,const Parameters& p,unsigned subdivisions,const std::array<double,3>& wind) {
    if(!subdivisions)throw std::invalid_argument("subdivisions must be positive");
    const double h=duration/subdivisions;
    State scratch,k1,k2,k3,k4;
    for(unsigned k=0;k<subdivisions;++k) {
        k1=derivative(s,u,p,wind);
        for(std::size_t j=0;j<7;++j)scratch[j]=s[j]+h*k1[j]/2;
        k2=derivative(scratch,u,p,wind);
        for(std::size_t j=0;j<7;++j)scratch[j]=s[j]+h*k2[j]/2;
        k3=derivative(scratch,u,p,wind);
        for(std::size_t j=0;j<7;++j)scratch[j]=s[j]+h*k3[j];
        k4=derivative(scratch,u,p,wind);
        for(std::size_t j=0;j<7;++j)s[j]+=h*(k1[j]+2*k2[j]+2*k3[j]+k4[j])/6;
    }
    s[4]=wrap(s[4]);return s;
}
bool envelope(const State& s,const Control& u,const Parameters& p,double eps) {
    for(double x:s)if(!std::isfinite(x))return false;
    for(double x:u)if(!std::isfinite(x))return false;
    if(s[3]<=0)return false;
    const State d=derivative(s,u,p);const double vz=s[3]*std::sin(s[5]);
    const double az=d[3]*std::sin(s[5])+s[3]*std::cos(s[5])*d[5];
    return s[3]>=p.min_speed-eps && s[3]<=p.max_speed+eps && std::abs(s[6])<=p.max_bank+eps && std::abs(s[5])<=p.max_path+eps && s[2]>=p.min_altitude-eps && s[2]<=p.max_altitude+eps && vz<=p.max_climb+eps && vz>=-p.max_descent-eps && std::abs(az)<=p.max_vertical_acceleration+eps && std::abs(d[5])<=p.max_path_rate+eps && u[0]>=-eps && u[0]<=p.max_thrust+eps && u[1]>=-eps && u[1]<=p.max_brake+eps && u[2]>=p.min_load-eps && u[2]<=ceiling(s[3],p)+eps && std::abs(u[3])<=p.max_roll+eps;
}
bool checked_step(const State& s,const Control& u,const Parameters& p,State& next) {
    next=propagate(s,u,p.dt,p);
    for(double fraction:{0.,.25,.5,.75,1.})if(!envelope(propagate(s,u,fraction*p.dt,p,4),u,p))return false;
    return envelope(next,u,p);
}
Control guidance(const State& s,const State& ref,double yaw,double speed_ff,const Parameters& p) {
    const double speed=s[3],gamma=s[5],bank=s[6];auto vel=velocity(ref);
    for(std::size_t j=0;j<3;++j)vel[j]+=.8*(ref[j]-s[j]);
    const double heading=std::atan2(vel[1],vel[0]);
    const double yaw_rate=yaw+1.5*wrap(heading-s[4]);
    const double desired_gamma=std::atan2(vel[2],std::sqrt(vel[0]*vel[0]+vel[1]*vel[1]));
    double gamma_rate=1.5*(desired_gamma-gamma);
    const double up=std::min(p.max_path,std::asin(std::min(1.,.98*p.max_climb/speed)));
    const double down=std::min(p.max_path,std::asin(std::min(1.,.98*p.max_descent/speed)));
    const double max_pitch=std::min(p.max_path_rate,.95*p.max_vertical_acceleration/std::max(speed,1e-6));
    gamma_rate=clip(gamma_rate,std::max(-max_pitch,(-down-gamma)/p.dt),std::min(max_pitch,(up-gamma)/p.dt));
    const double lift_vertical=gravity*std::cos(gamma)+speed*gamma_rate;
    double desired_bank=std::atan2(speed*std::cos(gamma)*yaw_rate,lift_vertical);
    const double cap=ceiling(speed,p);
    const double max_bank=std::min(p.max_bank,std::acos(clip(lift_vertical/(gravity*cap),0,1)));
    desired_bank=clip(desired_bank,-max_bank,max_bank);
    double roll=clip((desired_bank-bank)/p.dt,-p.max_roll,p.max_roll);
    const double budget=.85*p.max_vertical_acceleration/(gravity*std::cos(gamma));
    double lower=0,upper=0;
    for(int k=0;k<8;++k) {
        const double end=bank+roll*p.dt;
        const double min_cos=std::min(std::cos(bank),std::cos(end));
        const double max_cos=bank*end<=0?1.:std::max(std::cos(bank),std::cos(end));
        lower=std::max(p.min_load,(std::cos(gamma)-budget)/min_cos);
        upper=std::min(cap,(std::cos(gamma)+budget)/max_cos);
        if(lower<=upper)break;
        roll*=.5;
    }
    const double mid=bank+roll*p.dt/2;
    const double load=clip(lift_vertical/(gravity*std::cos(mid)),lower,std::max(lower,upper));
    double accel=1.5*(ref[3]-speed);if(speed_ff!=0)accel+=speed_ff;
    const double axial=accel+drag(speed,load,p)+gravity*std::sin(gamma);
    return {clip(axial,0,p.max_thrust),clip(-axial,0,p.max_brake),load,roll};
}
Path rollout(const State& initial,const std::vector<Control>& commands,const Parameters& p,bool check,unsigned subdivisions,const std::array<double,3>& wind) {
    validate(p);
    if(check&&(subdivisions!=2||wind!=std::array<double,3>{}))throw std::invalid_argument("checked nominal rollout requires two substeps and zero wind");
    Path out;out.states.reserve(commands.size()+1);out.controls.reserve(commands.size());out.states.push_back(initial);
    for(const auto& u:commands) {
        State next;
        if(check) { if(!checked_step(out.states.back(),u,p,next)) { out.rejection="envelope";return out; } }
        else next=propagate(out.states.back(),u,p.dt,p,subdivisions,wind);
        out.controls.push_back(u);out.states.push_back(next);
    }
    return out;
}
Path pullout(const State& initial,const Parameters& p,std::size_t max_steps) {
    Path out;out.states.reserve(max_steps+1);out.controls.reserve(max_steps);out.states.push_back(initial);
    if(initial[3]<=0||initial[5]>=0||initial[5]<=-pi/2||std::abs(initial[6])>1e-12) {out.rejection="pullout_entry";return out;}
    const double speed=initial[3], rate=std::min(p.max_path_rate,.97*p.max_vertical_acceleration/speed);
    max_steps=std::min(max_steps,static_cast<std::size_t>(std::ceil((std::abs(initial[5])/rate+2.)/p.dt)));
    for(std::size_t k=0;k<max_steps;++k) {
        const State& s=out.states.back();const double gamma=s[5];if(gamma>=-1e-4)break;
        double gamma_rate=std::min({p.max_path_rate,.97*p.max_vertical_acceleration/speed,-gamma/p.dt});
        const double rate_by_load=gravity*(ceiling(s[3],p)-std::cos(gamma))/s[3];
        if(rate_by_load<=0) {out.rejection="pullout_load";return out;}
        gamma_rate=std::min(gamma_rate,.90*rate_by_load);
        if(gamma_rate<=0){out.rejection="pullout_rate";return out;}
        const double load=std::cos(gamma)+s[3]*gamma_rate/gravity;
        const double axial=drag(s[3],load,p)+gravity*std::sin(gamma);
        if(axial < -p.max_brake || axial > p.max_thrust) {out.rejection="pullout_force";return out;}
        Control u{std::max(0.,axial),std::max(0.,-axial),load,0.};State next;
        if(!checked_step(s,u,p,next)){out.rejection="pullout_envelope";return out;}
        out.controls.push_back(u);out.states.push_back(next);
    }
    if(out.controls.empty()||std::abs(out.states.back()[5])>1e-4)out.rejection="pullout_horizon";
    return out;
}
Path prefix(const State& initial,const Parameters& p,const Recipe& r) {
    validate(p);
    if(r.steps==0||r.steps>1000000||r.waypoints.empty()||r.levels.empty()||r.levels.size()>3||r.speed_reference.size()!=r.steps+1||r.speed_derivative.size()!=r.steps+1)throw std::invalid_argument("invalid prefix recipe sizes");
    Builder b(initial,p,r);
    while(b.step()*p.dt<r.release-1e-10)if(!b.append(trim(b.state(),p)))return b.fail("physical_loiter");
    b.phase("loiter",0);
    if(r.rich) {
        auto levels=r.levels;if(levels.size()>1)levels.push_back(r.cruise);
        for(std::size_t index=0;index<levels.size();++index) {
            const auto first=b.step();std::size_t held=0;
            while(b.step()<r.steps) {
                held=settled(b.state(),levels[index])?held+1:0;
                if(held*p.dt>=r.minimum_hold && (index+1<levels.size()||b.step()*p.dt>=r.speed_end+3.))break;
                const auto ref=reference(b.state(),r.speed_reference[b.step()],levels[index],b.state()[4],p);
                const double yaw=(r.yaw_rate>0?1.:(r.yaw_rate<0?-1.:0.))*std::min(.15,.6*gravity*std::tan(p.max_bank)/b.state()[3]);
                if(!b.append(guidance(b.state(),ref,yaw,r.speed_derivative[b.step()],p)))return b.fail("physical_profile_envelope");
            }
            if(b.step()>=r.steps)return b.fail("physical_profile_horizon");
            b.phase("cruise_level_"+std::to_string(index),first);
        }
    }
    const auto transit=b.step();std::size_t waypoint=0,held=0;bool on_line=false;
    while(b.step()<r.steps) {
        State s=b.state();held=settled(s,r.cruise)?held+1:0;
        if(!on_line && distance_xy(s,r.waypoints[waypoint])<2*r.speed) {
            if(waypoint+1<r.waypoints.size())++waypoint;else on_line=true;
        }
        double heading;
        if(on_line) {
            double cross=along(s,r.gate,r.normal),a=along(s,r.gate,r.direction);
            heading=r.heading-std::atan2(cross,1.5*r.speed);
            if(a>=0) {
                const double prev=b.step()>transit?along(b.path.states[b.path.states.size()-2],r.gate,r.direction):a;
                if(std::abs(prev)<std::abs(a)) {b.path.states.pop_back();b.path.controls.pop_back();s=b.state();cross=along(s,r.gate,r.normal);}
                if(std::abs(wrap(s[4]-r.heading))>.4*pi/180||std::abs(cross)>.5||!settled(s,r.cruise)||std::abs(s[3]-r.speed)>.05||held*p.dt<r.minimum_hold)return b.fail("physical_approach_alignment");
                break;
            }
        } else heading=std::atan2(r.waypoints[waypoint][1]-s[1],r.waypoints[waypoint][0]-s[0]);
        if(!b.append(guidance(s,reference(s,r.speed,r.cruise,heading,p),0.,0.,p)))return b.fail("physical_transit_envelope");
    }
    if(b.step()>=r.steps)return b.fail("physical_transit_horizon");
    b.phase("transit",transit);
    auto first=b.step();
    if(!b.append(control(b.state(),r.speed,-b.state()[5]/p.dt,p,true))||std::abs(b.state()[6])>1e-12)return b.fail("physical_alignment_envelope");
    b.phase("alignment",first);first=b.step();
    if(r.angle<0) {
        if(!pitch(b,r.speed,r.angle,r.steps-b.step()))return b.fail("physical_descent_entry");
    } else {
        while(!settled(b.state(),r.target_z))if(!b.append(altitude_control(b.state(),r.speed,r.target_z,p)))return b.fail("physical_target_altitude");
    }
    b.phase(r.angle<0?"descent_entry":"target_altitude",first);first=b.step();
    while(b.step()<r.steps) {
        const auto& s=b.state();const auto v=velocity(s);
        if((s[0]-r.target[0])*v[0]+(s[1]-r.target[1])*v[1]+(s[2]-r.target_z)*v[2]>=0)break;
        if(!b.append(r.angle<0?control(s,r.speed,1.5*(r.angle-s[5]),p):altitude_control(s,r.speed,r.target_z,p)))return b.fail("physical_passage_envelope");
    }
    b.phase("passage",first);
    double closest=std::numeric_limits<double>::infinity();std::size_t arrival=0;
    for(std::size_t k=0;k<b.path.states.size();++k) {
        const auto& s=b.path.states[k];const double x=s[0]-r.target[0],y=s[1]-r.target[1],z=s[2]-r.target_z;
        const double distance=std::sqrt(x*x+y*y+z*z);if(distance<closest){closest=distance;arrival=k;}
    }
    b.path.arrival=arrival*p.dt;
    if(closest>r.target_radius-r.position_error)return b.fail("physical_target_missed");
    if(b.path.arrival>r.latest_arrival+1e-9)return b.fail("physical_arrival_timing");
    first=b.step();
    if(r.angle<0) {
        const auto recovered=pullout(b.state(),p,r.steps-b.step());
        if(!recovered.accepted())return b.fail("physical_pullout");
        for(std::size_t k=0;k<recovered.controls.size();++k)if(!b.append(recovered.controls[k])||b.state()!=recovered.states[k+1])return b.fail("physical_pullout_boundary");
    }
    b.phase("pullout",first);return std::move(b.path);
}
Path onward(const State& initial,const Parameters& p,const Recipe& r,std::size_t start,const std::vector<Point>& waypoints) {
    validate(p);
    if(r.steps>1000000||start>r.steps)throw std::invalid_argument("invalid onward horizon");
    Builder b(initial,p,r,start);std::size_t recovered=r.steps,waypoint=0;bool settled_once=false;
    while(b.step()<r.steps) {
        if(!settled_once && settled(b.state(),r.cruise)){recovered=b.step();settled_once=true;}
        auto u=altitude_control(b.state(),r.speed,r.cruise,p);
        if(!waypoints.empty()) {
            const auto& s=b.state();if(waypoint+1<waypoints.size()&&distance_xy(s,waypoints[waypoint])<2*r.speed)++waypoint;
            const double heading=std::atan2(waypoints[waypoint][1]-s[1],waypoints[waypoint][0]-s[0]);
            u=guidance(s,reference(s,r.speed,r.cruise,heading,p),0.,0.,p);
        }
        if(!b.append(u))return b.fail("physical_cruise_recovery");
    }
    if(!settled(b.state(),r.cruise))return b.fail("physical_recovery_horizon");
    if(recovered>start)b.path.phases.push_back({"cruise_recovery",start,recovered});
    if(r.steps>recovered)b.path.phases.push_back({"onward",recovered,r.steps});
    return std::move(b.path);
}
bool terminal_distance(double speed,double cruise,const Parameters& p,const Recipe& r,double& distance) {
    State initial{0.,0.,cruise,speed,0.,0.,0.};Recipe fixture=r;fixture.steps+=1;Builder b(initial,p,fixture);
    if(!b.append(control(initial,speed,0.,p)))return false;
    if(r.angle<0) {
        if(!pitch(b,speed,r.angle,r.steps))return false;
        auto s=b.state();if(s[2]-r.target_z<(r.target_radius+r.position_error)*std::sin(-r.angle))return false;
        for(std::size_t k=0;k<r.steps;++k) {
            // This proposal fixture has its own bounded loop, not a mission plan.
            State next;auto u=control(s,speed,1.5*(r.angle-s[5]),p);
            if(!checked_step(s,u,p,next))return false;
            if(next[2]<=r.target_z){double fraction=(s[2]-r.target_z)/(s[2]-next[2]);distance=s[0]+fraction*(next[0]-s[0]);return true;}s=next;
        }
    } else {
        auto s=b.state();for(std::size_t k=0;k<r.steps;++k) {
            if(settled(s,r.target_z)){distance=s[0]+std::max(2*r.target_radius,speed);return true;}
            State next;if(!checked_step(s,altitude_control(s,speed,r.target_z,p),p,next))return false;s=next;
        }
    }
    return false;
}
bool headroom(double speed,const Parameters& p,const Recipe& r,double& loss) {
    State initial{0.,0.,p.max_altitude,speed,0.,0.,0.};Builder b(initial,p,r);
    if(!pitch(b,speed,r.angle,r.steps))return false;
    loss=initial[2]-b.state()[2];return true;
}
}
