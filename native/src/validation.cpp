#include "swarmtraj/validation.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace swarmtraj {
namespace {
bool close(double a,double b,double tolerance) {
    return std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=tolerance;
}
template<std::size_t N> bool close(const std::array<double,N>& a,const std::array<double,N>& b,double tolerance) {
    for(std::size_t j=0;j<N;++j)if(!close(a[j],b[j],tolerance))return false;
    return true;
}
double distance(const State& s,const ValidationSettings& c) {
    const double x=s[0]-c.target[0],y=s[1]-c.target[1];
    const double z=c.target_altitude?s[2]-*c.target_altitude:0.;
    return std::sqrt(x*x+y*y+z*z);
}
bool sector(const State& s,const Candidate& path,const ValidationSettings& c) {
    return std::abs(wrap(std::atan2(s[1]-c.target[1],s[0]-c.target[0])-*path.approach))<=15*pi/180
        &&std::abs(wrap(s[4]-*path.approach-pi))<=15*pi/180;
}
bool approach_valid(const Candidate& path,const ValidationSettings& c) {
    if(!path.approach)return c.spread_degrees==0;
    for(std::size_t k=0;k<path.projected.size();++k) {
        const auto& s=path.projected[k];
        const double x=s[0]-c.target[0],y=s[1]-c.target[1];
        if(std::sqrt(x*x+y*y)<=c.target_radius) {
            return k*path.dt>=path.release
                &&std::abs(wrap(std::atan2(y,x)-*path.approach))<=15*pi/180
                &&std::abs(wrap(s[2]-*path.approach-pi))<=15*pi/180;
        }
    }
    return false;
}
struct Contact {double time;State state;};
std::optional<Contact> first_entry(const Candidate& path,const ValidationSettings& c) {
    std::size_t evaluations=0;
    std::function<std::optional<Contact>(double,const State&,double,double,const State&,double)> scan;
    scan=[&](double left,const State& a,double da,double right,const State& b,double db)->std::optional<Contact> {
        const double width=right-left;
        if(da<=c.target_radius)return Contact{left,a};
        if(std::min(da,db)-c.physics.max_speed*width/2>c.target_radius)return {};
        if(width<=1e-6)return db<=c.target_radius?std::optional<Contact>{{right,b}}:std::nullopt;
        const double middle=(left+right)/2;
        if(++evaluations>4096)throw std::runtime_error("contact evaluation budget exhausted");
        const auto state=sample_candidate(path,middle,c.physics);
        for(double v:state)if(!std::isfinite(v))throw std::runtime_error("nonfinite contact state");
        const double dm=distance(state,c);
        auto earlier=scan(left,a,da,middle,state,dm);
        return earlier?earlier:scan(middle,state,dm,right,b,db);
    };
    for(std::size_t k=0;k<path.controls.size();++k) {
        double left=k*path.dt;const double right=(k+1)*path.dt;
        State a=path.states[k];const auto& b=path.states[k+1];
        double da=distance(a,c);const double db=distance(b,c);
        if(std::min(da,db)-c.physics.max_speed*(right-left)/2>c.target_radius||right<c.activation)continue;
        if(left<c.activation){left=c.activation;a=sample_candidate(path,left,c.physics);da=distance(a,c);}
        auto contact=scan(left,a,da,right,b,db);if(contact)return contact;
    }
    return {};
}
}

Projection project_state(const State& s,const Control& u,const Parameters& p) {
    return {s[0],s[1],s[4],derivative(s,u,p)[4],s[2],s[3]*std::sin(s[5])};
}
State sample_candidate(const Candidate& path,double time,const Parameters& p) {
    auto index=static_cast<std::size_t>(std::max(0.,std::floor(std::max(time,0.)/path.dt+1e-9)));
    index=std::min(index,path.controls.size()-1);
    auto state=path.states[index];auto command=path.controls[index];double tau=time-index*path.dt;
    if(time<0||time>=path.controls.size()*path.dt) {
        state=time<0?path.states.front():path.states.back();
        const double load=std::cos(state[5])/std::cos(state[6]);
        const double axial=p.drag0*state[3]*state[3]+p.drag_lift*load*load/(state[3]*state[3])+gravity*std::sin(state[5]);
        command={std::max(0.,axial),std::max(0.,-axial),load,0.};
        tau=time<0?time:time-path.controls.size()*path.dt;
    }
    return propagate(state,command,tau,p,2);
}
double solid_distance(const Solid& o,const State& s) {
    double q[3]{};std::size_t dim=3;
    if(o.cylinder) {
        const double x=s[0]-o.center[0],y=s[1]-o.center[1];
        q[0]=std::sqrt(x*x+y*y)-o.half[0];q[1]=std::abs(s[2]-o.center[2])-o.half[2];dim=2;
    } else for(std::size_t j=0;j<3;++j)q[j]=std::abs(s[j]-o.center[j])-o.half[j];
    double square=0.,largest=-std::numeric_limits<double>::infinity();
    for(std::size_t j=0;j<dim;++j){const double v=std::max(q[j],0.);square+=v*v;largest=std::max(largest,q[j]);}
    return std::sqrt(square)+std::min(largest,0.);
}
double clearance(const Candidate& path,const std::vector<Solid>& obstacles) {
    double result=std::numeric_limits<double>::infinity();
    for(const auto& o:obstacles)for(const auto& p:path.projected) {
        const State position{p[0],p[1],p[4],0,0,0,0};result=std::min(result,solid_distance(o,position));
    }
    return result;
}

ValidationResult validate_candidate(const Candidate& path,const State& initial,double yaw_rate,
    std::optional<double> requested_cruise,const ValidationSettings& c) {
    validate(c.physics);const auto& p=c.physics;const auto& s=path.states;const auto& u=path.controls;
    auto bad=[](const char* reason){return ValidationResult{reason,{}};};
    if(!c.steps||s.size()!=c.steps+1||u.size()!=c.steps||path.projected.size()!=c.steps+1
        ||path.acceleration.size()!=c.steps||path.vertical_acceleration.size()!=c.steps
        ||path.speed!=initial[3]||path.max_speed!=p.max_speed||path.dt!=p.dt||!close(s.front(),initial,1e-9))return bad("validation_metadata");
    if(!(c.activation+c.min_delay-1e-9<=path.release&&path.release<=c.activation+c.max_delay+1e-9))return bad("validation_departure_window");
    if(!(path.release<=path.arrival&&path.arrival<=c.latest_arrival+1e-9))return bad("validation_arrival_horizon");
    for(std::size_t k=0;k<s.size();++k)if(k*path.dt<path.release) {
        const double t=k*path.dt,half=yaw_rate*t/2;
        const double scale=initial[3]*t*(half==0?1.:std::sin(half)/half);
        const double heading=initial[4]+half;
        if(!close(s[k][0],initial[0]+scale*std::cos(heading),1e-4)
            ||!close(s[k][1],initial[1]+scale*std::sin(heading),1e-4)
            ||std::abs(s[k][2]-initial[2])>1e-5||std::abs(s[k][5])>1e-5)return bad("validation_loiter");
    }
    for(std::size_t k=0;k<u.size();++k)if(!close(propagate(s[k],u[k],path.dt,p,2),s[k+1],1e-9))return bad("validation_integration");
    Projection previous{};
    for(std::size_t k=0;k<s.size();++k) {
        auto projection=project_state(s[k],u[std::min(k,u.size()-1)],p);
        if(!close(projection,path.projected[k],1e-9))return bad("validation_projection");
        if(k&&(!close((projection[3]-previous[3])/path.dt,path.acceleration[k-1],1e-9)
               ||!close((projection[5]-previous[5])/path.dt,path.vertical_acceleration[k-1],1e-9)))return bad("validation_projection");
        previous=projection;
    }
    for(double fraction:{0.,.25,.5,.75,1.})for(std::size_t k=0;k<u.size();++k)
        if(!envelope(propagate(s[k],u[k],fraction*path.dt,p,4),u[k],p,1e-7))return bad("validation_dense_envelope");
    if(!path.cruise)return bad("validation_cruise_metadata");
    const double cruise=*path.cruise;
    if(c.cruise_range) {
        if(!(c.cruise_range->at(0)<=cruise&&cruise<=c.cruise_range->at(1)))return bad("validation_cruise_range");
    } else if(std::abs(cruise-requested_cruise.value_or(initial[2]))>1e-8)return bad("validation_cruise_metadata");
    if(std::abs(s.back()[2]-cruise)>.2||std::abs(s.back()[3]*std::sin(s.back()[5]))>.1)return bad("validation_cruise_recovery");
    if(c.cruise_levels>1) {
        if(path.levels.size()!=c.cruise_levels||!c.cruise_range||!close(path.levels.front(),cruise,1e-8))return bad("validation_cruise_levels_metadata");
        for(std::size_t j=0;j<path.levels.size();++j) {
            const double z=path.levels[j];
            if(!std::isfinite(z)||z<c.cruise_range->at(0)||z>c.cruise_range->at(1))return bad("validation_cruise_levels_metadata");
            for(std::size_t k=j+1;k<path.levels.size();++k)if(std::abs(z-path.levels[k])<5-1e-8)return bad("validation_cruise_levels_metadata");
        }
        long previous_index=-1;
        for(double z:path.levels) {
            std::size_t run=0;bool found=false;
            for(std::size_t k=0;k<s.size();++k) {
                const bool reached=static_cast<long>(k)>previous_index&&k*path.dt>=path.release&&k*path.dt<path.arrival
                    &&std::abs(s[k][2]-z)<=.5&&std::abs(s[k][3]*std::sin(s[k][5]))<=.2;
                run=reached?run+1:0;
                if(run&&(run-1)*path.dt>=1-1e-9){previous_index=static_cast<long>(k);found=true;break;}
            }
            if(!found)return bad("validation_cruise_level_not_reached");
        }
    } else if(!path.levels.empty())return bad("validation_unrequested_cruise_levels");
    if(c.smooth_speed) {
        const auto& reference=path.speed_reference;
        if(!path.has_speed_reference||reference.size()!=s.size())return bad("validation_speed_reference");
        double low=std::numeric_limits<double>::infinity(),high=-low,actual_low=low,actual_high=high;
        for(std::size_t k=0;k<s.size();++k) {
            if(!std::isfinite(reference[k])||reference[k]<c.speed_range[0]-1e-8||reference[k]>c.speed_range[1]+1e-8
                ||(k&&std::abs((reference[k]-reference[k-1])/path.dt)>c.speed_acceleration+1e-5))return bad("validation_speed_reference");
            low=std::min(low,reference[k]);high=std::max(high,reference[k]);actual_low=std::min(actual_low,s[k][3]);actual_high=std::max(actual_high,s[k][3]);
        }
        if(!close(reference.front(),initial[3],1e-8)||!close(reference.back(),initial[3],1e-8)
            ||high-low<.2-1e-8||actual_high-actual_low<.1)return bad("validation_speed_reference");
    } else if(path.has_speed_reference)return bad("validation_unrequested_speed_reference");
    bool attained=false;
    for(std::size_t k=0;k<s.size();++k)if(k*path.dt>=path.release&&k*path.dt<=path.arrival
        &&std::abs(s[k][2]-cruise)<=.2&&std::abs(s[k][3]*std::sin(s[k][5]))<=.1){attained=true;break;}
    if(!attained)return bad("validation_cruise_before_arrival");
    const auto arrival=sample_candidate(path,path.arrival,p);
    if(distance(arrival,c)>c.target_radius-c.position_error||!approach_valid(path,c))return bad("validation_target_or_sector");
    std::optional<Contact> contact;
    try {contact=first_entry(path,c);}catch(const std::runtime_error&){return bad("validation_contact_event_unresolved");}
    if(!contact)return bad("validation_contact_event");
    if(contact->time<path.release-1e-7)return bad("validation_contact_before_departure");
    if(path.approach&&!sector(contact->state,path,c))return bad("validation_contact_sector");
    ValidationResult result{"",contact->time};
    if(c.descent_degrees>0&&(std::abs(-contact->state[5]*180/pi-c.descent_degrees)>1
        ||std::abs(-arrival[5]*180/pi-c.descent_degrees)>1))result.rejection="validation_descent_angle";
    if(!result.accepted())return result;
    if(!c.obstacles.empty()) {
        const double lower=clearance(path,c.obstacles)-p.max_speed*path.dt/2;
        const double required=c.aircraft_radius+c.obstacle_clearance+c.position_error+p.max_speed*c.clock_error;
        if(!std::isfinite(lower)||lower<required+1e-7)result.rejection="obstacle_clearance";
    }
    return result;
}
}
