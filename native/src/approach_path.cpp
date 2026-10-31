#include "swarmtraj/approach_path.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace swarmtraj {
namespace {
double positive(double a){double v=std::fmod(a,2*pi);return v<0?v+2*pi:v;}
}
std::vector<RouteSample> tangent_connection(const std::array<double,3>& initial,
    const std::array<double,3>& final,double radius,double spacing,double maximum_length) {
    for(const auto& point:{initial,final})for(double v:point)
        if(!std::isfinite(v))throw std::invalid_argument("nonfinite tangent pose");
    if(!std::isfinite(radius)||!std::isfinite(spacing)||!std::isfinite(maximum_length)
       ||radius<=0||spacing<=0||maximum_length<=0)throw std::invalid_argument("invalid tangent bounds");
    double best=std::numeric_limits<double>::infinity(),first=0,straight=0,last=0;
    int first_sign=0,last_sign=0;
    for(int a:{1,-1})for(int b:{1,-1}) {
        const Point ca{initial[0]-a*radius*std::sin(initial[2]),initial[1]+a*radius*std::cos(initial[2])};
        const Point cb{final[0]-b*radius*std::sin(final[2]),final[1]+b*radius*std::cos(final[2])};
        const double dx=cb[0]-ca[0],dy=cb[1]-ca[1],distance=std::hypot(dx,dy);
        double heading,run;
        if(distance<1e-10) {
            if(a!=b)continue;
            heading=final[2];run=0;
        } else {
            const double sine=(b-a)*radius/distance;
            if(std::abs(sine)>1.)continue;
            heading=std::atan2(dy,dx)-std::asin(sine);
            run=std::sqrt(std::max(0.,distance*distance-(b-a)*(b-a)*radius*radius));
        }
        const double begin=radius*positive(a*(heading-initial[2]));
        const double end=radius*positive(b*(final[2]-heading));
        const double length=begin+run+end;
        if(length<best&&length<=maximum_length){best=length;first=begin;straight=run;last=end;first_sign=a;last_sign=b;}
    }
    if(!std::isfinite(best)||best/spacing>1000000)return {};
    std::vector<RouteSample> samples;double x=initial[0],y=initial[1],heading=initial[2],distance=0.;
    samples.push_back({x,y,wrap(heading),first_sign/radius,0});
    auto append=[&](double length,int sign) {
        if(length<=1e-10)return;
        const auto steps=static_cast<std::size_t>(std::ceil(length/spacing));const double ds=length/steps;
        for(std::size_t i=0;i<steps;++i) {
            const double next=heading+sign*ds/radius;
            if(sign){x+=sign*radius*(std::sin(next)-std::sin(heading));y-=sign*radius*(std::cos(next)-std::cos(heading));}
            else{x+=ds*std::cos(heading);y+=ds*std::sin(heading);}
            heading=next;distance+=ds;samples.push_back({x,y,wrap(heading),sign/radius,distance});
        }
    };
    append(first,first_sign);append(straight,0);append(last,last_sign);
    if(std::hypot(x-final[0],y-final[1])>1e-6||std::abs(wrap(heading-final[2]))>1e-9)return {};
    samples.back().curvature=0.;return samples;
}
}
