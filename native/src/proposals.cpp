#include "swarmtraj/planning.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace swarmtraj {
namespace {
double clip(double v,double lo,double hi){return std::min(std::max(v,lo),hi);}
double norm(Point p){return std::sqrt(p[0]*p[0]+p[1]*p[1]);}
Point difference(Point a,Point b){return {a[0]-b[0],a[1]-b[1]};}
double dot(Point a,Point b){return a[0]*b[0]+a[1]*b[1];}
struct Rectangle{Point low,high;};
bool hits(Point a,Point b,const Rectangle& r) {
    double enter=0.,leave=1.;
    for(std::size_t k=0;k<2;++k) {
        const double delta=b[k]-a[k];
        if(std::abs(delta)<1e-12){if(!(r.low[k]<a[k]&&a[k]<r.high[k]))return false;}
        else {double first=(r.low[k]-a[k])/delta,last=(r.high[k]-a[k])/delta;
            if(first>last)std::swap(first,last);
            enter=std::max(enter,first);leave=std::min(leave,last);}
    }
    return enter<leave&&leave>0&&enter<1;
}
std::vector<Point> visibility(Point start,Point goal,const std::vector<Rectangle>& obstacles,bool reverse) {
    auto blocked=[&](Point a,Point b){return std::any_of(obstacles.begin(),obstacles.end(),[&](const auto& r){return hits(a,b,r);});};
    if(!blocked(start,goal))return {goal};
    std::vector<Point> corners;
    for(const auto& r:obstacles)for(Point p:{Point{r.low[0],r.low[1]},Point{r.low[0],r.high[1]},Point{r.high[0],r.high[1]},Point{r.high[0],r.low[1]}})corners.push_back(p);
    if(reverse)std::reverse(corners.begin(),corners.end());
    std::vector<Point> nodes{start,goal};nodes.insert(nodes.end(),corners.begin(),corners.end());
    std::vector<double> costs(nodes.size(),std::numeric_limits<double>::infinity());
    std::vector<std::size_t> parents(nodes.size());
    using Item=std::pair<double,std::size_t>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;costs[0]=0.;queue.emplace(0.,0);
    while(!queue.empty()) {
        auto [cost,i]=queue.top();queue.pop();if(cost>costs[i])continue;
        if(i==1){std::vector<Point> path;while(i){path.push_back(nodes[i]);i=parents[i];}std::reverse(path.begin(),path.end());return path;}
        for(std::size_t j=0;j<nodes.size();++j)if(i!=j&&!blocked(nodes[i],nodes[j])) {
            const double value=cost+norm(difference(nodes[j],nodes[i]));
            if(value<costs[j]-1e-9){costs[j]=value;parents[j]=i;queue.emplace(value,j);}
        }
    }
    return {goal}; // Proposal-only fallback; never a clearance approval.
}
double nominal_clearance(const PlanningSettings& c) {
    return c.aircraft_radius+c.obstacle_clearance+c.position_error+c.physics.max_speed*c.clock_error;
}
Recipe fixture_recipe(const PlanningSettings& c,double speed,double cruise) {
    Recipe r{};r.steps=c.steps;r.speed=speed;r.cruise=cruise;r.angle=-c.descent_degrees*(pi/180);
    r.target_z=c.target_altitude.value_or(cruise);r.target_radius=c.target_radius;r.position_error=c.position_error;return r;
}
std::optional<double> choose_cruise(const Designation& s,const PlanningSettings& c,long long id,Pcg64& rng,const Repair* repair,const std::string& mode) {
    if(!c.cruise_range)return s.cruise.value_or(s.initial[2]);
    double low=c.cruise_range->at(0),high=c.cruise_range->at(1);
    if(c.descent_degrees>0) {
        auto r=fixture_recipe(c,s.initial[3],0.);double drop;
        if(!headroom(s.initial[3],c.physics,r,drop))return {};
        low=std::max(low,*c.target_altitude+drop+(2*c.target_radius+s.initial[3]*c.physics.dt)*std::sin(c.descent_degrees*(pi/180)));
    }
    if(low>high)return {};
    if(repair&&mode=="altitude") {
        const auto side=repair->side();const double own=repair->conflict.positions[side][2],other=repair->conflict.positions[1-side][2];
        return (std::abs(own-other)>1e-9?own>other:side==1)?high:low;
    }
    if(!c.obstacles.empty()&&c.spatial&&id%3) {
        const double margin=nominal_clearance(c)+c.physics.max_speed*c.physics.dt/2+3.;
        const bool above=id%3==1;std::vector<double> choices;
        for(const auto& o:c.obstacles) {
            const double boundary=o.center[2]+(o.half[2]+margin)*(above?1:-1);
            const double level=above?std::max(low,boundary):std::min(high,boundary);
            if(low<=level&&level<=high)choices.push_back(level);
        }
        if(!choices.empty()) {
            std::sort(choices.begin(),choices.end());choices.erase(std::unique(choices.begin(),choices.end()),choices.end());
            std::stable_sort(choices.begin(),choices.end(),[&](double a,double b){const double da=std::abs(a-s.initial[2]),db=std::abs(b-s.initial[2]);return da==db?a<b:da<db;});
            return choices[(id/3)%choices.size()];
        }
    }
    return id%4==0?low:id%4==1?clip(s.initial[2],low,high):rng.uniform(low,high);
}
}
std::vector<Point> avoidance_route(Point start,const std::vector<Point>& waypoints,const PlanningSettings& c,double speed,long long id) {
    if(c.obstacles.empty())return waypoints;
    const double radius=speed/(gravity*std::tan(c.physics.max_bank)/speed);
    const double padding=nominal_clearance(c)+2*radius+8;
    std::vector<Rectangle> rectangles;
    for(const auto& o:c.obstacles)rectangles.push_back({{o.center[0]-o.half[0]-padding,o.center[1]-o.half[1]-padding},{o.center[0]+o.half[0]+padding,o.center[1]+o.half[1]+padding}});
    std::vector<Point> result;Point previous=start;
    for(auto point:waypoints){auto path=visibility(previous,point,rectangles,id%2);result.insert(result.end(),path.begin(),path.end());previous=point;}
    return result;
}

ProposalResult physical_proposal(const Designation& s,const PlanningSettings& c,long long id,Pcg64& rng,
    std::optional<double> approach,const Repair* repair,const std::string& mode,std::optional<double> departure,
    std::optional<double> hold,double extra_clearance) {
    auto fail=[](const char* reason){return ProposalResult{{},reason};};
    if((hold&&(!std::isfinite(*hold)||*hold<c.level_hold))
       ||!std::isfinite(extra_clearance)||extra_clearance<0.
       ||!std::isfinite(c.approach_lead_extra_s)||c.approach_lead_extra_s<0.)return fail("invalid_proposal_adjustment");
    const auto cruise=choose_cruise(s,c,id,rng,repair,mode);if(!cruise)return fail("physical_approach_headroom");
    auto r=fixture_recipe(c,s.initial[3],*cruise);double terminal;
    if(!terminal_distance(s.initial[3],*cruise,c.physics,r,terminal))return fail("physical_terminal_construction");
    double release=c.activation+rng.uniform(c.min_delay,c.max_delay);
    if(id%8==0)release=c.activation+c.min_delay;
    if(repair&&mode=="departure")release=c.activation+(repair->side()==0?c.min_delay:c.max_delay);
    if(departure) {
        if(*departure<c.activation+c.min_delay||*departure>c.activation+c.max_delay)return fail("interval_departure_out_of_bounds");
        release=*departure;
    }
    std::vector<double> levels{*cruise};
    for(std::size_t k=1;k<c.cruise_levels;++k) {
        const double last=levels.back();std::vector<double> choices;
        const double below=last-rng.uniform(5,9),above=last+rng.uniform(5,9);
        for(double v:{below,above,c.cruise_range->at(0),c.cruise_range->at(1)}) {
            if(v>=c.cruise_range->at(0)&&v<=c.cruise_range->at(1)
                &&std::all_of(levels.begin(),levels.end(),[&](double z){return std::abs(v-z)>=5;}))choices.push_back(v);
        }
        if(choices.empty())return fail("physical_cruise_levels_range");
        levels.push_back(choices[rng.integer(choices.size())]);
    }
    std::vector<double> speed(c.steps+1,s.initial[3]),speed_derivative(c.steps+1,0.);double speed_end=release;
    if(c.smooth_speed) {
        auto entropy=c.seed;append_entropy(entropy,s.drone_id);append_entropy(entropy,id);append_entropy(entropy,7301);Pcg64 own(entropy);
        int sign=own.integer(2)==0?-1:1;
        auto capacity=[&]{return sign<0?s.initial[3]-c.speed_range[0]:c.speed_range[1]-s.initial[3];};
        if(capacity()<.3)sign*=-1;
        const double start=release+4.,end=release+32.;
        const double amplitude=sign*std::min(capacity()*own.uniform(.6,.95),.8*c.speed_acceleration*(end-start)/pi);
        if(std::abs(amplitude)<.2||end>=c.latest_arrival)return fail("physical_speed_profile_range_or_time");
        for(std::size_t k=0;k<speed.size();++k) {
            const double time=k*c.physics.dt;
            if(time>start&&time<end){const double phase=(time-start)/(end-start);const double sine=std::sin(pi*phase);
                speed[k]+=amplitude*sine*sine;speed_derivative[k]=amplitude*pi/(end-start)*std::sin(2*pi*phase);}
        }
        speed_end=end;
    }
    double heading=approach?wrap(*approach+pi):std::atan2(c.target[1]-s.initial[1],c.target[0]-s.initial[0]);
    if(!c.obstacles.empty()&&!approach&&id>=5) {
        const double offsets[]{-30,30,-60,60,-90,90,180};heading=wrap(heading+offsets[(id-5)%7]*pi/180);
    }
    const Point direction{std::cos(heading),std::sin(heading)},normal{-direction[1],direction[0]};
    Point gate{c.target[0]-terminal*direction[0],c.target[1]-terminal*direction[1]};
    const double lead=s.initial[3]*(6+3*(id%5)+c.approach_lead_extra_s);Point join{gate[0]-lead*direction[0],gate[1]-lead*direction[1]};
    if(repair&&mode=="lateral") {
        const Point start{s.initial[0],s.initial[1]},delta=difference(join,start);
        const auto side=repair->side();const auto& own=repair->conflict.positions[side];const auto& other=repair->conflict.positions[1-side];
        const double fraction=clip(dot(difference({own[0],own[1]},start),delta)/std::max(dot(delta,delta),1e-9),.15,.85);
        const double away=dot({own[0]-other[0],own[1]-other[1]},normal);
        const double sign=std::abs(away)>1e-9?(away>0?1:-1):(side==0?-1:1);
        // Historical recipe writes one of two equal optional points, then uses
        // point zero as join. Keep even this no-op branch for exact parity.
        if(fraction<.475)join={start[0]+fraction*delta[0]+sign*c.detour*normal[0],start[1]+fraction*delta[1]+sign*c.detour*normal[1]};
    }
    std::vector<Point> waypoints{join};std::string routing="lateral";
    if(!c.obstacles.empty()) {
        auto route=c;route.obstacle_clearance+=extra_clearance;
        if(c.spatial&&id%3) {
            const double margin=nominal_clearance(route)+c.physics.max_speed*c.physics.dt/2+3.;
            route.obstacles.clear();for(const auto& o:c.obstacles)if(std::abs(*cruise-o.center[2])-o.half[2]<margin)route.obstacles.push_back(o);
            if(route.obstacles.size()<c.obstacles.size())routing=id%3==1?"overflight":"underflight";
        }
        waypoints=avoidance_route({s.initial[0],s.initial[1]},waypoints,route,s.initial[3],id);
    }
    r.yaw_rate=s.yaw_rate;r.release=release;r.minimum_hold=hold.value_or(c.level_hold);r.speed_end=speed_end;
    r.heading=heading;r.latest_arrival=c.latest_arrival;r.rich=c.cruise_levels>1||c.smooth_speed;
    r.gate=gate;r.direction=direction;r.normal=normal;r.target=c.target;r.levels=levels;
    r.speed_reference=speed;r.speed_derivative=speed_derivative;r.waypoints=waypoints;
    auto first=prefix(s.initial,c.physics,r);if(!first.accepted())return {{},first.rejection};
    const auto offset=first.controls.size();std::vector<Point> exit;
    if(!c.obstacles.empty()) {
        Point goal{first.states.back()[0]+(c.steps-offset)*c.physics.dt*s.initial[3]*direction[0],first.states.back()[1]+(c.steps-offset)*c.physics.dt*s.initial[3]*direction[1]};
        exit=avoidance_route({first.states.back()[0],first.states.back()[1]},std::vector<Point>{goal},c,s.initial[3],id);
    }
    auto tail=onward(first.states.back(),c.physics,r,offset,exit);if(!tail.accepted())return {{},tail.rejection};
    Candidate plan;plan.drone_id=s.drone_id;plan.candidate_id=id;plan.speed=s.initial[3];plan.dt=c.physics.dt;
    plan.max_speed=c.physics.max_speed;
    plan.release=release;plan.arrival=first.arrival;plan.cruise=*cruise;plan.approach=approach;
    plan.states=std::move(first.states);plan.states.insert(plan.states.end(),tail.states.begin()+1,tail.states.end());
    plan.controls=std::move(first.controls);plan.controls.insert(plan.controls.end(),tail.controls.begin(),tail.controls.end());
    plan.phases=std::move(first.phases);plan.phases.insert(plan.phases.end(),tail.phases.begin(),tail.phases.end());
    plan.routing_mode=routing;if(levels.size()>1)plan.levels=levels;
    if(c.smooth_speed){plan.speed_reference=std::move(speed);plan.has_speed_reference=true;}
    for(std::size_t k=0;k<plan.states.size();++k) {
        auto projected=project_state(plan.states[k],plan.controls[std::min(k,plan.controls.size()-1)],c.physics);
        if(k){plan.acceleration.push_back((projected[3]-plan.projected.back()[3])/plan.dt);plan.vertical_acceleration.push_back((projected[5]-plan.projected.back()[5])/plan.dt);}
        plan.projected.push_back(projected);
    }
    return {std::move(plan),""};
}
}
