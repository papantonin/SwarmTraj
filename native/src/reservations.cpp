#include "swarmtraj/search.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace swarmtraj {
Occupancy occupancy(const Candidate& p,const PlanningSettings& c) {
    const double radius=c.separation/2+c.aircraft_radius+c.position_error+c.physics.max_speed*(c.clock_error+p.dt/2);
    const double size=c.cell_size;
    if(!std::isfinite(radius)||std::max(1.,2*radius/size)>std::pow(static_cast<double>(c.max_entries),1./3))
        throw ReservationBudgetError("Inflated reservation exceeds the cell/time construction budget.");
    Occupancy out;out.drone_id=p.drone_id;out.candidate_id=p.candidate_id;out.inflation=radius;std::size_t entries=0;
    for(std::size_t n=0;n<p.projected.size();++n) {
        const auto& s=p.projected[n];const std::array<double,3> point{s[0],s[1],s[4]};
        double largest=0.;for(double v:point)largest=std::max(largest,std::abs(v/size));
        if(largest+radius/size>=static_cast<double>(std::numeric_limits<long long>::max())/2)
            throw ReservationBudgetError("Reservation coordinates exceed the supported integer grid range.");
        Cell low,high;double count=1.;
        for(std::size_t k=0;k<3;++k){low[k]=static_cast<long long>(std::floor((point[k]-radius)/size));high[k]=static_cast<long long>(std::floor((point[k]+radius)/size));count*=static_cast<double>(high[k])-low[k]+1;}
        if(count>c.max_entries-entries)throw ReservationBudgetError("Per-candidate cell/time construction budget exhausted; no infeasibility claim.");
        entries+=static_cast<std::size_t>(count);
        const double begin=std::max(0.,(static_cast<double>(n)-.5)*p.dt),end=std::min(p.controls.size()*p.dt,(n+.5)*p.dt);
        for(auto x=low[0];x<=high[0];++x)for(auto y=low[1];y<=high[1];++y)for(auto z=low[2];z<=high[2];++z) {
            auto& intervals=out.cells[{x,y,z}];
            if(!intervals.empty()&&begin<=intervals.back()[1]+1e-10)intervals.back()[1]=end;
            else intervals.push_back({begin,end});
        }
    }
    return out;
}
std::optional<ReservationConflict> first_conflict(const Occupancy& a,const Occupancy& b) {
    std::optional<ReservationConflict> best;
    for(const auto& [cell,first]:a.cells) {
        auto found=b.cells.find(cell);if(found==b.cells.end())continue;const auto& second=found->second;
        std::size_t i=0,j=0;
        while(i<first.size()&&j<second.size()) {
            const double begin=std::max(first[i][0],second[j][0]),end=std::min(first[i][1],second[j][1]);
            if(begin<=end+1e-10) {
                ReservationConflict item{cell,begin,std::max(begin,end),{a.drone_id,b.drone_id}};
                if(!best||begin<best->start||(begin==best->start&&cell<best->cell))best=item;
                break;
            }
            if(first[i][1]<second[j][1])++i;else ++j;
        }
    }
    return best;
}
Value conflict_record(const ReservationConflict& item) {
    return object({{"cell",array({item.cell[0],item.cell[1],item.cell[2]})},{"start_s",item.start},{"end_s",item.end},
        {"drone_ids",array({item.drones[0],item.drones[1]})}});
}
DepartureWindows departure_windows(const Candidate& parent,const Occupancy& own,const std::vector<const Occupancy*>& others,
    const PlanningSettings& c) {
    DepartureWindows result;result.parent=parent.candidate_id;
    for(auto* other:others)result.references.push_back(other->drone_id);
    std::sort(result.references.begin(),result.references.end());auto sorted=others;
    std::sort(sorted.begin(),sorted.end(),[](auto a,auto b){return a->drone_id<b->drone_id;});
    const double low=c.activation+c.min_delay,high=c.activation+c.max_delay;
    std::vector<Point> forbidden;
    for(auto* other:sorted)for(const auto& [cell,intervals]:own.cells) {
        auto found=other->cells.find(cell);if(found==other->cells.end())continue;
        for(auto own_interval:intervals) {
            if(own_interval[1]<parent.release)continue;
            own_interval[0]=std::max(own_interval[0],parent.release);
            for(auto blocked:found->second) {
                if(++result.constraints>c.max_constraints){result.exhausted=true;return result;}
                const double start=std::max(low,parent.release+blocked[0]-own_interval[1]-1e-6);
                const double end=std::min(high,parent.release+blocked[1]-own_interval[0]+1e-6);
                if(start<=end)forbidden.push_back({start,end});
            }
        }
    }
    if(low==high){if(forbidden.empty())result.intervals.push_back({low,high});return result;}
    std::sort(forbidden.begin(),forbidden.end());std::vector<Point> merged;
    for(auto interval:forbidden) {
        if(!merged.empty()&&interval[0]<=merged.back()[1])merged.back()[1]=std::max(merged.back()[1],interval[1]);
        else merged.push_back(interval);
    }
    std::vector<Point> free;double cursor=0.;
    for(auto interval:merged) {
        if(cursor<interval[0])free.push_back({cursor,std::nextafter(interval[0],-std::numeric_limits<double>::infinity())});
        cursor=std::nextafter(interval[1],std::numeric_limits<double>::infinity());
    }
    if(cursor<c.steps*c.physics.dt)free.push_back({cursor,c.steps*c.physics.dt});
    for(auto interval:free){interval[0]=std::max(interval[0],low);interval[1]=std::min(interval[1],high);if(interval[0]<=interval[1])result.intervals.push_back(interval);}
    return result;
}
}
