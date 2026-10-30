#include "swarmtraj/mission.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace swarmtraj {
namespace {
// Square shortest-augmenting-path assignment. Reverse remaining-column order
// and free-column ties match SciPy's rectangular_lsap algorithm (Crouse 2016).
std::vector<std::size_t> assign(const std::vector<std::vector<double>>& cost) {
    const auto n=cost.size();const auto infinity=std::numeric_limits<double>::infinity();
    std::vector<double> u(n),v(n),shortest(n);
    std::vector<long long> col4row(n,-1),row4col(n,-1),path(n);
    std::vector<bool> seen_rows(n),seen_cols(n);std::vector<std::size_t> remaining(n);
    for(std::size_t current=0;current<n;++current) {
        double minimum=0.;std::fill(shortest.begin(),shortest.end(),infinity);std::fill(seen_rows.begin(),seen_rows.end(),false);std::fill(seen_cols.begin(),seen_cols.end(),false);
        for(std::size_t k=0;k<n;++k)remaining[k]=n-k-1;
        std::size_t count=n,i=current;long long sink=-1;
        while(sink<0) {
            seen_rows[i]=true;double lowest=infinity;std::size_t best=0;
            for(std::size_t k=0;k<count;++k) {
                const auto j=remaining[k];const double reduced=minimum+cost[i][j]-u[i]-v[j];
                if(reduced<shortest[j]){shortest[j]=reduced;path[j]=i;}
                if(shortest[j]<lowest||(shortest[j]==lowest&&row4col[j]<0)){lowest=shortest[j];best=k;}
            }
            minimum=lowest;if(!std::isfinite(minimum))throw std::invalid_argument("infeasible approach assignment");
            const auto j=remaining[best];if(row4col[j]<0)sink=j;else i=row4col[j];
            seen_cols[j]=true;remaining[best]=remaining[--count];
        }
        u[current]+=minimum;
        for(std::size_t row=0;row<n;++row)if(seen_rows[row]&&row!=current)u[row]+=minimum-shortest[col4row[row]];
        for(std::size_t col=0;col<n;++col)if(seen_cols[col])v[col]-=minimum-shortest[col];
        auto j=sink;
        while(true){const auto row=path[j];row4col[j]=row;std::swap(col4row[row],j);if(static_cast<std::size_t>(row)==current)break;}
    }
    std::vector<std::size_t> result;for(auto col:col4row)result.push_back(col);return result;
}
}
std::map<long long,std::optional<double>> approach_angles(const std::vector<Designation>& snapshots,const std::vector<PlanningSettings>& settings) {
    std::map<long long,std::optional<double>> result;if(snapshots.empty())return result;
    const auto& c=settings.front();std::vector<std::size_t> order(snapshots.size());std::iota(order.begin(),order.end(),0);
    std::sort(order.begin(),order.end(),[&](auto a,auto b) {
        if(!c.id_approach){if(snapshots[a].initial[0]!=snapshots[b].initial[0])return snapshots[a].initial[0]<snapshots[b].initial[0];if(snapshots[a].initial[1]!=snapshots[b].initial[1])return snapshots[a].initial[1]<snapshots[b].initial[1];}
        return snapshots[a].drone_id<snapshots[b].drone_id;
    });
    if(c.spread_degrees==0){for(auto i:order)result[snapshots[i].drone_id]={};return result;}
    Point offset{};for(auto i:order){offset[0]+=snapshots[i].initial[0]-c.target[0];offset[1]+=snapshots[i].initial[1]-c.target[1];}
    offset[0]/=order.size();offset[1]/=order.size();
    const double center=std::sqrt(offset[0]*offset[0]+offset[1]*offset[1])>1e-9?std::atan2(offset[1],offset[0]):0.;
    const double spread=c.spread_degrees*(pi/180);std::vector<double> angles(order.size(),center);
    if(order.size()>1) {
        const auto divisor=c.spread_degrees<360?order.size()-1:order.size();const double step=spread/divisor;
        for(std::size_t k=0;k<angles.size();++k)angles[k]=center+(-spread/2+k*step);
        if(c.spread_degrees<360)angles.back()=center+spread/2;
    }
    std::vector<std::size_t> columns(order.size());std::iota(columns.begin(),columns.end(),0);
    if(!c.id_approach) {
        double radius=120.;for(auto i:order)radius=std::max(radius,6*snapshots[i].initial[3]/(gravity*std::tan(settings[i].physics.max_bank)/snapshots[i].initial[3]));
        std::vector<std::vector<double>> costs(order.size(),std::vector<double>(order.size()));
        for(std::size_t row=0;row<order.size();++row)for(std::size_t col=0;col<order.size();++col) {
            const auto i=order[row];const double x=snapshots[i].initial[0]-c.target[0]-radius*std::cos(angles[col]);
            const double y=snapshots[i].initial[1]-c.target[1]-radius*std::sin(angles[col]);costs[row][col]=std::sqrt(x*x+y*y);
        }
        columns=assign(costs);
    }
    for(std::size_t k=0;k<order.size();++k)result[snapshots[order[k]].drone_id]=wrap(angles[columns[k]]);
    return result;
}
}
