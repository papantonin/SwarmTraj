#include "swarmtraj/planning.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace swarmtraj;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main() {
    Pcg64 rng(std::vector<std::uint32_t>{7,3});require(rng.raw()==0xf99bcc89ba58d5c6ULL,"PCG64 golden raw word changed");
    PlanningSettings c;
    c.physics={6,1.15,2.5,.2,45*pi/180,8*pi/180,5,5,.003,20,8,16,50*pi/180,60*pi/180,0,200,3,8,1,.2};
    c.steps=600;c.latest_arrival=114.85;c.target={500,0};c.target_altitude=30;c.cruise_range=Point{70,100};
    c.max_delay=0;c.attempts=6;c.candidates=2;
    Designation s;s.initial={25,0,80,13,pi/2,0,std::atan(13*.52/gravity)};s.yaw_rate=.52;
    Catalog catalog;extend_catalog(catalog,s,c,1,{});
    require(catalog.plans.size()==1,"native complete proposal was not accepted");
    require(validate_candidate(catalog.plans[0],s.initial,s.yaw_rate,{},c).accepted(),"independent catalog replay failed");
    const auto before=catalog.plans[0].states;
    extend_catalog(catalog,s,c,2,{});
    require(catalog.plans.size()==2&&catalog.plans[0].states==before,"catalog resumption changed accepted history");
    require(catalog.rng.has_value(),"resumable RNG state missing");
    for(const double padding:{-1.,std::numeric_limits<double>::infinity()}) {
        Pcg64 unchanged(std::vector<std::uint32_t>{7,3});const auto initial=unchanged.state();
        auto invalid=physical_proposal(s,c,0,unchanged,{},nullptr,"",{},{},padding);
        require(!invalid.plan&&invalid.rejection=="invalid_proposal_adjustment","invalid padding was accepted");
        require(unchanged.state().state.high==initial.state.high&&unchanged.state().state.low==initial.state.low,
                "invalid adjustment consumed RNG state");
    }
    Pcg64 unchanged(std::vector<std::uint32_t>{7,3});
    require(physical_proposal(s,c,0,unchanged,{},nullptr,"",{},c.level_hold-.1).rejection=="invalid_proposal_adjustment",
            "shorter settling request was accepted");
    auto invalid_lead=c;invalid_lead.approach_lead_extra_s=-1.;
    require(physical_proposal(s,invalid_lead,0,unchanged,{}).rejection=="invalid_proposal_adjustment",
            "negative alignment extension accepted");
    auto feedback=c;feedback.rejection_feedback=true;Catalog retry;
    retry.rejections["physical_approach_alignment"]=2;
    extend_catalog(retry,s,feedback,1,0.);
    require(!retry.adjustments.empty(),"own alignment failures did not produce a bounded hint");
    const auto& hint=retry.adjustments.front();
    require(std::get<std::string>(hint.at("mode").data)=="longer_approach"
        &&std::get<double>(hint.at("approach_lead_extra_s").data)==5.,"wrong alignment hint");
    require(retry.attempted<=feedback.attempts,"alignment hint exceeded proposal budget");
    std::cout<<"native proposals and catalog tests passed\n";
}
