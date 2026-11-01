#include "aerodynamics.hpp"
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <sdf/Element.hh>
#include <fstream>
#include <iomanip>
#include <optional>
namespace compact {
// Physics-only, model-local component. No network callbacks, target, plan or
// aircraft-state sharing. Bench controls are immutable SDF inputs for now.
class PolarSystem final:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate {
    gz::sim::Link link;Polar polar{};double brake=0,elevator=0;
    std::optional<gz::math::Vector3d> initial;std::ofstream log;gz::math::Vector3d applied{};
public:
 void Configure(const gz::sim::Entity& e,const std::shared_ptr<const sdf::Element>& s,
                gz::sim::EntityComponentManager& ecm,gz::sim::EventManager&) override {
    auto value=[&](const char* key){if(!s->HasElement(key))throw std::invalid_argument(key);return s->Get<double>(key);};
    polar={value("density"),value("area"),value("cd0"),value("k"),value("clmax"),value("max_brake"),value("lift_slope"),value("zero_lift_angle")};
    brake=value("brake_fraction");elevator=value("elevator_fraction");evaluate(polar,{0,0,0},elevator,brake);
    link=gz::sim::Link(gz::sim::Model(e).LinkByName(ecm,"base_link"));
    if(!link.Valid(ecm))throw std::invalid_argument("missing own base_link");
    link.EnableVelocityChecks(ecm);
    if(s->HasElement("bench_initial_velocity"))initial=s->Get<gz::math::Vector3d>("bench_initial_velocity");
    if(s->HasElement("bench_csv")) {
        log.open(s->Get<std::string>("bench_csv"));if(!log)throw std::invalid_argument("cannot open bench log");
        log<<"time,vx,vy,vz,fx,fy,fz\n"<<std::setprecision(17);
    }
 }
 void PreUpdate(const gz::sim::UpdateInfo& info,gz::sim::EntityComponentManager& ecm) override {
    if(info.paused)return;
    if(initial){
        auto inertia=link.WorldInertial(ecm);
        const double dt=std::chrono::duration<double>(info.dt).count();
        if(!inertia||dt<=0)throw std::runtime_error("invalid impulse bench state");
        applied=*initial*(inertia->MassMatrix().Mass()/dt);
        link.AddWorldForce(ecm,applied);initial.reset();return;
    }
    auto pose=link.WorldPose(ecm);auto velocity=link.WorldLinearVelocity(ecm);
    if(!pose||!velocity)return;
    const auto local=pose->Rot().RotateVectorReverse(*velocity);
    auto result=evaluate(polar,{local.X(),local.Y(),local.Z()},elevator,brake);
    auto force=pose->Rot().RotateVector(gz::math::Vector3d(result.force[0],result.force[1],result.force[2]));
    applied=force;
    link.AddWorldForce(ecm,force);
 }
 void PostUpdate(const gz::sim::UpdateInfo& info,const gz::sim::EntityComponentManager& ecm) override {
    if(info.paused||!log)return;
    auto v=link.WorldLinearVelocity(ecm);if(!v)return;
    log<<std::chrono::duration<double>(info.simTime).count()<<','<<v->X()<<','<<v->Y()<<','<<v->Z()
       <<','<<applied.X()<<','<<applied.Y()<<','<<applied.Z()<<'\n';
 }
};
}
GZ_ADD_PLUGIN(compact::PolarSystem,gz::sim::System,compact::PolarSystem::ISystemConfigure,compact::PolarSystem::ISystemPreUpdate,compact::PolarSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(compact::PolarSystem,"swarm::CompactPolar")
