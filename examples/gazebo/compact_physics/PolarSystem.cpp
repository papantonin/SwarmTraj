#include "control_dynamics.hpp"
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <sdf/Element.hh>
#include <fstream>
#include <iomanip>
#include <optional>
namespace compact {
// Physics-only, model-local component. No network callbacks, target, plan or
// aircraft-state sharing. Inputs are fixed bench fractions or own joint positions.
class PolarSystem final:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate {
    gz::sim::Link link;Polar polar{};double brake=0,elevator=0;
    double aileron=0,rudder=0;
    double bench_thrust=0;
    std::optional<Stability> stability;bool joint_mode=false;
    std::array<gz::sim::Joint,5> joints;
    gz::sim::Joint rotor_joint;
    double surface_limit=.5,brake_limit=.7;std::size_t missing_positions=0;
    bool actuators_seen=false,physics_seen=false;
    Vec applied_torque{};
    Controls observed{};
    std::optional<gz::math::Vector3d> initial;std::ofstream log;gz::math::Vector3d applied{};
public:
 void Configure(const gz::sim::Entity& e,const std::shared_ptr<const sdf::Element>& s,
                gz::sim::EntityComponentManager& ecm,gz::sim::EventManager&) override {
    auto value=[&](const char* key){if(!s->HasElement(key))throw std::invalid_argument(key);return s->Get<double>(key);};
    polar={value("density"),value("area"),value("cd0"),value("k"),value("clmax"),value("max_brake"),value("lift_slope"),value("zero_lift_angle")};
    brake=value("brake_fraction");elevator=value("elevator_fraction");evaluate(polar,{0,0,0},elevator,brake);
    if(s->HasElement("aileron_fraction"))aileron=value("aileron_fraction");
    if(s->HasElement("rudder_fraction"))rudder=value("rudder_fraction");
    link=gz::sim::Link(gz::sim::Model(e).LinkByName(ecm,"base_link"));
    if(!link.Valid(ecm))throw std::invalid_argument("missing own base_link");
    link.EnableVelocityChecks(ecm);
    if(s->HasElement("control_mode")) {
        const auto mode=s->Get<std::string>("control_mode");
        if(mode!="joints"&&mode!="static")throw std::invalid_argument("unknown control mode");
        joint_mode=mode=="joints";
    }
    if(s->HasElement("pitch_stiffness")) {
        stability=Stability{value("span"),value("chord"),value("alpha_trim"),value("roll_beta"),value("roll_damping"),value("roll_control"),
            value("pitch_stiffness"),value("pitch_damping"),value("pitch_control"),value("yaw_stiffness"),value("yaw_damping"),value("yaw_control"),value("sideforce")};
        validate(*stability);
        wrench(polar,*stability,{0,0,0},{0,0,0},{aileron,elevator,rudder,brake});
    }
    if(!stability&&(aileron!=0||rudder!=0))throw std::invalid_argument("static roll/yaw inputs require stability model");
    if(s->HasElement("bench_thrust_n")) {
        bench_thrust=value("bench_thrust_n");const double maximum=value("bench_max_thrust_n");
        if(!std::isfinite(bench_thrust)||!std::isfinite(maximum)||bench_thrust<0||bench_thrust>maximum)
            throw std::invalid_argument("invalid manufactured thrust input");
    }
    if(joint_mode) {
        if(!stability)throw std::invalid_argument("own joint control requires stability model");
        surface_limit=value("surface_limit");brake_limit=value("brake_limit");positions({0,0,0,0,0},surface_limit,brake_limit);
        const char* names[]={"servo_0","servo_1","servo_2","rudder_joint","airbrake_joint"};
        for(std::size_t k=0;k<joints.size();++k) {
            joints[k]=gz::sim::Joint(gz::sim::Model(e).JointByName(ecm,names[k]));
            if(!joints[k].Valid(ecm))throw std::invalid_argument("missing own actuator joint");
            joints[k].EnablePositionCheck(ecm);
        }
        rotor_joint=gz::sim::Joint(gz::sim::Model(e).JointByName(ecm,"rotor_puller_joint"));
        if(!rotor_joint.Valid(ecm))throw std::invalid_argument("missing own rotor joint");
        rotor_joint.EnableVelocityCheck(ecm);
    }
    if(s->HasElement("bench_initial_velocity"))initial=s->Get<gz::math::Vector3d>("bench_initial_velocity");
    if(s->HasElement("bench_csv")) {
        log.open(s->Get<std::string>("bench_csv"));if(!log)throw std::invalid_argument("cannot open bench log");
        log<<"time,vx,vy,vz,fx,fy,fz,wx,wy,wz,tx,ty,tz,aileron,elevator,rudder,brake,rotor_joint_rad_s\n"<<std::setprecision(17);
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
    if(!pose||!velocity){if(physics_seen)throw std::runtime_error("own body kinematics unavailable");return;}
    const auto local=pose->Rot().RotateVectorReverse(*velocity);
    Controls controls{aileron,elevator,rudder,brake};
    if(joint_mode) {
        std::array<double,5> q;
        for(std::size_t k=0;k<joints.size();++k) {
            auto position=joints[k].Position(ecm);
            if(!position||position->size()!=1) {
                if(actuators_seen||++missing_positions>100)throw std::runtime_error("own actuator positions unavailable");
                return;
            }
            q[k]=position->at(0);
        }
        missing_positions=0;actuators_seen=true;controls=positions(q,surface_limit,brake_limit);
    }
    observed=controls;
    auto result=evaluate(polar,{local.X(),local.Y(),local.Z()},controls.elevator,controls.brake);
    applied_torque={0,0,0};
    if(stability) {
        auto rates=link.WorldAngularVelocity(ecm);
        if(!rates){if(physics_seen)throw std::runtime_error("own angular velocity unavailable");return;}
        auto r=pose->Rot().RotateVectorReverse(*rates);
        auto w=wrench(polar,*stability,{local.X(),local.Y(),local.Z()},{r.X(),r.Y(),r.Z()},controls);
        result=w.aerodynamic;applied_torque=w.torque;
    }
    auto force=pose->Rot().RotateVector(gz::math::Vector3d(result.force[0]+bench_thrust,result.force[1],result.force[2]));
    applied=force;
    auto torque=pose->Rot().RotateVector(gz::math::Vector3d(applied_torque[0],applied_torque[1],applied_torque[2]));
    applied_torque={torque.X(),torque.Y(),torque.Z()};
    link.AddWorldWrench(ecm,force,torque);
    physics_seen=true;
 }
 void PostUpdate(const gz::sim::UpdateInfo& info,const gz::sim::EntityComponentManager& ecm) override {
    if(info.paused||!log)return;
    auto v=link.WorldLinearVelocity(ecm);if(!v)return;
    auto w=link.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);
    double rotor_rate=0;
    if(joint_mode){auto v=rotor_joint.Velocity(ecm);if(v&&v->size()==1)rotor_rate=v->at(0);}
    log<<std::chrono::duration<double>(info.simTime).count()<<','<<v->X()<<','<<v->Y()<<','<<v->Z()
       <<','<<applied.X()<<','<<applied.Y()<<','<<applied.Z()<<','<<w.X()<<','<<w.Y()<<','<<w.Z()
       <<','<<applied_torque[0]<<','<<applied_torque[1]<<','<<applied_torque[2]
       <<','<<observed.aileron<<','<<observed.elevator<<','<<observed.rudder<<','<<observed.brake<<','<<rotor_rate<<'\n';
 }
};
}
GZ_ADD_PLUGIN(compact::PolarSystem,gz::sim::System,compact::PolarSystem::ISystemConfigure,compact::PolarSystem::ISystemPreUpdate,compact::PolarSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(compact::PolarSystem,"swarm::CompactPolar")
extern "C" int swarm_compact_control_api() {return 1;}
