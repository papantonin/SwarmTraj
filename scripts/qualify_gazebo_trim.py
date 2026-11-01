"""Uncontrolled free-body equilibrium and disturbance tests, before PX4 tuning.

Uses constant manufactured actuator/thrust inputs from analytical trim and one
starting impulse. No tracking, sustained velocity commands or real endpoints.
"""
from pathlib import Path
import argparse
import json
import shutil
import subprocess
import uuid
import xml.etree.ElementTree as ET
import numpy as np
from swarm_sim.px4_gazebo import IMAGE,ROOT
from swarm_sim.px4_prototype import child
from swarm_sim.gazebo_force_model import ForceModel
from swarm_sim.gazebo_control_model import ControlModel
from swarm_sim.trajectory2d import load_aircraft_profile
from qualify_control_model import trim


def qualify(output,library,seconds=20.):
    force=ForceModel(load_aircraft_profile(ROOT/'examples/aircraft/compact_5kg.json'))
    control=ControlModel.load(ROOT/'examples/gazebo/compact_physics/control_model.json',force)
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False);shutil.copyfile(library,output/'libcompact_polar.so')
    report=dict(**force.record(),controls=control.record(),trials=[],passed=False,px4_flight_qualified=False)
    for label,speed,slope,perturb,bank,yaw in [('cruise',force.cruise_test_speed(),0.,0.,0.,0.),
        ('dive',force.profile.airspeed_for('dive'),-30.,0.,0.,0.),('cruise-perturbed',force.cruise_test_speed(),0.,.01,0.,0.),
        ('bank-perturbed',force.cruise_test_speed(),0.,0.,.02,0.),('yaw-perturbed',force.cruise_test_speed(),0.,0.,0.,.02)]:
        row=dict(fixture=label,passed=False);case=output/label;case.mkdir()
        try:
            equilibrium=trim(force,control,speed,slope);row['trim']=equilibrium
            dt=.001;steps=round(seconds/dt);gamma=np.deg2rad(slope);pitch=-(gamma+equilibrium['alpha_rad']+perturb)
            root=ET.Element('sdf',version='1.9');world=child(root,'world',name='trim_bench')
            physics=child(world,'physics',type='ignored');child(physics,'max_step_size',dt);child(physics,'real_time_factor',0)
            child(world,'gravity','0 0 -9.81');child(world,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics')
            model=child(world,'model',name='trim_body');child(model,'pose',f'0 0 1000 {bank} {pitch} {yaw}')
            link=child(model,'link',name='base_link');inertial=child(link,'inertial');child(inertial,'mass',force.profile.airframe.mass_kg)
            inertia=child(inertial,'inertia')
            for axis,value in zip(('ixx','iyy','izz'),control.base_inertia_kg_m2):child(inertia,axis,value)
            sphere=child(child(child(link,'collision',name='body'),'geometry'),'sphere');child(sphere,'radius',.05)
            plugin=child(model,'plugin',filename='libcompact_polar.so',name='swarm::CompactPolar')
            values=force.plugin_values(brake_fraction=equilibrium['brake_fraction'],elevator_fraction=equilibrium['elevator_fraction'])
            values.update(control.plugin_values());values.update(control_mode='static',bench_thrust_n=equilibrium['thrust_n'],
                bench_max_thrust_n=force.profile.airframe.max_thrust_n,
                bench_initial_velocity=f'{speed*np.cos(gamma)} 0 {speed*np.sin(gamma)+9.81*dt}',bench_csv='/measurements/states.csv')
            for key,value in values.items():child(plugin,key,value)
            ET.indent(root);ET.ElementTree(root).write(case/'world.sdf',encoding='unicode')
            name='swarm-trim-bench-'+uuid.uuid4().hex
            command=['docker','run','--rm','--name',name,'--network','none','--mount',f'type=bind,src={output},dst=/fixture,readonly',
                '--mount',f'type=bind,src={case},dst=/measurements','--env','GZ_SIM_SYSTEM_PLUGIN_PATH=/fixture',
                '--env','GZ_PARTITION='+name,'--entrypoint','gz',IMAGE,'sim','-r','-s','--iterations',str(steps),f'/fixture/{label}/world.sdf']
            try:
                result=subprocess.run(command,capture_output=True,text=True,timeout=60)
                (case/'gazebo.log').write_text(result.stdout+result.stderr);result.check_returncode()
            finally:subprocess.run(['docker','rm','--force',name],capture_output=True,timeout=15)
            data=np.genfromtxt(case/'states.csv',delimiter=',',names=True)
            if data.size!=steps or not np.all(np.isfinite(np.column_stack([data[k] for k in data.dtype.names]))):raise RuntimeError('incomplete/nonfinite trim telemetry')
            velocities=np.column_stack([data[k] for k in ('vx','vy','vz')]);rates=np.column_stack([data[k] for k in ('wx','wy','wz')])
            nominal=speed*np.array([np.cos(gamma),0,np.sin(gamma)])
            error=np.linalg.norm(velocities-nominal,axis=1)
            lateral=bool(bank or yaw);disturbed=bool(perturb or lateral)
            norm_error=np.abs(np.linalg.norm(velocities,axis=1)-speed)
            selected=norm_error if lateral else error
            row.update(velocity_metric='airspeed norm' if lateral else 'inertial velocity vector',maximum_airspeed_error_m_s=float(norm_error.max()),maximum_velocity_error_m_s=float(error.max()),final_velocity_error_m_s=float(error[-1]),
                maximum_angular_rate_rad_s=float(np.linalg.norm(rates,axis=1).max()),seconds=seconds,
                passed=bool(selected.max()<(2. if disturbed else 1e-5) and np.linalg.norm(rates[-1])<(.05 if disturbed else 1e-5)))
        except Exception as error:row['error']=repr(error)
        report['trials'].append(row);(output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    report['passed']=all(t['passed'] for t in report['trials']);(output/'report.json').write_text(json.dumps(report,indent=2)+'\n');return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--library',type=Path,required=True)
    args=parser.parse_args();r=qualify(args.output,args.library);print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
