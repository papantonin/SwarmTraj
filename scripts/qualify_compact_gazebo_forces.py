"""Isolated force-to-acceleration checks in pinned Gazebo, without PX4.

A single free body receives one starting impulse. No continuous pose/velocity
commands, flight guidance, production aircraft or live endpoints are used.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import uuid
import xml.etree.ElementTree as ET
import numpy as np
from swarm_sim.px4_gazebo import IMAGE
from swarm_sim.px4_prototype import child
from swarm_sim.trajectory2d import load_aircraft_profile
from swarm_sim.gazebo_force_model import ForceModel,ForceAssumptions
ROOT=Path(__file__).resolve().parents[1]


def qualify(output,library,profile_path=None,assumptions=None):
    profile=load_aircraft_profile(profile_path or ROOT/'examples/aircraft/compact_5kg.json')
    model_binding=ForceModel(profile,assumptions if assumptions is not None else ForceAssumptions())
    a=profile.airframe;mass=a.mass_kg;speed=model_binding.cruise_test_speed()
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(library,output/'libcompact_polar.so')
    report=dict(image=IMAGE,**model_binding.record(),test_speed_m_s=speed,dt_s=.001,iterations_per_fixture=100,
                plugin_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),trials=[],passed=False,
                sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in [Path(__file__).resolve(),ROOT/'src/swarm_sim/gazebo_force_model.py',*sorted((ROOT/'examples/gazebo/compact_physics').glob('*'))] if p.is_file()},
                scope='no-gravity free-body force check, not PX4 trim or flight')
    neutral=model_binding.assumptions.lift_slope_per_rad*model_binding.assumptions.zero_lift_angle_rad
    zero_alpha=0. if abs(neutral)<=1 else model_binding.assumptions.zero_lift_angle_rad
    neutral=neutral if abs(neutral)<=1 else 0.
    fixtures=[('drag',0.,neutral,zero_alpha,0.),('brake',1.,neutral,zero_alpha,0.),
              ('lift',0.,0.,None,0.),('rotated-brake',1.,neutral,zero_alpha,np.pi/2)]
    for label,brake,elevator,alpha,yaw in fixtures:
        if alpha is None:
            try:alpha=model_binding.load_alpha(speed)
            except ValueError as error:
                report['trials'].append(dict(fixture=label,passed=False,error=str(error)))
                (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
                continue
        case=output/label;case.mkdir()
        root=ET.Element('sdf',version='1.9');world=child(root,'world',name='force_bench')
        physics=child(world,'physics',type='ignored');child(physics,'max_step_size',.001);child(world,'gravity','0 0 0')
        child(world,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics')
        model=child(world,'model',name='force_body');child(model,'pose',f'0 0 0 0 0 {yaw}')
        link=child(model,'link',name='base_link')
        sphere=child(child(child(link,'collision',name='body'),'geometry'),'sphere');child(sphere,'radius',.05)
        inertial=child(link,'inertial');child(inertial,'mass',mass);inertia=child(inertial,'inertia')
        for axis in ('ixx','iyy','izz'):child(inertia,axis,.4*mass*.05**2)
        plugin=child(model,'plugin',filename='libcompact_polar.so',name='swarm::CompactPolar')
        velocity=speed*np.array([np.cos(alpha),0.,-np.sin(alpha)])
        rotation=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
        initial=rotation@velocity
        values=model_binding.plugin_values(brake_fraction=brake,elevator_fraction=elevator)
        values.update(bench_initial_velocity=' '.join(map(str,initial)),bench_csv='/measurements/states.csv')
        for key,value in values.items():child(plugin,key,value)
        ET.indent(root);ET.ElementTree(root).write(case/'bench.sdf',encoding='unicode')
        name='swarm-force-bench-'+uuid.uuid4().hex
        command=['docker','run','--rm','--name',name,'--network','none',
            '--mount',f'type=bind,src={output},dst=/fixture,readonly',
            '--mount',f'type=bind,src={case},dst=/measurements',
            '--env','GZ_SIM_SYSTEM_PLUGIN_PATH=/fixture','--env','GZ_PARTITION='+name,
            '--entrypoint','gz',IMAGE,'sim','-r','-s','--iterations','100',f'/fixture/{case.name}/bench.sdf']
        try:
            result=subprocess.run(command,capture_output=True,text=True,timeout=60)
            (case/'gazebo.log').write_text(result.stdout+result.stderr);result.check_returncode()
            data=np.genfromtxt(case/'states.csv',delimiter=',',names=True)
            if data.size!=100:raise RuntimeError('incomplete Gazebo force telemetry')
            dt=np.diff(data['time']);dv=np.diff(np.column_stack([data[k] for k in ('vx','vy','vz')]),axis=0)
            if not np.all(np.isfinite(np.column_stack([data[k] for k in data.dtype.names]))):
                raise RuntimeError('nonfinite force telemetry')
            if not np.allclose(dt,.001,rtol=0,atol=1e-12) or abs(data['time'][-1]-.1)>1e-12:
                raise RuntimeError('unexpected force telemetry clock')
            if not np.allclose([data[k][0] for k in ('vx','vy','vz')],initial,rtol=0,atol=1e-8):
                raise RuntimeError('initial impulse did not establish the requested state')
            # The first row is the single starting impulse; every later row
            # participates in the force balance, including any unexpected force.
            force=np.column_stack([data[k] for k in ('fx','fy','fz')])[1:]
            residual=float(np.max(np.abs(mass*dv/dt[:,None]-force)))
            qS=.5*a.air_density_kg_m3*a.wing_area_m2*speed**2
            cl=np.clip(model_binding.assumptions.lift_slope_per_rad*(alpha-model_binding.assumptions.zero_lift_angle_rad)+elevator,-a.max_lift_coefficient,a.max_lift_coefficient)
            drag=qS*(a.zero_lift_drag_coefficient+a.induced_drag_factor*cl**2)+a.max_airbrake_drag_n*brake
            expected=rotation@(-drag*velocity/speed+qS*cl*np.array([np.sin(alpha),0,np.cos(alpha)]))
            first_error=float(np.max(np.abs(force[0]-expected)))
            row=dict(fixture=label,brake_fraction=brake,maximum_force_balance_residual_n=residual,
                applied_brake_force_n=a.max_airbrake_drag_n*brake,
                first_force_n=force[0].tolist(),expected_initial_force_n=expected.tolist(),
                initial_force_error_n=first_error,passed=bool(residual<1e-5 and first_error<1e-5))
        except Exception as error:
            row=dict(fixture=label,brake_fraction=brake,passed=False,error=repr(error))
        finally:
            subprocess.run(['docker','rm','--force',name],capture_output=True,timeout=15)
        report['trials'].append(row)
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    report['passed']=all(row['passed'] for row in report['trials'])
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--library',type=Path,required=True)
    parser.add_argument('--aircraft-profile',type=Path,help='Physical v3 JSON profile; defaults to compact 5 kg')
    parser.add_argument('--force-assumptions',type=Path,help='Separate versioned lift assumptions JSON')
    args=parser.parse_args();report=qualify(args.output,args.library,args.aircraft_profile,
        ForceAssumptions.load(args.force_assumptions) if args.force_assumptions else None)
    print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
