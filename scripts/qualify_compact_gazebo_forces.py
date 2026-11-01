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
ROOT=Path(__file__).resolve().parents[1]


def qualify(output,library):
    output=output.resolve();output.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(library,output/'libcompact_polar.so')
    profile=load_aircraft_profile(ROOT/'examples/aircraft/compact_5kg.json');a=profile.airframe
    report=dict(image=IMAGE,profile=profile.record(),dt_s=.001,iterations_per_fixture=100,
                plugin_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),trials=[],passed=False,
                sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in [Path(__file__).resolve(),*sorted((ROOT/'examples/gazebo/compact_physics').glob('*'))] if p.is_file()},
                scope='no-gravity free-body force check, not PX4 trim or flight')
    lift_alpha=(5*9.81/(.5*a.air_density_kg_m3*(100/3.6)**2*a.wing_area_m2))/4.75-.06
    fixtures=[('drag',0.,-.285,0.,0.),('brake',1.,-.285,0.,0.),
              ('lift',0.,0.,lift_alpha,0.),('rotated-brake',1.,-.285,0.,np.pi/2)]
    for label,brake,elevator,alpha,yaw in fixtures:
        case=output/label;case.mkdir()
        root=ET.Element('sdf',version='1.9');world=child(root,'world',name='force_bench')
        physics=child(world,'physics',type='ignored');child(physics,'max_step_size',.001);child(world,'gravity','0 0 0')
        child(world,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics')
        model=child(world,'model',name='force_body');child(model,'pose',f'0 0 0 0 0 {yaw}')
        link=child(model,'link',name='base_link')
        sphere=child(child(child(link,'collision',name='body'),'geometry'),'sphere');child(sphere,'radius',.05)
        inertial=child(link,'inertial');child(inertial,'mass',5);inertia=child(inertial,'inertia')
        for axis in ('ixx','iyy','izz'):child(inertia,axis,.05)
        plugin=child(model,'plugin',filename='libcompact_polar.so',name='swarm::CompactPolar')
        velocity=(100/3.6)*np.array([np.cos(alpha),0.,-np.sin(alpha)])
        rotation=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
        initial=rotation@velocity
        values=dict(density=a.air_density_kg_m3,area=a.wing_area_m2,cd0=a.zero_lift_drag_coefficient,
            k=a.induced_drag_factor,clmax=a.max_lift_coefficient,max_brake=a.max_airbrake_drag_n,
            lift_slope=4.75,zero_lift_angle=-.06,brake_fraction=brake,elevator_fraction=elevator,
            bench_initial_velocity=' '.join(map(str,initial)),bench_csv='/measurements/states.csv')
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
            residual=float(np.max(np.abs(5*dv/dt[:,None]-force)))
            qS=.5*a.air_density_kg_m3*a.wing_area_m2*(100/3.6)**2
            cl=np.clip(4.75*(alpha+.06)+elevator,-a.max_lift_coefficient,a.max_lift_coefficient)
            drag=qS*(a.zero_lift_drag_coefficient+a.induced_drag_factor*cl**2)+a.max_airbrake_drag_n*brake
            expected=rotation@(-drag*velocity/(100/3.6)+qS*cl*np.array([np.sin(alpha),0,np.cos(alpha)]))
            first_error=float(np.max(np.abs(force[0]-expected)))
            row=dict(fixture=label,brake_fraction=brake,maximum_force_balance_residual_n=residual,
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
    args=parser.parse_args();report=qualify(args.output,args.library)
    print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
