"""Independent moment signs, actuator saturation and steady trim algebra.

No PX4 or flight commands. Trim is conditional on an illustrative wrench model,
not evidence of dynamic stability, controller tuning or real-aircraft flight.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from swarm_sim.trajectory2d import load_aircraft_profile
from swarm_sim.trajectory2d.dynamics import G
from swarm_sim.gazebo_force_model import ForceModel,ForceAssumptions
from swarm_sim.gazebo_control_model import ControlModel,STABILITY_FIELDS
from swarm_sim.gazebo_trim import trim
ROOT=Path(__file__).resolve().parents[1]


def compile_probe(output):
    source=ROOT/'examples/gazebo/compact_physics/control_probe.cpp';binary=output/'control_probe'
    result=subprocess.run(['g++','-std=c++20','-O2','-fno-fast-math','-ffp-contract=off',str(source),'-o',str(binary)],capture_output=True,text=True)
    (output/'build.log').write_text(result.stdout+result.stderr);result.check_returncode();return binary


def probe(binary,force,control,velocity,rates=(0,0,0),positions=(0,0,0,0,0)):
    values=force.plugin_values()
    args=[*(values[k] for k in ('density','area','cd0','k','clmax','max_brake','lift_slope','zero_lift_angle')),
          *(getattr(control,k) for k in STABILITY_FIELDS),*velocity,*rates,*positions,
          control.surface_limit_rad,control.brake_limit_rad,1]
    result=subprocess.run([str(binary),*map(str,args)],check=True,capture_output=True,text=True)
    values=np.fromstring(result.stdout,sep=' ')
    if values.shape!=(10,) or not np.all(np.isfinite(values)):raise ValueError('invalid control probe result')
    return values


def qualify(output,profile_path,control_path,lift=None):
    force=ForceModel(load_aircraft_profile(profile_path),lift if lift is not None else ForceAssumptions())
    control=ControlModel.load(control_path,force)
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False);binary=compile_probe(output)
    report=dict(**force.record(),controls=control.record(),trials=[],trims=[],passed=False)
    v=force.cruise_test_speed();qS=.5*force.profile.airframe.air_density_kg_m3*v*v*force.profile.airframe.wing_area_m2
    # Zero AoA reference and symmetric per-axis impulse inputs, with independent FLU signs.
    for axis in range(3):
        for sign in (-1.,1.):
            rates=np.zeros(3);rates[axis]=sign*.1
            got=probe(binary,force,control,(v,0,0),rates)
            expected=np.array([0.,-qS*control.chord_m*control.pitch_stiffness*control.alpha_trim_rad,0.])
            lengths=(control.span_m,control.chord_m,control.span_m)
            damping=(control.roll_damping,control.pitch_damping,control.yaw_damping)
            expected[axis]-=qS*lengths[axis]**2*damping[axis]*rates[axis]/(2*v)
            error=float(np.max(np.abs(got[3:6]-expected)))
            report['trials'].append(dict(axis=axis,rate=float(rates[axis]),error_nm=error,passed=error<1e-10))
    limit=control.surface_limit_rad
    for label,positions,expected_control in [('roll',(-2*limit,2*limit,0,0,0),(1,0,0,0)),
        ('pitch',(0,0,2*limit,0,0),(0,1,0,0)),('yaw',(0,0,0,2*limit,0),(0,0,1,0)),
        ('brake',(0,0,0,0,2*control.brake_limit_rad),(0,0,0,1))]:
        got=probe(binary,force,control,(v,0,0),positions=positions)
        report['trials'].append(dict(control=label,passed=bool(np.allclose(got[6:],expected_control,rtol=0,atol=1e-12))))
    for speed,slope in [(v,0.),(force.profile.airspeed_for('dive'),-30.)]:
        row=dict(speed_m_s=speed,slope_deg=slope,passed=False)
        try:
            row.update(trim(force,control,speed,slope))
            alpha=row['alpha_rad'];gamma=np.deg2rad(slope);theta=gamma+alpha
            got=probe(binary,force,control,speed*np.array([np.cos(alpha),0,-np.sin(alpha)]),
                positions=(0,0,row['elevator_fraction']*limit,0,row['brake_fraction']*control.brake_limit_rad))
            residual=got[:3]+[row['thrust_n'],0,0]+force.profile.airframe.mass_kg*G*np.array([-np.sin(theta),0,-np.cos(theta)])
            row.update(force_residual_n=float(np.max(np.abs(residual))),moment_residual_nm=float(np.max(np.abs(got[3:6]))),
                       passed=bool(np.max(np.abs(residual))<1e-8 and np.max(np.abs(got[3:6]))<1e-8))
        except Exception as error:row['error']=repr(error)
        report['trims'].append(row)
    report['passed']=all(t['passed'] for t in report['trials']+report['trims'])
    report['source_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [Path(__file__).resolve(),ROOT/'src/swarm_sim/gazebo_control_model.py',*sorted((ROOT/'examples/gazebo/compact_physics').glob('*'))] if p.is_file()}
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n');return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--aircraft-profile',type=Path,default=ROOT/'examples/aircraft/compact_5kg.json')
    parser.add_argument('--control-model',type=Path,default=ROOT/'examples/gazebo/compact_physics/control_model.json')
    args=parser.parse_args();r=qualify(args.output,args.aircraft_profile,args.control_model)
    print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
