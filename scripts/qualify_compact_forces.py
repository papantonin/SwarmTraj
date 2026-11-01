"""Independent analytical comparison of the C++ force kernel for a v3 profile."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from swarm_sim.trajectory2d import load_aircraft_profile
from swarm_sim.trajectory2d.dynamics import G
from swarm_sim.gazebo_force_model import ForceModel,ForceAssumptions
ROOT=Path(__file__).resolve().parents[1]
DEFAULT_PROFILE=ROOT/'examples/aircraft/compact_5kg.json'


def qualify(output,profile_path=DEFAULT_PROFILE,assumptions=None):
    profile=load_aircraft_profile(profile_path)
    model=ForceModel(profile,assumptions if assumptions is not None else ForceAssumptions());a=profile.airframe
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    source=ROOT/'examples/gazebo/compact_physics'
    binary=output/'force_probe'
    compiled=subprocess.run(['g++','-std=c++20','-O2','-fno-fast-math','-ffp-contract=off',str(source/'force_probe.cpp'),'-o',str(binary)],capture_output=True,text=True)
    (output/'build.log').write_text(compiled.stdout+compiled.stderr);compiled.check_returncode()
    def command(velocity,brake=0.,elevator=0.):
        args=[a.air_density_kg_m3,a.wing_area_m2,a.zero_lift_drag_coefficient,a.induced_drag_factor,a.max_lift_coefficient,a.max_airbrake_drag_n,*velocity,elevator,brake,
              model.assumptions.lift_slope_per_rad,model.assumptions.zero_lift_angle_rad]
        return [str(binary),*map(str,args)]
    def probe(velocity,brake=0.,elevator=0.):
        run=subprocess.run(command(velocity,brake,elevator),check=True,capture_output=True,text=True)
        values=np.fromstring(run.stdout,sep=' ')
        if values.shape!=(7,) or not np.all(np.isfinite(values)):raise RuntimeError('invalid force probe output')
        return values
    rows=[]
    for speed in model.test_speeds():
        for slope in (0.,-30.):
            for brake in (0.,.5,1.):
                row=dict(speed_m_s=speed,slope_deg=slope,brake_fraction=brake,passed=False)
                try:
                    n=np.cos(np.deg2rad(slope));alpha=model.load_alpha(speed,n)
                    lift=a.mass_kg*G*n;q=.5*a.air_density_kg_m3*speed**2
                    cl=lift/(q*a.wing_area_m2)
                    velocity=np.array([speed*np.cos(alpha),0.,-speed*np.sin(alpha)])
                    got=probe(velocity,brake)
                    planner_drag=a.mass_kg*(profile.flight.parasitic_drag*speed**2+profile.flight.induced_drag*n*n/speed**2)
                    drag=q*a.wing_area_m2*(a.zero_lift_drag_coefficient+a.induced_drag_factor*cl**2)
                    expected_force=-(drag+a.max_airbrake_drag_n*brake)*velocity/speed+lift*np.array([np.sin(alpha),0.,np.cos(alpha)])
                    error=float(np.max(np.abs(got[:3]-expected_force)))
                    row.update(force_error_n=error,drag_error_vs_planner_n=float(abs(got[5]-planner_drag)),
                        passed=bool(error<1e-10 and abs(got[5]-planner_drag)<1e-10 and abs(got[6]-a.max_airbrake_drag_n*brake)<1e-12))
                except Exception as error:row['error']=repr(error)
                rows.append(row)
    zero=probe((0,0,0),1.)
    invalid=subprocess.run(command((model.cruise_test_speed(),0,0),1.1),capture_output=True)
    report=dict(**model.record(),trials=rows,zero_flow_verified=bool(np.all(zero==0)),
        invalid_brake_rejected=invalid.returncode!=0,
        sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in [Path(__file__).resolve(),ROOT/'src/swarm_sim/gazebo_force_model.py',*sorted(source.glob('*'))] if p.is_file()},
        scope='isolated force algebra; no moment, stall recovery, PX4 trim or flight qualification')
    report['passed']=all(r['passed'] for r in rows) and report['zero_flow_verified'] and report['invalid_brake_rejected']
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--aircraft-profile',type=Path,default=DEFAULT_PROFILE,help='Physical v3 JSON profile')
    parser.add_argument('--force-assumptions',type=Path,help='Separate versioned lift assumptions JSON')
    args=parser.parse_args()
    report=qualify(args.output,args.aircraft_profile,ForceAssumptions.load(args.force_assumptions) if args.force_assumptions else None)
    print(f"{len(report['trials'])} force fixtures: {'PASS' if report['passed'] else 'FAIL'}")
    raise SystemExit(0 if report['passed'] else 1)
