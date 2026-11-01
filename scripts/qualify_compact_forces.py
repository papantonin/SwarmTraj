"""Independent analytical comparison of the isolated C++ compact force kernel."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from swarm_sim.trajectory2d import load_aircraft_profile
from swarm_sim.trajectory2d.dynamics import G
ROOT=Path(__file__).resolve().parents[1]


def qualify(output):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    profile=load_aircraft_profile(ROOT/'examples/aircraft/compact_5kg.json');a=profile.airframe
    source=ROOT/'examples/gazebo/compact_physics'
    binary=output/'force_probe'
    compiled=subprocess.run(['g++','-std=c++20','-O2','-fno-fast-math','-ffp-contract=off',str(source/'force_probe.cpp'),'-o',str(binary)],capture_output=True,text=True)
    (output/'build.log').write_text(compiled.stdout+compiled.stderr);compiled.check_returncode()
    def probe(velocity,brake=0.,elevator=0.):
        args=[a.air_density_kg_m3,a.wing_area_m2,a.zero_lift_drag_coefficient,a.induced_drag_factor,a.max_lift_coefficient,a.max_airbrake_drag_n,*velocity,elevator,brake]
        run=subprocess.run([str(binary),*map(str,args)],check=True,capture_output=True,text=True)
        return np.fromstring(run.stdout,sep=' ')
    rows=[]
    for speed in (25.5,100/3.6,175/3.6,200/3.6):
        for slope in (0.,-30.):
            for brake in (0.,.5,1.):
                # Set angle of attack for L=mg*cos(gamma), independently from flight-path angle.
                lift=5*G*np.cos(np.deg2rad(slope));q=.5*a.air_density_kg_m3*speed**2
                cl=lift/(q*a.wing_area_m2);alpha=cl/4.75-.06
                velocity=np.array([speed*np.cos(alpha),0.,-speed*np.sin(alpha)])
                got=probe(velocity,brake)
                drag=q*a.wing_area_m2*(a.zero_lift_drag_coefficient+a.induced_drag_factor*cl**2)
                # Independent planner conversion, including induced drag at load cos(gamma).
                n=np.cos(np.deg2rad(slope))
                planner_drag=5*(profile.flight.parasitic_drag*speed**2+profile.flight.induced_drag*n*n/speed**2)
                expected_force=-(drag+20*brake)*velocity/speed+lift*np.array([np.sin(alpha),0.,np.cos(alpha)])
                error=float(np.max(np.abs(got[:3]-expected_force)))
                rows.append(dict(speed_m_s=speed,slope_deg=slope,brake_fraction=brake,
                    force_error_n=error,drag_error_vs_planner_n=float(abs(got[5]-planner_drag)),
                    passed=bool(error<1e-10 and abs(got[5]-planner_drag)<1e-10 and abs(got[6]-20*brake)<1e-12)))
    zero=probe((0,0,0),1.)
    invalid=subprocess.run([str(binary),'1.225','.12','.045','.18','1.4','20','30','0','0','0','1.1'],capture_output=True)
    report=dict(profile=profile.record(),trials=rows,zero_flow_verified=bool(np.all(zero==0)),
        invalid_brake_rejected=invalid.returncode!=0,
        sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.glob('*') if p.is_file()},
        scope='isolated force algebra; no moment, stall recovery, PX4 trim or flight qualification')
    report['passed']=all(r['passed'] for r in rows) and report['zero_flow_verified'] and report['invalid_brake_rejected']
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    report=qualify(parser.parse_args().output)
    print(f"{len(report['trials'])} force fixtures: {'PASS' if report['passed'] else 'FAIL'}")
    raise SystemExit(0 if report['passed'] else 1)
