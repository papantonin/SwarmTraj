"""Incremental compact-aircraft qualification; failed requests remain failures.

Control-step comparisons change guidance sampling. Held-command RK4 refinement
is reported separately and does not establish a certified integration error.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import platform
import resource
import time

import numpy as np
from swarm_sim.trajectory2d import Config, load_aircraft_profile, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.flight3d import propagate
from swarm_sim.trajectory2d.planning import validate_individual
from swarm_sim.trajectory2d.native_backend import identity


def trial(count, seed, dt):
    profile=load_aircraft_profile('examples/aircraft/compact_5kg.json')
    config=profile.apply(Config(target=(3000.,0.),target_altitude=200.,descent_angle_deg=30.,
        max_altitude=700.,cruise_altitude_range=(500.,550.),horizon=420.,dt=dt,
        candidates=4,attempts=24,seed=seed,approach_spread_deg=180. if count>1 else 0.,
        candidate_guidance='rejections',planning_strategy='adaptive',search_order='mrv',
        selection_policy='efficient',conflict_guidance='witness',obstacle_routing='spatial',
        approach_routing='tangent',speed_schedule='operating',speed_profile_acceleration=1.5))
    snapshots=tuple(replace(s,altitude=500.+25*k).with_speed(profile.airspeed_for('loiter'),profile.limits)
                    for k,s in enumerate(separate_loiter_snapshot(count,radius=400.,spacing=1000.)))
    start=time.perf_counter();cpu=time.process_time();result=plan_mission(snapshots,config)
    row=dict(count=count,seed=seed,control_dt_s=dt,status=result.status,reason=result.reason,
             planning_wall_s=time.perf_counter()-start,planning_cpu_s=time.process_time()-cpu,
             counts=result.counts,diagnostics=result.diagnostics,passed=False)
    if not result.validated:return row
    row['independent_replay']=all(validate_individual(p,s,config,backend='python') for p,s in zip(result.plans,snapshots))
    row['plan_signature']=hashlib.sha256(json.dumps([p.to_dict() for p in result.plans],sort_keys=True).encode()).hexdigest()
    row['held_command_max_state_difference_2_vs_8_substeps']=np.max(np.stack([
        np.max(np.abs(propagate(p.flight_states[:-1],p.flight_controls,dt,config.flight,substeps=8)-p.flight_states[1:]),axis=0)
        for p in result.plans]),axis=0).tolist()
    row['state_difference_columns']=['x_m','y_m','z_m','airspeed_m_s','heading_rad','flight_path_rad','bank_rad']
    row['execution']=simulate(result.plans,config).report
    row['passed']=bool(row['independent_replay'] and row['execution']['execution_validated'])
    return row


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seeds',type=int,nargs='+',default=[7,19])
    parser.add_argument('--steps',type=float,nargs='+',default=[.2,.1,.05])
    parser.add_argument('--counts',type=int,nargs='+',default=[1,4])
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    sources=sorted([*Path('src/swarm_sim/trajectory2d').glob('*.py'),*Path('native/src').glob('*'),
                    *Path('native/include/swarmtraj').glob('*.hpp'),Path(__file__),Path('examples/aircraft/compact_5kg.json')])
    def fingerprints():
        return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources if p.is_file()}
    original=fingerprints()
    metadata=dict(platform=platform.platform(),cpu=Path('/proc/cpuinfo').read_text().split('model name')[1].split('\n')[0],
                  native=identity(),sources=original,numpy=np.__version__,trials=[],all_passed=False,completed=False,
                  requested_trials=len(args.counts)*len(args.seeds)*len(args.steps))
    for count in args.counts:
        for seed in args.seeds:
            for dt in args.steps:
                # No filtering or retries: every requested case is persisted.
                if fingerprints()!=original:
                    raise RuntimeError('qualification sources changed during the campaign')
                try:row=trial(count,seed,dt)
                except Exception as error:
                    row=dict(count=count,seed=seed,control_dt_s=dt,passed=False,error=repr(error))
                metadata['trials'].append(row)
                metadata['process_peak_rss_kib']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
                (args.output/'report.json').write_text(json.dumps(metadata,indent=2,allow_nan=False)+'\n')
                print(f"n={count} seed={seed} dt={dt}: {'PASS' if row['passed'] else 'FAIL'}",flush=True)
    metadata['completed']=len(metadata['trials'])==metadata['requested_trials']
    metadata['all_passed']=metadata['completed'] and all(r['passed'] for r in metadata['trials'])
    (args.output/'report.json').write_text(json.dumps(metadata,indent=2,allow_nan=False)+'\n')
    return 0 if metadata['all_passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
