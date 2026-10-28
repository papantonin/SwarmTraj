"""Replay one saved mission and trace failed envelope predicates unchanged.

Observer instrumentation only: the wrapper returns the original predicate's
exact result. No limit, tolerance, controller or trajectory is changed.
"""

import argparse
import gzip
import hashlib
import inspect
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from benchmark_scaling_campaign import decode, sources
from swarm_sim.trajectory2d import Trajectory
from swarm_sim.trajectory2d.execution import simulate
import swarm_sim.trajectory2d.execution as execution
from swarm_sim.trajectory2d.execution_safety import envelope_margins
from swarm_sim.trajectory2d.flight3d import FlightLimits
from swarm_sim.trajectory2d.perturbations import position_bias_vectors
import swarm_sim.trajectory2d.flight3d as flight


def load_plan(record):
    mapping = {'approach_angle': 'approach_angle_rad', 'cruise_altitude': 'cruise_altitude_m',
               'airspeed_limit': 'airspeed_limit_m_s', 'cruise_levels_m': 'cruise_levels_m',
               'speed_reference': 'speed_reference_m_s'}
    values = {k: record[mapping.get(k, k)] for k in Trajectory.__dataclass_fields__
              if mapping.get(k, k) in record}
    values['flight_parameters'] = FlightLimits(**values['flight_parameters'])
    return Trajectory(**values)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('job_id')
    parser.add_argument('--profile', choices=('nominal', 'gust_sensor', 'outside_clock'), default='gust_sensor')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('output must be a new file')
    manifest = json.loads((args.directory / 'manifest.json').read_text())
    if sources() != manifest['source_sha256']:
        raise RuntimeError('planning/execution source differs from original campaign')
    job = next(j for j in manifest['jobs'] if j['id'] == args.job_id)
    snapshots, config = decode(job)
    with gzip.open(args.directory / 'missions' / (args.job_id+'.json.gz'), 'rt') as stream:
        doc = json.load(stream)
    canonical = {k: v for k, v in doc.items() if k != 'sha256'}
    if hashlib.sha256(json.dumps(canonical, sort_keys=True, allow_nan=False).encode()).hexdigest() != doc['sha256']:
        raise RuntimeError('original mission hash mismatch')
    plans = tuple(load_plan(p) for p in doc['plans'])
    if [p.to_dict() for p in plans] != doc['plans']:
        raise RuntimeError('loaded plans differ from original document')
    settings = {}
    if args.profile == 'gust_sensor':
        settings = {'wind': .1, 'wind_model': 'gust', 'wind_heading_deg': 15.,
            'clock_offsets': np.linspace(-.03, .03, len(plans)),
            'clock_drifts': np.linspace(-.0001, .0001, len(plans)),
            'position_biases': position_bias_vectors([p.drone_id for p in plans], .05, 3),
            'position_noise': .01, 'disturbance_seed': 23}
    elif args.profile == 'outside_clock':
        settings = {'clock_offsets': np.full(len(plans), .4)}
    original = flight.envelope_ok
    original_interval = execution._coupled_interval
    failures = []
    boundary_records = {p.drone_id: [] for p in plans}
    def trace_interval(*positional, **keywords):
        output = original_interval(*positional, **keywords)
        boundary_records[positional[0].plan.drone_id].extend(output[3])
        return output
    def trace(states, controls, cfg, tolerance=1e-7):
        accepted = original(states, controls, cfg, tolerance=tolerance)
        if not accepted and tolerance == 1e-5:
            caller = inspect.currentframe().f_back
            context = caller
            while context is not None and context.f_code.co_name != 'simulate':
                context = context.f_back
            observed_times = None
            if context is not None:
                times = context.f_locals.get('times')
                switches = context.f_locals.get('switches')
                if times is not None and len(states) == len(times):
                    observed_times = times
                elif times is not None and switches is not None and len(states) == len(switches):
                    observed_times = times[switches]
            drone_index = caller.f_locals.get('k')
            margins = envelope_margins(states, controls, cfg)
            bad = {}
            for name, values in margins.items():
                indices = np.flatnonzero(np.atleast_1d(values) < -tolerance)
                if len(indices):
                    first = int(indices[0])
                    bad[name] = {'minimum_margin': float(np.min(values)), 'first_array_index': first,
                        'state_at_first': np.asarray(states)[first].tolist(),
                        'control_at_first': np.asarray(controls)[first].tolist()}
                    if observed_times is not None and drone_index is not None:
                        at = float(observed_times[first])
                        bad[name]['observed_time_s'] = at
                        bad[name]['actual_boundary_checks'] = [
                            {'time_s': float(t), 'control': u.tolist(),
                             'matches_tested_control': bool(np.allclose(u, np.asarray(controls)[first], atol=1e-10, rtol=0)),
                             'envelope_accepted': original(s, u, cfg, tolerance=tolerance),
                             'constraint_margin': float(envelope_margins(s, u, cfg)[name])}
                            for t, s, u in boundary_records[plans[drone_index].drone_id] if abs(t-at) < 1e-8]
            failures.append({'caller': caller.f_code.co_name, 'line': caller.f_lineno,
                'caller_drone_index': drone_index, 'shape': list(np.shape(states)),
                'tolerance': tolerance, 'constraints': bad})
        return accepted
    flight.envelope_ok = trace
    execution._coupled_interval = trace_interval
    start = perf_counter()
    try:
        report = simulate(plans, config, **settings).report
    finally:
        flight.envelope_ok = original
        execution._coupled_interval = original_interval
    data = {'job_id': args.job_id, 'profile': args.profile, 'original_mission_sha256': doc['sha256'],
        'execution_wall_s': perf_counter()-start, 'failed_predicates': failures, 'report': report,
        'scope': 'observer-only predicate trace; exact original result returned; no criteria relaxed'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
    print(json.dumps({'failed_predicates': failures, 'execution_failures': report['execution_failures']}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
