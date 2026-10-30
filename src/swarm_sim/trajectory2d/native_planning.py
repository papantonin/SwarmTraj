"""Typed input/output adapters for the standalone native physical planner.

No physical integration, random draw or candidate decision is performed here.
Python binds own aircraft identities and owns public immutable plan objects.
"""
from dataclasses import asdict
import numpy as np

from . import native_backend as native


def seed_words(seed):
    seed=int(seed)
    result=[]
    while True:
        result.append(seed & 0xffffffff)
        seed >>= 32
        if not seed:
            return result


def options(config):
    return {**native.validation_options(config), 'seed_words': seed_words(config.seed),
        'candidates': config.candidates, 'attempts': config.attempts,
        'initial_candidates': config.initial_candidates, 'max_nodes': config.max_search_nodes,
        'margin_nodes': config.margin_search_nodes, 'margin_rounds': config.margin_expansion_rounds,
        'max_entries': config.reservation_max_entries, 'max_constraints': config.interval_max_constraints,
        'separation': float(config.separation), 'detour': float(config.detour),
        'level_hold': float(config.cruise_level_hold), 'cell_size': float(config.reservation_cell_size),
        'post_arrival': float(config.post_arrival_time), 'deadline': config.arrival_deadline,
        'spatial': config.obstacle_routing == 'spatial', 'adaptive': config.planning_strategy == 'adaptive',
        'mrv': config.search_order == 'mrv', 'max_margin': config.selection_policy == 'max_margin',
        'witnesses': config.conflict_guidance == 'witness', 'reservations': config.reservation_policy == 'grid',
        'windows': config.interval_guidance == 'safe_windows', 'mixed': config.profile_policy == 'adaptive_mixed',
        'rejection_feedback': config.candidate_guidance == 'rejections',
        'id_approach': config.approach_assignment == 'id'}


def designation(snapshot):
    from .flight3d import from_snapshot
    return dict(drone_id=int(snapshot.drone_id), initial=native._array(from_snapshot(snapshot)),
                yaw_rate=float(snapshot.state[3]), cruise=snapshot.cruise_altitude)


def candidate_record(p):
    return dict(drone_id=p.drone_id, candidate_id=p.candidate_id, speed=p.speed, dt=p.dt,
        release=p.release_time, arrival=p.arrival_time, cruise=p.cruise_altitude, approach=p.approach_angle,
        states=native._array(p.flight_states), controls=native._array(p.flight_controls),
        projected=native._array(np.column_stack((p.states, p.vertical_states))),
        acceleration=native._array(p.accelerations), vertical_acceleration=native._array(p.vertical_accelerations),
        levels=native._array(p.cruise_levels_m),
        speed_reference=None if p.speed_reference is None else native._array(p.speed_reference),
        family=p.profile_family, routing_mode=p.routing_mode, phases=p.generation_phases)


def candidate_from_record(record, config, profile=None):
    from .planning import Trajectory
    def array(key, width=None):
        result=np.frombuffer(record[key], dtype=np.float64)
        return result.reshape(-1,width) if width else result
    projected=array('projected',6)
    return Trajectory(record['drone_id'], record['candidate_id'], record['speed'], record['dt'],
        projected[:,:4], array('acceleration'), record['arrival'], record['release'], record['approach'],
        projected[:,4:], array('vertical_acceleration'), record['cruise'], array('states',7),
        array('controls',4), config.flight, config.limits.max_speed, record['routing_mode'],
        cruise_levels_m=tuple(record['levels']),
        speed_reference=None if record['speed_reference'] is None else array('speed_reference'),
        profile_family=record['family'], aircraft_profile=profile,
        generation_phases=tuple(tuple(p) for p in record['phases']))


def rng_record(state):
    if state is None:
        return None
    if state.get('bit_generator') != 'PCG64':
        raise ValueError('native physical catalogs require PCG64 state')
    value=state['state'];mask=(1<<64)-1
    return (value['state']>>64, value['state']&mask, value['inc']>>64, value['inc']&mask,
            state['has_uint32'], state['uinteger'])


def rng_from_record(state):
    if state is None:
        return None
    a,b,c,d,has,cached=state
    return dict(bit_generator='PCG64',state=dict(state=(a<<64)|b,inc=(c<<64)|d),has_uint32=has,uinteger=cached)


def catalog_record(catalog):
    return dict(plans=[candidate_record(p) for p in catalog.plans], attempted=catalog.attempted,
        profile_stage=catalog.profile_stage, profile_cursor=catalog.profile_cursor,
        profile_failures=catalog.profile_failures, rng=rng_record(catalog.rng_state),
        rejections=catalog.rejections, details=catalog.rejection_details,
        generation_s=catalog.generation_s, validation_s=catalog.validation_s,
        minimum_horizon=catalog.minimum_required_horizon_s, guided=catalog.guided_proposals,
        interval_contexts=catalog.interval_contexts, interval_proposals=catalog.interval_proposals,
        profile_events=catalog.profile_events, profile_proposals=catalog.profile_proposals,
        adjustments=catalog.candidate_proposals)


def restore_catalog_types(record):
    for proposal in record['guided']:
        witness=proposal['witness']
        for key in ('drone_ids','candidate_ids'):
            witness[key]=tuple(witness[key])
        witness['positions']=tuple(tuple(p) for p in witness['positions'])
    for context in record['interval_contexts']:
        context['reference_drone_ids']=tuple(context['reference_drone_ids'])
        context['intervals_s']=tuple(tuple(i) for i in context['intervals_s'])
    for proposal in record['interval_proposals']:
        proposal['departure_interval_s']=tuple(proposal['departure_interval_s'])


def extend_catalog(task):
    snapshot, full_config, angle, catalog, goal, *extra = task
    profile=full_config.profile_for(snapshot.drone_id)
    config=full_config.for_drone(snapshot.drone_id)
    module=native.require_native()
    if module.build_info().get('catalog_api') != 1:
        raise RuntimeError('native catalog kernel unavailable; rebuild with ./scripts/build_native.sh')
    if config.generator_mode not in ('physical_mission','physical_recovery_seed'):
        raise ValueError('native catalogs require physical mission generation')
    hint=extra[0] if extra else None
    windows=extra[1] if len(extra)>1 and config.interval_guidance == 'safe_windows' else None
    expansion=extra[2] if len(extra)>2 else 'initial'
    if len(extra)>3 and extra[3] is not None:
        raise ValueError('physical catalogs do not support geometric passage fitting')
    hint=asdict(hint) if hint is not None else None
    if hint is not None:
        hint['conflict']['positions']=native._array(hint['conflict']['positions'])
    windows=asdict(windows) if windows is not None else None
    if windows is not None:
        windows['intervals_s']=native._array(windows['intervals_s']).reshape(-1,2)
    result=module.extend_catalog(catalog_record(catalog),designation(snapshot),options(config),
        native.parameters(config),angle,min(config.candidates,max(0,goal)),hint,windows,expansion)
    restore_catalog_types(result)
    old_count=len(catalog.plans)
    if len(result['plans'])<old_count:
        raise RuntimeError('native catalog lost previously validated plans')
    catalog.plans.extend(candidate_from_record(r,config,profile) for r in result['plans'][old_count:])
    for key in ('attempted','profile_stage','profile_cursor','profile_failures','generation_s','validation_s'):
        setattr(catalog,key,result[key])
    for name,key in (('rejections','rejections'),('rejection_details','details'),
            ('minimum_required_horizon_s','minimum_horizon'),('guided_proposals','guided'),
            ('interval_contexts','interval_contexts'),('interval_proposals','interval_proposals'),
            ('profile_events','profile_events'),('profile_proposals','profile_proposals'),('candidate_proposals','adjustments')):
        setattr(catalog,name,result[key])
    catalog.rng_state=rng_from_record(result['rng'])
    return catalog
