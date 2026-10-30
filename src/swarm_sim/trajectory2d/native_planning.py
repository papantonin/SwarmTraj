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
        'horizon': float(config.horizon),
        'max_path_degrees': float(config.flight.max_flight_path_deg),
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
        release=p.release_time, arrival=p.arrival_time, cruise=p.cruise_altitude, approach=p.approach_angle,airspeed_limit=p.airspeed_limit,
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
        array('controls',4), config.flight, record['airspeed_limit'], record['routing_mode'],
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


def require_planner():
    module=native.require_native()
    if module.build_info().get('planning_api') != 1:
        raise RuntimeError('complete native planner unavailable; rebuild with ./scripts/build_native.sh')
    return module


def propose_candidate(snapshot, config, candidate_id, rng, approach_angle=None, diagnostics=None,
                      repair_hint=None, repair_mode=None, departure_override=None,
                      level_hold_override=None, routing_extra_margin=0.):
    """Own-input conversion around one native proposal; no Python construction."""
    from .production import require_production
    from .catalog import reject
    require_production(config)
    if config.aircraft_profiles:
        raise ValueError('physical construction requires resolved own aircraft physics')
    if ((level_hold_override is not None and (not np.isfinite(level_hold_override)
            or level_hold_override < config.cruise_level_hold))
            or not np.isfinite(routing_extra_margin) or routing_extra_margin < 0):
        return reject(diagnostics, 'invalid_proposal_adjustment')
    module = require_planner()
    if module.build_info().get('proposal_api') != 1:
        raise RuntimeError('native proposal API unavailable; rebuild with ./scripts/build_native.sh')
    hint = asdict(repair_hint) if repair_hint is not None else None
    if hint is not None:
        hint['conflict']['positions'] = native._array(hint['conflict']['positions'])
    record, state, reason = module.physical_proposal(designation(snapshot), options(config),
        native.parameters(config), int(candidate_id), rng_record(rng.bit_generator.state),
        approach_angle, hint, repair_mode or '', departure_override, level_hold_override,
        float(routing_extra_margin))
    rng.bit_generator.state = rng_from_record(state)
    if reason:
        return reject(diagnostics, reason)
    return candidate_from_record(record, config)


def mission_inputs(snapshots,config):
    from .production import require_production
    require_production(config)
    if not config.coupled_flight or config.generator_mode not in ('physical_mission','physical_recovery_seed'):
        raise ValueError('native mission planning requires coupled 3D physical generation')
    if not snapshots:
        raise ValueError('at least one snapshot is required')
    if len({s.drone_id for s in snapshots}) != len(snapshots):
        raise ValueError('drone IDs must be unique')
    config.require_aircraft_ids(s.drone_id for s in snapshots)
    own=[config.for_drone(s.drone_id) for s in snapshots]
    return [designation(s) for s in snapshots],[options(c) for c in own],[native.parameters(c) for c in own]


def approach_angles(snapshots,config):
    records=require_planner().approach_angles(*mission_inputs(snapshots,config))
    return {int(key):angle for key,angle in records.items()}


def plan_mission(snapshots,config):
    """One binding call for a complete serial mission; no numerical Python callback."""
    from .planning import Result
    record=require_planner().plan_mission(*mission_inputs(snapshots,config))
    plans=tuple(candidate_from_record(p,config.for_drone(p['drone_id']),config.profile_for(p['drone_id'])) for p in record['plans'])
    diagnostic=record['diagnostics']
    for row in diagnostic.get('rounds',[]):
        row['counts']=tuple(row['counts'])
    for step in diagnostic.get('selection',{}).get('steps',[]):
        step['counts']=tuple(step['counts'])
    for catalog in diagnostic.get('catalogs',[]):
        restore_catalog_types(dict(guided=catalog['guided_proposals'],interval_contexts=catalog['interval_contexts'],interval_proposals=catalog['interval_proposals']))
    return Result(record['status'],record['reason'],plans,record['minimum_distance_lower_bound'],
        record['required_distance'],tuple(record['counts']),record['search_nodes'],diagnostic,record['timings'])


def extend_catalog(task):
    snapshot, full_config, angle, catalog, goal, *extra = task
    profile=full_config.profile_for(snapshot.drone_id)
    config=full_config.for_drone(snapshot.drone_id)
    from .production import require_production
    require_production(config)
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


class NativeJointSearch:
    """Python object ownership around native search; all decisions run in C++."""
    def __init__(self,config):
        self.config=config
        self._module=native.require_native()
        if self._module.build_info().get('search_api') != 1:
            raise RuntimeError('native search kernel unavailable; rebuild with ./scripts/build_native.sh')
        base=config.for_drone(config.aircraft_profiles[0][0]) if config.aircraft_profiles else config
        own=[(int(identifier),options(config.for_drone(identifier)),native.parameters(config.for_drone(identifier)))
             for identifier,_ in config.aircraft_profiles]
        self._handle=self._module.search_new(options(base),native.parameters(base),own)
        self._cache={}
        self._metrics=dict(nodes=0,exhausted=False,optimization_nodes=0,optimization_budget_exhausted=False)

    def _plans(self,plans):
        records=[]
        for plan in plans:
            self._cache[(plan.drone_id,plan.candidate_id)]=plan
            records.append(candidate_record(plan))
        return records

    def _catalogs(self,catalogs):
        return [dict(attempted=c.attempted,plans=self._plans(c.plans)) for c in catalogs]

    def _call(self,operation,**data):
        from .reservations import ReservationBudgetError
        try:
            result=self._module.search_call(self._handle,operation,data)
        except self._module.ReservationBudgetError as error:
            raise ReservationBudgetError(str(error)) from error
        self._metrics=result['metrics']
        return result['value']

    def _selection(self,keys):
        return None if keys is None else tuple(self._cache[tuple(key)] for key in keys)

    @property
    def nodes(self):return self._metrics['nodes']

    @property
    def exhausted(self):return self._metrics['exhausted']

    @exhausted.setter
    def exhausted(self,value):self._call('mark_exhausted',value=value)

    @property
    def optimization_nodes(self):return self._metrics['optimization_nodes']

    @property
    def optimization_budget_exhausted(self):return self._metrics['optimization_budget_exhausted']

    def compatible(self,first,second):return self._call('compatible',plans=self._plans((first,second)))
    def margin(self,first,second):return self._call('margin',plans=self._plans((first,second)))
    def score(self,plans):return tuple(self._call('score',plans=self._plans(plans)))
    def pair_scores(self,plans):return self._call('pair_scores',plans=self._plans(plans))
    def optimization_available(self):return self._call('optimization_available')
    def diagnostics(self):return self._call('diagnostics')
    def verify_joint(self,plans):return self._call('verify_joint',plans=self._plans(plans))

    def solve(self,catalogs):
        return self._selection(self._call('solve',domains=[self._plans(d) for d in catalogs]))

    def refine(self,catalogs,incumbent):
        result=self._call('refine',domains=[self._plans(d) for d in catalogs],incumbent=self._plans(incumbent))
        return self._selection(result['plans']),result['quality']

    def expansion_indices(self,catalogs,snapshots):
        return self._call('expansion_indices',catalogs=self._catalogs(catalogs),snapshot_ids=[int(s.drone_id) for s in snapshots])

    def margin_expansion_indices(self,catalogs,snapshots,plans):
        return self._call('margin_expansion_indices',catalogs=self._catalogs(catalogs),
            snapshot_ids=[int(s.drone_id) for s in snapshots],plans=self._plans(plans))

    def repair_hints(self,drone_ids,plans=None):
        from .conflict_guidance import ConflictWitness,RepairHint
        records=self._call('repair_hints',drone_ids=[int(i) for i in drone_ids],
            incumbent=None if plans is None else self._plans(plans))
        result={}
        for key,record in records.items():
            w=record['conflict']
            witness=ConflictWitness(tuple(w['drone_ids']),tuple(w['candidate_ids']),w['sample_time_s'],
                tuple(tuple(p) for p in w['positions']),w['lower_bound_m'],w['required_m'])
            result[int(key)]=RepairHint(witness,record['drone_id'])
        return result

    def interval_hints(self,catalogs,indices,incumbent=None):
        from .interval_guidance import DepartureWindows
        records=self._call('interval_hints',catalogs=self._catalogs(catalogs),indices=list(indices),
            incumbent=None if incumbent is None else self._plans(incumbent))
        return {int(key):DepartureWindows(r['parent_candidate_id'],tuple(r['reference_drone_ids']),
            tuple(tuple(i) for i in r['intervals_s']),r['constraints'],r['budget_exhausted']) for key,r in records.items()}

    def passage_hints(self,catalogs,indices,incumbent=None):
        # Physical generation rejects geometric passage fitting at configuration.
        return {}
