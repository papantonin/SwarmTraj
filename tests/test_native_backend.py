"""Native kernels compared against an explicitly selected Python reference."""
from dataclasses import replace
import importlib.util
import pickle
import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d import flight3d as py
from swarm_sim.trajectory2d import native_backend as native

HAS_NATIVE = importlib.util.find_spec('swarm_sim._trajectory_native') is not None
requires_native = pytest.mark.skipif(not HAS_NATIVE, reason='optional C++ extension not built')


def case(**kwargs):
    c=Config(dimensions=3, numerical_backend='python',generator_mode='physical_mission',target=(500.,0.),
        target_altitude=30.,descent_angle_deg=30.,max_descent_rate=8.,horizon=120.,
        cruise_altitude_range=(70.,100.),max_extra_loiter=0.,candidates=1,attempts=3)
    c=replace(c,**kwargs)
    s=replace(separate_loiter_snapshot(1)[0],altitude=80.).with_speed(13.,c.limits)
    return s,c


def test_backend_is_explicit_and_physical_only(monkeypatch):
    assert Config().numerical_backend=='python'
    for changes in ({'numerical_backend':'bad'},{'numerical_backend':'cpp'},
                    {'dimensions':3,'generator_mode':'geometric','numerical_backend':'cpp'}):
        with pytest.raises(ValueError):Config(**changes)
    native.require_native.cache_clear()
    original=native.import_module
    def unavailable(name):raise ModuleNotFoundError(name)
    monkeypatch.setattr(native,'import_module',unavailable)
    with pytest.raises(RuntimeError,match='build_native'):
        native.require_native()
    monkeypatch.setattr(native,'import_module',original)
    native.require_native.cache_clear()


@requires_native
@pytest.mark.parametrize('substeps',[1,2,4,8])
def test_rhs_rk4_and_held_history_against_python(substeps):
    _,c=case()
    rng=np.random.default_rng(84)
    for _ in range(40):
        state=rng.uniform([-100,-100,30,10,-3,-.3,-.4],[100,100,130,15,3,.3,.4])
        u=rng.uniform([0,0,.7,-.2],[1,1,1.4,.2]);wind=np.array([.1,-.2,0.])
        rhs=np.frombuffer(native.evaluate('rhs',state,u,c,wind=wind),dtype=float)
        np.testing.assert_allclose(rhs,py.derivative(state,u,c.flight,wind),rtol=0,atol=2e-14)
        expected=py.propagate(state,u,.2,c.flight,wind=wind,substeps=substeps)
        actual=native.propagate(state,u,.2,c,substeps=substeps,wind=wind)
        np.testing.assert_allclose(actual,expected,rtol=0,atol=1e-12)
    state=np.array([0.,0.,80.,13.,0.,0.,0.]);u=py.trim_control(state,c.flight)
    result,reason=native.rollout(state,np.tile(u,(500,1)),c,check=False,substeps=substeps,wind=wind)
    assert reason is None
    expected=[state]
    for _ in range(500):expected.append(py.propagate(expected[-1],u,c.dt,c.flight,substeps=substeps,wind=wind))
    np.testing.assert_allclose(result[0],expected,atol=1e-9,rtol=0)
    assert not result[0].flags.writeable
    with pytest.raises(ValueError):result[0].setflags(write=True)


@requires_native
def test_envelope_boundaries_and_both_control_switch_sides():
    _,c=case();state=np.array([0.,0.,80.,13.,0.,0.,0.]);u=py.trim_control(state,c.flight)
    for v in (c.limits.min_speed-2e-7,c.limits.min_speed,c.limits.max_speed,c.limits.max_speed+2e-7):
        for z in (c.min_altitude-2e-7,c.min_altitude,c.max_altitude,c.max_altitude+2e-7):
            s=state.copy();s[3]=v;s[2]=z
            assert native.evaluate('envelope',s,u,c)==py.envelope_ok(s,u,c)
    from swarm_sim.trajectory2d.physical_generation import _step
    for field,value in ((0,6.),(1,6.),(2,100.),(3,10.)):
        bad=u.copy();bad[field]=value
        assert native.checked_step(state,bad,c) is None and _step(state,bad,c) is None
        for commands in (np.vstack([bad,u]),np.vstack([u,bad])):
            path,reason=native.rollout(state,commands,c)
            assert path is None and reason=='envelope'
    # Start and separately integrated nominal/refined endpoints retained.
    np.testing.assert_allclose(native.checked_step(state,u,c),_step(state,u,c),atol=1e-12,rtol=0)


@requires_native
def test_strict_binding_shapes_layout_finiteness_and_bounds():
    _,c=case();m=native.require_native();params=native.parameters(c)
    s=np.array([0.,0.,80.,13.,0.,0.,0.]);u=py.trim_control(s,c.flight)
    for bad in (s.astype('f'),s[::-1],np.zeros(6),np.full(7,np.nan)):
        with pytest.raises(ValueError):m.evaluate('propagate',bad,u,params,{})
    for options in ({'substeps':0},{'substeps':-1},{'substeps':1025},{'duration':np.nan}, {'steps':-1}):
        with pytest.raises(ValueError):m.evaluate('propagate',s,u,params,options)
    for params_bad in (params[:-1],(*params[:-1],-1.)):
        with pytest.raises(ValueError):m.evaluate('propagate',s,u,params_bad,{})
    with pytest.raises(ValueError):native.rollout(s,np.tile(u,(2,1)),c,wind=(1.,0.,0.))
    # Call boundary copies cannot mutate caller memory.
    original=s.copy();native.propagate(s,u,.2,c);np.testing.assert_array_equal(s,original)


@requires_native
@pytest.mark.parametrize('changes',[{}, {'cruise_levels':2,'speed_profile':'smooth','speed_profile_range':(10.,14.),
    'horizon':210.,'target':(500.,500.),'cruise_altitude_range':(30.,120.),'max_descent_rate':10.},
    {'descent_angle_deg':0.},{'horizon':45.},{'arrival_deadline':20.},{'post_arrival_time':110.}])
def test_complete_missions_acceptance_profiles_and_independent_validation(changes):
    from swarm_sim.trajectory2d.planning import validate_individual,mission_document
    snapshot,c=case(**changes)
    reference=plan_mission((snapshot,),c)
    actual=plan_mission((snapshot,),replace(c,numerical_backend='cpp'))
    assert actual.status==reference.status and actual.counts==reference.counts
    assert actual.search_nodes==reference.search_nodes
    for a,b in zip(actual.plans,reference.plans):
        assert a.generation_phases==b.generation_phases and a.arrival_time==b.arrival_time
        np.testing.assert_allclose(a.flight_states,b.flight_states,atol=1e-9,rtol=0)
        np.testing.assert_allclose(a.flight_controls,b.flight_controls,atol=1e-10,rtol=0)
        assert validate_individual(a,snapshot,c)
        controls=a.flight_controls.copy();controls[-1,2]=100.
        assert not validate_individual(replace(a,flight_controls=controls),snapshot,c)
        received=pickle.loads(pickle.dumps(a));assert not received.flight_states.flags.writeable
    document=mission_document(actual,replace(c,numerical_backend='cpp'),(snapshot,))
    assert document['config']['numerical_backend']=='cpp'


@requires_native
@pytest.mark.parametrize('profiles',[False,True])
def test_spawn_native_catalogs_are_exact_and_read_only(profiles):
    from swarm_sim.trajectory2d.parallel import generate_catalogs
    from swarm_sim.trajectory2d.planning import _generate_catalog
    _,c=case(numerical_backend='cpp',attempts=3,horizon=150.)
    snapshots=tuple(replace(s,altitude=80.+10*k).with_speed(13.,c.limits) for k,s in enumerate(separate_loiter_snapshot(2)))
    if profiles:
        from swarm_sim.trajectory2d.aircraft_profiles import reference_aircraft_profile
        first=replace(reference_aircraft_profile(),bounding_radius_m=1.2)
        second=replace(first,name='different native own physics',bounding_radius_m=1.8,
            flight=replace(first.flight,parasitic_drag=.0033,max_thrust_acceleration=4.5))
        c=replace(c,aircraft_profiles=((0,first),(1,second)))
        assert native.parameters(c.for_drone(0))!=native.parameters(c.for_drone(1))
        with pytest.raises(ValueError,match='resolved own'):
            native.parameters(c)
    tasks=[(s,c,None) for s in snapshots]
    serial=generate_catalogs(_generate_catalog,tasks,1)
    parallel=generate_catalogs(_generate_catalog,tasks,2)
    assert all(serial)
    for a,b in zip(serial,parallel):
        assert [p.to_dict() for p in a]==[p.to_dict() for p in b]
        assert all(not p.flight_states.flags.writeable for p in b)
        if profiles:
            for p in b:
                assert p.flight_parameters==c.for_drone(p.drone_id).flight
                assert p.aircraft_profile==c.profile_for(p.drone_id)


@requires_native
@pytest.mark.parametrize('kind',['obstacle','three_levels','profile'])
def test_obstacles_rich_levels_and_resolved_own_airframe(kind):
    from swarm_sim.trajectory2d.obstacles import Obstacle
    from swarm_sim.trajectory2d.aircraft_profiles import reference_aircraft_profile
    from swarm_sim.trajectory2d.planning import validate_individual
    changes=dict(horizon=210.,target=(500.,500.),max_descent_rate=10.,cruise_altitude_range=(30.,120.),attempts=6)
    if kind=='obstacle':changes.update(obstacles=(Obstacle.cylinder(700.,0.,30.,0.,180.),),obstacle_routing='spatial',target=(500.,0.))
    if kind=='three_levels':changes.update(cruise_levels=3,speed_profile='smooth',speed_profile_range=(10.,14.))
    s,c=case(**changes)
    if kind=='profile':
        profile=replace(reference_aircraft_profile(),name='native own fixture',bounding_radius_m=1.2)
        c=replace(c,aircraft_profiles=((s.drone_id,profile),))
    a=plan_mission((s,),c);b=plan_mission((s,),replace(c,numerical_backend='cpp'))
    assert a.validated and b.validated
    assert a.counts==b.counts and a.search_nodes==b.search_nodes
    assert b.plans[0].aircraft_profile==a.plans[0].aircraft_profile
    np.testing.assert_allclose(a.plans[0].flight_states,b.plans[0].flight_states,atol=1e-9,rtol=0)
    assert validate_individual(b.plans[0],s,c)


@requires_native
def test_python_validator_is_not_routed_to_native(monkeypatch):
    from swarm_sim.trajectory2d.planning import validate_individual
    s,c=case(numerical_backend='cpp');r=plan_mission((s,),c);assert r.validated
    monkeypatch.setattr(native,'require_native',lambda:pytest.fail('validator must remain Python'))
    assert validate_individual(r.plans[0],s,c,backend='python')


def test_shared_px4_schema_persists_explicit_backend_choice(tmp_path):
    import json
    from swarm_sim.px4_scenario import default_scenario,scenario_record,load_scenario
    s,c=default_scenario()
    c=replace(c,generator_mode='physical_mission',numerical_backend='cpp')
    record=scenario_record(s,c)
    assert record['mission']['numerical_backend']=='cpp'
    path=tmp_path/'cpp.json';path.write_text(json.dumps(record))
    assert load_scenario(path)==(s,c)


@requires_native
def test_random_envelope_classification_and_heading_wrap():
    _,c=case();rng=np.random.default_rng(931)
    for _ in range(1000):
        s=rng.uniform([-50,-50,-1,7.9,-3.2,-.8,-1.0],[50,50,201,16.1,3.2,.8,1.0])
        u=rng.uniform([-.01,-.01,.19,-1.1],[5.01,5.01,2.51,1.1])
        assert native.evaluate('envelope',s,u,c)==py.envelope_ok(s,u,c)
    for yaw in (-100*np.pi,-np.pi-1e-12,-np.pi,np.pi,np.pi+1e-12,100*np.pi):
        s=np.array([0.,0.,80.,13.,yaw,0.,0.]);u=py.trim_control(s,c.flight)
        for dt in (-.2,0.,.2):
            np.testing.assert_allclose(native.propagate(s,u,dt,c),py.propagate(s,u,dt,c.flight),rtol=0,atol=1e-12)


@requires_native
def test_native_pullout_is_independently_reintegrated():
    from swarm_sim.trajectory2d.kinodynamic_recovery import Pullout,validate_pullout,pullout_from_state
    _,c=case();s=np.array([14.,-42.,73.,13.1,.7,-.45,0.])
    path,reason=native._path(native.evaluate('pullout',s,(),c,steps=100))
    assert reason is None
    states,controls,_,_=path
    primitive=Pullout(states,controls,len(controls)*c.dt,float(s[2]-states[:,2].min()))
    assert validate_pullout(primitive,c)
    reference=pullout_from_state(s,c)
    np.testing.assert_allclose(states,reference.states,atol=1e-10,rtol=0)
    short,reason=native._path(native.evaluate('pullout',s,(),c,steps=len(controls)-1))
    assert short is None and reason
    s[6]=.01
    path,reason=native._path(native.evaluate('pullout',s,(),c,steps=100))
    assert path is None and reason=='pullout_entry'
