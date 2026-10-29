"""Warm-start ordering/freshness contracts; no SDK, Docker or actuator calls."""
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from swarm_sim import px4_gazebo as single
from swarm_sim import px4_fleet as fleet


def samples(height=80.,east=15.,north=0.,receipt=10.2,sim_time=1.2):
    m=single.Measurements()
    m.latest={'position_velocity_ned':NS(position=NS(down_m=-height),
              velocity=NS(east_m_s=east,north_m_s=north)),
              'ground_truth':NS(timestamp_us=int(sim_time*1e6))}
    m.received_at={'position_velocity_ned':receipt,'ground_truth':receipt}
    return m


def check(m,now=10.3):
    return single.airborne_estimate_check(m,np.array([300.,0.,80.,15.,0.,0.,0.]),1.,0.,10.,now)


def test_estimator_gate_requires_fresh_post_release_coherent_samples():
    m=samples();assert check(m)['passed']
    assert not check(single.Measurements())['passed']
    for key in ('position_velocity_ned','ground_truth'):
        m=samples();m.received_at[key]=9.9
        assert check(m)['reason']=='pre_release_sample'
        m=samples();m.received_at[key]=10.1
        assert check(m,now=11.2)['reason']=='stale_sample'
    m=samples(sim_time=.9)
    assert check(m)['reason']=='pre_release_sample'
    m=samples();m.latest['position_velocity_ned'].velocity.east_m_s=6.
    assert check(m)['reason']=='nominal_state_mismatch'  # No acceptance of arbitrary slowed flight.
    m=samples(east=float('nan'));result=check(m)
    assert result['reason']=='nonfinite_sample' and not result['passed']
    json.dumps(result,allow_nan=False)


@pytest.mark.parametrize('field,value,expected',[
    ('height',82.999999,True),('height',83.,False),('height',77.,False),
    ('east',12.000001,True),('east',12.,False),('east',18.,False),
    ('north',2.999999,True),('north',3.,False),('north',-3.,False)])
def test_original_strict_estimated_state_bounds_are_retained(field,value,expected):
    assert check(samples(**{field:value}))['passed']==expected


def test_nominal_loiter_velocity_uses_same_mission_clock():
    m=samples(east=15*np.cos(.15*.2),north=-15*np.sin(.15*.2))
    r=single.airborne_estimate_check(m,np.array([300.,0.,80.,15.,0.,0.,0.]),1.,-.15,10.,10.3)
    assert r['passed'] and r['elapsed_since_initial_state_s']==pytest.approx(.2)
    assert r['estimate_source_timestamp_available'] is False


def test_callback_records_receipt_time_without_mutating_observer_state(monkeypatch):
    m=single.Measurements();clock=iter((10.1,10.2))
    monkeypatch.setattr(single.time,'monotonic',lambda:next(clock))
    estimate=samples().latest['position_velocity_ned']
    truth=NS(timestamp_us=1200000,latitude_deg=47.,longitude_deg=8.,absolute_altitude_m=80.)
    m.callback('position_velocity_ned')(estimate,None)
    m.callback('ground_truth')(truth,None)
    assert m.received_at=={'position_velocity_ned':10.1,'ground_truth':10.2}
    assert len(m.rows)==1 and m.rows[0]['sim_time_s']==1.2
    assert check(m)['passed']


def test_release_receipt_cutoff_excludes_callback_during_publication(monkeypatch,tmp_path):
    from pathlib import Path
    m=samples();clock=iter((10.1,10.2));rename=Path.rename
    monkeypatch.setattr(single.time,'monotonic',lambda:next(clock))
    def racing_rename(path,target):
        assert not target.exists()
        m.callback('position_velocity_ned')(m.latest['position_velocity_ned'],None)
        return rename(path,target)
    monkeypatch.setattr(Path,'rename',racing_rename)
    cutoff=single.publish_airborne_release(tmp_path,'frozen mission')
    assert cutoff==10.2 and m.received_at['position_velocity_ned']==10.1
    record=single.airborne_estimate_check(m,np.array([300.,0.,80.,15.,0.,0.,0.]),1.,0.,cutoff,10.3)
    assert record['reason']=='pre_release_sample' and not record['passed']
    assert json.loads((tmp_path/'airborne-release.json').read_text())=={'mission_sha256':'frozen mission','release':True}
    assert not (tmp_path/'airborne-release.pending.json').exists()


def setup_fleet(tmp_path):
    for index in range(4):
        folder=tmp_path/f'drone-{index}';folder.mkdir()
        for phase in ('ready-to-arm','armed-ready'):
            (folder/(phase+'.json')).write_text('{"ready":true}')
    return {'airborne_initialization':{'initial_state_passed':True},
        'airborne_estimator_acquisition':{'passed':True},'airborne_controller_reset_count':1,'mission_started':True}


def test_ordered_bootstrap_waits_for_predecessor_and_both_original_barriers(tmp_path):
    evidence=setup_fleet(tmp_path)
    first=tmp_path/'drone-0'
    (tmp_path/'drone-3/armed-ready.json').unlink()
    with pytest.raises(TimeoutError):fleet.wait_bootstrap_turn(first,0,'parent',timeout=.02)
    (tmp_path/'drone-3/armed-ready.json').write_text('{"ready":true}')
    for index in range(4):
        folder=tmp_path/f'drone-{index}'
        if index:
            with pytest.raises(TimeoutError):fleet.wait_bootstrap_turn(folder,index,'parent',timeout=.02)
            fleet.complete_bootstrap_turn(tmp_path/f'drone-{index-1}',index-1,'parent',evidence)
        result=fleet.wait_bootstrap_turn(folder,index,'parent',timeout=.02)
        assert result['index']==index and result['timed_departure_supported'] is False
        assert not (folder/'airborne-release.json').exists()  # Coordination itself issues no release/control.


@pytest.mark.parametrize('field',['actual','estimated','reset','mission'])
def test_no_next_bootstrap_without_own_gates_single_reset_and_mission_ack(tmp_path,field):
    evidence=setup_fleet(tmp_path)
    if field=='actual':evidence['airborne_initialization']['initial_state_passed']=False
    if field=='estimated':evidence['airborne_estimator_acquisition']['passed']=False
    if field=='reset':evidence['airborne_controller_reset_count']=0
    if field=='mission':evidence['mission_started']=False
    with pytest.raises(ValueError):fleet.complete_bootstrap_turn(tmp_path/'drone-0',0,'parent',evidence)
    assert not (tmp_path/'drone-0/warm-start-complete.json').exists()
    with pytest.raises(TimeoutError):fleet.wait_bootstrap_turn(tmp_path/'drone-1',1,'parent',timeout=.02)


def test_ordered_bootstrap_rejects_wrong_mission_or_abort(tmp_path):
    evidence=setup_fleet(tmp_path)
    fleet.complete_bootstrap_turn(tmp_path/'drone-0',0,'different parent',evidence)
    with pytest.raises(ValueError,match='predecessor'):
        fleet.wait_bootstrap_turn(tmp_path/'drone-1',1,'parent',timeout=.02)
    (tmp_path/'abort.json').write_text('{}')
    with pytest.raises(RuntimeError,match='failed'):
        fleet.wait_bootstrap_turn(tmp_path/'drone-0',0,'parent',timeout=.02)
