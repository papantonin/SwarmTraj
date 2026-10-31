"""Operating speed requests select setpoints without weakening physical limits."""
from dataclasses import replace
import json
from pathlib import Path
import runpy
import sys

import pytest
from swarm_sim.trajectory2d import load_aircraft_profile
from swarm_sim.trajectory2d.aircraft_profiles import aircraft_profile_from_dict

ROOT=Path(__file__).resolve().parents[1]
EXAMPLE=ROOT/'examples/aircraft/custom_fixed_wing.json'


@pytest.mark.parametrize('regime,expected', [('loiter',10.),('climb',11.),('cruise',12.),('descent',12.),('dive',15.)])
def test_operating_speed_roundtrip_and_lookup(regime,expected):
    profile=load_aircraft_profile(EXAMPLE)
    assert profile.airspeed_for(regime)==expected
    assert aircraft_profile_from_dict(profile.to_dict())==profile
    assert profile.limits.max_speed==16
    updated=replace(profile,speeds=replace(profile.speeds,climb_airspeed_m_s=11.5))
    assert updated.sha256!=profile.sha256
    assert updated.physical_config()==profile.physical_config()


@pytest.mark.parametrize('key,value', [('cruise_airspeed_m_s',0),('climb_airspeed_m_s',True),
    ('loiter_airspeed_m_s',7.),('dive_airspeed_m_s',17.),('dive_airspeed_m_s',11.),
    ('descent_airspeed_m_s',float('nan')),('cruise_airspeed_m_s','12')])
def test_invalid_operating_speed_rejects(key,value):
    record=json.loads(EXAMPLE.read_text());record['speeds'][key]=value
    with pytest.raises(ValueError):aircraft_profile_from_dict(record)


def test_incomplete_unknown_and_undeclared_speeds_reject():
    record=json.loads(EXAMPLE.read_text());del record['speeds']['dive_airspeed_m_s']
    with pytest.raises(ValueError,match='missing'):aircraft_profile_from_dict(record)
    record=json.loads(EXAMPLE.read_text());record['speeds']['landing_groundspeed_m_s']=10
    with pytest.raises(ValueError,match='unknown'):aircraft_profile_from_dict(record)
    record=json.loads(EXAMPLE.read_text());del record['speeds']
    legacy=aircraft_profile_from_dict(record)
    assert legacy.to_dict()==record and legacy.speeds is None
    with pytest.raises(ValueError,match='does not declare'):legacy.airspeed_for()
    with pytest.raises(ValueError,match='unknown'):load_aircraft_profile(EXAMPLE).airspeed_for('landing')


@pytest.mark.parametrize('options,expected', [([],13.),(['--flight-regime','dive'],15.),
    (['--flight-regime','climb'],11.),(['--airspeed','14'],14.),(['--drone-speed','11'],11.)])
def test_cli_uses_profile_cruise_or_explicit_requested_speed(tmp_path,monkeypatch,options,expected):
    record=json.loads(EXAMPLE.read_text());record['speeds']['cruise_airspeed_m_s']=13
    source=tmp_path/'aircraft.json';source.write_text(json.dumps(record))
    main=runpy.run_path(str(ROOT/'examples/trajectory_2d.py'))['main']
    class Captured(Exception):pass
    def inspect(snapshots,config):
        assert snapshots[0].speed==expected
        assert config.limits.max_speed==16
        assert config.flight.max_brake_acceleration==0
        raise Captured
    monkeypatch.setitem(main.__globals__,'plan_mission',inspect)
    monkeypatch.setattr(sys,'argv',['trajectory_3d.py','--no-gui','--num-drones','1',
        '--aircraft-profile',str(source),'--output',str(tmp_path/'run'),*options])
    with pytest.raises(Captured):main()


@pytest.mark.parametrize('options', [[],['--airspeed','12'],['--drone-speed','12']])
def test_regime_requires_declared_speeds_and_unambiguous_request(tmp_path,monkeypatch,options):
    main=runpy.run_path(str(ROOT/'examples/trajectory_2d.py'))['main']
    args=['--aircraft-profile',str(EXAMPLE)] if options else []
    monkeypatch.setattr(sys,'argv',['trajectory_3d.py','--no-gui','--num-drones','1',
        '--output',str(tmp_path/'run'),'--flight-regime','dive',*args,*options])
    with pytest.raises(SystemExit) as error:main()
    assert error.value.code==2 and not (tmp_path/'run').exists()
