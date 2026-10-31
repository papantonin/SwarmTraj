"""SI airframe files derive the same independently validated native physics."""
from dataclasses import replace
import json
from pathlib import Path
import pickle

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, load_aircraft_profile, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.aircraft_profiles import aircraft_profile_from_dict
from swarm_sim.trajectory2d.dynamics import G
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.planning import mission_document, validate_individual

EXAMPLE = Path(__file__).resolve().parents[1]/'examples/aircraft/custom_fixed_wing.json'


def test_force_and_polar_conversion_matches_independent_aerodynamic_equations():
    profile = load_aircraft_profile(EXAMPLE)
    a = profile.airframe
    for speed in (10.,12.,15.):
        for load in (.5,1.,2.):
            dynamic_pressure = .5*a.air_density_kg_m3*speed**2
            lift_coefficient = load*a.mass_kg*G/(dynamic_pressure*a.wing_area_m2)
            drag = dynamic_pressure*a.wing_area_m2*(a.zero_lift_drag_coefficient+a.induced_drag_factor*lift_coefficient**2)
            assert profile.flight.drag(speed,load) == pytest.approx(drag/a.mass_kg,rel=1e-14)
    assert .5*a.air_density_kg_m3*profile.flight.stall_speed**2*a.wing_area_m2*a.max_lift_coefficient == pytest.approx(a.mass_kg*G)
    assert profile.flight.max_thrust_acceleration == a.max_thrust_n/a.mass_kg
    assert profile.flight.max_brake_acceleration == 0
    assert pickle.loads(pickle.dumps(profile)) == profile
    assert aircraft_profile_from_dict(profile.to_dict()) == profile
    with pytest.raises(ValueError,match='derivation'):
        replace(profile,flight=replace(profile.flight,parasitic_drag=.1))


@pytest.mark.parametrize('field,value', [('mass_kg',0),('wing_area_m2',-1),('max_thrust_n',0),
    ('max_lift_coefficient',0),('air_density_kg_m3',True),('induced_drag_factor',-.1),
    ('max_airbrake_drag_n',-1),('mass_kg',float('inf')),('mass_kg',1e308),
    ('wing_area_m2',1e-320),('air_density_kg_m3','1.225')])
def test_invalid_physical_inputs_reject(field,value):
    record=json.loads(EXAMPLE.read_text());record['airframe'][field]=value
    with pytest.raises(ValueError):aircraft_profile_from_dict(record)


@pytest.mark.parametrize('section,key', [('airframe','mass_kg'),('limits','stall_margin'),('geometry','bounding_radius_m')])
def test_incomplete_physical_profile_rejects(section,key):
    record=json.loads(EXAMPLE.read_text());del record[section][key]
    with pytest.raises(ValueError):aircraft_profile_from_dict(record)


@pytest.mark.parametrize('section,key', [('airframe','battery_capacity_wh'),('limits','stall_speed_1g_m_s'),(None,'drag'),(None,'actuators')])
def test_unused_or_duplicate_sources_of_physics_reject(section,key):
    record=json.loads(EXAMPLE.read_text());(record if section is None else record[section])[key]=1
    with pytest.raises(ValueError,match='unknown'):aircraft_profile_from_dict(record)


def test_native_mission_matches_normalized_equivalent_and_retains_own_identity():
    physical=load_aircraft_profile(EXAMPLE)
    normalized=replace(physical,airframe=None,speeds=None)
    base=Config(target=(500.,0.),target_altitude=80.,descent_angle_deg=0.,
        horizon=120.,cruise_altitude_range=(80.,80.),max_extra_loiter=0.,candidates=1,attempts=8)
    snapshot=replace(separate_loiter_snapshot(1)[0],altitude=80.)
    config=replace(base,aircraft_profiles=((0,physical),))
    result=plan_mission((snapshot,),config)
    other=plan_mission((snapshot,),replace(base,aircraft_profiles=((0,normalized),)))
    assert result.validated and other.validated
    np.testing.assert_array_equal(result.plans[0].flight_states,other.plans[0].flight_states)
    np.testing.assert_array_equal(result.plans[0].flight_controls,other.plans[0].flight_controls)
    assert validate_individual(result.plans[0],snapshot,config)
    assert simulate(result.plans,config).report['execution_validated']
    document=mission_document(result,config,(snapshot,))
    assert document['plans'][0]['aircraft_profile']==physical.record()
    larger=replace(physical,airframe=replace(physical.airframe,mass_kg=3.),
        flight=replace(physical.flight,**replace(physical.airframe,mass_kg=3.).derived()))
    assert larger.sha256 != physical.sha256
