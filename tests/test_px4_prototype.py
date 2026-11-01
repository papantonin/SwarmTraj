"""Hypothetical fixtures cannot silently become qualified aircraft adapters."""
from dataclasses import replace
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import pytest
from swarm_sim.px4_prototype import build_model
from swarm_sim.trajectory2d import load_aircraft_profile


def template():
    # Minimal isolated upstream-interface fixture, with external mesh/sensor names.
    root=ET.Element('sdf');model=ET.SubElement(root,'model',name='rc_cessna')
    base=ET.SubElement(model,'link',name='base_link')
    ET.SubElement(base,'sensor',name='imu_sensor',type='imu')
    for index in range(6):
        plugin=ET.SubElement(model,'plugin',name='gz::sim::systems::LiftDrag',filename='gz-sim-lift-drag-system')
        for tag,text in [('area','.6'),('cp','0 0 0'),('air_density','1.2'),('link_name','base_link')]:
            ET.SubElement(plugin,tag).text=text
        if index in (2,3):ET.SubElement(plugin,'control_joint_name').text=('left_flap_joint','right_flap_joint')[index-2]
    return ET.tostring(root,encoding='unicode')


def test_mass_enclosure_references_and_nonqualification_are_explicit():
    profile=load_aircraft_profile('examples/aircraft/compact_5kg.json')
    source=template();lidar=Path('examples/gazebo/models/rc_cessna_lidar/model.sdf').read_text()
    document,record=build_model(profile,source,lidar)
    assert build_model(profile,source,lidar)==(document,record)
    model=ET.fromstring(document).find('model')
    assert sum(float(e.text) for e in model.findall('link/inertial/mass'))==pytest.approx(5.,abs=1e-12)
    assert not record['px4_flight_qualified'] and not record['ideal_airbrake_implemented']
    assert record['profile']==profile.record()
    for link in model.findall('link'):
        inertial=link.find('inertial');diagonal=[float(inertial.findtext('inertia/'+a)) for a in ('ixx','iyy','izz')]
        assert min(diagonal)>0 and max(diagonal)<=sum(diagonal)-max(diagonal)+1e-12
        # All authored ellipsoids are centered in own link; a norm-plus-radius
        # bound covers arbitrary joint rotation as well as the neutral pose.
        pose=link.find('pose');position=[0,0,0] if pose is None else list(map(float,pose.text.split()[:3]))
        for radii in link.findall('*/geometry/ellipsoid/radii'):
            assert math.sqrt(sum(x*x for x in position))+max(map(float,radii.text.split()))<=.3+1e-12
    wing=model.find("link[@name='base_link']/collision[@name='wing_collision']/geometry/ellipsoid/radii")
    a,b,_=map(float,wing.text.split());assert math.pi*a*b==pytest.approx(.12)
    names={j.get('name') for j in model.findall('joint')}
    for name in model.findall('plugin/control_joint_name'):assert name.text in names
    assert model.findall('.//mesh')==[]


def test_incompatible_mass_and_unenclosable_wing_fail():
    profile=load_aircraft_profile('examples/aircraft/compact_5kg.json')
    lidar=Path('examples/gazebo/models/rc_cessna_lidar/model.sdf').read_text()
    with pytest.raises(ValueError,match='restricted'):
        build_model(replace(profile,bounding_radius_m=.5),template(),lidar)
    airframe=replace(profile.airframe,wing_area_m2=.4)
    large=replace(profile,airframe=airframe,flight=replace(profile.flight,**airframe.derived()))
    with pytest.raises(ValueError,match='wing does not fit'):
        build_model(large,template(),lidar)


def test_explicit_polar_replaces_inherited_forces_without_claiming_px4_controls():
    profile=load_aircraft_profile('examples/aircraft/compact_5kg.json')
    sdf,record=build_model(profile,template(),Path('examples/gazebo/models/rc_cessna_lidar/model.sdf').read_text(),matched_polar=True)
    plugins=ET.fromstring(sdf).findall('model/plugin')
    assert all(p.get('name')!='gz::sim::systems::LiftDrag' for p in plugins)
    own=[p for p in plugins if p.get('name')=='swarm::CompactPolar'];assert len(own)==1
    assert float(own[0].findtext('max_brake'))==20
    assert float(own[0].findtext('brake_fraction'))==0
    assert record['ideal_airbrake_implemented'] and not record['px4_flight_qualified']
    assert not record['wind_supported'] and 'bench' in record['airbrake_control']
