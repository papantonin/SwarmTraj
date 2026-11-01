from copy import deepcopy
import hashlib
import json
import numpy as np
import pytest
from swarm_sim.px4_compact_reference import generate_compact_reference,checked_reference
from swarm_sim.trajectory2d import load_aircraft_profile


def test_world_coordinates_keep_absolute_ground_and_commands_keep_zero_altitude():
    from swarm_sim.px4_compact_reference import native_altitude_fields
    assert native_altitude_fields(True,0.,.205,True)==(5,0.)
    assert native_altitude_fields(False,0.,.205,True)==(2,0.)
    assert native_altitude_fields(True,80.,.205,True)==(5,80.)
    assert native_altitude_fields(True,80.205,.205,False)==(6,80.)


@pytest.fixture(scope='module')
def reference():
    profile=load_aircraft_profile('examples/aircraft/compact_5kg.json')
    document=json.loads(json.dumps(generate_compact_reference(profile)))
    return profile,document


def test_real_cpp_reference_retains_full_horizon_and_target(reference):
    profile,document=reference;before=deepcopy(document)
    xyz=checked_reference(document,profile)
    assert document==before and document['status']=='validated'
    assert len(xyz)==501 and xyz[-1,0]>1800.
    assert document['plans'][0]['arrival_time']<document['config']['horizon']
    assert np.min(np.linalg.norm(xyz-[1800,0,80],axis=1))<6.


def test_compact_scene_shows_target_and_full_reference_without_collision_or_lidar_returns(reference):
    from swarm_sim.px4_scene import scene_tree
    profile,document=reference;before=deepcopy(document)
    tree=scene_tree(document,'examples/gazebo/worlds/default.sdf',reference_radius_m=.12)
    world=tree.find('world')
    target=world.find("model[@name='swarm_target']")
    assert list(map(float,target.findtext('pose').split()[:3]))==[1800.,0.,80.]
    assert float(target.findtext('link/visual/geometry/sphere/radius'))==6.
    assert world.find("model[@name='swarm_target_beacon']") is not None
    for name in ['swarm_reference_arrival','swarm_reference_onward','swarm_target','swarm_target_ground','swarm_target_beacon']:
        model=world.find(f"model[@name='{name}']");assert model is not None
        assert model.findtext('static')=='true' and not model.findall('.//collision')
        assert all(v.findtext('visibility_flags')=='1' for v in model.findall('.//visual'))
    for name in ['arrival','onward']:
        assert float(world.findtext(f"model[@name='swarm_reference_{name}']/link/visual/geometry/cylinder/radius"))==.12
    assert document==before


def test_reference_fingerprint_is_checked_before_conversion(reference):
    profile,document=reference;changed=deepcopy(document)
    changed['plans'][0]['flight_states'][10][2]=75.
    with pytest.raises(ValueError,match='fingerprint'):checked_reference(changed,profile)


@pytest.mark.parametrize('mutation',['dive','fleet','backend','target','obstacle'])
def test_adapter_rejects_unsupported_reference_families_even_with_new_hash(reference,mutation):
    profile,document=reference;changed=deepcopy(document)
    if mutation=='dive':changed['plans'][0]['flight_states'][10][2]=75.
    if mutation=='fleet':changed['plans'].append(deepcopy(changed['plans'][0]))
    if mutation=='backend':changed['config']['numerical_backend']='python'
    if mutation=='target':changed['config']['target'][1]=500.
    if mutation=='obstacle':changed['config']['obstacles']=[{}]
    changed.pop('sha256');changed['sha256']=hashlib.sha256(json.dumps(changed,sort_keys=True,allow_nan=False).encode()).hexdigest()
    with pytest.raises(ValueError):checked_reference(changed,profile)
