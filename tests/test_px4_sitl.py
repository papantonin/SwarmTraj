"""Offline/SITL boundary contracts; tests never start a simulator or arm a vehicle."""

from dataclasses import replace
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from swarm_sim.px4_reference import (LocalOrigin, ecef_to_geodetic, geodetic_to_ecef,
    generate_reference, mission_items, polyline_distances, waypoint_indices)
from swarm_sim import px4_gazebo as adapter


@pytest.mark.parametrize('point', ((0., 0., 0.), (300., 0., 80.), (1200., 500., 80.), (-1500., -500., 100.)))
def test_wgs84_enu_round_trip_and_axis_conventions(point):
    origin = LocalOrigin(47.397971, 8.546164, 488.)
    llh = origin.from_enu(point)
    np.testing.assert_allclose(origin.to_enu(*llh), point, atol=2e-8)
    if point[0] > 0:
        assert llh[1] > origin.longitude_deg
    if point[1] > 0:
        assert llh[0] > origin.latitude_deg
    assert llh[2] == pytest.approx(origin.altitude_m+point[2], abs=.3)


@pytest.mark.parametrize('coordinates', ((0., 0., 0.), (-35., 149., 100.), (60., -120., 2000.)))
def test_geodetic_ecef_round_trip(coordinates):
    np.testing.assert_allclose(ecef_to_geodetic(geodetic_to_ecef(*coordinates)), coordinates, atol=2e-8)


@pytest.mark.parametrize('coordinates', ((float('nan'), 0, 0), (90, 0, 0), (0, 181, 0)))
def test_invalid_geodetic_origins_rejected(coordinates):
    with pytest.raises(ValueError):
        LocalOrigin(*coordinates)


def test_reference_geographic_extent_is_limited():
    with pytest.raises(ValueError):
        LocalOrigin(47, 8, 0).from_enu((6000, 0, 0))


def test_mission_contains_takeoff_air_speed_reference_and_reachable_landing():
    origin = LocalOrigin(47., 8., 100.)
    points = np.column_stack((np.arange(0, 121, 3), np.zeros(41), np.full(41, 80.)))
    items, indices = mission_items(points, origin)
    assert [r['seq'] for r in items] == list(range(len(items)))
    assert [r['command'] for r in items] == [22, 178, *[16]*len(indices), 189, 31, 21]
    assert indices[0] == 0 and indices[-1] == len(points)-1
    assert items[1]['param1'] == 0 and items[1]['param2'] == 15  # Airspeed, not groundspeed.
    assert items[0]['current'] == 1 and all(r['current'] == 0 for r in items[1:])
    assert all(r['frame'] == 6 for r in items if r['command'] in (22, 16, 31, 21))
    assert items[-2]['param2'] == 100 and items[-2]['param4'] == 1
    assert items[-1]['z'] == pytest.approx(0., abs=.001)
    assert all(r['z'] == pytest.approx(80., abs=.01) for r in items if r['command'] in (22, 16))
    assert math.degrees(math.atan2(60, math.sqrt(600**2-100**2))) < 8
    json.dumps(items, allow_nan=False)  # Missing yaw is stored as null, never invalid JSON NaN.


@pytest.mark.parametrize('speed', (0, 9, 25, float('nan')))
def test_demo_does_not_extrapolate_fixed_wing_speed_limits(speed):
    with pytest.raises(ValueError):
        mission_items(np.array([[0, 0, 80], [100, 0, 80]]), LocalOrigin(47, 8, 0), airspeed=speed)


@pytest.mark.parametrize('points', ([], [[0, 0, 80]], [[0, 0, 80], [1, 1, 70]], [[0, 0, 80], [float('nan'), 1, 80]]))
def test_invalid_or_nonlevel_references_rejected(points):
    with pytest.raises(ValueError):
        waypoint_indices(points)


def test_spatial_error_is_distance_to_segments_not_only_waypoint_endpoints():
    actual = np.array([[5, 2, 80], [10, 0, 83], [-2, 0, 80]])
    reference = np.array([[0, 0, 80], [10, 0, 80]])
    np.testing.assert_allclose(polyline_distances(actual, reference), [2, 3, 2])
    duplicate = np.vstack((reference[0], reference))
    np.testing.assert_allclose(polyline_distances(actual, duplicate), [2, 3, 2])


def test_small_coupled_vertical_transients_are_retained_not_flattened():
    points = np.array([[0., 0., 80.], [100., 0., 79.98], [200., 0., 80.01]])
    original = points.copy()
    origin = LocalOrigin(47., 8., 0.)
    items, indices = mission_items(points, origin)
    np.testing.assert_array_equal(points, original)
    for item, index in zip(items[2:-3], indices):
        actual = origin.to_enu(item['x']/1e7, item['y']/1e7, item['z']+origin.altitude_m)
        assert actual[2] == pytest.approx(points[index, 2], abs=.001)


def test_runtime_is_pinned_isolated_and_mounts_only_project_and_run(tmp_path):
    arguments = adapter.container_arguments('swarm-test-name', tmp_path)
    assert arguments[arguments.index('--network')+1] == 'none'
    assert '@sha256:' in arguments[-2]
    assert not any(x in arguments for x in ('--privileged', '--device', '--publish', '--network=host'))
    assert not any('/dev/tty' in x or '14540:' in x for x in arguments)
    mounts = [arguments[i+1] for i, value in enumerate(arguments[:-1]) if value == '--mount']
    assert len(mounts) == 2
    assert 'dst=/workspace,readonly' in mounts[0]
    assert f'src={tmp_path},dst=/runresults' in mounts[1]
    assert 'PX4_SYS_AUTOSTART=4003' in arguments
    assert 'PX4_GZ_MODELS=/workspace/examples/gazebo/models' in arguments
    assert '--headless-rendering' in (adapter.ROOT/'examples/gazebo/start_sitl.sh').read_text()
    assert 'LIBGL_ALWAYS_SOFTWARE=1' in arguments
    assert 'PX4_GZ_NO_FOLLOW=1' in arguments  # Adapter uses acknowledged GUI setup instead.


def test_gpu_opt_in_exposes_only_render_node_and_preserves_network_isolation(monkeypatch, tmp_path):
    monkeypatch.setattr(Path, 'is_char_device', lambda p: p == adapter.GPU_RENDER_NODE)
    monkeypatch.setattr(adapter.os, 'access', lambda p, mode: p == adapter.GPU_RENDER_NODE)
    arguments = adapter.container_arguments('swarm-test', tmp_path, gpu=True)
    devices = [arguments[i+1] for i, v in enumerate(arguments[:-1]) if v == '--device']
    assert devices == ['/dev/dri/renderD128:/dev/dri/renderD128:rw']
    assert 'LIBGL_ALWAYS_SOFTWARE=0' in arguments
    assert 'LIBGL_ALWAYS_SOFTWARE=1' not in arguments
    assert arguments[arguments.index('--network')+1] == 'none'
    assert not any(v in arguments for v in ('--privileged', '--publish', '--network=host'))


@pytest.mark.parametrize('character_device,accessible', ((False, True), (True, False)))
def test_gpu_mode_rejects_missing_or_inaccessible_node(monkeypatch, tmp_path, character_device, accessible):
    monkeypatch.setattr(Path, 'is_char_device', lambda p: character_device)
    monkeypatch.setattr(adapter.os, 'access', lambda p, mode: accessible)
    with pytest.raises(ValueError, match='render node'):
        adapter.container_arguments('swarm-test', tmp_path, gpu=True)


def test_sensor_variant_inherits_official_airframe_without_replacing_aerodynamics():
    model = ET.parse(adapter.ROOT/'examples/gazebo/models/rc_cessna_lidar/model.sdf')
    assert model.findtext('.//include/uri') == 'model://rc_cessna'
    assert model.find('.//include').attrib['merge'] == 'true'
    assert not model.findall('.//plugin')  # Original aircraft plugins are inherited.
    assert model.findtext('.//sensor/ray/range/max') == '100'
    assert model.findtext('.//sensor/pose') == '0 0 0 3.141592653589793 0 0'
    assert model.findtext('.//link/pose').endswith('1.5707963267948966 0')
    assert model.findtext('.//link/inertial/mass') == '0.001'


def test_lidar_ground_footprint_covers_reference_and_landing_pattern():
    world = ET.parse(adapter.ROOT/'examples/gazebo/worlds/default.sdf')
    assert world.findtext('.//physics/max_step_size') == '0.004'
    assert world.findtext('.//gravity') == '0 0 -9.8'
    assert world.findtext('.//collision/geometry/plane/size') == '6000 6000'
    assert world.findtext('.//visual/geometry/mesh/uri') == '../meshes/ground.dae'
    mesh = ET.parse(adapter.ROOT/'examples/gazebo/meshes/ground.dae')
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    positions = np.fromstring(mesh.findtext('.//c:float_array[@id="ground_positions_array"]', namespaces=ns), sep=' ').reshape(-1, 3)
    np.testing.assert_array_equal(positions, [[-3000, -3000, 0], [3000, -3000, 0], [3000, 3000, 0], [-3000, 3000, 0]])
    assert mesh.find('.//c:triangles', ns).attrib['count'] == '2'
    assert mesh.findtext('.//c:up_axis', namespaces=ns) == 'Z_UP'


def test_lightweight_ground_texture_and_sky_are_local_visual_assets():
    world_path = adapter.ROOT/'examples/gazebo/worlds/default.sdf'
    world = ET.parse(world_path)
    assert world.find('.//scene/sky') is not None
    background = list(map(float, world.findtext('.//scene/background').split()))
    assert background[2] > background[0]  # Blue fallback, not the old white/gray.
    assert len(world.findall('.//collision')) == 1
    assert world.find('.//heightmap') is None
    texture = world_path.parent/world.findtext('.//visual/material/pbr/metal/albedo_map')
    assert texture.is_file() and texture.stat().st_size < 16000
    tokens = ' '.join(l for l in texture.read_text().splitlines() if not l.startswith('#')).split()
    assert tokens[:4] == ['P3', '32', '32', '255']
    pixels = np.array(list(map(int, tokens[4:])), dtype=int).reshape(32, 32, 3)
    assert pixels.min() >= 0 and pixels.max() <= 255
    assert np.unique(pixels.reshape(-1, 3), axis=0).shape[0] > 8
    assert pixels[..., 1].mean() > pixels[..., 0].mean() > pixels[..., 2].mean()
    assert world.findtext('.//visual/material/pbr/metal/roughness') == '1'
    assert world.findtext('.//visual/material/pbr/metal/metalness') == '0'
    mesh = ET.parse(adapter.ROOT/'examples/gazebo/meshes/ground.dae')
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    uv = np.fromstring(mesh.findtext('.//c:float_array[@id="ground_uv_array"]', namespaces=ns), sep=' ')
    assert uv.min() == 0 and uv.max() == 150  # 6000/150 = 40 m repeat.
    hashes = adapter.source_hashes()
    assert 'examples/gazebo/materials/textures/grass.ppm' in hashes
    assert 'examples/gazebo/meshes/ground.dae' in hashes


def test_gui_requires_explicit_local_display_authorization(monkeypatch, tmp_path):
    monkeypatch.delenv('DISPLAY', raising=False)
    with pytest.raises(ValueError, match='GUI requires'):
        adapter.container_arguments('swarm-test', tmp_path, gui=True)


def test_worker_refuses_to_connect_on_host(monkeypatch, tmp_path):
    monkeypatch.delenv('SWARM_SITL_WORKER', raising=False)
    with pytest.raises(ValueError, match='simulation-only'):
        adapter.execute_worker(tmp_path, 30)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('text,ready', (
    ('    valid: True\n    seq_total: 42\n', True),
    ('    valid: False\n    seq_total: 42\n', False),
    ('    valid: True\n    seq_total: 4\n', False),
    ('    valid: True\n    seq_total: 420\n', False),
    ('', False),
))
def test_mission_upload_ack_is_not_feasibility_confirmation(text, ready):
    assert adapter.mission_is_ready(text, 42) is ready


def camera_status(**changes):
    data = {'trackMode': 'FOLLOW', 'followTarget': {'name': adapter.CAMERA_FOLLOW_TARGET},
            'followPgain': 1.,
            'followOffset': dict(zip(('x', 'y', 'z'), adapter.CAMERA_FOLLOW_OFFSET))}
    data.update(changes)
    return json.dumps(data)


@pytest.mark.parametrize('status,expected', (
    (camera_status(), True),
    (camera_status(trackMode='TRACK'), False),
    (camera_status(followPgain=.01), False),
    (camera_status(followTarget={'name': 'wrong_drone'}), False),
    (camera_status(followOffset={}), False),
    (camera_status(followOffset={'x': 'bad', 'y': -3, 'z': 3}), False),
    ('{}', False), ('not JSON', False), ('[]', False),
))
def test_camera_follow_status_must_match_target_mode_and_offset(status, expected):
    assert adapter.camera_status_matches(status) is expected


def test_camera_initialization_retries_late_gui_and_stops_after_confirmation(monkeypatch):
    calls = []
    responses = iter(('', 'data: true', 'data: true', '', camera_status()))
    def fake(args):
        calls.append(args)
        return next(responses)
    monkeypatch.setattr(adapter, 'camera_gz', fake)
    monkeypatch.setattr(adapter.time, 'sleep', lambda _: None)
    result = adapter.initialize_camera_follow(.5)
    assert result['startup_status_confirmed']
    assert len(calls) == 5
    assert calls[0] == calls[1] and '/gui/follow' in calls[0]
    assert '/gui/follow/offset' in calls[2]
    assert '/gui/track' in calls[3] and 'follow_pgain: 1.0' in calls[3][-1]
    assert '/gui/currently_tracked' in calls[4]
    assert not any('mavlink' in str(c) or 'arm' in c for c in calls)


def test_camera_service_ack_without_matching_status_is_not_success(monkeypatch):
    monkeypatch.setattr(adapter, 'camera_gz', lambda args: '{}' if args[0] == 'topic' else 'data: true')
    with pytest.raises(TimeoutError, match='before arming'):
        adapter.initialize_camera_follow(.01)


@pytest.mark.parametrize('value', (0., -1., float('nan')))
def test_camera_initialization_rejects_invalid_budget(value):
    with pytest.raises(ValueError):
        adapter.initialize_camera_follow(value)


def test_offline_cli_uses_real_generator_without_any_simulator_or_network(monkeypatch, tmp_path):
    def forbidden(*args, **kwargs):
        pytest.fail('offline generation must never start or connect to a simulator')
    monkeypatch.setattr(adapter, 'managed_run', forbidden)
    output = tmp_path/'offline'
    assert adapter.main(['--output', str(output)]) == 0
    reference = json.loads((output/'reference.json').read_text())
    mission = json.loads((output/'point_mass_mission.json').read_text())
    assert mission['status'] == 'validated' and len(mission['plans']) == 1
    assert reference['mission_sha256'] == mission['sha256']
    assert reference['timing_preserved_by_px4_mission'] is False
    assert reference['single_aircraft_only'] and reference['obstacles'] == []
    scene = ET.parse(output/'scene.sdf')
    assert scene.find('.//model[@name="swarm_target"]') is not None
    assert json.loads((output/'scene.json').read_text())['mission_sha256'] == mission['sha256']
    np.testing.assert_array_equal(reference['positions_enu_m'], np.array(mission['plans'][0]['flight_states'])[:, :3])
    with pytest.raises(SystemExit):
        adapter.main(['--output', str(output)])


@pytest.mark.parametrize('options', (['--gui'], ['--gpu'], ['--seed', '-1'], ['--timeout', 'nan'], ['--timeout', '10']))
def test_invalid_cli_options_have_no_external_effect(tmp_path, options):
    with pytest.raises(SystemExit):
        adapter.main([*options, '--output', str(tmp_path/'never')])
    assert not list(tmp_path.iterdir())


def test_obstacles_and_target_share_original_mission_geometry(tmp_path):
    from swarm_sim.px4_scene import scene_tree
    from swarm_sim.trajectory2d.obstacles import load_obstacles
    obstacles = load_obstacles(adapter.ROOT/'examples/scenarios/static_obstacles.json')
    document, times, points = generate_reference(19, obstacles)
    assert document['status'] == 'validated'
    from swarm_sim.trajectory2d.obstacles import Obstacle
    assert tuple(Obstacle(**o) for o in document['config']['obstacles']) == obstacles
    world = scene_tree(document, adapter.ROOT/'examples/gazebo/worlds/default.sdf').getroot().find('world')
    for index, obstacle in enumerate(obstacles):
        entity = world.find(f'model[@name="swarm_obstacle_{index}"]')
        np.testing.assert_array_equal(np.fromstring(entity.findtext('pose'), sep=' ')[:3], obstacle.center)
        visual = entity.find('link/visual/geometry')
        collision = entity.find('link/collision/geometry')
        assert ET.canonicalize(ET.tostring(visual), strip_text=True) == ET.canonicalize(ET.tostring(collision), strip_text=True)
        if obstacle.kind == 'box':
            np.testing.assert_array_equal(np.fromstring(visual.findtext('box/size'), sep=' '), np.array(obstacle.half_extents)*2)
        else:
            assert float(visual.findtext('cylinder/radius')) == obstacle.half_extents[0]
            assert float(visual.findtext('cylinder/length')) == obstacle.half_extents[2]*2
    for name in ('swarm_target', 'swarm_target_ground', 'swarm_target_beacon'):
        marker = world.find(f'model[@name="{name}"]')
        assert marker.find('link/collision') is None
        assert marker.findtext('link/visual/visibility_flags') == '1'
    target = world.find('model[@name="swarm_target"]')
    np.testing.assert_allclose(np.fromstring(target.findtext('pose'), sep=' ')[:3], [1200, 500, 80], atol=.1)
    assert float(target.findtext('link/visual/geometry/sphere/radius')) == document['config']['target_radius']
    assert world.findtext('.//albedo_map').startswith('/workspace/examples/gazebo/')
    assert world.findtext('.//mesh/uri').startswith('/workspace/examples/gazebo/')


def test_target_annotation_visibility_is_excluded_from_range_sensor():
    sensor = ET.parse(adapter.ROOT/'examples/gazebo/models/rc_cessna_lidar/model.sdf')
    mask = int(sensor.findtext('.//ray/visibility_mask'))
    assert mask & 1 == 0
    assert mask & 0xffffffff != 0  # Ordinary terrain/obstacles remain visible.


def test_obstacle_cli_exports_same_scene_and_rejects_invalid_input(monkeypatch, tmp_path):
    monkeypatch.setattr(adapter, 'managed_run', lambda *a, **k: pytest.fail('offline only'))
    output = tmp_path/'scene'
    assert adapter.main(['--output', str(output), '--obstacles', str(adapter.ROOT/'examples/scenarios/static_obstacles.json')]) == 0
    mission = json.loads((output/'point_mass_mission.json').read_text())
    reference = json.loads((output/'reference.json').read_text())
    assert len(reference['obstacles']) == 2
    assert reference['obstacles'] == mission['config']['obstacles']
    assert len(ET.parse(output/'scene.sdf').findall('.//collision')) == 3  # ground + two solids
    assert adapter.main(['--output', str(tmp_path/'invalid'), '--obstacles', str(tmp_path/'missing.json')]) == 2
    assert not (tmp_path/'invalid').exists()


def test_obstructed_snapshot_fails_instead_of_creating_unchecked_scene():
    from swarm_sim.trajectory2d.obstacles import Obstacle
    with pytest.raises(ValueError, match='initial_obstacle_violation'):
        generate_reference(obstacles=(Obstacle.box(290, -10, 70, 310, 10, 90),))


def test_display_reduction_is_bounded_and_does_not_modify_reference():
    from swarm_sim.px4_scene import display_polyline
    x = np.linspace(0, 100, 501)
    points = np.column_stack((x, 10*np.sin(x/20), 80+np.sin(x/10)))
    before = points.copy()
    reduced = display_polyline(points)
    np.testing.assert_array_equal(points, before)
    np.testing.assert_array_equal(reduced[[0, -1]], points[[0, -1]])
    assert len(reduced) < len(points)/4
    assert max(polyline_distances(points, reduced)) <= .5+1e-10
    np.testing.assert_array_equal(display_polyline([[1, 2, 3]]), [[1, 2, 3]])


def test_theoretical_path_visuals_are_oriented_noncolliding_and_lidar_invisible():
    from swarm_sim.px4_scene import add_reference_visuals
    # Includes a genuinely 3D segment and fractional arrival time.
    points = np.array([[0, 0, 80], [10, 10, 85], [20, 10, 85], [30, 10, 85]])
    world = ET.Element('world')
    plan = {'flight_states': points.tolist(), 'arrival_time': 1.5, 'dt': 1.}
    original = json.dumps(plan)
    add_reference_visuals(world, plan)
    assert json.dumps(plan) == original
    assert not world.findall('.//collision')
    reconstructed = {}
    for name in ('arrival', 'onward'):
        model = world.find(f'model[@name="swarm_reference_{name}"]')
        assert model.findtext('static') == 'true'
        segments = []
        for visual in model.findall('link/visual'):
            assert visual.findtext('visibility_flags') == '1'
            assert visual.findtext('cast_shadows') == 'false'
            pose = np.fromstring(visual.findtext('pose'), sep=' ')
            pitch, yaw = pose[4:]
            direction = np.array([math.sin(pitch)*math.cos(yaw), math.sin(pitch)*math.sin(yaw), math.cos(pitch)])
            half = float(visual.findtext('geometry/cylinder/length'))/2
            segments.append((pose[:3]-direction*half, pose[:3]+direction*half))
        reconstructed[name] = segments
    np.testing.assert_allclose(reconstructed['arrival'][0][0], points[0], atol=1e-10)
    np.testing.assert_allclose(reconstructed['arrival'][-1][1], [15, 10, 85], atol=1e-10)
    np.testing.assert_allclose(reconstructed['onward'][0][0], [15, 10, 85], atol=1e-10)
    np.testing.assert_allclose(reconstructed['onward'][-1][1], points[-1], atol=1e-10)


@pytest.mark.parametrize('points,tolerance', (([], .5), ([[0, 0, float('nan')]], .5), ([[0, 0, 1]], 0)))
def test_display_polyline_rejects_invalid_input(points, tolerance):
    from swarm_sim.px4_scene import display_polyline
    with pytest.raises(ValueError):
        display_polyline(points, tolerance)


def test_airborne_container_is_opt_in_and_attaches_only_simulated_model(tmp_path):
    ordinary = adapter.container_arguments('bench', tmp_path)
    airborne = adapter.container_arguments('bench', tmp_path, airborne_start=True)
    assert 'SWARM_AIRBORNE_START=1' not in ordinary
    assert 'SWARM_AIRBORNE_START=1' in airborne
    assert 'PX4_GZ_MODEL_NAME=rc_cessna_lidar_0' in airborne
    assert 'PYTHONPATH=/workspace/examples/gazebo' in airborne
    assert airborne[airborne.index('--network')+1] == 'none'
    assert '--privileged' not in airborne and '--device' not in airborne


def test_airborne_offline_scene_keeps_reference_and_ground_preflight(monkeypatch, tmp_path):
    monkeypatch.setattr(adapter, 'managed_run', lambda *a, **k: pytest.fail('offline only'))
    output = tmp_path/'airborne'
    assert adapter.main(['--airborne-start', '--output', str(output)]) == 0
    reference = json.loads((output/'reference.json').read_text())
    original = json.loads((output/'point_mass_mission.json').read_text())
    assert reference['airborne_start'] is True
    np.testing.assert_array_equal(reference['positions_enu_m'], np.array(original['plans'][0]['flight_states'])[:, :3])
    model = ET.parse(output/'scene.sdf').find('.//model[@name="rc_cessna_lidar_0"]')
    assert model.findtext('pose') == '300.0 0.0 0 0 0 0.0'
    assert model.findtext('plugin/module_name') == 'airborne_bootstrap'
    assert model.findtext('include/uri').endswith('/rc_cessna_lidar/model.sdf')
    assert not (output/'airborne-release.json').exists()  # No offline flight command.


def load_bootstrap_for_test(monkeypatch):
    import importlib.util
    import sys
    from types import ModuleType
    # No native Gazebo libraries needed to test the one-shot state machine.
    math_module, sim_module = ModuleType('gz.math7'), ModuleType('gz.sim8')
    math_module.Pose3d = lambda *args: args
    math_module.Vector3d = lambda *args: np.asarray(args)
    sim_module.Link = sim_module.Model = object
    monkeypatch.setitem(sys.modules, 'gz', ModuleType('gz'))
    monkeypatch.setitem(sys.modules, 'gz.math7', math_module)
    monkeypatch.setitem(sys.modules, 'gz.sim8', sim_module)
    spec = importlib.util.spec_from_file_location('bootstrap_under_test', adapter.ROOT/'examples/gazebo/airborne_bootstrap.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_airborne_plugin_refuses_unmanaged_process(monkeypatch):
    monkeypatch.delenv('SWARM_AIRBORNE_START', raising=False)
    plugin = load_bootstrap_for_test(monkeypatch).AirborneBootstrap()
    with pytest.raises(RuntimeError, match='managed simulator'):
        plugin.configure(None, None, None, None)


def test_airborne_impulse_is_once_only_after_separate_pose_step(monkeypatch, tmp_path):
    from types import SimpleNamespace
    from datetime import timedelta
    plugin = load_bootstrap_for_test(monkeypatch).AirborneBootstrap()
    poses, forces = [], []
    link = SimpleNamespace(world_pose=lambda ecm: SimpleNamespace(z=lambda: .24),
                           world_linear_velocity=lambda ecm: np.zeros(3),
                           add_world_force=lambda ecm, force: forces.append(force.copy()))
    plugin.output = tmp_path
    plugin.state = [300, 0, 80, 15, 0, 0, 0]
    plugin.identity = 'identity'
    plugin.velocity = np.array([15., 0, 0])
    plugin.link = link
    plugin.links = [(link, 2.)]
    plugin.model = SimpleNamespace(set_world_pose_cmd=lambda ecm, pose: poses.append(pose))
    plugin.positioned = plugin.applied = False
    info = SimpleNamespace(paused=False, dt=timedelta(seconds=.004), sim_time=timedelta(seconds=10))
    plugin.pre_update(info, None)
    assert not poses and not forces  # No request, no intervention.
    (tmp_path/'airborne-release.json').write_text(json.dumps({'mission_sha256': 'identity', 'release': True}))
    plugin.pre_update(info, None)
    assert len(poses) == 1 and not forces and plugin.positioned and not plugin.applied
    np.testing.assert_allclose(poses[0], [300, 0, 79.76, 0, 0, 0])
    plugin.pre_update(info, None)
    assert plugin.applied
    np.testing.assert_allclose(forces, [[7500, 0, 0]])
    for _ in range(100):
        plugin.pre_update(info, None)
    assert len(poses) == len(forces) == 1  # Free physics thereafter, no replay.


def test_translated_spawn_keeps_same_world_reference():
    origin = LocalOrigin(47.397971, 8.550138, .13)
    point, offset = np.array([500., 100., 80.]), np.array([300., 0., 0.])
    geographic = origin.from_enu(point-offset)
    np.testing.assert_allclose(origin.to_enu(*geographic)+offset, point, atol=1e-7)


def test_airborne_camera_failure_does_not_claim_no_flight_commands(monkeypatch):
    monkeypatch.setattr(adapter, 'camera_gz', lambda args: '')
    with pytest.raises(TimeoutError, match='after airborne Mission activation'):
        adapter.initialize_camera_follow(.001, before_arming=False)


def test_reference_observations_exclude_pre_release_not_startup_deviations():
    rows = [{'sim_time_s': t, 'mission_current': seq, 'up_m': z}
            for t, seq, z in [(7.9, 3, 0), (8.0, 3, 80), (8.1, 4, 30), (100, 40, 80)]]
    assert adapter.reference_observation_rows(rows, 42, 7.912) == rows[1:3]
    assert adapter.reference_observation_rows(rows, 42) == rows[:3]
