"""Camera geometry/UI configuration only; tests send no aircraft commands."""

import math
import xml.etree.ElementTree as ET

import pytest

from swarm_sim.px4_camera import View, moved, overview, gui_config, view_matches, restore_free_view


def test_camera_translation_azerty_qwerty_and_height_limits():
    view = View(100, 200, 50, math.pi/2, .6)
    for key in ('Z', 'W'):
        result = moved(view, ord(key), 50)
        assert result.x == pytest.approx(100) and result.y == pytest.approx(250)
        assert result.z == 50 and result.pitch == .6
    assert moved(view, ord('S'), 50).y == pytest.approx(150)
    assert moved(view, ord('Q'), 50).x == pytest.approx(50)
    assert moved(view, ord('A'), 50) == moved(view, ord('Q'), 50)
    assert moved(view, ord('D'), 50).x == pytest.approx(150)
    assert moved(view, ord('R'), 50).z == 100
    assert moved(view, ord('F'), 100).z == 2
    assert moved(view, 16777238, 50) == moved(view, ord('R'), 50)
    assert moved(view, 16777239, 50) == moved(view, ord('F'), 50)
    assert moved(view, ord('X'), 50) is None


def test_camera_rotation_and_quaternion_are_bounded():
    view = View(0, 0, 100, 0, 0)
    assert moved(view, 16777234, 50).yaw == pytest.approx(math.radians(10))
    assert moved(view, 16777236, 50).yaw == pytest.approx(-math.radians(10))
    assert moved(view, 16777235, 50).pitch == pytest.approx(-math.radians(10))
    for _ in range(30):
        view = moved(view, 16777237, 50)
    assert view.pitch == pytest.approx(math.radians(85))
    assert sum(v*v for v in view.quaternion()) == pytest.approx(1)


def test_overview_verification_checks_orientation_and_wrapped_yaw():
    desired = View(10, 20, 30, math.pi, .7)
    assert view_matches(View(10, 20, 30, -math.pi, .7), desired)
    assert not view_matches(None, desired)
    assert not view_matches(View(13, 20, 30, math.pi, .7), desired)
    assert not view_matches(View(10, 20, 30, math.pi, .8), desired)


def test_reset_retries_acked_but_discarded_animation_until_observed():
    view = View(10, 20, 30, 0, .7)
    now, requests = [0.], []

    def request(pose):
        requests.append(pose)
        return True  # First ACK is not proof the camera actually moved.

    def observe():
        return (view, len(requests) >= 2)

    assert restore_free_view(view, lambda: True, request, observe,
                             clock=lambda: now[0], sleep=lambda dt: now.__setitem__(0, now[0]+dt))
    assert len(requests) == 2


def test_reset_fails_if_detachment_or_render_pose_is_not_confirmed():
    view = View(10, 20, 30, 0, .7)
    now, requests = [0.], []
    assert not restore_free_view(view, lambda: False, requests.append, lambda: (view, True))
    assert not requests
    assert not restore_free_view(view, lambda: True, lambda pose: requests.append(pose),
                                 lambda: (View(100, 100, 100, 0, 0), True), timeout=1.,
                                 clock=lambda: now[0], sleep=lambda dt: now.__setitem__(0, now[0]+dt))
    assert requests


def test_overview_uses_all_references_obstacles_and_target_without_mutation():
    document = {'plans': [{'flight_states': [[0, 0, 60], [1000, 300, 80]]},
                          {'flight_states': [[-400, -300, 90], [800, 100, 80]]}],
                'config': {'target': [1200, 500], 'target_altitude': 80,
                           'obstacles': [{'center': [800, 200, 75], 'half_extents': [125, 125, 75]}]}}
    view = overview(document)
    assert view.x == 400 and view.y < -300 and view.z > 1000
    assert view.yaw == pytest.approx(math.pi/2)
    assert document['plans'][0]['flight_states'][0] == [0, 0, 60]


def test_gui_config_retains_mouse_scene_tools_and_adds_keyboard_not_teleop():
    template = '''<?xml version="1.0"?><window><width>1000</width></window>
    <plugin filename="MinimalScene"><camera_pose>0 0 5 0 0 0</camera_pose></plugin>
    <plugin filename="InteractiveViewControl"/><plugin filename="CameraTracking"/>'''
    view = View(100, -700, 600, 1, .7)
    result = gui_config(template, view)
    root = ET.fromstring('<settings>'+result.split('?>', 1)[1]+'</settings>')
    assert root.find("plugin[@filename='KeyPublisher']") is not None
    assert root.find("plugin[@filename='InteractiveViewControl']") is not None
    assert root.find("plugin[@filename='CameraTracking']") is not None
    assert root.find("plugin[@filename='Teleop']") is None
    scene = root.find("plugin[@filename='MinimalScene']")
    assert scene.find('camera_clip/far').text == '20000'
    assert scene.find('camera_pose').text == '100 -700 600 0 0.7 1'
    assert gui_config(result, view).count('filename="KeyPublisher"') == 1
