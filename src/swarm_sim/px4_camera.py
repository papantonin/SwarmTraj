"""GUI-only keyboard camera. Never imports MAVSDK or writes simulator entities.

Run as a script with the container's system Python (Gazebo transport bindings).
The geometry/configuration helpers are also importable in offline unit tests.
"""

import argparse
from dataclasses import dataclass, replace
import json
import math
import os
from pathlib import Path
import re
import subprocess
import threading
import time
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class View:
    x: float
    y: float
    z: float
    yaw: float
    pitch: float

    def quaternion(self):
        cy, sy = math.cos(self.yaw/2), math.sin(self.yaw/2)
        cp, sp = math.cos(self.pitch/2), math.sin(self.pitch/2)
        return (-sy*sp, cy*sp, sy*cp, cy*cp)


def overview(document):
    points = [state[:3] for plan in document['plans'] for state in plan['flight_states']]
    for obstacle in document['config']['obstacles']:
        center, half = obstacle['center'], obstacle['half_extents']
        points += [[center[k]+sign*half[k] for k in range(3)] for sign in (-1, 1)]
    points.append([*document['config']['target'], document['config']['target_altitude'] or 80.])
    if not points or not all(math.isfinite(v) for p in points for v in p):
        raise ValueError('Finite scenario geometry required for camera overview.')
    low = [min(p[k] for p in points) for k in range(3)]
    high = [max(p[k] for p in points) for k in range(3)]
    extent = max(high[0]-low[0], high[1]-low[1], 400.)+400.
    x, y, z = (low[0]+high[0])/2, (low[1]+high[1])/2, max(high[2], 80.)
    distance, height = .85*extent, .8*extent
    return View(x, y-distance, z+height, math.pi/2, math.atan2(height, distance))


def moved(view, key, step):
    """Qt key codes; ZQSD and WASD, arrows rotate, R/F change world altitude."""
    forward = (math.cos(view.yaw), math.sin(view.yaw))
    left = (-forward[1], forward[0])
    if key in (ord('Z'), ord('W'), ord('S'), ord('Q'), ord('A'), ord('D')):
        if key in (ord('Z'), ord('W')):
            delta = forward
        elif key == ord('S'):
            delta = tuple(-v for v in forward)
        elif key in (ord('Q'), ord('A')):
            delta = left
        else:
            delta = tuple(-v for v in left)
        return replace(view, x=max(-10000., min(10000., view.x+step*delta[0])),
                       y=max(-10000., min(10000., view.y+step*delta[1])))
    if key in (ord('R'), ord('F'), 16777238, 16777239):  # PageUp/PageDown
        return replace(view, z=max(2., min(10000., view.z+step*(1 if key in (ord('R'), 16777238) else -1))))
    if key in (16777234, 16777236):  # left/right
        return replace(view, yaw=(view.yaw+math.radians(10)*(1 if key == 16777234 else -1)+math.pi)%(2*math.pi)-math.pi)
    if key in (16777235, 16777237):  # up/down
        return replace(view, pitch=max(math.radians(-85), min(math.radians(85),
                       view.pitch+math.radians(10)*(-1 if key == 16777235 else 1))))
    return None


def view_matches(actual, desired):
    """Observed render pose, not a transport ACK, establishes overview return."""
    if actual is None:
        return False
    distance = math.sqrt(sum((getattr(actual, k)-getattr(desired, k))**2 for k in ('x', 'y', 'z')))
    angles = [abs((getattr(actual, k)-getattr(desired, k)+math.pi)%(2*math.pi)-math.pi)
              for k in ('yaw', 'pitch')]
    return distance <= 2. and max(angles) <= .02


def restore_free_view(view, detach, request, observe, timeout=4., *,
                      clock=time.monotonic, sleep=time.sleep):
    """Bounded GUI-only reset, including a reset during an earlier animation.

    Gazebo may ACK a move that an earlier animation's completion then clears.
    Retry only this explicit reset until a fresh free render pose is observed.
    observe returns (pose, detached); no aircraft state or command is involved.
    """
    if not detach():
        return False
    deadline, retry = clock()+timeout, -math.inf
    while clock() < deadline:
        if clock() >= retry:
            request(view)
            retry = clock()+.75
        actual, detached = observe()
        if detached and view_matches(actual, view):
            return True
        sleep(.03)
    return False


def gui_config(template, initial):
    """Retain native scene/mouse/tools; add key events and a long view clip."""
    text = re.sub(r'<\?xml[^>]*\?>', '', template)
    root = ET.fromstring('<settings>'+text+'</settings>')
    scene = root.find("plugin[@filename='MinimalScene']")
    if scene is None:
        raise ValueError('Native GUI template has no MinimalScene plugin.')
    for tag, value in (('camera_pose', f'{initial.x} {initial.y} {initial.z} 0 {initial.pitch} {initial.yaw}'),
                       ('horizontal_fov', '90')):
        element = scene.find(tag)
        if element is None:
            element = ET.SubElement(scene, tag)
        element.text = value
    old = scene.find('camera_clip')
    if old is not None:
        scene.remove(old)
    clip = ET.SubElement(scene, 'camera_clip')
    ET.SubElement(clip, 'near').text = '.1'
    ET.SubElement(clip, 'far').text = '20000'
    if root.find("plugin[@filename='KeyPublisher']") is None:
        plugin = ET.SubElement(root, 'plugin', filename='KeyPublisher', name='Camera keyboard')
        properties = ET.SubElement(plugin, 'gz-gui')
        for name, kind, value in (('state', 'string', 'floating'), ('width', 'double', '5'),
                                  ('height', 'double', '5'), ('showTitleBar', 'bool', 'false')):
            ET.SubElement(properties, 'property', key=name, type=kind).text = value
    return '<?xml version="1.0"?>\n'+'\n'.join(ET.tostring(node, encoding='unicode') for node in root)


def run(output, launch_gui=True, initial_mode='free'):
    if os.environ.get('SWARM_MANAGED_SIM') != '1' or not Path('/.dockerenv').exists():
        raise ValueError('Camera helper is restricted to the managed simulator.')
    from gz.transport13 import Node
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.cameratrack_pb2 import CameraTrack
    from gz.msgs10.gui_camera_pb2 import GUICamera
    from gz.msgs10.int32_pb2 import Int32
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.stringmsg_pb2 import StringMsg

    document = json.loads((output/'point_mass_mission.json').read_text())
    home = overview(document)
    if launch_gui:
        template = Path('/usr/share/gz/gz-sim8/gui/gui.config').read_text()
        path = output/'free-camera.config'
        with path.open('x') as stream:
            stream.write(gui_config(template, home))
        gui = subprocess.Popen(['gz', 'sim', '-g', '--gui-config', str(path)])
    else:
        gui = None
    node = Node()
    track = node.advertise('/gui/track', CameraTrack)
    lock = threading.Lock()
    state = {'pose': None, 'key': None, 'key_time': 0., 'enabled': True,
             'pose_at': 0., 'track_at': 0., 'detached': False, 'detach_at': 0.}
    def on_pose(message):
        q, p = message.orientation, message.position
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        with lock:
            state['pose'] = View(p.x, p.y, p.z, yaw, pitch)
            state['pose_at'] = time.monotonic()
    def on_key(message):
        with lock:
            state.update(key=message.data, key_time=time.monotonic())
    node.subscribe(Pose, '/gui/camera/pose', on_pose)
    node.subscribe(Int32, '/keyboard/keypress', on_key)
    def on_track(message):
        with lock:
            state['detached'] = (message.track_mode == CameraTrack.NONE
                                 and not message.follow_target.name and not message.track_target.name)
            state['track_at'] = time.monotonic()
    node.subscribe(CameraTrack, '/gui/currently_tracked', on_track)
    def request(view):
        message = GUICamera()
        message.pose.position.x, message.pose.position.y, message.pose.position.z = view.x, view.y, view.z
        q = message.pose.orientation
        q.x, q.y, q.z, q.w = view.quaternion()
        success, response = node.request('/gui/move_to/pose', message, GUICamera, Boolean, 1000)
        return success and response.data
    def free():
        with lock:
            state['detach_at'] = time.monotonic()
        # NONE alone does not clear CameraTracking's remembered follow name.
        # Use its acknowledged GUI follow service to clear that selection first.
        success, response = node.request('/gui/follow', StringMsg(data=''), StringMsg, Boolean, 1000)
        if not (success and response.data):
            return False
        message = CameraTrack(track_mode=CameraTrack.NONE)
        # NONE is protobuf's zero/default value: alone it serializes to an empty
        # payload. Include explicit empty submessages for a nonempty reset packet.
        message.follow_target.SetInParent()
        message.track_target.SetInParent()
        return track.publish(message)
    def observed_free_pose():
        with lock:
            fresh = min(state['pose_at'], state['track_at']) >= state['detach_at']
            return state['pose'], fresh and state['detached']
    ready = time.monotonic()+40.
    while state['pose'] is None:
        if time.monotonic() > ready or (gui is not None and gui.poll() is not None):
            raise TimeoutError('Free camera GUI pose was not observed.')
        time.sleep(.1)
    time.sleep(.3)  # Allow the track subscription to discover the publisher.
    if initial_mode == 'follow':
        track.publish(CameraTrack(track_mode=CameraTrack.FOLLOW,
            follow_target={'name': 'rc_cessna_lidar_0'}, follow_offset={'x': -6., 'y': -3., 'z': 3.},
            follow_pgain=1., track_pgain=1.))
    else:
        # A pose publication can precede service discovery / the first rendered
        # scene update on a loaded host. Keep the readiness gate, but retry
        # transport acquisition instead of leaving a GUI without its helper.
        deadline = time.monotonic()+15.
        while not restore_free_view(home, free, request, observed_free_pose):
            if time.monotonic() > deadline:
                raise RuntimeError('Camera overview was not observed after GUI service acquisition retries.')
            time.sleep(.2)
    (output/'camera-ready.json').write_text(json.dumps({'mode': initial_mode, 'keyboard': True,
        'aircraft_commands': False, 'movement_step_m': 50., 'animation_s': .5}))
    print('Camera: ZQSD/WASD move, R/F up/down, arrows turn/look, '
          '+/- step size, 0/Home free overview, 1–4 follow an aircraft, F8 enable/disable.', flush=True)
    step, busy_until = 50., time.monotonic()+.65
    while gui is None or gui.poll() is None:
        time.sleep(.03)
        with lock:
            key, pose, age = state['key'], state['pose'], time.monotonic()-state['key_time']
            # Reset/toggle are commands, not stale repeated movement. In
            # particular, a quick 1 -> 0 must not expire during the follow delay.
            if time.monotonic() < busy_until and key not in (48, 16777232, 16777271):
                continue
            state['key'] = None
        if key is None or age > (2. if key in (48, 16777232, 16777271) else .35):
            continue  # No replay of queued movement after key release/focus loss.
        if key == 16777271:  # F8
            state['enabled'] = not state['enabled']
            continue
        if not state['enabled']:
            continue
        if key in (43, 61, 45):
            step = max(5., min(400., step*(.5 if key == 45 else 2)))
            continue
        if key in range(49, 49+len(document['plans'])):
            track.publish(CameraTrack(track_mode=CameraTrack.FOLLOW,
                follow_target={'name': f'rc_cessna_lidar_{key-49}'},
                follow_offset={'x': -6., 'y': -3., 'z': 3.}, follow_pgain=1., track_pgain=1.))
            busy_until = time.monotonic()+.65
            continue
        if key in (48, 16777232):
            verified = restore_free_view(home, free, request, observed_free_pose)
            with (output/'camera-events.jsonl').open('a') as stream:
                stream.write(json.dumps({'key': key, 'requested_position': [home.x, home.y, home.z],
                    'mode': 'free-overview', 'render_pose_verified': verified})+'\n')
            if not verified:
                print('Camera reset not confirmed; press 0/Home to retry. Aircraft unaffected.', flush=True)
            busy_until = time.monotonic()+.65
            continue
        view = moved(pose, key, step)
        if view is None:
            continue
        if free() and request(view):
            busy_until = time.monotonic()+.65
            with (output/'camera-events.jsonl').open('a') as stream:
                stream.write(json.dumps({'key': key, 'requested_position': [view.x, view.y, view.z]})+'\n')
    if gui is not None:
        (output/'camera-exit.json').write_text(json.dumps({'gui_returncode': gui.returncode,
                                                        'aircraft_commands': False}))
        if gui.returncode != 0:
            raise RuntimeError(f'Gazebo GUI exited with code {gui.returncode}; see camera.log.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--attach', action='store_true', help='attach to an existing GUI with KeyPublisher loaded')
    parser.add_argument('--mode', choices=['free', 'follow'], default='free')
    args = parser.parse_args()
    run(args.output, not args.attach, args.mode)
