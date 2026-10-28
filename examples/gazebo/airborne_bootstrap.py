"""One-shot warm-start impulse for the isolated Cessna experiment.

No trajectory replay, persistent velocity command, estimator injection or
autopilot interface. Ordinary forces/torques and PX4 resume after ONE step.
The initial airborne pitch is zero in this level-only fixture, not calibrated
angle-of-attack trim or a restored complete airframe/controller checkpoint.
"""

import csv
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from gz.math7 import Pose3d, Vector3d
from gz.sim8 import Link, Model


class AirborneBootstrap:
    def configure(self, entity, sdf, ecm, event_mgr):
        if os.environ.get('SWARM_AIRBORNE_START') != '1' or not Path('/.dockerenv').exists():
            raise RuntimeError('airborne bootstrap is restricted to the managed simulator')
        self.model = Model(entity)
        model_name = self.model.name(ecm)
        fleet = os.environ.get('SWARM_FLEET_SIZE') == '4'
        names = [f'rc_cessna_lidar_{k}' for k in range(4)] if fleet else ['rc_cessna_lidar_0']
        if model_name not in names:
            raise ValueError('unexpected bootstrap model')
        self.output = Path('/runresults')/(f'drone-{names.index(model_name)}' if fleet else '')
        reference = json.loads((self.output/'reference.json').read_text())
        mission = json.loads((self.output/'point_mass_mission.json').read_text())
        if not reference.get('airborne_start') or len(mission['plans']) != 1:
            raise ValueError('single-aircraft airborne experiment required')
        self.identity = reference['mission_sha256']
        self.state = list(map(float, mission['plans'][0]['flight_states'][0]))
        x, y, z, speed, yaw, gamma, bank = self.state
        if not all(map(math.isfinite, self.state)) or not (10 <= speed <= 20 and 20 <= z <= 100):
            raise ValueError('invalid airborne bench state')
        if abs(gamma) > 1e-8 or abs(bank) > math.radians(35):
            raise ValueError('bootstrap requires level initial flight with bank within 35 degrees')
        self.yaw_rate = float(mission['snapshots'][0]['state'][3])
        if not math.isfinite(self.yaw_rate) or abs(self.yaw_rate) > .3:
            raise ValueError('unsupported initial loiter yaw rate')
        self.pose = Pose3d(x, y, z, 0, 0, yaw)
        self.velocity = Vector3d(speed*math.cos(yaw), speed*math.sin(yaw), 0)
        self.link = Link(self.model.canonical_link(ecm))
        self.link.enable_velocity_checks(ecm)
        # The exact pinned aircraft has no nested included models. Apply the
        # same translational impulse to every link in proportion to its mass,
        # avoiding an artificial sustained velocity constraint / joint shock.
        base = ET.parse('/opt/px4-gazebo/share/gz/models/rc_cessna/model.sdf')
        accessory = ET.parse('/workspace/examples/gazebo/models/rc_cessna_lidar/model.sdf')
        self.links = []
        for element in base.findall('./model/link') + accessory.findall('./model/link'):
            mass = float(element.findtext('inertial/mass'))
            link = Link(self.model.link_by_name(ecm, element.attrib['name']))
            if not link.valid(ecm) or not math.isfinite(mass) or mass <= 0:
                raise ValueError('invalid pinned airframe mass/link')
            link.enable_velocity_checks(ecm)
            self.links.append((link, mass))
        self.applied = False
        self.positioned = False
        self.acknowledged = False
        self.last_sample = -1.
        with (self.output/'airborne-bootstrap-ready.json').open('x') as stream:
            json.dump({'mission_sha256': self.identity, 'total_mass_kg': sum(m for _, m in self.links),
                       'method': 'one step of physical impulse; no velocity constraint'}, stream)

    def pre_update(self, info, ecm):
        if info.paused or self.applied:
            return
        request_path = self.output/'airborne-release.json'
        if not request_path.exists():
            return
        # The worker publishes a complete file by atomic rename only AFTER
        # normal readiness, mission feasibility, arm and start-mission ACKs.
        request = json.loads(request_path.read_text())
        if request != {'mission_sha256': self.identity, 'release': True}:
            raise ValueError('invalid airborne release request')
        dt = info.dt.total_seconds()
        if not 0 < dt <= .02:
            raise ValueError('unexpected impulse physics step')
        if not self.positioned:
            # Move clear of ground contacts first. Otherwise the ground's
            # reaction impulse from this step contaminates the requested Vz.
            current = self.link.world_pose(ecm)
            x, y, z, _, yaw, _, bank = self.state
            # Planner bank is positive for ENU counter-clockwise left turns;
            # the native FLU model's positive X roll banks the opposite way.
            self.model.set_world_pose_cmd(ecm, Pose3d(x, y, z-current.z(), -bank, 0, yaw))
            self.positioned = True
            return
        for link, mass in self.links:
            velocity = link.world_linear_velocity(ecm)
            if velocity is None:
                raise ValueError('missing pre-release velocity')
            link.add_world_force(ecm, (self.velocity-velocity)*(mass/dt))
            if abs(getattr(self, 'yaw_rate', 0)) > 1e-9:
                inertia = link.world_inertia_matrix(ecm)
                if inertia is None:
                    raise ValueError('missing native inertia for loiter initialization')
                delta = Vector3d(0, 0, self.yaw_rate)-self.link.world_angular_velocity(ecm)
                link.add_world_wrench(ecm, Vector3d(0, 0, 0), inertia*(delta/dt))
        self.release_time = info.sim_time.total_seconds()
        self.applied = True

    def post_update(self, info, ecm):
        if not self.applied or info.paused:
            return
        pose = self.link.world_pose(ecm)
        velocity = self.link.world_linear_velocity(ecm)
        angular = self.link.world_angular_velocity(ecm)
        if pose is None or velocity is None or angular is None:
            return
        now = info.sim_time.total_seconds()
        state = {'sim_time_s': now, 'elapsed_s': now-self.release_time,
                 'position_world_m': [pose.x(), pose.y(), pose.z()],
                 'velocity_world_m_s': [velocity.x(), velocity.y(), velocity.z()],
                 'rpy_rad': [pose.rot().roll(), pose.rot().pitch(), pose.rot().yaw()],
                 'angular_velocity_rad_s': [angular.x(), angular.y(), angular.z()]}
        if not self.acknowledged:
            with (self.output/'airborne-initial-state.json').open('x') as stream:
                json.dump({'mission_sha256': self.identity, 'requested_state': self.state,
                           'snapshot': state, 'impulse_steps': 1,
                           'persistent_velocity_control': False}, stream, allow_nan=False)
            self.acknowledged = True
        # Observer-only startup evidence, including the estimator transient.
        if now-self.release_time <= 30 and now-self.last_sample >= .1:
            first = self.last_sample < 0
            with (self.output/'airborne-physics.csv').open('x' if first else 'a', newline='') as stream:
                writer = csv.writer(stream)
                if first:
                    writer.writerow(['elapsed_s', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw'])
                writer.writerow([state['elapsed_s'], *state['position_world_m'],
                                 *state['velocity_world_m_s'], *state['rpy_rad']])
            self.last_sample = now


def get_system():
    return AirborneBootstrap()
