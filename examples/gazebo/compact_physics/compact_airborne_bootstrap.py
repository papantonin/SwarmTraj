"""One-shot warm initialization of the owned compact Gazebo airframe.

Standard-library-only: loaded by Gazebo's system Python, not the project venv.
After one pose change and one impulse per link this system only observes.
"""
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from gz.math7 import Pose3d, Vector3d
from gz.sim8 import Link, Model


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


class CompactAirborneBootstrap:
    def configure(self, entity, sdf, ecm, event_mgr):
        if (os.environ.get('SWARM_COMPACT_PX4_STARTUP') != '1'
                or os.environ.get('SWARM_COMPACT_AIRBORNE') != '1'
                or not Path('/.dockerenv').exists()):
            raise RuntimeError('owned compact warm initialization only')
        self.output = Path('/runresults')
        self.model = Model(entity)
        if self.model.name(ecm) != 'compact_5kg_prototype_0':
            raise ValueError('unexpected compact bootstrap model')
        self.config = json.loads((self.output/'airborne-config.json').read_text())
        expected = {'schema', 'model_sdf_sha256', 'force_binding_sha256', 'control_sha256',
                    'state', 'body_pitch_rad'}
        if set(self.config) != expected or self.config['schema'] != 'swarm-compact-warm-start-v1':
            raise ValueError('invalid compact warm-start configuration')
        self.identity = digest(self.config)
        model_path = self.output/'models/compact_5kg_prototype/model.sdf'
        assumptions = json.loads((self.output/'assumptions.json').read_text())
        if (hashlib.sha256(model_path.read_bytes()).hexdigest() != self.config['model_sdf_sha256']
                or assumptions['force_model']['binding_sha256'] != self.config['force_binding_sha256']
                or assumptions['control_model']['sha256'] != self.config['control_sha256']
                or assumptions['airborne_config_sha256'] != self.identity):
            raise ValueError('compact warm-start physical identity mismatch')
        self.state = self.config['state']
        pitch = self.config['body_pitch_rad']
        numbers = [*self.state, pitch]
        if (len(self.state) != 7 or any(type(v) not in (float, int) or not math.isfinite(v) for v in numbers)
                or self.state != [0., 0., 80., assumptions['airframe_defaults']['FW_AIRSPD_TRIM'], 0., 0., 0.]
                or not -.25 < pitch < 0):
            raise ValueError('only the declared straight level compact cruise fixture is supported')
        x, y, z, speed, yaw, _, _ = self.state
        self.pose = Pose3d(x, y, z, 0., pitch, yaw)
        self.velocity = Vector3d(speed, 0., 0.)
        self.link = Link(self.model.canonical_link(ecm))
        self.link.enable_velocity_checks(ecm)
        self.links = []
        for element in ET.parse(model_path).findall('./model/link'):
            mass = float(element.findtext('inertial/mass'))
            link = Link(self.model.link_by_name(ecm, element.attrib['name']))
            if not link.valid(ecm) or not math.isfinite(mass) or mass <= 0:
                raise ValueError('invalid own compact link or mass')
            link.enable_velocity_checks(ecm)
            self.links.append((link, mass))
        total = sum(m for _, m in self.links)
        if not self.links or not math.isclose(total, 5., rel_tol=0., abs_tol=1e-12):
            raise ValueError('invalid compact mass accounting')
        self.positioned = self.applied = self.acknowledged = False
        self.last_sample = -1.
        self.save('airborne-bootstrap-ready.json', dict(config_sha256=self.identity, total_mass_kg=total))

    def save(self, name, value):
        # Readers never see a partially written acknowledgement.
        pending = self.output/(name+'.pending')
        with pending.open('x') as stream:
            json.dump(value, stream, allow_nan=False)
        pending.rename(self.output/name)

    def pre_update(self, info, ecm):
        if info.paused or self.applied:
            return
        path = self.output/'airborne-release.json'
        if not path.exists():
            return
        request = json.loads(path.read_text())
        mission = json.loads((self.output/'native-flight-mission.json').read_text())
        if (request != dict(config_sha256=self.identity, mission_sha256=digest(mission), release=True)
                or request.get('release') is not True):
            raise ValueError('invalid own compact airborne release')
        self.mission_identity = request['mission_sha256']
        dt = info.dt.total_seconds()
        if not 0 < dt <= .00401:
            raise ValueError('unexpected compact impulse step')
        if not self.positioned:
            # base_link has zero pose in this authored, non-nested SDF.
            # Clear ground contact for one step before the physical impulse.
            self.model.set_world_pose_cmd(ecm, self.pose)
            self.positioned = True
            return
        for link, mass in self.links:
            velocity = link.world_linear_velocity(ecm)
            if velocity is None:
                raise ValueError('missing own pre-release velocity')
            link.add_world_force(ecm, (self.velocity-velocity)*(mass/dt))
        self.release_time = info.sim_time.total_seconds()
        self.applied = True

    def post_update(self, info, ecm):
        if not self.applied or info.paused:
            return
        pose = self.link.world_pose(ecm)
        velocity = self.link.world_linear_velocity(ecm)
        angular = self.link.world_angular_velocity(ecm)
        if pose is None or velocity is None or angular is None:
            raise ValueError('missing own post-release observation')
        now = info.sim_time.total_seconds()
        state = dict(sim_time_s=now, elapsed_s=now-self.release_time,
                     position_world_m=[pose.x(), pose.y(), pose.z()],
                     velocity_world_m_s=[velocity.x(), velocity.y(), velocity.z()],
                     rpy_rad=[pose.rot().roll(), pose.rot().pitch(), pose.rot().yaw()],
                     angular_velocity_rad_s=[angular.x(), angular.y(), angular.z()])
        if not all(math.isfinite(v) for v in [now, *state['position_world_m'],
                *state['velocity_world_m_s'], *state['rpy_rad'], *state['angular_velocity_rad_s']]):
            raise ValueError('nonfinite own airborne state')
        if not self.acknowledged:
            self.save('airborne-initial-state.json', dict(config_sha256=self.identity,
                mission_sha256=self.mission_identity, snapshot=state, impulse_steps=1,
                persistent_velocity_control=False))
            self.acknowledged = True
        if now-self.last_sample >= .1:
            first = self.last_sample < 0
            with (self.output/'airborne-physics.csv').open('x' if first else 'a', newline='') as stream:
                writer = csv.writer(stream)
                if first:
                    writer.writerow(['elapsed_s', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw'])
                writer.writerow([state['elapsed_s'], *state['position_world_m'],
                                 *state['velocity_world_m_s'], *state['rpy_rad']])
            self.last_sample = now


def get_system():
    return CompactAirborneBootstrap()
