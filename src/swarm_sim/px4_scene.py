"""Gazebo scene derived from the same immutable mission as the reference.

Ideal boxes/cylinders have physical collisions. Gold destination annotations
are non-colliding and occupy visibility bit 1 only (excluded by our lidar).
This is scene construction, not a PX4 obstacle-avoidance controller.
"""

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np

from .trajectory2d.obstacles import Obstacle


def display_polyline(points, tolerance=.5):
    """RDP reduction for rendering only, never for flight commands/validation.

    Every input sample lies within tolerance of its replacement segment. This
    concerns the sampled reference, not the continuous ODE or a tracking tube.
    """
    points = np.asarray(points, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or not len(points)
            or not np.all(np.isfinite(points)) or not math.isfinite(tolerance) or tolerance <= 0):
        raise ValueError('finite nonempty XYZ polyline and positive tolerance required')
    keep = {0, len(points)-1}
    pending = [(0, len(points)-1)]
    while pending:
        start, end = pending.pop()
        if end <= start+1:
            continue
        delta = points[end]-points[start]
        residual = points[start+1:end]-points[start]
        fraction = np.clip(residual @ delta / max(float(delta @ delta), 1e-20), 0, 1)
        distance = np.linalg.norm(residual-fraction[:, None]*delta, axis=1)
        split = int(np.argmax(distance))+start+1
        if float(distance.max()) > tolerance:
            keep.add(split)
            pending.extend(((start, split), (split, end)))
    return points[sorted(keep)].copy()


def add_reference_visuals(world, plan, trajectory_only=False, radius_m=.6):
    """Cyan to arrival, muted blue afterwards; no takeoff/landing invented."""
    if not math.isfinite(radius_m) or radius_m<=0:
        raise ValueError('positive finite reference display radius required')
    points = np.asarray(plan['flight_states'], dtype=float)[:, :3]
    phase = np.clip(plan['arrival_time']/plan['dt'], 0, len(points)-1)
    index = int(math.floor(phase))
    arrival = points[index] + (phase-index)*(points[min(index+1, len(points)-1)]-points[index])
    legs = (('arrival', np.vstack((points[:index+1], arrival)), '0.02 0.85 1 1'),
            ('onward', np.vstack((arrival, points[index+1:])), '0.25 0.42 0.60 1'))
    if trajectory_only:
        legs = legs[:1]
    for name, samples, color in legs:
        entity = ET.SubElement(world, 'model', name='swarm_reference_'+name)
        ET.SubElement(entity, 'static').text = 'true'
        link = ET.SubElement(entity, 'link', name='link')
        reduced = display_polyline(samples)
        for number, (a, b) in enumerate(zip(reduced[:-1], reduced[1:])):
            delta = b-a
            length = float(np.linalg.norm(delta))
            if length < 1e-8:
                continue
            visual = ET.SubElement(link, 'visual', name=f'segment_{number}')
            yaw = math.atan2(delta[1], delta[0])
            pitch = math.atan2(math.hypot(delta[0], delta[1]), delta[2])
            ET.SubElement(visual, 'pose').text = ' '.join(map(str, (*(a+b)/2, 0, pitch, yaw)))
            ET.SubElement(visual, 'visibility_flags').text = '1'
            ET.SubElement(visual, 'cast_shadows').text = 'false'
            cylinder = ET.SubElement(ET.SubElement(visual, 'geometry'), 'cylinder')
            ET.SubElement(cylinder, 'radius').text = str(radius_m)
            ET.SubElement(cylinder, 'length').text = str(length)
            material = ET.SubElement(visual, 'material')
            for field in ('ambient', 'diffuse', 'emissive'):
                ET.SubElement(material, field).text = color


def scene_tree(document, template, asset_root='/workspace/examples/gazebo', airborne_start=False, reference_radius_m=.6):
    tree = ET.parse(template)
    world = tree.getroot().find('world')
    # The generated SDF lives in /runresults, not next to the checked-in assets.
    for node in world.findall('.//uri') + world.findall('.//albedo_map'):
        if node.text.startswith('../'):
            node.text = str(Path(asset_root)/node.text[3:])

    def model(name, position, kind, size, color, collision=False, annotation=False):
        entity = ET.SubElement(world, 'model', name=name)
        ET.SubElement(entity, 'static').text = 'true'
        ET.SubElement(entity, 'pose').text = ' '.join(map(str, (*position, 0, 0, 0)))
        link = ET.SubElement(entity, 'link', name='link')
        visual = ET.SubElement(link, 'visual', name='visual')
        if annotation:
            ET.SubElement(visual, 'visibility_flags').text = '1'
            ET.SubElement(visual, 'cast_shadows').text = 'false'
        for node in ([visual, ET.SubElement(link, 'collision', name='collision')] if collision else [visual]):
            shape = ET.SubElement(ET.SubElement(node, 'geometry'), kind)
            if kind == 'box':
                ET.SubElement(shape, 'size').text = ' '.join(map(str, size))
            elif kind == 'cylinder':
                ET.SubElement(shape, 'radius').text = str(size[0])
                ET.SubElement(shape, 'length').text = str(size[1])
            else:
                ET.SubElement(shape, 'radius').text = str(size[0])
        material = ET.SubElement(visual, 'material')
        for field in ('ambient', 'diffuse'):
            ET.SubElement(material, field).text = color
        return entity

    config = document['config']
    for index, record in enumerate(config['obstacles']):
        obstacle = Obstacle(**record)
        half = obstacle.half_extents
        size = tuple(2*v for v in half) if obstacle.kind == 'box' else (half[0], 2*half[2])
        model(f'swarm_obstacle_{index}', obstacle.center,
              'box' if obstacle.kind == 'box' else 'cylinder', size, '0.45 0.48 0.52 1', collision=True)

    # With target_altitude=None the planner defines an XY destination. Its
    # displayed height is the selected aircraft's actual reference arrival Z,
    # exactly as in Matplotlib, not an invented common-altitude requirement.
    plan = document['plans'][0]
    arrival_index = min(len(plan['flight_states'])-1, round(plan['arrival_time']/config['dt']))
    z = config['target_altitude']
    if z is None:
        z = plan['flight_states'][arrival_index][2]
    x, y = config['target']
    radius = config['target_radius']
    model('swarm_target', (x, y, z), 'sphere', (radius,), '1 0.72 0.02 1', annotation=True)
    # Ground beacon makes an airborne destination easy to locate from far away.
    model('swarm_target_ground', (x, y, .1), 'cylinder', (2*radius, .15), '1 0.72 0.02 1', annotation=True)
    if z > 0:
        model('swarm_target_beacon', (x, y, z/2), 'cylinder', (.3, z), '1 0.72 0.02 1', annotation=True)
    from .px4_termination import end_behavior
    add_reference_visuals(world, plan, end_behavior(document) == 'remove-at-target',radius_m=reference_radius_m)
    if airborne_start:
        # PX4 attaches to this existing model. It is first at ground level for
        # ordinary estimator/arming initialization; the plugin is inert until
        # the worker's one-shot release request.
        entity = ET.SubElement(world, 'model', name='rc_cessna_lidar_0')
        initial = plan['flight_states'][0]
        ET.SubElement(entity, 'pose').text = f'{initial[0]} {initial[1]} 0 0 0 {initial[4]}'
        include = ET.SubElement(entity, 'include', merge='true')
        ET.SubElement(include, 'uri').text = '/workspace/examples/gazebo/models/rc_cessna_lidar/model.sdf'
        plugin = ET.SubElement(entity, 'plugin', filename='gz-sim-python-system-loader-system',
                               name='gz::sim::systems::PythonSystemLoader')
        ET.SubElement(plugin, 'module_name').text = 'airborne_bootstrap'
    ET.indent(tree, space='  ')
    return tree
