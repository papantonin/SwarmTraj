"""Build-only hypothetical compact Gazebo fixture; never a production adapter.

The pinned image supplies sensor/plugin conventions. Geometry, inertias and
control derivatives below are declared assumptions, not measured airframe data.
"""
from copy import deepcopy
import math
import xml.etree.ElementTree as ET
from .gazebo_force_model import ForceModel,ForceAssumptions

MODEL='compact_5kg_prototype'


def child(parent,tag,text=None,**attributes):
    node=ET.SubElement(parent,tag,attributes)
    if text is not None:node.text=str(text)
    return node


def build_model(profile,official_sdf,lidar_sdf,*,matched_polar=False,force_assumptions=None):
    if force_assumptions is not None and not matched_polar:
        raise ValueError('force assumptions require the explicit experimental polar component')
    force_model=ForceModel(profile,force_assumptions if force_assumptions is not None else ForceAssumptions()) if matched_polar else None
    if profile.airframe is None or profile.bounding_radius_m != .3 or profile.airframe.mass_kg != 5:
        raise ValueError('this hypothetical fixture is restricted to the compact 5 kg, 0.30 m profile')
    source=ET.fromstring(official_sdf).find('model')
    if source is None or source.get('name')!='rc_cessna':
        raise ValueError('expected the pinned official rc_cessna model')
    root=ET.Element('sdf',version='1.9');model=child(root,'model',name=MODEL)
    child(model,'pose','0 0 0.17 0 0 0')
    bounds=[]
    def link(name,mass,position=(0,0,0),radii=(.02,.02,.02)):
        node=child(model,'link',name=name);child(node,'pose',' '.join(map(str,(*position,0,0,0))))
        inertial=child(node,'inertial');child(inertial,'mass',mass)
        inertia=child(inertial,'inertia')
        for axis,other in [('ixx',(1,2)),('iyy',(0,2)),('izz',(0,1))]:
            child(inertia,axis,mass*sum(radii[j]**2 for j in other)/5)
        for axis in ('ixy','ixz','iyz'):child(inertia,axis,0)
        # Rotation-independent outer bound, valid also during servo rotation.
        bound=math.sqrt(sum(v*v for v in position))+max(radii)
        if bound>profile.bounding_radius_m+1e-12:raise ValueError('prototype component exceeds the enclosing sphere')
        bounds.append(dict(link=name,radial_bound_m=bound))
        for kind in ('visual','collision'):
            shape=child(node,kind,name=name+'_'+kind);geometry=child(shape,'geometry')
            ellipsoid=child(geometry,'ellipsoid');child(ellipsoid,'radii',' '.join(map(str,radii)))
            if kind=='visual':
                material=child(shape,'material');child(material,'ambient','0.15 0.4 0.8 1');child(material,'diffuse','0.15 0.4 0.8 1')
        return node
    base=link('base_link',4.942,radii=(.22,.05,.045))
    # Elliptical planform area pi*a*b = 0.12 m², full span 0.56 m.
    wing_radii=(profile.airframe.wing_area_m2/(math.pi*.28),.28,.007)
    bounds.append(dict(link='wing',radial_bound_m=max(wing_radii)))
    if max(wing_radii)>.3:raise ValueError('wing does not fit the compact prototype')
    for kind in ('visual','collision'):
        shape=child(base,kind,name='wing_'+kind);geometry=child(shape,'geometry')
        ellipsoid=child(geometry,'ellipsoid');child(ellipsoid,'radii',' '.join(map(str,wing_radii)))
    for sensor in source.find("link[@name='base_link']").findall('sensor'):
        base.append(deepcopy(sensor))
    rotor=link('rotor_puller',.02,(.235,0,0),(.03,.03,.005))
    rotor.find('pose').text=f'.235 0 0 0 {math.pi/2} 0'
    joint=child(model,'joint',name='rotor_puller_joint',type='revolute')
    child(joint,'parent','base_link');child(joint,'child','rotor_puller')
    axis=child(joint,'axis');child(axis,'xyz','0 0 1');limit=child(axis,'limit');child(limit,'lower',-1e16);child(limit,'upper',1e16)
    controls=[('left_elevon','servo_0',(0,.18,0),'0 1 0'),
              ('right_elevon','servo_1',(0,-.18,0),'0 1 0'),
              ('elevator','servo_2',(-.18,0,0),'0 1 0'),
              ('rudder','rudder_joint',(-.18,0,0),'0 0 1')]
    for name,joint_name,position,direction in controls:
        link(name,.004,position,(.025,.025,.004))
        joint=child(model,'joint',name=joint_name,type='revolute')
        child(joint,'parent','base_link');child(joint,'child',name)
        axis=child(joint,'axis');child(axis,'xyz',direction)
        limit=child(axis,'limit');child(limit,'lower',-.78);child(limit,'upper',.78)
        dynamics=child(axis,'dynamics');child(dynamics,'damping',.01)
    for k,position in enumerate(((.12,0,-.14),(-.1,.1,-.14),(-.1,-.1,-.14))):
        name=f'gear_{k}';link(name,.006,position,(.02,.02,.02))
        joint=child(model,'joint',name=name+'_joint',type='fixed')
        child(joint,'parent','base_link');child(joint,'child',name)
    lidar=ET.fromstring(lidar_sdf).find('model')
    lidar_link=deepcopy(lidar.find("link[@name='lidar_sensor_link']"))
    lidar_link.find('inertial/mass').text='0.004'
    model.append(lidar_link);model.append(deepcopy(lidar.find("joint[@name='lidar_sensor_joint']")))
    bounds.append(dict(link='lidar_sensor_link',radial_bound_m=.05))
    areas=[.045,.045,.015,.015,.012,.006]
    centers=[(0,.14,0),(0,-.14,0),(0,.07,0),(0,-.07,0),(-.18,0,0),(-.18,0,0)]
    aero=[p for p in source.findall('plugin') if p.get('name')=='gz::sim::systems::LiftDrag']
    if len(aero)!=6:raise ValueError('pinned LiftDrag template changed')
    for plugin,area,center in zip(aero,areas,centers):
        plugin=deepcopy(plugin)
        plugin.find('area').text=str(area)
        plugin.find('cp').text=' '.join(map(str,center))
        plugin.find('air_density').text=str(profile.airframe.air_density_kg_m3)
        if plugin.findtext('control_joint_name') in ('left_flap_joint','right_flap_joint'):
            for tag in ('control_joint_name','control_joint_rad_to_cl'):
                element=plugin.find(tag)
                if element is not None:plugin.remove(element)
        # Unmeasured derivatives deliberately remain distinct from CD0+k*CL².
        model.append(plugin)
    for plugin in source.findall('plugin'):
        if plugin.get('name')=='gz::sim::systems::JointPositionController':model.append(deepcopy(plugin))
        elif plugin.get('name')=='gz::sim::systems::MulticopterMotorModel':
            plugin=deepcopy(plugin)
            plugin.find('motorConstant').text=str(profile.airframe.max_thrust_n/1000**2)
            model.append(plugin)
    if matched_polar:
        for plugin in list(model.findall('plugin')):
            if plugin.get('name')=='gz::sim::systems::LiftDrag':model.remove(plugin)
        plugin=child(model,'plugin',filename='libcompact_polar.so',name='swarm::CompactPolar')
        for key,value in force_model.plugin_values().items():child(plugin,key,value)
    child(model,'static','false')
    masses=[float(p.text) for p in model.findall('link/inertial/mass')]
    if not math.isclose(sum(masses),5,abs_tol=1e-12):raise ValueError('prototype mass accounting failed')
    ET.indent(root)
    assumptions=dict(model=MODEL,profile=profile.record(),status='hypothetical_build_only',
        total_mass_kg=sum(masses),component_radial_bounds=bounds,
        wing_span_m=.56,wing_planform_m2=profile.airframe.wing_area_m2,
        inertia_model='illustrative solid ellipsoids; base inertia is not a measured mass distribution',
        aerodynamics='six inherited LiftDrag derivatives with hypothetical areas/centers; not the planner polar',
        thrust=f'{profile.airframe.max_thrust_n:g} N nominal static maximum; inherited rotor dynamics, no calibrated propeller curve',
        ideal_airbrake_implemented=False,px4_flight_qualified=False,
        limitations=['no matched planner drag polar or 20 N deployable airbrake',
                     'no trim, longitudinal/lateral stability or PX4 controller tuning qualification',
                     'no transfer of point-mass timing, tracking or separation validation'])
    if matched_polar:
        assumptions.update(force_model=force_model.record(),aerodynamics='C++ CD0+k*CL² force kernel; assumed lift slope and clipped CL',
            ideal_airbrake_implemented=True,airbrake_control='fixed bench fraction only; no PX4 actuator binding',
            wind_supported=False,
            limitations=['no aerodynamic moments, sideslip/post-stall or trim qualification',
                'brake and elevator fractions are immutable bench inputs, not PX4 controls',
                'no transfer of point-mass timing, tracking or separation validation'])
    return ET.tostring(root,encoding='unicode'),assumptions
