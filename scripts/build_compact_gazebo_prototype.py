"""Export an explicitly unqualified compact-aircraft Gazebo build fixture.

Only the pinned isolated simulator is accessed. No PX4 process is started and
no tuning values are applied. --check runs schema and 250-step load checks.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import uuid
import xml.etree.ElementTree as ET
from swarm_sim.px4_gazebo import IMAGE,ROOT
from swarm_sim.px4_prototype import MODEL,build_model,child
from swarm_sim.trajectory2d import load_aircraft_profile


def run(arguments,timeout=60):
    return subprocess.run(arguments,check=True,text=True,capture_output=True,timeout=timeout)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args();output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    official=run(['docker','run','--rm','--network','none','--entrypoint','/bin/cat',IMAGE,
                  '/opt/px4-gazebo/share/gz/models/rc_cessna/model.sdf']).stdout
    profile=load_aircraft_profile(ROOT/'examples/aircraft/compact_5kg.json')
    sdf,assumptions=build_model(profile,official,(ROOT/'examples/gazebo/models/rc_cessna_lidar/model.sdf').read_text())
    (output/'model.sdf').write_text(sdf+'\n')
    (output/'LICENSE.PX4').write_text((ROOT/'examples/gazebo/LICENSE.PX4').read_text())
    assumptions['image']=IMAGE
    assumptions['official_template_sha256']=hashlib.sha256(official.encode()).hexdigest()
    assumptions['source_sha256']={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (ROOT/'src/swarm_sim/px4_prototype.py',Path(__file__).resolve(),
                     ROOT/'examples/gazebo/models/rc_cessna_lidar/model.sdf')}
    assumptions['model_sha256']=hashlib.sha256(sdf.encode()).hexdigest()
    assumptions['checks']={'sdf_schema':False,'gazebo_250_step_load':False}
    # A standalone load fixture contains no PX4, waypoints or flight commands.
    world=ET.Element('sdf',version='1.9');node=child(world,'world',name='compact_build_check')
    physics=child(node,'physics',type='ignored');child(physics,'max_step_size',.004)
    for filename,name in [('gz-sim-physics-system','gz::sim::systems::Physics'),
                          ('gz-sim-user-commands-system','gz::sim::systems::UserCommands'),
                          ('gz-sim-scene-broadcaster-system','gz::sim::systems::SceneBroadcaster')]:
        child(node,'plugin',filename=filename,name=name)
    child(node,'gravity','0 0 -9.8')
    # Plane-only ground keeps this build check independent of rendering assets.
    ground=child(node,'model',name='ground');child(ground,'static','true');link=child(ground,'link',name='ground')
    collision=child(link,'collision',name='ground');plane=child(child(collision,'geometry'),'plane')
    child(plane,'normal','0 0 1');child(plane,'size','20 20')
    node.append(deepcopy(ET.fromstring(sdf).find('model')))
    ET.indent(world);ET.ElementTree(world).write(output/'load-check.sdf',encoding='unicode')
    # Proposals only: a future dedicated adapter must apply/read back and qualify them.
    parameters={'status':'proposed_not_applied','FW_AIRSPD_MIN':25.,'FW_AIRSPD_TRIM':100/3.6,
                'FW_AIRSPD_MAX':200/3.6,'FW_T_CLMB_MAX':8.,'FW_T_SINK_MAX':30.}
    (output/'px4-parameters.proposed.json').write_text(json.dumps(parameters,indent=2)+'\n')
    (output/'assumptions.json').write_text(json.dumps(assumptions,indent=2)+'\n')
    if args.check:
        name='swarm-compact-check-'+uuid.uuid4().hex
        base=['docker','run','--rm','--name',name,'--network','none','--mount',f'type=bind,src={output},dst=/prototype,readonly',
              '--env','LIBGL_ALWAYS_SOFTWARE=1','--env','GZ_PARTITION=compact-build-check','--entrypoint','/bin/sh',IMAGE]
        try:
            schema=run(base+['-c','gz sdf -k /prototype/model.sdf'])
            (output/'sdf-check.log').write_text(schema.stdout+schema.stderr)
            if 'Valid.' not in schema.stdout or 'Error Code' in schema.stderr:
                raise RuntimeError('SDF validation did not confirm validity')
            assumptions['checks']['sdf_schema']=True
            smoke=run(base+['-c','gz sim -r -s --iterations 250 /prototype/load-check.sdf'],timeout=60)
            (output/'gazebo-load.log').write_text(smoke.stdout+smoke.stderr)
            errors=('Failed to load','Error Code','does not exist','Could not find','not found')
            if any(message in smoke.stdout+smoke.stderr for message in errors):
                raise RuntimeError('Gazebo reported a model/plugin loading error; inspect gazebo-load.log')
            assumptions['checks']['gazebo_250_step_load']=True
        finally:
            subprocess.run(['docker','rm','--force',name],capture_output=True,timeout=15)
            (output/'assumptions.json').write_text(json.dumps(assumptions,indent=2)+'\n')
    print(output)
    print('Build fixture only: PX4 flight, drag matching and airbrake remain unqualified.')


if __name__=='__main__':main()
