"""Own physical joint-to-wrench input qualification; no PX4 or flight."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import subprocess
import uuid
import xml.etree.ElementTree as ET
import numpy as np
from swarm_sim.px4_gazebo import IMAGE,ROOT
from swarm_sim.px4_prototype import build_model,child
from swarm_sim.gazebo_force_model import ForceModel
from swarm_sim.gazebo_control_model import ControlModel,verify_control_plugin
from swarm_sim.trajectory2d import load_aircraft_profile


def qualify(output,library):
    force=ForceModel(load_aircraft_profile(ROOT/'examples/aircraft/compact_5kg.json'))
    control=ControlModel.load(ROOT/'examples/gazebo/compact_physics/control_model.json',force)
    verify_control_plugin(library)
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    official=subprocess.run(['docker','run','--rm','--network','none','--entrypoint','cat',IMAGE,
        '/opt/px4-gazebo/share/gz/models/rc_cessna/model.sdf'],check=True,capture_output=True,text=True).stdout
    sdf,assumptions=build_model(force.profile,official,(ROOT/'examples/gazebo/models/rc_cessna_lidar/model.sdf').read_text(),matched_polar=True,control_model=control)
    root=ET.fromstring(sdf);plugin=root.find("model/plugin[@name='swarm::CompactPolar']");child(plugin,'bench_csv','/bench/states.csv')
    world=ET.Element('sdf',version='1.9');node=child(world,'world',name='actuator_bench')
    physics=child(node,'physics',type='ignored');child(physics,'max_step_size',.001);child(physics,'real_time_factor',1)
    for filename,name in [('gz-sim-physics-system','gz::sim::systems::Physics'),('gz-sim-user-commands-system','gz::sim::systems::UserCommands')]:child(node,'plugin',filename=filename,name=name)
    child(node,'gravity','0 0 0');node.append(root.find('model'))
    ET.indent(world);ET.ElementTree(world).write(output/'world.sdf',encoding='unicode')
    shutil.copyfile(library,output/'libcompact_polar.so')
    shutil.copyfile(ROOT/'examples/gazebo/compact_physics/actuator_bench.py',output/'actuator_bench.py')
    report=dict(**assumptions,passed=False,px4_allocator_verified=False,control_api=1,
        plugin_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
          for p in [Path(__file__).resolve(),ROOT/'src/swarm_sim/gazebo_control_model.py',ROOT/'src/swarm_sim/px4_prototype.py',
                    *sorted((ROOT/'examples/gazebo/compact_physics').glob('*'))] if p.is_file()})
    name='swarm-actuator-bench-'+uuid.uuid4().hex
    command=['docker','run','--rm','--name',name,'--network','none','--mount',f'type=bind,src={output},dst=/bench',
        '--env','GZ_SIM_SYSTEM_PLUGIN_PATH=/bench','--env','GZ_PARTITION='+name,'--env','SWARM_COMPACT_ACTUATOR_BENCH=1',
        '--entrypoint','/usr/bin/python3',IMAGE,'/bench/actuator_bench.py']
    try:
        result=subprocess.run(command,capture_output=True,text=True,timeout=45)
        (output/'container.log').write_text(result.stdout+result.stderr);result.check_returncode()
        data=np.genfromtxt(output/'states.csv',delimiter=',',names=True)
        if data.size<5900 or data['time'][-1]<5.99:raise RuntimeError('incomplete actuator telemetry')
        if not np.all(np.isfinite(np.column_stack([data[k] for k in data.dtype.names]))):raise RuntimeError('nonfinite actuator telemetry')
        checks=[]
        for key,wanted in [('aileron',.4),('elevator',.3),('rudder',-.3),('brake',.5)]:
            got=float(np.median(data[key][-500:]));checks.append(dict(control=key,observed=got,expected=wanted,passed=abs(got-wanted)<.03))
        report.update(checks=checks,passed=all(row['passed'] for row in checks))
    except Exception as error:report['error']=repr(error)
    finally:
        subprocess.run(['docker','rm','--force',name],capture_output=True,timeout=15)
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--library',type=Path,required=True)
    args=parser.parse_args();r=qualify(args.output,args.library);print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
