"""Isolated compact PX4 qualification: startup, optional actuators and flight.

The default is unarmed startup. Explicit --actuator-test drives only the owned
simulated effectors; --flight-test additionally attempts ordinary takeoff,
circuit and landing after all readiness/identity/actuator/mission gates pass.
"""
import argparse
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time
import uuid
import io
import xml.etree.ElementTree as ET
import numpy as np
from swarm_sim.px4_gazebo import IMAGE,ROOT,PX4_HASH
from swarm_sim.px4_prototype import build_model
from swarm_sim.gazebo_force_model import ForceModel
from swarm_sim.gazebo_control_model import ControlModel,verify_control_plugin
from swarm_sim.trajectory2d import load_aircraft_profile


def worker(actuator_test=False,flight_test=False):
    if not Path('/.dockerenv').exists() or os.environ.get('SWARM_COMPACT_PX4_STARTUP')!='1' or os.environ.get('PX4_SIM_MODEL')!='gz_compact_5kg_prototype':
        raise ValueError('managed isolated compact SITL startup only')
    from mavsdk import Mavsdk,Configuration,ComponentType
    from mavsdk.plugins.telemetry import Telemetry
    from mavsdk.plugins.param import Param
    output=Path('/runresults');report=dict(passed=False,armed=False,flight_qualified=False)
    force=ForceModel(load_aircraft_profile(ROOT/'examples/aircraft/compact_5kg.json'))
    controls=ControlModel.load(ROOT/'examples/gazebo/compact_physics/control_model.json',force)
    manifest=json.loads((output/'assumptions.json').read_text())
    if manifest['force_model']['binding_sha256']!=force.record()['binding_sha256'] or manifest['control_model']['sha256']!=controls.record()['sha256']:
        raise ValueError('own physical/control binding changed after fixture creation')
    sdk=Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
    try:
        if importlib.metadata.version('mavsdk')!='4.1.0':raise ValueError('verified MAVSDK 4.1.0 required')
        sdk.add_any_connection('udpin://127.0.0.1:14540');system=sdk.first_autopilot(60.)
        if system is None:raise TimeoutError('no local PX4 autopilot')
        version=subprocess.run(['/opt/px4-gazebo/bin/px4-ver','all'],check=True,capture_output=True,text=True,timeout=10).stdout
        if PX4_HASH not in version or 'PX4_SITL' not in version:raise ValueError('unexpected PX4 SITL identity')
        report['px4_identity']=version;telemetry=Telemetry(system);params=Param(system);latest={}
        telemetry.subscribe_health(lambda value,_:latest.update(health=value))
        telemetry.subscribe_home(lambda value,_:latest.update(home=value))
        telemetry.subscribe_armed(lambda value,_:latest.update(armed=value))
        deadline=time.monotonic()+90
        while time.monotonic()<deadline:
            health=latest.get('health')
            if latest.get('armed'):raise RuntimeError('startup fixture unexpectedly armed')
            if health is not None and health.is_armable and health.is_global_position_ok and health.is_home_position_ok and 'home' in latest and latest.get('armed') is False:break
            time.sleep(.2)
        else:raise TimeoutError('unmodified PX4 readiness gates did not pass')
        if params.get_param_int('SYS_AUTOSTART')!=4999:raise ValueError('unexpected compact experiment airframe identity')
        report.update(passed=True,health=str(latest['health']),
            airspeed_parameters={key:params.get_param_float(key) for key in ('FW_AIRSPD_MIN','FW_AIRSPD_TRIM','FW_AIRSPD_MAX')},
            scope='unarmed startup only; derived compact airframe defaults, no flight qualification')
        if actuator_test:
            force=ForceModel(load_aircraft_profile(ROOT/'examples/aircraft/compact_5kg.json'))
            controls=ControlModel.load(ROOT/'examples/gazebo/compact_physics/control_model.json',force)
            bridge={}
            for index in range(1,6):
                travel=controls.surface_limit_rad if index<5 else controls.brake_limit_rad
                for boundary,wanted in [('MIN',-travel),('MAX',travel)]:
                    key=f'SIM_GZ_SV_{boundary}A{index}';effective=params.get_param_float(key)
                    if abs(effective-math.degrees(wanted))>1e-4:raise RuntimeError('own servo angle readback failed')
                    bridge[key]=dict(expected_deg=math.degrees(wanted),effective_deg=effective)
            report['own_simulated_servo_travel']=bridge
            def observed():
                text=(output/'actuators.csv').read_text().rsplit('\n',1)[0]
                data=np.genfromtxt(io.StringIO(text),delimiter=',',names=True)
                if data.size<10:raise RuntimeError('own actuator observations unavailable')
                return data
            checks=[]
            for selector,index,value,field,expected in [('-s',1,.2,'aileron',-.1),('-s',2,.2,'aileron',.1),
                ('-s',3,.15,'elevator',.15),('-s',4,.15,'rudder',.15),('-s',5,.35,'brake',.35),
                ('-m',1,.2,'rotor_joint_rad_s',-20.)]:
                if latest.get('armed') is not False:raise RuntimeError('actuator experiment requires unarmed simulated state')
                before=observed()['time'][-1]
                deadline=time.monotonic()+12
                while True:
                    if latest.get('armed') is not False:raise RuntimeError('actuator experiment unexpectedly armed')
                    result=subprocess.run(['/opt/px4-gazebo/bin/px4-actuator_test','set',selector,str(index),'-v',str(value),'-t','2'],check=True,capture_output=True,text=True,timeout=10)
                    time.sleep(.2);data=observed()
                    if data['time'][-1]>before+1.6:break
                    if time.monotonic()>deadline:raise TimeoutError('actuator response clock did not advance')
                window=data[(data['time']>before+.75)&(data['time']<before+1.5)]
                if window.size<20:raise RuntimeError('insufficient own actuator response samples')
                measured=float(np.median(window[field]));tolerance=1. if selector=='-m' else .03
                checks.append(dict(selector=selector,index=index,command=value,observed_field=field,observed=measured,
                    expected=expected,tolerance=tolerance,passed=abs(measured-expected)<=tolerance,native_status=result.stdout+result.stderr))
                subprocess.run(['/opt/px4-gazebo/bin/px4-actuator_test','set',selector,str(index),'-v','0','-t','1'],check=True,capture_output=True,text=True,timeout=10)
                report['actuator_checks']=checks
            report.update(actuator_checks=checks,actuator_chain_verified=all(c['passed'] for c in checks))
            report['passed']=report['passed'] and report['actuator_chain_verified']
        if flight_test:
            if not report.get('actuator_chain_verified'):raise RuntimeError('own PX4 actuator chain must pass before a flight trial')
            from swarm_sim.px4_compact_trial import trial
            flight=trial(system,telemetry,params,latest,output)
            report.update(flight_trial=flight,passed=report['passed'] and flight['passed'],armed=bool(latest.get('armed')))
            report['scope']='startup and actuator gates followed by an opt-in native flight trial; see individual outcomes'
    except BaseException as error:
        report.update(passed=False,error=repr(error))
        if not isinstance(error,Exception):raise
    finally:
        try:sdk.destroy()
        except Exception as error:report.update(passed=False,cleanup_error=repr(error))
        (output/'startup-report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def qualify(output,library,actuator_test=False,flight_test=False):
    if importlib.metadata.version('mavsdk')!='4.1.0':raise ValueError('verified MAVSDK 4.1.0 required')
    force=ForceModel(load_aircraft_profile(ROOT/'examples/aircraft/compact_5kg.json'))
    controls=ControlModel.load(ROOT/'examples/gazebo/compact_physics/control_model.json',force)
    control_api=verify_control_plugin(library)
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    official=subprocess.run(['docker','run','--rm','--network','none','--entrypoint','cat',IMAGE,
        '/opt/px4-gazebo/share/gz/models/rc_cessna/model.sdf'],check=True,capture_output=True,text=True).stdout
    sdf,assumptions=build_model(force.profile,official,(ROOT/'examples/gazebo/models/rc_cessna_lidar/model.sdf').read_text(),matched_polar=True,control_model=controls)
    root=ET.fromstring(sdf)
    plugin=root.find("model/plugin[@name='swarm::CompactPolar']")
    ET.SubElement(plugin,'bench_csv').text='/runresults/actuators.csv'
    sdf=ET.tostring(root,encoding='unicode')
    model=output/'models/compact_5kg_prototype';model.mkdir(parents=True);(model/'model.sdf').write_text(sdf)
    shutil.copyfile(library,output/'libcompact_polar.so')
    tree=ET.parse(ROOT/'examples/gazebo/worlds/default.sdf')
    tree.find('world/physics/max_step_size').text='0.001'
    tree.find('world/physics/real_time_update_rate').text='1000'
    tree.find('world/gravity').text='0 0 -9.81'
    for node in tree.findall('.//uri')+tree.findall('.//albedo_map'):
        if node.text.startswith('..'):node.text='/workspace/examples/gazebo/'+str(Path('worlds')/node.text)
    tree.write(output/'world.sdf',encoding='unicode')
    assumptions.update(image=IMAGE,control_api=control_api,plugin_sha256=hashlib.sha256(library.read_bytes()).hexdigest())
    (output/'LICENSE.PX4').write_text((ROOT/'examples/gazebo/LICENSE.PX4').read_text())
    assumptions['source_sha256']={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [Path(__file__).resolve(),ROOT/'src/swarm_sim/px4_compact_trial.py',ROOT/'src/swarm_sim/px4_prototype.py',
                     ROOT/'src/swarm_sim/gazebo_control_model.py',ROOT/'src/swarm_sim/gazebo_force_model.py',
                     *sorted((ROOT/'examples/gazebo/compact_physics').glob('*'))] if path.is_file()}
    inherited=subprocess.run(['docker','run','--rm','--network','none','--entrypoint','cat',IMAGE,
        '/opt/px4-gazebo/etc/init.d-posix/airframes/4003_gz_rc_cessna'],check=True,capture_output=True,text=True).stdout
    defaults={}
    for index in range(1,6):
        travel=controls.surface_limit_rad if index<5 else controls.brake_limit_rad
        defaults[f'SIM_GZ_SV_MINA{index}']=-math.degrees(travel)
        defaults[f'SIM_GZ_SV_MAXA{index}']=math.degrees(travel)
    defaults.update(FW_AIRSPD_MIN=force.profile.limits.min_speed,FW_AIRSPD_TRIM=force.cruise_test_speed(),FW_AIRSPD_MAX=force.profile.limits.max_speed)
    airframe=output/'4999_gz_compact_prototype'
    airframe.write_text(inherited+'\n# Explicit compact simulation defaults, applied before bridge initialization.\n'+
                       ''.join(f'param set-default {key} {value:.15g}\n' for key,value in defaults.items()))
    assumptions.update(airframe_id=4999,inherited_airframe_id=4003,airframe_defaults=defaults,
        airframe_sha256=hashlib.sha256(airframe.read_bytes()).hexdigest(),inherited_airframe_sha256=hashlib.sha256(inherited.encode()).hexdigest())
    (output/'assumptions.json').write_text(json.dumps(assumptions,indent=2)+'\n')
    name='swarm-compact-startup-'+uuid.uuid4().hex
    args=['docker','run','--detach','--name',name,'--network','none',
        '--mount',f'type=bind,src={ROOT},dst=/workspace,readonly','--mount',f'type=bind,src={output},dst=/runresults',
        '--mount',f'type=bind,src={airframe},dst=/opt/px4-gazebo/etc/init.d-posix/airframes/4999_gz_compact_prototype,readonly',
        '--env','SWARM_COMPACT_PX4_STARTUP=1','--env','PX4_SYS_AUTOSTART=4999','--env','PX4_SIM_MODEL=gz_compact_5kg_prototype',
        '--env','PX4_GZ_MODELS=/runresults/models','--env','PX4_GZ_NO_FOLLOW=1','--env','LIBGL_ALWAYS_SOFTWARE=1',
        '--env','GZ_SIM_SYSTEM_PLUGIN_PATH=/runresults:/opt/px4-gazebo/lib/gz/plugins','--env','GZ_PARTITION='+name,
        '--entrypoint','/bin/sh',IMAGE,'/workspace/examples/gazebo/compact_physics/start_px4.sh']
    try:
        subprocess.run(args,check=True,capture_output=True,text=True,timeout=30)
        metadata=json.loads(subprocess.run(['docker','inspect',name],check=True,capture_output=True,text=True).stdout)[0]
        if metadata['HostConfig']['NetworkMode']!='none':raise RuntimeError('non-isolated simulator')
        command=['docker','exec','--env','PYTHONPATH=/workspace/src','--env','PYTHONDONTWRITEBYTECODE=1',name,
            '/workspace/.venv/bin/python','/workspace/scripts/qualify_compact_px4_startup.py','--worker']
        if actuator_test:command+=['--actuator-test']
        if flight_test:command+=['--flight-test']
        result=subprocess.run(command,capture_output=True,text=True,timeout=720 if flight_test else 180)
        (output/'worker.log').write_text(result.stdout+result.stderr)
        report=json.loads((output/'startup-report.json').read_text())
        if result.returncode!=0 and report.get('passed'):raise RuntimeError('startup worker status contradicts its report')
        return report
    except BaseException as error:
        failure=dict(passed=False,flight_qualified=False,error=repr(error),scope='host qualification failed; see worker/native diagnostics')
        (output/'host-failure.json').write_text(json.dumps(failure,indent=2)+'\n')
        if not isinstance(error,Exception):raise
        return failure
    finally:
        logs=subprocess.run(['docker','logs',name],capture_output=True,text=True,timeout=15)
        (output/'px4-gazebo.log').write_text(logs.stdout+logs.stderr)
        copied=subprocess.run(['docker','cp',name+':/root/.local/share/px4/rootfs/fs/log',str(output/'px4-logs')],capture_output=True,text=True,timeout=30)
        (output/'runtime-end.json').write_text(json.dumps(dict(native_logs_copied=copied.returncode==0,
            runtime_ended_by='owned simulation container shutdown; no in-air disarm command'),indent=2)+'\n')
        subprocess.run(['docker','rm','--force',name],capture_output=True,timeout=15)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path);parser.add_argument('--library',type=Path);parser.add_argument('--worker',action='store_true')
    parser.add_argument('--actuator-test',action='store_true',help='Unarmed own servo/motor response checks in isolated simulation')
    parser.add_argument('--flight-test',action='store_true',help='Experimental native takeoff/circuit/landing after all actuator and native readiness gates')
    args=parser.parse_args()
    if not args.worker and (not args.output or not args.library):parser.error('output and library required')
    r=worker(args.actuator_test or args.flight_test,args.flight_test) if args.worker else qualify(args.output,args.library,args.actuator_test or args.flight_test,args.flight_test)
    print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
