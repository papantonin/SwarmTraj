"""Isolated compact-airframe cold-takeoff experiment, not a plan executor.

Only called inside the owned simulation. No caller-selectable connection,
readiness bypass or forced arming. The landing mission stays mandatory.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
from .px4_gazebo import Measurements,mission_is_ready
from .px4_reference import LocalOrigin
from .trajectory2d import load_aircraft_profile


def own_physics_velocity(path,stamp):
    """Bounded observer read, independent of PX4 control and sensor noise."""
    with path.open('rb') as stream:
        stream.seek(0,2);end=stream.tell();begin=max(0,end-32768);stream.seek(begin);data=stream.read()
    lines=data.split(b'\n')
    if begin:lines=lines[1:]
    rows=[line.split(b',') for line in lines[:-1] if line and not line.startswith(b'time,')]
    if not rows:raise RuntimeError('own physics velocity unavailable')
    values=[float(x) for x in rows[-1]]
    if len(values)<7 or not all(math.isfinite(v) for v in values) or abs(values[0]-stamp)>.25:
        raise RuntimeError('own physics observation is invalid or stale')
    return values[1:4]


def trial(system,telemetry,params,latest,output):
    if not Path('/.dockerenv').exists() or os.environ.get('SWARM_COMPACT_PX4_STARTUP')!='1' or params.get_param_int('SYS_AUTOSTART')!=4999:
        raise ValueError('owned compact SITL flight qualification only')
    from mavsdk.plugins.action import Action
    from mavsdk.plugins.mission_raw import MissionRaw,MissionItem
    profile=load_aircraft_profile('/workspace/examples/aircraft/compact_5kg.json')
    action=Action(system);mission=MissionRaw(system);observations=Measurements()
    report=dict(passed=False,completed=False,point_mass_reference_used=False,spatial_tracking_qualified=False,
                scope='one cold takeoff, native spatial circuit and landing; no swarm or reference safety transfer')
    try:
        for name in ('ground_truth','attitude_euler','fixedwing_metrics','in_air','armed','flight_mode','status_text'):
            getattr(telemetry,'subscribe_'+name)(observations.callback(name))
        telemetry.set_rate_ground_truth(10.);telemetry.set_rate_fixedwing_metrics(10.);telemetry.set_rate_attitude_euler(10.)
        own_defaults={'FW_R_LIM':15.,'FW_T_CLMB_MAX':.7*profile.performance.max_climb_rate,
                      'FW_T_SINK_MAX':3.5,'FW_AIRSPD_STALL':profile.flight.stall_speed,
                      'FW_THR_MAX':.75,'FW_THR_TRIM':.53,'TRIM_PITCH':-.005,'TRIM_ROLL':-.034,
                      'RWTO_MAX_THR':.7,'RWTO_RAMP_TIME':4.,'RWTO_ROT_AIRSPD':27.,'RWTO_ROT_TIME':3.,
                      'FW_TKO_AIRSPD':profile.airspeed_for('cruise'),
                      'FW_PR_P':.2,'FW_PR_I':.1,'FW_PR_FF':.06,
                      'FW_RR_P':.1,'FW_RR_I':.2,'FW_RR_FF':.06,
                      'FW_YR_P':.1,'FW_YR_I':.05,'FW_YR_FF':.03,
                      'FW_R_RMAX':30.,'FW_P_RMAX_POS':6.,'FW_P_RMAX_NEG':6.,'FW_LND_AIRSPD':27.}
        effective={}
        for key,wanted in own_defaults.items():
            before=params.get_param_float(key);params.set_param_float(key,wanted);after=params.get_param_float(key)
            if abs(after-wanted)>1e-5:raise ValueError('own controller parameter readback failed')
            effective[key]=dict(before=before,effective=after)
        report['controller_parameters']=effective
        home=latest['home'];origin=LocalOrigin(home.latitude_deg,home.longitude_deg,home.absolute_altitude_m)
        items=[]
        def add(command,point=None,args=(0.,0.,0.,float('nan'))):
            lat,lon,alt=origin.from_enu(point) if point is not None else (0,0,origin.altitude_m)
            items.append(dict(seq=len(items),frame=6 if point is not None else 2,command=command,current=int(not items),autocontinue=1,
                param1=args[0],param2=args[1],param3=args[2],param4=args[3],x=round(lat*1e7),y=round(lon*1e7),z=alt-origin.altitude_m,mission_type=0))
        add(22,(500.,0.,80.),(10.,0.,0.,float('nan')))
        add(178,args=(0.,profile.airspeed_for('cruise'),-1.,0.))
        for point in [(1000.,0.,80.),(1500.,800.,80.),(300.,1200.,80.),(-500.,300.,80.)]:add(16,point,(0.,15.,0.,float('nan')))
        add(189);add(31,(-800.,0.,60.),(1.,400.,0.,1.));add(21,(0.,0.,0.),(0.,0.,0.,90.))
        exported=[{k:None if isinstance(v,float) and math.isnan(v) else v for k,v in item.items()} for item in items]
        (output/'native-flight-mission.json').write_text(json.dumps(exported,indent=2)+'\n')
        report['native_mission_sha256']=hashlib.sha256(json.dumps(exported,sort_keys=True).encode()).hexdigest()
        mission.upload_mission([MissionItem(**item) for item in items])
        mission.subscribe_mission_progress(observations.callback('mission_progress'))
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            result=subprocess.run(['/opt/px4-gazebo/bin/px4-listener','mission_result','-n','1'],capture_output=True,text=True,timeout=5)
            if result.returncode==0 and mission_is_ready(result.stdout,len(items)):break
            time.sleep(.2)
        else:raise RuntimeError('native mission feasibility did not pass; no arm command sent')
        if not latest['health'].is_armable or latest.get('armed') is not False:raise RuntimeError('not ready for ordinary simulated arming')
        action.arm();deadline=time.monotonic()+5
        while latest.get('armed') is not True:
            if time.monotonic()>deadline:raise RuntimeError('ordinary arming not observed')
            time.sleep(.05)
        mission.start_mission();start=time.monotonic();cruise_hold=0.;steady_since=None;last_time=None;ever_airborne=False
        while time.monotonic()-start<480:
            if observations.rows:
                row=observations.rows[-1];height=row['absolute_altitude_m']-home.absolute_altitude_m
                stamp=row['sim_time_s']
                if stamp==last_time:time.sleep(.05);continue
                if observations.latest.get('in_air'):ever_airborne=True
                roll=row.get('roll_deg');pitch=row.get('pitch_deg')
                if time.monotonic()-start>1 and ((roll is not None and abs(roll)>profile.limits.max_bank_deg) or (pitch is not None and abs(pitch)>80)):
                    raise RuntimeError('attitude outside the qualification domain, including ground transition')
                if time.monotonic()-start>2 and latest.get('armed') is False and not ever_airborne:
                    raise RuntimeError('PX4 disarmed before takeoff; inspect native failsafe evidence')
                if height>20:
                    velocity=own_physics_velocity(output/'actuators.csv',stamp)
                    speed=math.sqrt(sum(v*v for v in velocity));roll=row.get('roll_deg')
                    if not profile.limits.min_speed<=speed<=profile.limits.max_speed:raise RuntimeError('true airspeed outside own envelope above 20 m (zero-wind fixture)')
                    if roll is not None and abs(roll)>profile.limits.max_bank_deg:raise RuntimeError('bank outside own envelope')
                    steady=speed is not None and abs(speed-profile.airspeed_for('cruise'))<=2 and 60<=height<=100
                    if steady:
                        if steady_since is None or last_time is None or stamp-last_time>.25:steady_since=stamp
                        cruise_hold=max(cruise_hold,stamp-steady_since)
                    else:steady_since=None
                else:steady_since=None
                last_time=stamp
                (output/'flight-status.json').write_text(json.dumps(dict(elapsed_wall_s=time.monotonic()-start,last_observation=row,ever_airborne=ever_airborne,cruise_hold_s=cruise_hold),indent=2)+'\n')
                if ever_airborne and latest.get('armed') is False:
                    progress=observations.latest.get('mission_progress')
                    if observations.latest.get('in_air') is not False or height>2 or progress is None or progress.current<progress.total-1:
                        raise RuntimeError('PX4 disarmed before verified native mission landing')
                    report.update(completed=True,passed=cruise_hold>=5);break
            time.sleep(.1)
        else:raise TimeoutError('native circuit/landing did not finish within the qualification budget')
        report.update(maximum_cruise_hold_s=cruise_hold,ever_airborne=ever_airborne)
    except Exception as error:report.update(passed=False,error=repr(error))
    finally:
        (output/'flight-observations.json').write_text(json.dumps(observations.rows,indent=2,allow_nan=False)+'\n')
        report['status_text']=observations.status
        (output/'flight-report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
