"""Isolated compact-airframe flight experiment, not a plan executor.

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
from .px4_gazebo import Measurements,mission_is_ready,airborne_estimate_check,wait_for,initialize_camera_follow
from .px4_reference import LocalOrigin
from .trajectory2d import load_aircraft_profile


def own_physics_row(path,stamp):
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
    return values


def own_physics_velocity(path,stamp):
    return own_physics_row(path,stamp)[1:4]


def own_physics_brake(path,stamp):
    values=own_physics_row(path,stamp)
    if len(values)!=18 or not 0<=values[16]<=1:
        raise RuntimeError('invalid own measured brake feedback')
    return values[16]


def brake_hold(since,last,stamp,value):
    if not math.isfinite(value) or not math.isfinite(stamp) or not 0<=value<=1:
        raise ValueError('finite measured brake fraction and clock required')
    if abs(value-.4)>.03:return None
    return stamp if since is None or last is None or not 0<stamp-last<=.25 else since


def initial_state_check(config,ack,mission_sha256):
    """Check physical initialization independently of the later estimator gate."""
    identity=hashlib.sha256(json.dumps(config,sort_keys=True,allow_nan=False).encode()).hexdigest()
    if (ack['config_sha256']!=identity or ack['mission_sha256']!=mission_sha256
            or type(ack['impulse_steps']) is not int or ack['impulse_steps']!=1
            or ack['persistent_velocity_control'] is not False):
        raise ValueError('invalid own warm-initialization acknowledgement')
    snapshot=ack['snapshot'];desired=config['state'];speed=desired[3]
    position=snapshot['position_world_m'];velocity=snapshot['velocity_world_m_s']
    angles=snapshot['rpy_rad'];rates=snapshot['angular_velocity_rad_s']
    if any(len(v)!=3 for v in (position,velocity,angles,rates)) or not all(
            type(v) in (int,float) and math.isfinite(v) for v in [snapshot['sim_time_s'],*position,*velocity,*angles,*rates]):
        raise ValueError('invalid own physical initial state')
    distance=lambda a,b:math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
    angle_error=max(abs((a-b+math.pi)%(2*math.pi)-math.pi) for a,b in zip(angles,[0.,config['body_pitch_rad'],desired[4]]))
    errors=dict(position_error_m=distance(position,desired[:3]),velocity_error_m_s=distance(velocity,[speed,0.,0.]),
                attitude_error_deg=math.degrees(angle_error),angular_rate_rad_s=distance(rates,[0.,0.,0.]))
    return dict(**errors,passed=errors['position_error_m']<=.5 and errors['velocity_error_m_s']<=.5
                and errors['attitude_error_deg']<=2. and errors['angular_rate_rad_s']<=.1)


def warm_initialize(mission,observations,output,config,mission_sha256,report):
    """One owned simulator release; observations never become control inputs."""
    identity=hashlib.sha256(json.dumps(config,sort_keys=True,allow_nan=False).encode()).hexdigest()
    # Keep the native takeoff/landing records for feasibility but skip takeoff.
    mission.set_current_mission_item(1)
    pending=output/'airborne-release.pending.json'
    with pending.open('x') as stream:json.dump(dict(config_sha256=identity,mission_sha256=mission_sha256,release=True),stream)
    pending.rename(output/'airborne-release.json');released=time.monotonic()
    wait_for(lambda:(output/'airborne-initial-state.json').exists(),5.,'compact airborne impulse not acknowledged',interval=.02)
    ack=json.loads((output/'airborne-initial-state.json').read_text())
    check=initial_state_check(config,ack,mission_sha256)
    report['airborne_initialization']=dict(**ack,check=check)
    (output/'airborne-check.json').write_text(json.dumps(check,indent=2)+'\n')
    if not check['passed']:raise RuntimeError('compact physical warm-start state did not pass')
    checks=[]
    def estimated_ready():
        check=airborne_estimate_check(observations,config['state'],ack['snapshot']['sim_time_s'],0.,released)
        checks.append(check)
        return check['passed'] and observations.latest.get('in_air') is True
    try:
        wait_for(estimated_ready,3.,'compact estimated airborne state did not converge',interval=.02)
    finally:
        report['airborne_estimator_acquisition']=dict(passed=bool(checks and checks[-1]['passed'] and observations.latest.get('in_air') is True),
            checks=checks,timeout_wall_s=3.,receipt_freshness_is_not_source_time_alignment=True)
        (output/'airborne-estimator-check.json').write_text(json.dumps(report['airborne_estimator_acquisition'],indent=2)+'\n')
    for operation in ('stop','start'):
        subprocess.run(['/opt/px4-gazebo/bin/px4-fw_lat_lon_control',operation],check=True,capture_output=True,text=True,timeout=3)
    report['airborne_controller_reset_count']=1
    report['airborne_first_mission_sequence']=1


def cruise_turns_passed(progress,cruise_hold,heading_change_deg,maximum_bank_deg):
    """A bounded phase result, never a completed landing or full qualification."""
    return (progress is not None and progress.total==9 and progress.current>=6
            and cruise_hold>=5. and heading_change_deg>=45. and maximum_bank_deg>=5.)


def trial(system,telemetry,params,latest,output,*,airborne_start=False,flight_phase='circuit-landing'):
    if flight_phase not in ('circuit-landing','cruise-turns','native-brake','swarm-reference') or (flight_phase in ('cruise-turns','native-brake','swarm-reference') and not airborne_start):
        raise ValueError('cruise/turn phase requires the explicit warm airborne bench')
    if not Path('/.dockerenv').exists() or os.environ.get('SWARM_COMPACT_PX4_STARTUP')!='1' or params.get_param_int('SYS_AUTOSTART')!=4999:
        raise ValueError('owned compact SITL flight qualification only')
    from mavsdk.plugins.action import Action
    from mavsdk.plugins.mission_raw import MissionRaw,MissionItem
    profile=load_aircraft_profile('/workspace/examples/aircraft/compact_5kg.json')
    action=Action(system);mission=MissionRaw(system);observations=Measurements()
    report=dict(passed=False,completed=False,native_mission_completed=False,point_mass_reference_used=False,spatial_tracking_qualified=False,
                airborne_start=airborne_start,flight_phase=flight_phase,
                scope=('warm airborne initialization' if airborne_start else 'cold takeoff')+'; native spatial circuit and landing; no swarm or reference safety transfer')
    if flight_phase=='cruise-turns':report['scope']='warm initialization and native cruise/turn phase only; simulator shutdown before landing, no full mission/flight qualification'
    try:
        for name in ('ground_truth','attitude_euler','fixedwing_metrics','in_air','armed','flight_mode','status_text'):
            getattr(telemetry,'subscribe_'+name)(observations.callback(name))
        telemetry.set_rate_ground_truth(10.);telemetry.set_rate_fixedwing_metrics(10.);telemetry.set_rate_attitude_euler(10.)
        config=None
        if airborne_start:
            if os.environ.get('SWARM_COMPACT_AIRBORNE')!='1':raise ValueError('compact warm-start fixture was not enabled at creation')
            config=json.loads((output/'airborne-config.json').read_text())
            manifest=json.loads((output/'assumptions.json').read_text())
            identity=hashlib.sha256(json.dumps(config,sort_keys=True,allow_nan=False).encode()).hexdigest()
            if identity!=manifest['airborne_config_sha256']:raise ValueError('compact warm-start identity changed')
            wait_for(lambda:(output/'airborne-bootstrap-ready.json').exists(),10.,'own compact bootstrap unavailable before arm')
            if json.loads((output/'airborne-bootstrap-ready.json').read_text())['config_sha256']!=identity:
                raise ValueError('unexpected own compact bootstrap identity')
            telemetry.subscribe_position_velocity_ned(observations.callback('position_velocity_ned'))
            telemetry.set_rate_position_velocity_ned(50.)
            telemetry.set_rate_in_air(10.)
        own_defaults={'FW_R_LIM':15.,'FW_T_CLMB_MAX':.7*profile.performance.max_climb_rate,
                      'FW_T_SINK_MAX':3.5,'FW_AIRSPD_STALL':profile.flight.stall_speed,
                      'FW_THR_MAX':.75,'FW_THR_TRIM':.53,'TRIM_PITCH':-.005,'TRIM_ROLL':-.034,
                      'RWTO_MAX_THR':.7,'RWTO_RAMP_TIME':4.,'RWTO_ROT_AIRSPD':27.,'RWTO_ROT_TIME':3.,
                      'FW_TKO_AIRSPD':profile.airspeed_for('cruise'),
                      'FW_PR_P':.2,'FW_PR_I':.1,'FW_PR_FF':.06,
                      'FW_RR_P':.1,'FW_RR_I':.2,'FW_RR_FF':.06,
                      'FW_YR_P':.1,'FW_YR_I':.05,'FW_YR_FF':.03,
                      'FW_R_RMAX':30.,'FW_P_RMAX_POS':6.,'FW_P_RMAX_NEG':6.,'FW_LND_AIRSPD':27.}
        if flight_phase=='native-brake':own_defaults.update(FW_SPOILERS_LND=.4)
        effective={}
        for key,wanted in own_defaults.items():
            before=params.get_param_float(key);params.set_param_float(key,wanted);after=params.get_param_float(key)
            if abs(after-wanted)>1e-5:raise ValueError('own controller parameter readback failed')
            effective[key]=dict(before=before,effective=after)
        report['controller_parameters']=effective
        if flight_phase=='native-brake':
            before=params.get_param_int('FW_LND_EARLYCFG')
            params.set_param_int('FW_LND_EARLYCFG',1)
            if params.get_param_int('FW_LND_EARLYCFG')!=1:raise RuntimeError('native early landing configuration required for brake bench')
            report['native_early_landing_configuration']=dict(before=before,effective=1,scope='native brake bench only')
            report['scope']='native landing-configuration brake response only; no dive, recovery or completed landing qualification'
        home=latest['home'];origin=LocalOrigin(home.latitude_deg,home.longitude_deg,home.absolute_altitude_m)
        reference=None;tracking_errors=[];target_distance=float('inf')
        if flight_phase=='swarm-reference':
            from .px4_compact_reference import checked_reference
            from .px4_reference import waypoint_indices,polyline_distances
            document=json.loads((output/'point_mass_mission.json').read_text())
            if document['sha256']!=manifest['point_mass_mission_sha256']:raise ValueError('unexpected compact reference identity')
            reference=checked_reference(document,profile)
            origin=LocalOrigin(47.397971057728974,8.546163739800146,0.)
            report.update(point_mass_reference_used=True,point_mass_mission_sha256=document['sha256'],
                          scope='single-aircraft generated straight level spatial reference; no timing, separation or obstacle certificate transfer')
        items=[]
        def add(command,point=None,args=(0.,0.,0.,float('nan'))):
            from .px4_compact_reference import native_altitude_fields
            lat,lon,alt=origin.from_enu(point) if point is not None else (0,0,origin.altitude_m)
            frame,z=native_altitude_fields(point is not None,alt,home.absolute_altitude_m,reference is not None)
            items.append(dict(seq=len(items),frame=frame,command=command,current=int(not items),autocontinue=1,
                param1=args[0],param2=args[1],param3=args[2],param4=args[3],x=round(lat*1e7),y=round(lon*1e7),z=z,mission_type=0))
        add(22,(500.,0.,80.),(10.,0.,0.,float('nan')))
        add(178,args=(0.,profile.airspeed_for('cruise'),-1.,0.))
        points=([(1000.,0.,80.),(1500.,800.,80.),(300.,1200.,80.),(-500.,300.,80.)] if reference is None
                else reference[waypoint_indices(reference,spacing_m=100.)[1:]])
        for point in points:add(16,point,(0.,15.,0.,float('nan')))
        reference_end_sequence=len(items)
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
        if os.environ.get('SWARM_COMPACT_GUI')=='1':
            subprocess.Popen(['gz','sim','-g'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            report['gui_requested']=True
            report['camera_follow']=initialize_camera_follow(target='compact_5kg_prototype_0')
        if not latest['health'].is_armable or latest.get('armed') is not False:raise RuntimeError('not ready for ordinary simulated arming')
        action.arm();deadline=time.monotonic()+5
        while latest.get('armed') is not True:
            if time.monotonic()>deadline:raise RuntimeError('ordinary arming not observed')
            time.sleep(.05)
        if airborne_start:warm_initialize(mission,observations,output,config,report['native_mission_sha256'],report)
        mission.start_mission();start=time.monotonic();cruise_hold=0.;steady_since=None;last_time=None;ever_airborne=False
        heading_change=maximum_bank=0.
        brake_since=None
        while time.monotonic()-start<480:
            if observations.rows:
                row=observations.rows[-1];height=row['absolute_altitude_m']-home.absolute_altitude_m
                stamp=row['sim_time_s']
                if airborne_start and stamp<report['airborne_initialization']['snapshot']['sim_time_s']:
                    time.sleep(.05);continue
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
                    heading_change=max(heading_change,abs(math.degrees(math.atan2(velocity[1],velocity[0]))))
                    if roll is not None:maximum_bank=max(maximum_bank,abs(roll))
                    steady=speed is not None and abs(speed-profile.airspeed_for('cruise'))<=2 and 60<=height<=100
                    if steady:
                        if steady_since is None or last_time is None or stamp-last_time>.25:steady_since=stamp
                        cruise_hold=max(cruise_hold,stamp-steady_since)
                    else:steady_since=None
                else:steady_since=None
                previous_stamp=last_time;last_time=stamp
                report.update(maximum_cruise_hold_s=cruise_hold,cruise_hold_passed=cruise_hold>=5.,ever_airborne=ever_airborne,
                              maximum_heading_change_deg=heading_change,maximum_bank_deg=maximum_bank)
                (output/'flight-status.json').write_text(json.dumps(dict(elapsed_wall_s=time.monotonic()-start,last_observation=row,ever_airborne=ever_airborne,cruise_hold_s=cruise_hold),indent=2)+'\n')
                if ever_airborne and latest.get('armed') is False:
                    progress=observations.latest.get('mission_progress')
                    if observations.latest.get('in_air') is not False or height>2 or progress is None or progress.current<progress.total-1:
                        raise RuntimeError('PX4 disarmed before verified native mission landing')
                    report.update(completed=True,native_mission_completed=True,passed=cruise_hold>=5);break
                progress=observations.latest.get('mission_progress')
                if flight_phase=='swarm-reference':
                    actual=origin.to_enu(row['latitude_deg'],row['longitude_deg'],row['absolute_altitude_m'])
                    error=float(polyline_distances(actual[None],reference)[0]);tracking_errors.append(error)
                    target_distance=min(target_distance,float(math.dist(actual,[1800.,0.,80.])))
                    report.update(maximum_spatial_error_m=max(tracking_errors),minimum_target_distance_m=target_distance,
                                  spatial_error_limit_m=10.,target_radius_m=document['config']['target_radius'])
                    if error>10.:raise RuntimeError('compact generated-reference spatial error exceeded 10 m')
                    if progress is not None and progress.current>=reference_end_sequence:
                        if latest.get('armed') is not True or observations.latest.get('in_air') is not True:raise RuntimeError('reference did not finish in armed flight')
                        reached=target_distance<=document['config']['target_radius']
                        report.update(completed=True,passed=reached,spatial_tracking_qualified=reached,
                                      completion='generated spatial reference finished; simulator shutdown before landing')
                        if not reached:report['error']='target passage was not observed within the original target radius'
                        break
                if flight_phase=='native-brake' and progress is not None and progress.current>=7:
                    if progress.current!=7 or height<40:
                        raise RuntimeError('native brake response not established in the elevated pre-landing loiter')
                    brake=own_physics_brake(output/'actuators.csv',stamp)
                    brake_since=brake_hold(brake_since,previous_stamp,stamp,brake)
                    report['native_brake_response']=dict(expected_fraction=.4,observed_fraction=brake,held_s=0. if brake_since is None else stamp-brake_since)
                    if brake_since is not None and stamp-brake_since>=2.:
                        if latest.get('armed') is not True or observations.latest.get('in_air') is not True:raise RuntimeError('native brake response requires armed flight')
                        report.update(completed=True,passed=True,completion='native brake response observed; simulator shutdown before landing')
                        break
                if flight_phase=='cruise-turns' and cruise_turns_passed(progress,cruise_hold,heading_change,maximum_bank):
                    if observations.latest.get('in_air') is not True or latest.get('armed') is not True:
                        raise RuntimeError('cruise/turn phase did not finish in native armed flight')
                    report.update(completed=True,passed=True,completion='bounded cruise/turn phase; native mission not completed')
                    break
            time.sleep(.1)
        else:raise TimeoutError('native circuit/landing did not finish within the qualification budget')
        report.update(maximum_cruise_hold_s=cruise_hold,ever_airborne=ever_airborne)
    except Exception as error:report.update(passed=False,error=repr(error))
    finally:
        (output/'flight-observations.json').write_text(json.dumps(observations.rows,indent=2,allow_nan=False)+'\n')
        report['status_text']=observations.status
        (output/'flight-report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
