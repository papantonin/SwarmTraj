"""Own-airframe, versioned illustrative moments and actuator conventions.

The point-mass profile cannot identify these derivatives or inertias. A separate
record binds them to one exact physical-profile/lift-assumption pairing.
"""
from dataclasses import dataclass,fields
import json
import math
from pathlib import Path
from .gazebo_force_model import ForceModel,identity

SCHEMA='swarm-gazebo-control-model-v1'
STABILITY_FIELDS=('span_m','chord_m','alpha_trim_rad','roll_beta','roll_damping','roll_control',
                  'pitch_stiffness','pitch_damping','pitch_control','yaw_stiffness','yaw_damping','yaw_control','sideforce')


def verify_control_plugin(library):
    """Check the experimental capability inside the pinned runtime, not the host."""
    import subprocess
    from .px4_gazebo import IMAGE
    library=Path(library).resolve()
    if not library.is_file():raise ValueError('control plugin library is missing; build it first')
    check='import ctypes; lib=ctypes.CDLL("/plugin/libcompact_polar.so"); version=lib.swarm_compact_control_api(); assert version==1, version; print(version)'
    result=subprocess.run(['docker','run','--rm','--network','none','--mount',f'type=bind,src={library},dst=/plugin/libcompact_polar.so,readonly',
        '--entrypoint','/usr/bin/python3',IMAGE,'-c',check],capture_output=True,text=True,timeout=30)
    if result.returncode!=0:raise RuntimeError('C++ joint/moment API unavailable; rebuild with scripts/build_compact_gazebo_physics.sh: '+result.stderr)
    return 1


@dataclass(frozen=True)
class ControlModel:
    force_binding_sha256: str
    calibration_status: str
    span_m: float
    chord_m: float
    alpha_trim_rad: float
    roll_beta: float
    roll_damping: float
    roll_control: float
    pitch_stiffness: float
    pitch_damping: float
    pitch_control: float
    yaw_stiffness: float
    yaw_damping: float
    yaw_control: float
    sideforce: float
    surface_limit_rad: float
    brake_limit_rad: float
    servo_p_gain: float
    servo_damping_nms: float
    servo_effort_nm: float
    servo_rate_rad_s: float
    wheel_damping_nms: float
    wheel_radius_m: float
    wheel_mass_kg: float
    base_inertia_kg_m2: tuple

    def __post_init__(self):
        if not isinstance(self.force_binding_sha256,str) or len(self.force_binding_sha256)!=64 or any(c not in '0123456789abcdef' for c in self.force_binding_sha256):
            raise ValueError('invalid own force-binding identity')
        if self.calibration_status not in ('illustrative','user_supplied','measured'):raise ValueError('invalid calibration status')
        def number(value):
            if type(value) not in (int,float):raise ValueError('control coefficients must be finite numbers')
            try:value=float(value)
            except OverflowError as error:raise ValueError('control coefficient out of range') from error
            if not math.isfinite(value):raise ValueError('control coefficient must be finite')
            return value
        for key in (*STABILITY_FIELDS,'surface_limit_rad','brake_limit_rad','servo_p_gain','servo_damping_nms','servo_effort_nm','servo_rate_rad_s','wheel_damping_nms','wheel_radius_m','wheel_mass_kg'):
            object.__setattr__(self,key,number(getattr(self,key)))
        if min(self.span_m,self.chord_m,self.surface_limit_rad,self.brake_limit_rad)<=0:raise ValueError('invalid geometry or actuator travel')
        if min(self.servo_p_gain,self.servo_effort_nm,self.servo_rate_rad_s)<=0 or self.servo_damping_nms<0:raise ValueError('invalid servo dynamics')
        if self.wheel_damping_nms<0:raise ValueError('negative wheel damping')
        if self.wheel_radius_m<=0:raise ValueError('nonpositive wheel radius')
        if self.wheel_mass_kg<=0:raise ValueError('nonpositive wheel mass')
        if min(self.roll_damping,self.pitch_damping,self.yaw_damping,self.sideforce)<0:raise ValueError('negative damping/sideforce')
        if not isinstance(self.base_inertia_kg_m2,(tuple,list)) or len(self.base_inertia_kg_m2)!=3:raise ValueError('three principal base inertias required')
        inertia=tuple(number(v) for v in self.base_inertia_kg_m2)
        if min(inertia)<=0 or 2*max(inertia)>sum(inertia):raise ValueError('nonphysical principal inertias')
        object.__setattr__(self,'base_inertia_kg_m2',inertia)

    def require_matches(self,force_model):
        if not isinstance(force_model,ForceModel) or self.force_binding_sha256!=force_model.record()['binding_sha256']:
            raise ValueError('control model does not match the own physical/lift binding')

    def to_dict(self):
        result=dict(schema=SCHEMA,**{f.name:getattr(self,f.name) for f in fields(self)})
        result['base_inertia_kg_m2']=list(self.base_inertia_kg_m2)
        return result

    def record(self):
        record=self.to_dict();return dict(sha256=identity(record),control_model=record)

    def plugin_values(self):
        return {**{key.removesuffix('_m').removesuffix('_rad'):getattr(self,key) for key in STABILITY_FIELDS},
                'surface_limit':self.surface_limit_rad,'brake_limit':self.brake_limit_rad}

    @classmethod
    def load(cls,path,force_model):
        def unique(pairs):
            result={}
            for key,value in pairs:
                if key in result:raise ValueError('duplicate control-model field')
                result[key]=value
            return result
        data=json.loads(Path(path).read_text(),object_pairs_hook=unique)
        if not isinstance(data,dict) or set(data)!={'schema',*(f.name for f in fields(cls))} or data['schema']!=SCHEMA:
            raise ValueError('control model requires exact versioned fields')
        model=cls(**{k:v for k,v in data.items() if k!='schema'});model.require_matches(force_model);return model
