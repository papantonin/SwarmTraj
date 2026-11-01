"""Reusable physical-profile binding for experimental Gazebo force benches.

These assumptions add no geometry, inertias, moments or PX4 controller tuning.
They remain separate from the authoritative point-mass aircraft profile.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from .trajectory2d.aircraft_profiles import AircraftProfile
from .trajectory2d.dynamics import G

SCHEMA = 'swarm-gazebo-force-assumptions-v1'


def identity(document):
    return hashlib.sha256(json.dumps(document,sort_keys=True,separators=(',', ':'),allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class ForceAssumptions:
    lift_slope_per_rad: float = 4.75
    zero_lift_angle_rad: float = -.06
    calibration_status: str = 'illustrative'

    def __post_init__(self):
        for key in ('lift_slope_per_rad','zero_lift_angle_rad'):
            value=getattr(self,key)
            if type(value) not in (float,int):
                raise ValueError(f'{key} must be a finite number')
            try:value=float(value)
            except OverflowError as error:raise ValueError(f'{key} is out of numerical range') from error
            if not math.isfinite(value):raise ValueError(f'{key} must be finite')
            object.__setattr__(self,key,value)
        if self.lift_slope_per_rad<=0 or abs(self.zero_lift_angle_rad)>=math.pi/2:
            raise ValueError('invalid lift slope or zero-lift angle')
        if self.calibration_status not in ('illustrative','user_supplied','measured'):
            raise ValueError('unsupported force-assumption calibration status')

    def to_dict(self):
        return dict(schema=SCHEMA,lift_slope_per_rad=self.lift_slope_per_rad,
                    zero_lift_angle_rad=self.zero_lift_angle_rad,calibration_status=self.calibration_status)

    def record(self):
        document=self.to_dict()
        return dict(sha256=identity(document),assumptions=document)

    @classmethod
    def load(cls,path):
        def unique(pairs):
            result={}
            for key,value in pairs:
                if key in result:raise ValueError(f'duplicate force-assumption field: {key}')
                result[key]=value
            return result
        document=json.loads(Path(path).read_text(),object_pairs_hook=unique)
        fields={'schema','lift_slope_per_rad','zero_lift_angle_rad','calibration_status'}
        if not isinstance(document,dict) or set(document)!=fields or document['schema']!=SCHEMA:
            raise ValueError('force assumptions require the exact versioned schema and fields')
        return cls(**{key:value for key,value in document.items() if key!='schema'})


@dataclass(frozen=True)
class ForceModel:
    profile: AircraftProfile
    assumptions: ForceAssumptions = ForceAssumptions()

    def __post_init__(self):
        if not isinstance(self.profile,AircraftProfile) or self.profile.airframe is None:
            raise ValueError('Gazebo forces require a v3 physical airframe; normalized v1/v2 values do not identify mass/wing area')
        if not isinstance(self.assumptions,ForceAssumptions):
            raise ValueError('force assumptions must be validated')

    def record(self):
        document=dict(profile=self.profile.record(),force_assumptions=self.assumptions.record())
        return dict(**document,binding_sha256=identity(document))

    def plugin_values(self,*,brake_fraction=0.,elevator_fraction=0.):
        for key,value in [('brake_fraction',brake_fraction),('elevator_fraction',elevator_fraction)]:
            if type(value) not in (int,float) or not math.isfinite(value):raise ValueError(f'invalid {key}')
        if not 0<=brake_fraction<=1 or abs(elevator_fraction)>1:raise ValueError('actuator fraction outside bench bounds')
        a=self.profile.airframe
        return dict(density=a.air_density_kg_m3,area=a.wing_area_m2,cd0=a.zero_lift_drag_coefficient,
            k=a.induced_drag_factor,clmax=a.max_lift_coefficient,max_brake=a.max_airbrake_drag_n,
            lift_slope=self.assumptions.lift_slope_per_rad,zero_lift_angle=self.assumptions.zero_lift_angle_rad,
            brake_fraction=float(brake_fraction),elevator_fraction=float(elevator_fraction))

    def test_speeds(self):
        p=self.profile
        if p.speeds:
            values=[p.airspeed_for(key) for key in ('loiter','cruise','dive')]+[p.limits.max_speed]
        else:
            low=max(p.limits.min_speed,p.flight.stall_speed*p.flight.stall_margin)
            values=[low+(p.limits.max_speed-low)*fraction for fraction in (0.,1/3,2/3,1.)]
        return tuple(dict.fromkeys(values))

    def cruise_test_speed(self):
        if self.profile.speeds:return self.profile.airspeed_for('cruise')
        speeds=self.test_speeds()
        return (speeds[0]+speeds[-1])/2

    def load_alpha(self,speed,load=1.):
        if not math.isfinite(speed) or speed<=0 or not math.isfinite(load) or load<0:
            raise ValueError('invalid fixture speed or load')
        a=self.profile.airframe
        cl=a.mass_kg*G*load/(.5*a.air_density_kg_m3*speed**2*a.wing_area_m2)
        alpha=cl/self.assumptions.lift_slope_per_rad+self.assumptions.zero_lift_angle_rad
        if not math.isfinite(alpha) or abs(cl)>a.max_lift_coefficient or abs(alpha)>=math.pi/2:
            raise ValueError('requested load exceeds the force fixture lift/forward-flow domain')
        return alpha
