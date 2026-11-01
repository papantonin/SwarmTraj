"""Immutable, versioned fixed-wing parameter profiles, separate from missions.

Only the existing normalized coordinated point-mass backend is supported.
Unknown model types or unused physical fields must never be silently ignored.
"""

from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path

from .dynamics import Limits
from .dynamics import G
from .flight3d import FlightLimits


SCHEMA = 'swarm-aircraft-profile-v1'
GEOMETRY_SCHEMA = 'swarm-aircraft-profile-v2'
PHYSICAL_SCHEMA = 'swarm-aircraft-profile-v3'
MODEL = 'coordinated_point_mass_v1'
LIMIT_FIELDS = {
    'min_airspeed_m_s': ('limits', 'min_speed'),
    'max_airspeed_m_s': ('limits', 'max_speed'),
    'max_bank_deg': ('limits', 'max_bank_deg'),
    'max_roll_rate_deg_s': ('limits', 'max_roll_rate_deg_s'),
    'stall_speed_1g_m_s': ('flight', 'stall_speed'),
    'stall_margin': ('flight', 'stall_margin'),
    'min_load_factor': ('flight', 'min_load_factor'),
    'max_load_factor': ('flight', 'max_load_factor'),
    'max_flight_path_deg': ('flight', 'max_flight_path_deg'),
    'max_flight_path_rate_deg_s': ('flight', 'max_flight_path_rate_deg_s'),
    'max_climb_rate_m_s': ('performance', 'max_climb_rate'),
    'max_descent_rate_m_s': ('performance', 'max_descent_rate'),
    'max_vertical_acceleration_m_s2': ('performance', 'max_vertical_acceleration'),
}
ACTUATOR_FIELDS = {
    'max_thrust_acceleration_m_s2': ('flight', 'max_thrust_acceleration'),
    'max_airbrake_acceleration_m_s2': ('flight', 'max_brake_acceleration'),
}
DRAG_FIELDS = {
    'parasitic_coefficient_per_m': ('flight', 'parasitic_drag'),
    'induced_coefficient_m3_per_s4': ('flight', 'induced_drag'),
}


def _number(value, name):
    if type(value) not in (int, float):
        raise ValueError(f'{name} must be a finite number, not a boolean/string')
    try:
        number = float(value)
    except OverflowError as error:
        raise ValueError(f'{name} is out of range') from error
    if not math.isfinite(number):
        raise ValueError(f'{name} must be finite')
    return number


def _keys(value, required, name):
    if not isinstance(value, dict):
        raise ValueError(f'{name} must be an object')
    if any(not isinstance(key, str) for key in value):
        raise ValueError(f'{name} keys must be strings')
    missing, unknown = set(required) - value.keys(), value.keys() - set(required)
    if missing or unknown:
        raise ValueError(f'{name}: missing fields {sorted(missing)}; unknown fields {sorted(unknown)}')


@dataclass(frozen=True)
class PerformanceLimits:
    max_climb_rate: float = 3.
    max_descent_rate: float = 10.
    max_vertical_acceleration: float = 1.

    def __post_init__(self):
        for name, value in vars(self).items():
            value = _number(value, name)
            if value <= 0:
                raise ValueError(f'{name} must be positive')
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class PhysicalAirframe:
    """Constant-density parabolic polar and force limits, in SI units."""
    mass_kg: float
    wing_area_m2: float
    air_density_kg_m3: float
    zero_lift_drag_coefficient: float
    induced_drag_factor: float
    max_lift_coefficient: float
    max_thrust_n: float
    max_airbrake_drag_n: float

    def __post_init__(self):
        for name, value in vars(self).items():
            number = _number(value, name)
            minimum_allowed = name in ('zero_lift_drag_coefficient', 'induced_drag_factor', 'max_airbrake_drag_n')
            if number < 0 or (number == 0 and not minimum_allowed):
                raise ValueError(f'{name} must be {"nonnegative" if minimum_allowed else "positive"}')
            object.__setattr__(self, name, number)
        self.derived()

    def derived(self):
        try:
            loading = self.mass_kg / (self.air_density_kg_m3 * self.wing_area_m2)
            result = dict(
                stall_speed=math.sqrt(2 * G * loading / self.max_lift_coefficient),
                parasitic_drag=self.zero_lift_drag_coefficient / (2 * loading),
                induced_drag=2 * self.induced_drag_factor * G**2 * loading,
                max_thrust_acceleration=self.max_thrust_n / self.mass_kg,
                max_brake_acceleration=self.max_airbrake_drag_n / self.mass_kg)
        except (OverflowError, ZeroDivisionError, ValueError) as error:
            raise ValueError('physical airframe conversion is out of numerical range') from error
        if any(not math.isfinite(value) for value in result.values()) or result['stall_speed'] <= 0:
            raise ValueError('physical airframe conversion is out of numerical range')
        return result


@dataclass(frozen=True)
class OperatingSpeeds:
    """Recommended total airspeeds, not changes to the physical envelope."""
    loiter_airspeed_m_s: float
    climb_airspeed_m_s: float
    cruise_airspeed_m_s: float
    descent_airspeed_m_s: float
    dive_airspeed_m_s: float

    def __post_init__(self):
        for name, value in vars(self).items():
            number = _number(value, name)
            if number <= 0:
                raise ValueError(f'{name} must be positive')
            object.__setattr__(self, name, number)
        if self.dive_airspeed_m_s < self.cruise_airspeed_m_s:
            raise ValueError('recommended dive airspeed must not be below cruise airspeed')


@dataclass(frozen=True)
class AircraftProfile:
    name: str
    description: str
    calibration_status: str
    limits: Limits
    flight: FlightLimits
    performance: PerformanceLimits
    bounding_radius_m: float | None = None
    airframe: PhysicalAirframe | None = None
    speeds: OperatingSpeeds | None = None

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name.strip() or len(self.name) > 120:
            raise ValueError('profile name must be nonempty and at most 120 characters')
        if not isinstance(self.description, str) or not self.description.strip():
            raise ValueError('profile description must be nonempty')
        if self.calibration_status not in ('illustrative', 'user_supplied', 'measured'):
            raise ValueError('unsupported calibration status (not a certification label)')
        if not isinstance(self.limits, Limits) or not isinstance(self.flight, FlightLimits) or not isinstance(self.performance, PerformanceLimits):
            raise ValueError('profile requires validated limit objects')
        # Revalidate and normalize direct Python constructors too, including
        # nested legacy parameter objects that may otherwise accept booleans.
        object.__setattr__(self, 'limits', Limits(**{k: _number(v, k) for k, v in vars(self.limits).items()}))
        object.__setattr__(self, 'flight', FlightLimits(**{k: _number(v, k) for k, v in vars(self.flight).items()}))
        if not self.flight.min_load_factor <= 1 <= self.flight.max_load_factor:
            raise ValueError('this fixed-wing profile backend requires support for 1g level flight')
        if self.flight.stall_speed * self.flight.stall_margin > self.limits.max_speed:
            raise ValueError('maximum airspeed is below the model 1g stall margin')
        if self.bounding_radius_m is not None:
            radius = _number(self.bounding_radius_m, 'bounding_radius_m')
            if radius <= 0:
                raise ValueError('bounding_radius_m must be positive')
            object.__setattr__(self, 'bounding_radius_m', radius)
        if self.airframe is not None:
            if not isinstance(self.airframe, PhysicalAirframe) or self.bounding_radius_m is None:
                raise ValueError('physical airframe requires validated parameters and enclosing geometry')
            if any(getattr(self.flight, key) != value for key, value in self.airframe.derived().items()):
                raise ValueError('effective flight parameters differ from physical airframe derivation')
        if self.speeds is not None:
            if self.airframe is None or not isinstance(self.speeds, OperatingSpeeds):
                raise ValueError('operating speeds require a physical v3 airframe')
            lower = max(self.limits.min_speed, self.flight.stall_speed * self.flight.stall_margin)
            if any(not lower <= value <= self.limits.max_speed for value in vars(self.speeds).values()):
                raise ValueError('operating airspeeds must lie within the physical speed and 1g stall limits')

    def airspeed_for(self, regime='cruise'):
        if regime not in ('loiter', 'climb', 'cruise', 'descent', 'dive'):
            raise ValueError('unknown flight regime')
        if self.speeds is None:
            raise ValueError('this aircraft profile does not declare operating speeds')
        return getattr(self.speeds, regime + '_airspeed_m_s')

    def operating_speed_values(self):
        return tuple(self.airspeed_for(regime) for regime in ('loiter','climb','cruise','descent','dive'))

    def to_dict(self):
        def group(mapping):
            return {external: float(getattr(getattr(self, owner), attribute))
                    for external, (owner, attribute) in mapping.items()}
        document = {'schema': SCHEMA, 'model': MODEL, 'unit_system': 'SI', 'name': self.name,
                'description': self.description, 'calibration_status': self.calibration_status,
                'limits': group(LIMIT_FIELDS), 'actuators': group(ACTUATOR_FIELDS), 'drag': group(DRAG_FIELDS)}
        if self.bounding_radius_m is not None:
            document.update(schema=GEOMETRY_SCHEMA, geometry={'bounding_radius_m': self.bounding_radius_m})
        if self.airframe is not None:
            document['schema'] = PHYSICAL_SCHEMA
            document['airframe'] = dict(vars(self.airframe))
            del document['limits']['stall_speed_1g_m_s']
            del document['actuators']
            del document['drag']
            if self.speeds is not None:
                document['speeds'] = dict(vars(self.speeds))
        return document

    @property
    def sha256(self):
        canonical = json.dumps(self.to_dict(), sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
        return hashlib.sha256(canonical).hexdigest()

    def record(self):
        return {'sha256': self.sha256, 'profile': self.to_dict()}

    def physical_config(self):
        return {'limits': self.limits, 'flight': self.flight,
                'aircraft_radius': self.bounding_radius_m or 0.,
                'max_climb_rate': self.performance.max_climb_rate,
                'max_descent_rate': self.performance.max_descent_rate,
                'max_vertical_acceleration': self.performance.max_vertical_acceleration}

    def apply(self, config):
        """Bind one common airframe without changing mission or environment fields."""
        from .planning import Config
        if not isinstance(config, Config) or not config.coupled_flight:
            raise ValueError('aircraft profiles require the coupled 3D point-mass backend')
        if config.aircraft_profiles:
            raise ValueError('apply a common profile before assigning per-drone profiles')
        schedule={'operating_airspeeds':self.operating_speed_values()} if config.speed_schedule=='operating' else {}
        return replace(config, **self.physical_config(), **schedule)

    def require_matches(self, config):
        if not config.coupled_flight or config.aircraft_profiles or any(getattr(config, key) != value for key, value in self.physical_config().items()):
            raise ValueError('effective physical configuration differs from the recorded aircraft profile')
        if config.speed_schedule=='operating' and config.operating_airspeeds!=self.operating_speed_values():
            raise ValueError('effective operating speeds differ from the recorded aircraft profile')


def reference_aircraft_profile():
    return AircraftProfile('fixed_wing_reference', 'Historical illustrative point-mass parameters; not a calibrated aircraft.',
                           'illustrative', Limits(), FlightLimits(), PerformanceLimits())


def aircraft_profile_from_dict(document):
    physical = isinstance(document, dict) and document.get('schema') == PHYSICAL_SCHEMA
    has_speeds = physical and 'speeds' in document
    geometry = isinstance(document, dict) and document.get('schema') in (GEOMETRY_SCHEMA, PHYSICAL_SCHEMA)
    _keys(document, ('schema', 'model', 'unit_system', 'name', 'description', 'calibration_status',
                     'limits', *(('airframe',) if physical else ('actuators', 'drag')),
                     *(('geometry',) if geometry else ()), *(('speeds',) if has_speeds else ())), 'profile')
    if document['schema'] not in (SCHEMA, GEOMETRY_SCHEMA, PHYSICAL_SCHEMA) or document['model'] != MODEL or document['unit_system'] != 'SI':
        raise ValueError('unsupported aircraft schema, model or unit system')
    values = {'limits': {}, 'flight': {}, 'performance': {}}
    mappings = [('limits', {key: value for key, value in LIMIT_FIELDS.items()
                           if not physical or key != 'stall_speed_1g_m_s'})]
    if not physical:
        mappings.extend((('actuators', ACTUATOR_FIELDS), ('drag', DRAG_FIELDS)))
    for name, mapping in mappings:
        _keys(document[name], mapping, name)
        for external, (owner, attribute) in mapping.items():
            values[owner][attribute] = _number(document[name][external], f'{name}.{external}')
    airframe = None
    if physical:
        _keys(document['airframe'], PhysicalAirframe.__dataclass_fields__, 'airframe')
        airframe = PhysicalAirframe(**document['airframe'])
        values['flight'].update(airframe.derived())
    radius = None
    if geometry:
        _keys(document['geometry'], ('bounding_radius_m',), 'geometry')
        radius = _number(document['geometry']['bounding_radius_m'], 'bounding_radius_m')
    speeds = None
    if has_speeds:
        _keys(document['speeds'], OperatingSpeeds.__dataclass_fields__, 'speeds')
        speeds = OperatingSpeeds(**document['speeds'])
    return AircraftProfile(document['name'], document['description'], document['calibration_status'],
                           Limits(**values['limits']), FlightLimits(**values['flight']), PerformanceLimits(**values['performance']), radius, airframe, speeds)


def _unique_object(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f'duplicate aircraft profile key: {key}')
        value[key] = item
    return value


def _invalid_constant(value):
    raise ValueError(f'non-finite JSON constant: {value}')


def load_aircraft_profile(path):
    path = Path(path)
    # Profiles are small documents. Bound the actual read, not just stat(), so
    # concurrent file replacement/growth cannot bypass the size check.
    with path.open('rb') as stream:
        raw = stream.read(1024 * 1024 + 1)
    if len(raw) > 1024 * 1024:
        raise ValueError('aircraft profile exceeds the 1 MiB limit')
    try:
        document = json.loads(raw.decode('utf-8'), object_pairs_hook=_unique_object,
                              parse_constant=_invalid_constant)
        return aircraft_profile_from_dict(document)
    except (UnicodeError, RecursionError) as error:
        raise ValueError('invalid aircraft profile encoding or nesting') from error
