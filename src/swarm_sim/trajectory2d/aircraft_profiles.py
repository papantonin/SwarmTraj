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
from .flight3d import FlightLimits


SCHEMA = 'swarm-aircraft-profile-v1'
GEOMETRY_SCHEMA = 'swarm-aircraft-profile-v2'
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
class AircraftProfile:
    name: str
    description: str
    calibration_status: str
    limits: Limits
    flight: FlightLimits
    performance: PerformanceLimits
    bounding_radius_m: float | None = None

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

    def to_dict(self):
        def group(mapping):
            return {external: float(getattr(getattr(self, owner), attribute))
                    for external, (owner, attribute) in mapping.items()}
        document = {'schema': SCHEMA, 'model': MODEL, 'unit_system': 'SI', 'name': self.name,
                'description': self.description, 'calibration_status': self.calibration_status,
                'limits': group(LIMIT_FIELDS), 'actuators': group(ACTUATOR_FIELDS), 'drag': group(DRAG_FIELDS)}
        if self.bounding_radius_m is not None:
            document.update(schema=GEOMETRY_SCHEMA, geometry={'bounding_radius_m': self.bounding_radius_m})
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
        return replace(config, **self.physical_config())

    def require_matches(self, config):
        if not config.coupled_flight or config.aircraft_profiles or any(getattr(config, key) != value for key, value in self.physical_config().items()):
            raise ValueError('effective physical configuration differs from the recorded aircraft profile')


def reference_aircraft_profile():
    return AircraftProfile('fixed_wing_reference', 'Historical illustrative point-mass parameters; not a calibrated aircraft.',
                           'illustrative', Limits(), FlightLimits(), PerformanceLimits())


def aircraft_profile_from_dict(document):
    geometry = isinstance(document, dict) and document.get('schema') == GEOMETRY_SCHEMA
    _keys(document, ('schema', 'model', 'unit_system', 'name', 'description', 'calibration_status',
                     'limits', 'actuators', 'drag', *(('geometry',) if geometry else ())), 'profile')
    if document['schema'] not in (SCHEMA, GEOMETRY_SCHEMA) or document['model'] != MODEL or document['unit_system'] != 'SI':
        raise ValueError('unsupported aircraft schema, model or unit system')
    values = {'limits': {}, 'flight': {}, 'performance': {}}
    for name, mapping in (('limits', LIMIT_FIELDS), ('actuators', ACTUATOR_FIELDS), ('drag', DRAG_FIELDS)):
        _keys(document[name], mapping, name)
        for external, (owner, attribute) in mapping.items():
            values[owner][attribute] = _number(document[name][external], f'{name}.{external}')
    radius = None
    if geometry:
        _keys(document['geometry'], ('bounding_radius_m',), 'geometry')
        radius = _number(document['geometry']['bounding_radius_m'], 'bounding_radius_m')
    return AircraftProfile(document['name'], document['description'], document['calibration_status'],
                           Limits(**values['limits']), FlightLimits(**values['flight']), PerformanceLimits(**values['performance']), radius)


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
