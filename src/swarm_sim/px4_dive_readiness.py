"""Read-only prerequisites for a native dive trial, not flight qualification."""
import math
from .gazebo_trim import trim


BRAKE_DEFAULTS = {'CA_SV_CS_COUNT': 5, 'CA_SV_CS4_TYPE': 11,
                  'CA_SV_CS4_FLAP': 0., 'CA_SV_CS4_SPOIL': 1.,
                  'SIM_GZ_SV_FUNC6': 0}


def dive_readiness(force, control, parameters):
    control.require_matches(force)
    speed = force.profile.airspeed_for('dive')
    angle = -30.
    equilibrium = trim(force, control, speed, angle)
    sink = -speed * math.sin(math.radians(angle))
    pitch = angle + math.degrees(equilibrium['alpha_rad'])
    required = {'FW_AIRSPD_MAX', 'FW_T_SINK_MAX', 'FW_P_LIM_MIN', *BRAKE_DEFAULTS}
    if set(parameters) != required or any(type(v) not in (int, float) or not math.isfinite(v)
                                          for v in parameters.values()):
        raise ValueError('complete finite native dive parameter readbacks required')
    blockers = []
    if parameters['FW_AIRSPD_MAX'] < speed: blockers.append('native_airspeed_limit')
    if parameters['FW_T_SINK_MAX'] < sink: blockers.append('native_sink_setting')
    if parameters['FW_P_LIM_MIN'] > pitch: blockers.append('native_pitch_limit')
    if any(abs(parameters[k]-v) > 1e-6 for k, v in BRAKE_DEFAULTS.items()):
        blockers.append('native_airbrake_allocation')
    return dict(schema='swarm-compact-dive-readiness-v1',
                force_binding_sha256=force.record()['binding_sha256'],
                control_sha256=control.record()['sha256'], trim=equilibrium,
                requested_sink_m_s=sink, requested_pitch_deg=pitch,
                native_parameters=dict(parameters), blockers=blockers,
                settings_compatible=not blockers, closed_loop_qualified=False,
                documented_sink_setting_max_m_s=15.,
                documented_range_compatible=sink<=15. and parameters['FW_T_SINK_MAX']<=15.,
                autonomous_waypoint_braking_supported=False,
                flight_trial_ready=False,
                scope='necessary settings and illustrative equilibrium only; no native dive or recovery flown')
