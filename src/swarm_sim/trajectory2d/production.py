"""Production policy; archived configurations remain data, never implicit migrations."""


def production_error(config):
    """Return an actionable reason without changing a recorded configuration."""
    if (config.coupled_flight
            and config.generator_mode in ('physical_mission', 'physical_recovery_seed')
            and config.numerical_backend == 'cpp'):
        return None
    return (
        'Production requires the coupled 3D physical C++ planner. '
        f'This configuration records dimensions={config.dimensions}, '
        f'flight_model={config.flight_model}, generator_mode={config.generator_mode}, '
        f'numerical_backend={config.numerical_backend}. '
        'The Python, geometric and planar/legacy production backends have been retired; '
        'the recorded settings have not been converted. Create a new physical C++ mission '
        'or explicitly migrate a shared scenario with scripts/migrate_scenario.py '
        'to a new output file. Historical implementations remain in Git at 300de37.'
    )


def require_production(config):
    reason = production_error(config)
    if reason:
        raise ValueError(reason)
