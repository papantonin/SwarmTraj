"""Read-only Phase 2 pull-out stress bench: no plans or mission acceptances."""

import json
import numpy as np

from swarm_sim.trajectory2d.kinodynamic_recovery import (
    recovery_seed_acceleration, simulate_pullout,
)
from swarm_sim.trajectory2d.planning import Config


def main():
    config = Config(dimensions=3, target_altitude=30.0,
                    descent_angle_deg=30.0, min_altitude=0.0,
                    max_descent_rate=8.0, horizon=120.0)
    records = []
    for speed in (10.0, 11.0, 12.0, 13.0, 14.0):
        pullout = simulate_pullout(speed, config)
        rate = speed * np.tan(np.deg2rad(config.descent_angle_deg))
        old_ramp = int(np.ceil(rate / (0.9 * config.max_vertical_acceleration * config.dt)))
        old_loss = rate * old_ramp * config.dt / 2
        seed = recovery_seed_acceleration(speed, config)
        seeded_loss = None
        if seed is not None:
            ramp = int(np.ceil(rate / (seed * config.dt)))
            seeded_loss = rate * ramp * config.dt / 2
        records.append({
            'speed_m_s': speed,
            'legacy_template_loss_m': old_loss,
            'physical_straight_pullout_loss_m': None if pullout is None else pullout.altitude_loss_m,
            'physical_straight_pullout_duration_s': None if pullout is None else pullout.duration_s,
            'seeded_template_loss_m': seeded_loss,
            'mission_feasibility_established': False,
        })
    print(json.dumps({'scope': 'straight unbanked local pullout only',
                      'parameters': {'target_altitude_m': 30., 'descent_angle_deg': 30.,
                                     'max_descent_rate_m_s': 8.}, 'records': records},
                     indent=2))


if __name__ == '__main__':
    main()
