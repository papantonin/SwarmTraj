"""Independent test-only integration/envelope oracle; no candidate construction."""
import numpy as np
from swarm_sim.trajectory2d.flight3d import envelope_ok, propagate


def checked_step(state, control, config):
    following = propagate(state, control, config.dt, config.flight)
    refined = propagate(state, control, np.array([0., .25, .5, .75, 1.]) * config.dt,
                        config.flight, substeps=4)
    if not envelope_ok(np.concatenate((refined, following[None])), control, config):
        return None
    return following
