"""Independent held-control audit; production protection executes in C++."""
import numpy as np
from .flight3d import envelope_ok, propagate


def held_control_ok(state, control, config, dt):
    samples = propagate(state, control, np.linspace(0, dt, 21), config.flight, substeps=4)
    return envelope_ok(samples, control, config, tolerance=1e-8)
