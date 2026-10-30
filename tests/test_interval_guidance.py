"""Native departure-window proposals preserve streams through spawned workers."""
from dataclasses import replace
from swarm_sim.trajectory2d import Config, separate_loiter_snapshot
from swarm_sim.trajectory2d.catalog import Catalog, extend_catalog
from swarm_sim.trajectory2d.interval_guidance import DepartureWindows
from swarm_sim.trajectory2d.parallel import generate_catalogs


def test_window_proposals_are_spawn_deterministic():
    config = Config(dimensions=3, target=(500., 0.), target_altitude=30., max_descent_rate=10.,
                    horizon=150., cruise_altitude_range=(60.,100.), reservation_policy='grid', interval_guidance='safe_windows', candidates=2, attempts=12)
    snapshots = tuple(replace(s, altitude=80.) for s in separate_loiter_snapshot()[:2])
    windows = DepartureWindows(0, (2, 3), ((8., 10.),), 1)
    serial = [extend_catalog((s, config, None, Catalog(), 2, None, windows)) for s in snapshots]
    parallel = generate_catalogs(extend_catalog,
        [(s, config, None, Catalog(), 2, None, windows) for s in snapshots], 2)
    for a, b in zip(serial, parallel):
        assert a.interval_proposals == b.interval_proposals and a.attempted == b.attempted
        assert [p.to_dict() for p in a.plans] == [p.to_dict() for p in b.plans]
        assert a.plans and all(not p.states.flags.writeable for p in b.plans)
