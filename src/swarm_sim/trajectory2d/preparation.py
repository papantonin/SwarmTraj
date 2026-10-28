"""In-flight preparation timeline on the designation-time mission clock.

An all-or-none simulated activation gate, not a real distributed commit protocol.
No telemetry is refreshed after designation. Aircraft follow the already-known
holding motion while plans are computed and transferred before radio cutoff.
"""

from dataclasses import asdict, dataclass, replace
from time import perf_counter
import numpy as np

from .dynamics import propagate


@dataclass(frozen=True)
class Preparation:
    upload_delay_s: float = .25  # Shared link: sequential uploads in sorted ID order.
    acknowledgement_delay_s: float = .1
    guard_s: float = .5
    simulated_planning_s: float | None = None  # None uses measured wall time.
    missing_ack_ids: tuple[int, ...] = ()

    def __post_init__(self):
        values = [self.upload_delay_s, self.acknowledgement_delay_s, self.guard_s]
        if self.simulated_planning_s is not None:
            values.append(self.simulated_planning_s)
        if not np.all(np.isfinite(values)) or min(values) < 0:
            raise ValueError("preparation delays must be finite and nonnegative")
        if (len(set(self.missing_ack_ids)) != len(self.missing_ack_ids)
                or any(type(i) is not int or i < 0 for i in self.missing_ack_ids)):
            raise ValueError("missing acknowledgements require unique nonnegative IDs")


def readiness(snapshots, config, preparation, planning_s):
    if not np.isfinite(planning_s) or planning_s < 0:
        raise ValueError("planning duration must be finite and nonnegative")
    snapshots = sorted(snapshots, key=lambda s: s.drone_id)
    if set(preparation.missing_ack_ids) - {s.drone_id for s in snapshots}:
        raise ValueError("unknown drone ID in missing acknowledgements")
    deadline = config.activation_time - config.clock_error - preparation.guard_s
    aircraft = []
    for index, snapshot in enumerate(snapshots):
        uploaded = planning_s + (index + 1) * preparation.upload_delay_s
        acknowledged = None if snapshot.drone_id in preparation.missing_ack_ids else (
            uploaded + preparation.acknowledgement_delay_s)
        state = propagate(snapshot.state, snapshot.speed, 0., config.activation_time)
        aircraft.append({"drone_id": snapshot.drone_id, "uploaded_at_s": uploaded,
                         "acknowledged_at_s": acknowledged,
                         "ready": acknowledged is not None and acknowledged <= deadline,
                         "predicted_activation_state": state.tolist(),
                         "predicted_activation_altitude_m": snapshot.altitude})
    return {"mode": "in_flight_simulated_all_or_none", "activation_time_s": config.activation_time,
            "radio_cutoff_s": config.activation_time, "latest_ready_time_s": deadline,
            "clock_guard_s": config.clock_error, "settings": asdict(preparation),
            "aircraft": aircraft, "all_ready": all(a["ready"] for a in aircraft),
            "holding_motion_requires_full_horizon_validation": True,
            "live_network_or_distributed_commit": False}


def prepare_mission(snapshots, config, preparation=Preparation()):
    from .planning import plan_mission
    if set(preparation.missing_ack_ids) - {s.drone_id for s in snapshots}:
        raise ValueError("unknown drone ID in missing acknowledgements")
    started = perf_counter()
    result = plan_mission(snapshots, config)
    measured = perf_counter() - started
    duration = measured if preparation.simulated_planning_s is None else preparation.simulated_planning_s
    gate = readiness(snapshots, config, preparation, duration)
    gate["holding_motion_validated"] = result.validated
    # Real measured durations never enter the reproducible manifest. For a
    # deterministic experiment, use an explicitly simulated planning latency.
    deterministic = gate if preparation.simulated_planning_s is not None else {
        k: v for k, v in gate.items() if k != "aircraft"}
    diagnostics = {**result.diagnostics, "preparation": deterministic}
    timings = {**result.timings, "preparation_timeline": gate, "planning_measured_wall_s": measured}
    if result.validated and not gate["all_ready"]:
        return replace(result, status="preparation_not_ready", plans=(),
                       reason="No mission activation: every plan must be loaded and acknowledged before the guarded activation time.",
                       diagnostics=diagnostics, timings=timings)
    return replace(result, diagnostics=diagnostics, timings=timings)
