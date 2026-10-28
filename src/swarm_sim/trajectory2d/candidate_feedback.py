"""Own-catalog rejection feedback changes proposals, never validation limits."""

from dataclasses import dataclass


@dataclass(frozen=True)
class CandidateAdjustment:
    mode: str
    trigger: str
    trigger_count: int
    level_hold_s: float | None = None
    routing_extra_m: float = 0.

    def kwargs(self):
        return {"level_hold_override": self.level_hold_s, "routing_extra_margin": self.routing_extra_m}


def adjustment(config, rejections, trial_index):
    """Cycle eligible bounded recipes after at least two matching rejections.

    Longer reference holds cannot reduce the requested level count or enlarge
    the horizon; wider visibility routing does not change actual obstacle
    clearance requirements. Either recipe can still be dynamically invalid.
    """
    available = []
    levels = rejections.get("validation_cruise_level_not_reached", 0)
    obstacles = rejections.get("obstacle_clearance", 0)
    if config.cruise_levels > 1 and levels >= 2:
        available.append(("settle_levels", "validation_cruise_level_not_reached", levels))
    if config.obstacles and obstacles >= 2:
        available.append(("wider_route", "obstacle_clearance", obstacles))
    if not available:
        return None
    mode, trigger, count = available[trial_index % len(available)]
    cycle = trial_index // len(available)
    if mode == "settle_levels":
        return CandidateAdjustment(mode, trigger, count,
            level_hold_s=max(5., config.cruise_level_hold + 2 * (1 + cycle % 2)))
    return CandidateAdjustment(mode, trigger, count, routing_extra_m=5. * (1 + cycle % 3))
