"""Resumable per-drone catalogs, with deterministic diagnostics and separate timings."""

from dataclasses import dataclass, field


def reject(diagnostics, reason, detail=None):
    """Record the first failing stage for this proposal, without changing behavior."""
    if diagnostics is not None:
        diagnostics.setdefault("rejection", reason)
        if detail is not None:
            diagnostics.setdefault("rejection_detail", detail)
    return None


def invalid(diagnostics, reason, detail=None):
    reject(diagnostics, reason, detail)
    return False


@dataclass
class Catalog:
    plans: list = field(default_factory=list)
    attempted: int = 0
    rng_state: dict | None = None
    rejections: dict = field(default_factory=dict)
    generation_s: float = 0.0
    validation_s: float = 0.0
    rejection_details: dict = field(default_factory=dict)
    minimum_required_horizon_s: float | None = None
    guided_proposals: list = field(default_factory=list)
    interval_contexts: list = field(default_factory=list)
    interval_proposals: list = field(default_factory=list)
    profile_stage: int = 0
    profile_cursor: int = 0
    profile_failures: int = 0
    profile_events: list = field(default_factory=list)
    profile_proposals: list = field(default_factory=list)
    passage_contexts: list = field(default_factory=list)
    passage_proposals: list = field(default_factory=list)
    candidate_proposals: list = field(default_factory=list)


def extend_catalog(task):
    """Continue a native catalog in a spawn-safe worker; failures propagate."""
    from .native_planning import extend_catalog as native_extend
    return native_extend(task)
