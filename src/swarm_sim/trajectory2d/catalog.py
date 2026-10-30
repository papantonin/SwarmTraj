"""Resumable per-drone catalogs, with deterministic diagnostics and separate timings."""

from dataclasses import asdict, dataclass, field, replace
from time import perf_counter

import numpy as np


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
    geometric_references: dict = field(default_factory=dict)  # Offline only; never mission/aircraft payloads.
    passage_contexts: list = field(default_factory=list)
    passage_proposals: list = field(default_factory=list)
    passage_trials: set = field(default_factory=set)  # Identical deterministic fits must not consume budgets twice.
    candidate_proposals: list = field(default_factory=list)


def extend_catalog(task, *, backend=None):
    """Continue the exact RNG stream; cumulative budgets are never reset.

    This callable is importable by spawn workers. Plans are appended, never
    retimed or modified after validation. Worker exceptions propagate normally.
    """
    if (task[1].numerical_backend if backend is None else backend) == 'cpp':
        from .native_planning import extend_catalog as native_extend
        return native_extend(task)
    if backend not in (None, 'python'):
        raise ValueError('catalog backend must be python or cpp')
    from .planning import _candidate, validate_individual

    snapshot, config, angle, catalog, goal, *extra = task
    aircraft_profile = config.profile_for(snapshot.drone_id)
    config = config.for_drone(snapshot.drone_id)
    from .profile_policy import families, family_config
    mixed = config.profile_policy == "adaptive_mixed"
    menu = families(config) if mixed else ()

    def unlock(reason):
        if mixed and catalog.profile_stage + 1 < len(menu):
            catalog.profile_stage += 1
            catalog.profile_cursor = 0
            catalog.profile_failures = 0
            catalog.profile_events.append({"before_candidate_id": catalog.attempted,
                "reason": reason, "unlocked_family": menu[catalog.profile_stage].name})

    expansion = extra[2] if len(extra) > 2 else "initial"
    if expansion in ("conflict", "margin"):
        unlock(expansion)
    hint = extra[0] if extra else None
    windows = extra[1] if len(extra) > 1 and config.interval_guidance == "safe_windows" else None
    passage = extra[3] if len(extra) > 3 and config.passage_guidance == "safe_windows" else None
    if passage is not None:
        catalog.passage_contexts.append(asdict(passage))
    if windows is not None:
        catalog.interval_contexts.append(asdict(windows))
    modes = hint.modes(config) if hint is not None and config.conflict_guidance == "witness" else []
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([config.seed, snapshot.drone_id])))
    if catalog.rng_state is not None:
        rng.bit_generator.state = catalog.rng_state
    batch_attempt = 0
    while catalog.attempted < config.attempts and len(catalog.plans) < min(goal, config.candidates):
        diagnostic = {}
        proposed = None
        proposal_config = config
        if mixed:
            # Newest family first, then cycle all unlocked families. Resumption
            # and process transfer preserve this cursor and cumulative budget.
            family = menu[(catalog.profile_stage - catalog.profile_cursor) % (catalog.profile_stage + 1)]
            catalog.profile_cursor += 1
            proposal_config = family_config(config, family)
            proposed = {"candidate_id": catalog.attempted, "family": family.name, "accepted": False}
            catalog.profile_proposals.append(proposed)
        # Alternate a guided attempt with ordinary exploration if it fails.
        # Rejections consume the same mission-wide attempt allowance.
        guided = None
        timed = None
        fitted = None
        adjusted = None
        reference = catalog.geometric_references.get(passage.parent_candidate_id) if passage is not None else None
        trial_key = (passage, proposal_config.cruise_levels, proposal_config.speed_profile)
        fit_passage = (passage is not None and passage.intervals_s and not passage.budget_exhausted
                       and reference is not None and proposal_config.speed_profile == "smooth"
                       and (len(reference.cruise_levels_m) or 1) == proposal_config.cruise_levels
                       and batch_attempt % 2 == 0 and trial_key not in catalog.passage_trials)
        kwargs = {}
        if modes and batch_attempt % 2 == 0 and not fit_passage:
            mode = modes[len(catalog.guided_proposals) % len(modes)]
            guided = {"candidate_id": catalog.attempted, "mode": mode,
                      "witness": asdict(hint.conflict), "accepted": False}
            catalog.guided_proposals.append(guided)
            kwargs = {"repair_hint": hint, "repair_mode": mode}
        if windows is not None and windows.intervals_s and batch_attempt % 2 == 0 and not fit_passage:
            release, interval = windows.choose(rng)
            timed = {"candidate_id": catalog.attempted, "departure_interval_s": interval,
                     "requested_departure_s": release, "accepted": False}
            catalog.interval_proposals.append(timed)
            kwargs["departure_override"] = release
        start = perf_counter()
        if fit_passage:
            from .coupled_planning import convert_candidate
            catalog.passage_trials.add(trial_key)
            reference = replace(reference, candidate_id=catalog.attempted)
            fitted = {"candidate_id": catalog.attempted, "parent_candidate_id": passage.parent_candidate_id,
                      "cell": passage.cell, "accepted": False}
            catalog.passage_proposals.append(fitted)
            plan = convert_candidate(reference, snapshot, proposal_config, diagnostic, passage=passage)
            fitted.update(diagnostic.get("passage_fit", {}))
        else:
            if config.candidate_guidance == "rejections" and batch_attempt % 2 == 0:
                from .candidate_feedback import adjustment
                recipe = adjustment(proposal_config, catalog.rejections, len(catalog.candidate_proposals))
                if recipe is not None:
                    adjusted = {"candidate_id": catalog.attempted, **asdict(recipe), "accepted": False}
                    catalog.candidate_proposals.append(adjusted)
                    kwargs.update(recipe.kwargs())
            if config.passage_guidance != "none":
                kwargs["cache_reference"] = True
            plan = _candidate(snapshot, proposal_config, catalog.attempted, rng, angle, diagnostics=diagnostic, **kwargs)
            reference = diagnostic.pop("_geometric_reference", None)
        if plan is not None and aircraft_profile is not None:
            plan = replace(plan, aircraft_profile=aircraft_profile)
        if mixed and plan is not None:
            plan = replace(plan, profile_family=family.name)
        catalog.generation_s += perf_counter() - start
        catalog.attempted += 1
        batch_attempt += 1
        if plan is not None:
            start = perf_counter()
            valid = validate_individual(plan, snapshot, config, diagnostics=diagnostic)
            if valid and fitted is not None:
                from .passage_guidance import verify_passage
                valid, visits = verify_passage(plan, passage, fitted["selected_window_s"], config)
                fitted["actual_occupancy_s"] = visits
                fitted["window_verified"] = valid
                if not valid:
                    invalid(diagnostic, "validation_passage_window")
            catalog.validation_s += perf_counter() - start
            if valid:
                catalog.profile_failures = 0
                if proposed is not None:
                    proposed["accepted"] = True
                if guided is not None:
                    guided["accepted"] = True
                if timed is not None:
                    timed["accepted"] = True
                if fitted is not None:
                    fitted["accepted"] = True
                if adjusted is not None:
                    adjusted["accepted"] = True
                if reference is not None and config.passage_guidance != "none":
                    catalog.geometric_references[plan.candidate_id] = reference
                catalog.plans.append(plan)
                continue
        if fitted is not None:
            fitted["rejection"] = diagnostic.get("rejection", "individual_validation")
        if adjusted is not None:
            adjusted["rejection"] = diagnostic.get("rejection", "individual_validation")
        if mixed:
            catalog.profile_failures += 1
            if catalog.profile_failures >= max(2, config.initial_candidates):
                unlock("consecutive_rejections")
        reason = diagnostic.get("rejection", "individual_validation" if plan is not None else "proposal_rejected")
        catalog.rejections[reason] = catalog.rejections.get(reason, 0) + 1
        detail = diagnostic.get("rejection_detail", reason)
        catalog.rejection_details[detail] = catalog.rejection_details.get(detail, 0) + 1
        required = diagnostic.get("required_horizon_s")
        if required is not None:
            catalog.minimum_required_horizon_s = min(required, catalog.minimum_required_horizon_s or required)
    catalog.rng_state = rng.bit_generator.state
    return catalog
