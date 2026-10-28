"""Bounded deterministic joint search, reusable across catalog expansions."""

from collections import Counter
from itertools import combinations
import numpy as np


class JointSearch:
    def __init__(self, config):
        self.config = config
        self.cache = {}
        self.nodes = 0
        self.exhausted = False
        self.cache_hits = 0
        self.wipeouts = Counter()
        self.pairs = {}
        self.margins = {}
        self.optimization_nodes = 0
        self.optimization_budget_exhausted = False
        self.conflict_witnesses = {}
        self.occupancies = {}
        self.reservation_conflicts = 0
        self.first_reservation_conflict = None

    def occupied(self, plan):
        from .reservations import occupancy
        key = (plan.drone_id, plan.candidate_id)
        if key not in self.occupancies:
            self.occupancies[key] = occupancy(plan, self.config)
        return self.occupancies[key]

    def interval_hints(self, catalogs, indices, incumbent=None):
        if self.config.interval_guidance == "none" or any(not c.plans for c in catalogs):
            return {}
        from .interval_guidance import departure_windows
        references = ({p.drone_id: p for p in incumbent} if incumbent is not None else
                      {c.plans[0].drone_id: c.plans[0] for c in catalogs})
        hints = {}
        for i in indices:
            drone_id = catalogs[i].plans[0].drone_id
            parent = references[drone_id]
            blockers = [self.occupied(p) for drone, p in sorted(references.items()) if drone != drone_id]
            hints[drone_id] = departure_windows(parent, self.occupied(parent), blockers, self.config)
        return hints

    def passage_hints(self, catalogs, indices, incumbent=None):
        if self.config.passage_guidance == "none" or any(not c.plans for c in catalogs):
            return {}
        from .passage_guidance import passage_windows
        references = ({p.drone_id: p for p in incumbent} if incumbent is not None else
                      {c.plans[0].drone_id: c.plans[0] for c in catalogs})
        hints = {}
        for i in indices:
            drone_id = catalogs[i].plans[0].drone_id
            parent = references[drone_id]
            if parent.candidate_id not in catalogs[i].geometric_references:
                continue
            blockers = [self.occupied(p) for drone, p in sorted(references.items()) if drone != drone_id]
            hint = passage_windows(parent, self.occupied(parent), blockers, self.config)
            if hint is not None:
                hints[drone_id] = hint
        return hints

    def compatible(self, first, second):
        from .planning import continuous_distance_lower_bound, required_distance

        a, b = sorted((first, second), key=lambda p: p.drone_id)
        key = (a.drone_id, a.candidate_id, b.drone_id, b.candidate_id)
        if key in self.cache:
            self.cache_hits += 1
            return self.cache[key]
        lower = continuous_distance_lower_bound(a, b)
        required = required_distance(a, b, self.config)
        valid = lower >= required + 1e-7
        if valid and self.config.reservation_policy == "grid":
            from .reservations import first_conflict
            conflict = first_conflict(self.occupied(a), self.occupied(b))
            if conflict is not None:
                valid = False
                self.reservation_conflicts += 1
                if self.first_reservation_conflict is None:
                    self.first_reservation_conflict = conflict
        self.cache[key] = valid
        self.margins[key] = lower - required
        if not valid and self.config.conflict_guidance == "witness":
            from .conflict_guidance import witness
            pair_key = (a.drone_id, b.drone_id)
            previous = self.conflict_witnesses.get(pair_key)
            if previous is None or lower - required < previous.slack:
                self.conflict_witnesses[pair_key] = witness(a, b, lower, required)
        pair = self.pairs.setdefault((a.drone_id, b.drone_id), {
            "drone_ids": [a.drone_id, b.drone_id], "checks": 0, "conflicts": 0,
            "worst_lower_bound_m": lower, "required_at_worst_m": required,
            "closest_sample_time_s": None})
        pair["checks"] += 1
        pair["conflicts"] += int(not valid)
        if pair["closest_sample_time_s"] is None or lower < pair["worst_lower_bound_m"]:
            pair["worst_lower_bound_m"] = lower
            pair["required_at_worst_m"] = required
            pair["closest_sample_time_s"] = float(np.argmin(np.linalg.norm(a.positions - b.positions, axis=1)) * a.dt)
        return valid

    def solve(self, catalogs):
        """Global node budget and pair cache persist between calls."""
        if any(not domain for domain in catalogs):
            return None

        def visit(selected, domains):
            if not domains:
                return tuple(sorted(selected, key=lambda p: p.drone_id))
            index = (min(range(len(domains)), key=lambda i: (len(domains[i]), domains[i][0].drone_id))
                     if self.config.search_order == "mrv" else 0)
            domain = domains[index]
            others = domains[:index] + domains[index + 1:]
            for candidate in domain:
                if self.nodes >= self.config.max_search_nodes:
                    self.exhausted = True
                    return None
                self.nodes += 1
                remaining = []
                for future in others:
                    filtered = [other for other in future if self.compatible(candidate, other)]
                    if not filtered:
                        self.wipeouts[candidate.drone_id] += 1
                        self.wipeouts[future[0].drone_id] += 1
                        break
                    remaining.append(filtered)
                else:
                    solution = visit(selected + [candidate], remaining)
                    if solution is not None:
                        return solution
                    if self.exhausted:
                        return None
            return None

        return visit([], catalogs)

    def margin(self, first, second):
        a, b = sorted((first, second), key=lambda p: p.drone_id)
        key = (a.drone_id, a.candidate_id, b.drone_id, b.candidate_id)
        self.compatible(a, b)
        return self.margins[key]

    def score(self, plans):
        """Leximin robust pair slack in meters; obstacles remain hard constraints."""
        return tuple(sorted(self.margin(a, b) for a, b in combinations(plans, 2)))

    def pair_scores(self, plans):
        return [{"drone_ids": [a.drone_id, b.drone_id], "extra_margin_m": self.margin(a, b)}
                for a, b in combinations(plans, 2)]

    def optimization_available(self):
        return (self.nodes < self.config.max_search_nodes
                and self.optimization_nodes < self.config.margin_search_nodes)

    def refine(self, catalogs, incumbent):
        """Branch-and-bound over a finite catalog, retaining a feasible incumbent.

        The node limit is shared across refinement calls and also consumes the
        mission-wide search budget. Reaching it does not invalidate the incumbent.
        A sorted partial score padded with +infinity is an optimistic upper bound:
        adding pair margins cannot improve any sorted component already fixed.
        """
        best, best_score = tuple(incumbent), self.score(incumbent)
        pair_count = len(best_score)
        complete = True
        improvements = 0

        def visit(selected, domains, margins):
            nonlocal best, best_score, complete, improvements
            upper = tuple(sorted(margins)) + (float("inf"),) * (pair_count - len(margins))
            if upper <= best_score:
                return
            if not domains:
                best, best_score = tuple(sorted(selected, key=lambda p: p.drone_id)), upper
                improvements += 1
                return
            index = (min(range(len(domains)), key=lambda i: (len(domains[i]), domains[i][0].drone_id))
                     if self.config.search_order == "mrv" else 0)
            others = domains[:index] + domains[index + 1:]
            # Promising partial margins first; stable sort preserves candidate
            # order for ties and makes the bounded result reproducible.
            ranked = sorted(domains[index], key=lambda candidate: tuple(sorted(
                self.margin(candidate, previous) for previous in selected)), reverse=True)
            for candidate in ranked:
                added = [self.margin(candidate, previous) for previous in selected]
                if any(not self.compatible(candidate, previous) for previous in selected):
                    continue
                optimistic = tuple(sorted(margins + added)) + (float("inf"),) * (pair_count - len(margins) - len(added))
                if optimistic <= best_score:
                    continue
                if not self.optimization_available():
                    complete = False
                    self.optimization_budget_exhausted = True
                    return
                self.nodes += 1
                self.optimization_nodes += 1
                remaining = []
                for domain in others:
                    filtered = [other for other in domain if self.compatible(candidate, other)]
                    if not filtered:
                        break
                    remaining.append(filtered)
                else:
                    visit(selected + [candidate], remaining, margins + added)
                    if not complete:
                        return

        if pair_count:
            visit([], catalogs, [])
        return best, {"catalog_optimal": complete, "improvements": improvements,
                      "optimization_nodes": self.optimization_nodes,
                      "sorted_slack_m": list(best_score)}

    def margin_expansion_indices(self, catalogs, snapshots, plans):
        """Prioritize expandable endpoints of the current lowest-margin pairs."""
        indices = {s.drone_id: i for i, s in enumerate(snapshots)}
        selected = []
        ranked = sorted(combinations(plans, 2), key=lambda pair: (
            self.margin(*pair), pair[0].drone_id, pair[1].drone_id))
        for pair in ranked:
            for plan in pair:
                i = indices[plan.drone_id]
                c = catalogs[i]
                if (i not in selected and c.attempted < self.config.attempts
                        and len(c.plans) < self.config.candidates):
                    selected.append(i)
                    if len(selected) == 2:
                        return selected
        return selected

    def expansion_indices(self, catalogs, snapshots):
        """Expand at most two expandable conflict endpoints, deterministically.

        This targets WHICH catalogs to enrich, not a continuous-space repair
        of the conflicting path. Uninvolved catalogs keep their previous work.
        """
        available = [i for i, c in enumerate(catalogs)
                     if c.attempted < self.config.attempts and len(c.plans) < self.config.candidates]
        if not available:
            return []
        scores = Counter()
        for pair in self.pairs.values():
            for drone in pair["drone_ids"]:
                scores[drone] += pair["conflicts"]
        return sorted(available, key=lambda i: (-self.wipeouts[snapshots[i].drone_id],
                                               -scores[snapshots[i].drone_id],
                                               len(catalogs[i].plans), snapshots[i].drone_id))[:2]

    def diagnostics(self):
        return {"pair_checks": len(self.cache), "pair_cache_hits": self.cache_hits,
                "reservation_conflicts": self.reservation_conflicts,
                "first_reservation_conflict": self.first_reservation_conflict,
                "domain_wipeouts": {str(k): v for k, v in sorted(self.wipeouts.items())},
                "pairs": [self.pairs[k] for k in sorted(self.pairs)]}

    def repair_hints(self, drone_ids, plans=None):
        """Give each expanded catalog its worst explored pair's sampled witness.

        For margin refinement, use the current incumbent rather than historical
        rejected pairs. These offline hints contain no runtime telemetry.
        """
        if self.config.conflict_guidance != "witness":
            return {}
        from .conflict_guidance import RepairHint, witness
        from .planning import required_distance
        if plans is None:
            witnesses = list(self.conflict_witnesses.values())
        else:
            witnesses = [witness(a, b, self.margin(a, b) + required_distance(a, b, self.config),
                                 required_distance(a, b, self.config))
                         for a, b in combinations(plans, 2)]
        hints = {}
        for item in sorted(witnesses, key=lambda w: (w.slack, w.drone_ids, w.candidate_ids)):
            for drone_id in item.drone_ids:
                if drone_id in drone_ids and drone_id not in hints:
                    hints[drone_id] = RepairHint(item, drone_id)
        return hints
