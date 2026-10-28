"""Explicit mixed proposal families; no flight or safety limits are relaxed."""

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class ProfileFamily:
    name: str
    levels: int
    speed: str


def families(config):
    """Ordered finite menu permitted by the explicit mixed-policy configuration."""
    result = [ProfileFamily("baseline", 1, "constant")]
    result += [ProfileFamily(f"levels_{n}", n, "constant")
               for n in range(2, config.cruise_levels + 1)]
    if config.speed_profile == "smooth":
        result.append(ProfileFamily("speed", 1, "smooth"))
        result += [ProfileFamily(f"levels_{n}_speed", n, "smooth")
                   for n in range(2, config.cruise_levels + 1)]
    return tuple(result)


def family_config(config, family):
    """Change reference requirements only; leave every physical/safety cap intact."""
    return replace(config, profile_policy="strict", cruise_levels=family.levels,
                   speed_profile=family.speed,
                   passage_guidance=config.passage_guidance if family.speed == "smooth" else "none")


def validation_config(plan, config):
    """Resolve declared family, then use the existing strict physical validator.

    A missing/unknown declaration is an error, not permission to infer a less
    demanding profile from missing metadata. The strict validator checks actual
    level attainment and speed-reference data against the declared family.
    """
    if config.profile_policy == "strict":
        if plan.profile_family is not None:
            raise ValueError("mixed family declaration requires adaptive_mixed policy")
        return config
    for family in families(config):
        if family.name == plan.profile_family:
            return family_config(config, family)
    raise ValueError("missing or unsupported mixed profile family")
