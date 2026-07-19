"""Keep-rule for autoresearch candidates (plan 'Objective function' — enforced by the
evaluator, never by the proposing LLM).

A candidate is KEPT only if:
  * no auto-fail (hard collision, 10 m moving-object violation),
  * objective 1 and 2 do not regress past safety thresholds vs the baseline,
  * objective 2 or 3 improves, without an unjustified tradeoff against the other.

Never let a change 'optimize' collision avoidance by making Crusader so conservative
it fails mission tasks — the completion-drop guard below is that rule in code.

Single-episode wins never promote: callers must aggregate metrics across the full
suite x seeds first (aggregate() below) — evaluate() refuses n_episodes < min_episodes.
"""
from dataclasses import dataclass, field


@dataclass
class Thresholds:
    min_clearance_floor_m: float = 0.5   # absolute safety floor, any source
    clearance_regress_m: float = 0.25    # max allowed min-clearance drop vs baseline
    violation_regress: int = 0           # no new clearance violations allowed
    stall_regress: int = 0               # no new stalls allowed
    flag_regress: int = 2                # small at-risk flag increase tolerated...
    completion_drop: float = 0.0         # ...but partial-credit may never drop
    min_improvement: float = 1e-6
    min_episodes: int = 5                # small-sample fragility guard


@dataclass
class Decision:
    keep: bool
    reasons: list = field(default_factory=list)


def aggregate(metric_list):
    """Aggregate per-episode metrics dicts (same schema) into one comparable dict.
    Worst-case for safety numbers, mean for completion."""
    n = len(metric_list)
    if n == 0:
        raise ValueError("no episodes to aggregate")
    o1 = [m["objective1"] for m in metric_list]
    o2 = [m["objective2"] for m in metric_list]
    o3 = [m["objective3"] for m in metric_list]
    mins = [x["min_clearance_m"] for x in o1 if x["min_clearance_m"] is not None]
    cmins = [x["min_comms_clearance_m"] for x in o1 if x["min_comms_clearance_m"] is not None]
    return {
        "n_episodes": n,
        "objective1": {
            "min_clearance_m": min(mins) if mins else None,
            "min_comms_clearance_m": min(cmins) if cmins else None,
            "clearance_violations": sum(x["clearance_violations"] for x in o1),
            "keepout_violations": sum(x["keepout_violations"] for x in o1),
            "moving_10m_violations": sum(x["moving_10m_violations"] for x in o1),
            "hard_collisions": sum(x["hard_collisions"] for x in o1),
            "auto_fail": any(x["auto_fail"] for x in o1),
        },
        "objective2": {
            "at_risk_flags": sum(x["at_risk_flags"] for x in o2),
            "stalls": sum(x["stalls"] for x in o2),
            "flagged_s": round(sum(x["flagged_s"] for x in o2), 2),
        },
        "objective3": {
            "partial_credit": round(sum(x["partial_credit"] for x in o3) / n, 4),
            "completed_all": all(x["completed"] for x in o3),
        },
    }


def evaluate(candidate, baseline, th: Thresholds = None) -> Decision:
    """candidate/baseline are aggregate() outputs."""
    th = th or Thresholds()
    reasons = []

    if candidate["n_episodes"] < th.min_episodes:
        return Decision(False, [f"only {candidate['n_episodes']} episodes "
                                f"(< {th.min_episodes}) — small-sample guard"])

    c1, b1 = candidate["objective1"], baseline["objective1"]
    c2, b2 = candidate["objective2"], baseline["objective2"]
    c3, b3 = candidate["objective3"], baseline["objective3"]

    # hard fails
    if c1["auto_fail"]:
        return Decision(False, ["auto-fail: hard collision or 10 m moving-object violation"])
    for key in ("min_clearance_m", "min_comms_clearance_m"):
        if c1[key] is not None and c1[key] < th.min_clearance_floor_m:
            return Decision(False, [f"{key}={c1[key]} below safety floor "
                                    f"{th.min_clearance_floor_m}"])

    # regression guards (objectives 1 & 2)
    for key in ("min_clearance_m", "min_comms_clearance_m"):
        if c1[key] is not None and b1[key] is not None \
                and b1[key] - c1[key] > th.clearance_regress_m:
            return Decision(False, [f"{key} regressed {b1[key]}->{c1[key]}"])
    for key in ("clearance_violations", "keepout_violations"):
        if c1[key] - b1[key] > th.violation_regress:
            return Decision(False, [f"{key} regressed {b1[key]}->{c1[key]}"])
    if c2["stalls"] - b2["stalls"] > th.stall_regress:
        return Decision(False, [f"stalls regressed {b2['stalls']}->{c2['stalls']}"])
    if c2["at_risk_flags"] - b2["at_risk_flags"] > th.flag_regress:
        return Decision(False, [f"at-risk flags regressed "
                                f"{b2['at_risk_flags']}->{c2['at_risk_flags']}"])

    # blended-score-gaming guard: completion may never drop
    if b3["partial_credit"] - c3["partial_credit"] > th.completion_drop:
        return Decision(False, [f"completion dropped {b3['partial_credit']}"
                                f"->{c3['partial_credit']} (conservatism tradeoff rejected)"])

    # must actually improve objective 2 or 3
    improved = []
    if b2["flagged_s"] - c2["flagged_s"] > th.min_improvement:
        improved.append(f"flagged_s {b2['flagged_s']}->{c2['flagged_s']}")
    if b2["stalls"] - c2["stalls"] > 0:
        improved.append(f"stalls {b2['stalls']}->{c2['stalls']}")
    if c3["partial_credit"] - b3["partial_credit"] > th.min_improvement:
        improved.append(f"completion {b3['partial_credit']}->{c3['partial_credit']}")
    if not improved:
        return Decision(False, ["no improvement in objective 2 or 3"])

    return Decision(True, ["improved: " + "; ".join(improved)])
