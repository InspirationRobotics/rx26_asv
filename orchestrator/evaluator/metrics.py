"""3-objective episode metrics (plan 'Objective function' — never collapse to one scalar).

Objective 1 — collision probability proxy: min clearance + violation counts + hard
collisions, tracked SEPARATELY for perception-detected obstacles and comms-reported
virtual obstacles (keep-outs, moving objects with the hard 10 m rule).
Objective 2 — local-minima incidence: preventative at-risk flags (low speed +
heading oscillation + no progress over a rolling window), escalation to stall,
recovery time.
Objective 3 — mission/task completion: per-element pass/fail + partial credit;
Mission-4 sub-metrics (ack timing, resume fidelity, comms compliance) emit as None
until the mission planner exists (Phase 4) — explicitly null, never silently absent.

`objective1.trusted` stays False until the buoy model is retrained on Crusader's own
buoys (Gate G2): sim/bench collision metrics are not representative before that.
"""
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path
from statistics import pstdev

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # repo root

MOVING_CLEARANCE_M = 10.0    # Mission 4 Disruptive hard rule (scenario.MovingObject)


def _shared_thresholds():
    """Objective-2 thresholds from config/crusader_params.yaml — the SAME file
    the onboard roa_apf_node reads, so the scorer and the boat's preventative
    detector cannot drift apart (Phase 3.5 single-source rule)."""
    from robotx_2026.api.common import config as crsd_config
    return crsd_config.monitor_kwargs()


# ---------- objective 1 ----------

def objective1(result, scenario, trusted=False):
    per_min = math.inf          # perception-source min clearance
    comms_min = math.inf        # comms-source min clearance
    per_viol = comms_viol = hard = moving_viol = 0
    in_per_viol = in_comms_viol = in_moving_viol = False

    for s in result.trace:
        # physical obstacles (perception-detected in the real pipeline)
        d = min((math.hypot(o.x - s.x, o.y - s.y) - o.radius
                 for o in scenario.obstacles), default=math.inf)
        per_min = min(per_min, d)
        if d <= 0:
            hard += 1
        below = d < scenario.avoid_margin
        if below and not in_per_viol:
            per_viol += 1
        in_per_viol = below

        # comms keep-outs (active window only)
        dk = min((math.hypot(k.x - s.x, k.y - s.y) - k.radius
                  for k in result.keepouts
                  if k.active_from <= s.t <= k.active_until), default=math.inf)
        comms_min = min(comms_min, dk)
        below_k = dk < 0            # inside the zone = violation (zone edge is the line)
        if below_k and not in_comms_viol:
            comms_viol += 1
        in_comms_viol = below_k

        # moving objects: hard 10 m threshold from CURRENT position
        # (projected-position clearance lands with the Phase-3 avoidance layer)
        dm = min((math.hypot(mx - s.x, my - s.y)
                  for m in result.moving_objects if s.t >= m.active_from
                  for (mx, my) in [m.position(s.t)]), default=math.inf)
        if dm is not math.inf:
            comms_min = min(comms_min, dm - MOVING_CLEARANCE_M)
        below_m = dm < MOVING_CLEARANCE_M
        if below_m and not in_moving_viol:
            moving_viol += 1
        in_moving_viol = below_m

    return {
        "min_clearance_m": None if per_min is math.inf else round(per_min, 3),
        "min_comms_clearance_m": None if comms_min is math.inf else round(comms_min, 3),
        "clearance_violations": per_viol,
        "keepout_violations": comms_viol,
        "moving_10m_violations": moving_viol,
        "hard_collisions": hard,
        "auto_fail": hard > 0 or moving_viol > 0,   # both are hard thresholds
        "trusted": trusted,
    }


# ---------- objective 2 ----------

def objective2(result, scenario, window_s=None, speed_floor=None,
               progress_floor=None, heading_osc_floor=None, stall_after_s=None):
    """Explicit args override the shared config (tests only — production runs
    must score with the boat's own thresholds)."""
    cfg = _shared_thresholds()
    window_s = cfg["window_s"] if window_s is None else window_s
    speed_floor = cfg["speed_floor"] if speed_floor is None else speed_floor
    progress_floor = (cfg["progress_floor"] if progress_floor is None
                      else progress_floor)
    heading_osc_floor = (cfg["heading_osc_floor"] if heading_osc_floor is None
                         else heading_osc_floor)
    stall_after_s = (cfg["stall_after_s"] if stall_after_s is None
                     else stall_after_s)
    trace = result.trace
    if len(trace) < 3:
        return {"at_risk_flags": 0, "stalls": 0, "escapes": 0,
                "mean_recovery_s": None, "flagged_s": 0.0, "causes": {}}
    flags = stalls = 0
    flagged_s = 0.0
    recoveries = []
    causes = {"obstacle": 0, "comms": 0, "other": 0}
    in_flag = False
    flag_start = 0.0

    fallback_goal = scenario.waypoints[-1]
    dt = trace[1].t - trace[0].t if len(trace) > 1 else 0.1
    win = max(2, int(window_s / dt))

    def goal_of(sample):
        if sample.goal_x is None:
            return fallback_goal
        return (sample.goal_x, sample.goal_y)

    for i in range(win, len(trace)):
        s = trace[i]
        past = trace[i - win]
        window = trace[i - win:i]
        gx, gy = goal_of(s)
        # commanded-hold guard (same semantics as ProgressMonitor.goal_radius):
        # sitting ON the commanded goal — loiter, station-keep — is not a stall
        at_goal = math.hypot(gx - s.x, gy - s.y) <= scenario.wp_radius * 1.5
        # goal changed inside the window (task switch / interrupt divert):
        # progress-vs-goal is ill-defined across the change — skip
        goal_stable = all(goal_of(p) == (gx, gy) for p in window)
        speed_avg = sum(p.speed for p in window) / win
        progress = (math.hypot(gx - past.x, gy - past.y)
                    - math.hypot(gx - s.x, gy - s.y))
        osc = pstdev([p.heading for p in window])
        at_risk = (not at_goal and goal_stable
                   and speed_avg < speed_floor and progress < progress_floor
                   and osc > heading_osc_floor)
        if at_risk and not in_flag:
            flags += 1
            in_flag = True
            flag_start = s.t
            # attribute a cause at flag time (named failure-mode: keep sources separate)
            d_obs = min((math.hypot(o.x - s.x, o.y - s.y) for o in scenario.obstacles),
                        default=math.inf)
            d_com = min((math.hypot(k.x - s.x, k.y - s.y) for k in result.keepouts
                         if k.active_from <= s.t <= k.active_until), default=math.inf)
            if d_obs < scenario.avoid_margin * 2:
                causes["obstacle"] += 1
            elif d_com < scenario.avoid_margin * 2:
                causes["comms"] += 1
            else:
                causes["other"] += 1
        elif not at_risk and in_flag:
            in_flag = False
            dur = s.t - flag_start
            flagged_s += dur
            recoveries.append(dur)
            if dur > stall_after_s:
                stalls += 1
    if in_flag:                       # episode ended while flagged = unrecovered stall
        dur = trace[-1].t - flag_start
        flagged_s += dur
        if dur > stall_after_s:
            stalls += 1

    return {
        "at_risk_flags": flags,
        "stalls": stalls,
        "escapes": len(recoveries),
        "mean_recovery_s": round(sum(recoveries) / len(recoveries), 2) if recoveries else None,
        "flagged_s": round(flagged_s, 2),
        "causes": causes,
    }


# ---------- mission 4 sub-metrics (from MissionPlanner logs, not claims) ----------

# required outbound acks per inbound event type. ACK_INTENT is a COMMITMENT
# sent when servicing actually begins (may be deferred behind an active
# interrupt), so like READINESS/RESUMPTION it counts toward correctness and
# compliance but not toward immediate-ack latency.
_IMMEDIATE = {"assistance_request": ["ACK_RECEIPT"],
              "keep_out_zone": ["KEEPOUT_ACK"],
              "all_clear": ["ALLCLEAR_ACK"]}
_EVENTUAL = {"assistance_request": ["ACK_INTENT", "READINESS", "RESUMPTION"]}


def score_mission4(planner):
    """planner: MissionPlanner (or any object with event_log, comms_log,
    resume_record). Returns the objective-3 mission4 dict."""
    required = []                       # (kind, ref, t_event, immediate)
    for (t, etype, ref) in planner.event_log:
        for kind in _IMMEDIATE.get(etype, []):
            required.append((kind, ref, t, True))
        for kind in _EVENTUAL.get(etype, []):
            required.append((kind, ref, t, False))
    if not required:
        return {"ack_correct": None, "ack_latency_s": None,
                "resume_fidelity": None, "comms_compliance": None}

    sent = list(planner.comms_log)      # (t, kind, ref)
    found, latencies, order_ok = 0, [], True
    for kind, ref, t_event, immediate in required:
        match = next((s for s in sent
                      if s[1] == kind and s[2] == ref and s[0] >= t_event), None)
        if match is None:
            continue
        found += 1
        if immediate:
            latencies.append(match[0] - t_event)
    # ordering per assistance request: receipt <= intent <= readiness <= resumption
    for (_, etype, ref) in planner.event_log:
        if etype != "assistance_request":
            continue
        ts = {}
        for (t, kind, r) in sent:
            if r == ref and kind not in ts:
                ts[kind] = t
        seq = ["ACK_RECEIPT", "ACK_INTENT", "READINESS", "RESUMPTION"]
        got = [ts.get(k) for k in seq]
        if None in got or got != sorted(got):
            order_ok = False

    compliance = found / len(required)
    fidelity = None
    if any(e[1] == "assistance_request" for e in planner.event_log):
        rr = planner.resume_record
        fidelity = bool(rr and rr["pre_suspend"] == rr["post_resume"])
    return {
        "ack_correct": order_ok and compliance == 1.0,
        "ack_latency_s": round(max(latencies), 3) if latencies else None,
        "resume_fidelity": fidelity,
        "comms_compliance": round(compliance, 3),
    }


# ---------- objective 3 ----------

def objective3(result, mission4=None):
    total = len(result.waypoints_reached)
    done = sum(result.waypoints_reached)
    return {
        "elements": {f"waypoint_{i}": bool(r)
                     for i, r in enumerate(result.waypoints_reached)},
        "partial_credit": round(done / total, 3) if total else 0.0,
        "completed": bool(total and done == total and not result.timed_out),
        "timed_out": result.timed_out,
        # filled by score_mission4(planner) on planner-driven episodes;
        # explicit nulls on plain scripted episodes
        "mission4": mission4 or {"ack_correct": None, "ack_latency_s": None,
                                 "resume_fidelity": None,
                                 "comms_compliance": None},
    }


# ---------- assembly ----------

def _git_sha():
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                              capture_output=True, text=True, timeout=5
                              ).stdout.strip() or "unknown"
    except Exception:
        return "unknown"


def _param_hash(path):
    if not path:
        return None
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()[:12]
    except OSError:
        return None


def assemble(result, scenario, fidelity="sim", params_file=None,
             perception_trusted=False, planner=None):
    """planner: pass the MissionPlanner from a PlannerEpisodeRunner run to
    score the mission4 sub-metrics from its logs."""
    from robotx_2026.api.common import config as crsd_config
    mission4 = score_mission4(planner) if planner is not None else None
    return {
        "schema": "rx26-episode-metrics/1",
        "ros_config_hash": crsd_config.config_hash(),
        "scenario": scenario.name,
        "scenario_version": scenario.version,
        "seed": result.seed,
        "fidelity": fidelity,                # sim | bench | field
        "git_sha": _git_sha(),
        "param_hash": _param_hash(params_file),
        "wall_clock_s": round(result.wall_clock_s, 2),
        "stopped_early": result.stopped_early,
        "objective1": objective1(result, scenario, trusted=perception_trusted),
        "objective2": objective2(result, scenario),
        "objective3": objective3(result, mission4=mission4),
    }


def write_json(metrics, path):
    with open(path, "w") as f:
        json.dump(metrics, f, indent=2)
    return path
