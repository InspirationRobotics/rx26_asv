"""Proof of Readiness scorer — USV autonomous navigation demo (Handbook 3.1.2).

    *** SUBMISSION DEADLINE: 31 AUGUST 2026 ***

Scores an EpisodeResult against RoboNation's Proof of Readiness criteria, which
are a different question from the three objectives in evaluator/metrics.py.
Objectives ask "is this mechanism better?"; PoR asks the binary "will RoboNation
let this vehicle on the course?". Both read the same episode trace, so a PoR run
is an ordinary episode — no new backend, no new runner, no new scenario format.

Fail PoR and the USV does not deploy in Singapore, and the team loses
travel/shipping stipend eligibility. It is pass/fail, reviewed on a ROLLING
basis, with UNLIMITED resubmissions until the window closes — so the correct
strategy is to submit early and deliberately: a fail costs nothing and buys
reviewer feedback. Most teams waste that by submitting once, late.

HANDBOOK 3.1.2 REQUIREMENT
    In a fully autonomous run, the USV starts 3 m BEHIND the Gate and passes
    through BOTH sets of Gates.
      - must not strike any buoys
      - must demonstrate good autonomous control throughout

HANDBOOK 3.1.1 SUBMISSION CONSTRAINTS (checked here because they are easy and
cheap to fail on, and a resubmission cycle costs days)
      - one continuous recording, no breaks
      - <= 5 minutes, including the team-name intro
      - English, or English subtitles
      - school/organisation and team name at the beginning

ASV SCOPE
    This repo is the ASV only. The UUV Proof of Readiness (Handbook 3.1.3) —
    including the no-surface-breach vs positive-buoyancy tension that makes it
    the harder of the two — belongs in the UUV repo, which owns that vehicle.
    Nothing here reaches across domains.

WHAT SIMULATION CANNOT TELL YOU
    This scores the RUN. It cannot score the SUBMISSION: kill-switch wiring
    drawings, tow/lift point photos, propeller shrouds, the R/C block diagram
    and the visual-feedback-system video are physical artifacts. A pass here
    means the autonomy is ready, not that the package is complete.

Thresholds come from config/crusader_params.yaml (`por_usv` section) via
robotx_2026.api.common.config.por_usv_kwargs(), so they cannot drift from the
run the boat is flown against and are covered by the config sha256 already
recorded in every metrics JSON. tests/test_config_shared.py asserts agreement.

Stdlib only — the orchestrator must run anywhere (dev laptop, CI, Jetson host)
and never imports rclpy.
"""
import math


class PorUsvScorer:
    """Scores one episode trace against Handbook 3.1.2 / 3.1.1.

    Construct with config.por_usv_kwargs(); call score() with an EpisodeResult
    and its Scenario.
    """

    def __init__(self, start_distance_m=3.0, start_tolerance_m=1.5,
                 max_video_s=300.0, video_warn_s=240.0,
                 gate_centre_tolerance=0.6, contact_margin_m=0.0):
        """
        Args:
            start_distance_m (float): handbook start offset behind gate 1.
            start_tolerance_m (float): acceptance band around it.
            max_video_s (float): hard 5-minute single-take limit.
            video_warn_s (float): warn threshold ("tight once intro is added").
            gate_centre_tolerance (float): fraction of half-gate-width beyond
                which a transit is reported as grazing (diagnostic only).
            contact_margin_m (float): clearance below this counts as contact.
        """
        self.start_distance_m = start_distance_m
        self.start_tolerance_m = start_tolerance_m
        self.max_video_s = max_video_s
        self.video_warn_s = video_warn_s
        self.gate_centre_tolerance = gate_centre_tolerance
        self.contact_margin_m = contact_margin_m

    # ------------------------------------------------------------------ api #

    def score(self, result, scenario):
        """Score an episode against the PoR criteria.

        Args:
            result (EpisodeResult): trace + flags from EpisodeRunner.
            scenario (Scenario): the PoR gate course (obstacles = gate buoys).

        Returns:
            dict: {"passed": bool, "criteria": {name: {...}}, "notes": [str]}
                  Every criterion carries pass/fail plus a detail string, so a
                  failed submission says WHY without re-running.
        """
        track = [(s.x, s.y) for s in result.trace]
        gates = self._gates(scenario)
        criteria = {}
        notes = []

        if not track:
            return {"passed": False,
                    "criteria": {"has_trace": self._c(False, "empty trace")},
                    "notes": ["no trace recorded"]}

        if len(gates) < 2:
            notes.append(
                f"scenario declares {len(gates)} gate(s); PoR needs 2 — pair "
                "obstacles by y with labels containing 'red'/'green'")

        # --- start 3 m behind gate 1 ---------------------------------------
        criteria["start_behind_gate"] = self._score_start(track, gates)

        # --- transit both gates, in order ----------------------------------
        crossings = []
        for i, g in enumerate(gates[:2], start=1):
            xs = self._gate_crossings(track, g[0], g[1])
            crossings.append(xs)
            criteria[f"pass_through_gate_{i}"] = self._c(
                bool(xs), self._crossing_detail(xs, f"gate {i}"))

        if len(crossings) >= 2:
            ordered = bool(crossings[0] and crossings[1]
                           and crossings[0][0]["index"] < crossings[1][0]["index"])
            criteria["gates_in_order"] = self._c(
                ordered, "gate 1 must be transited before gate 2")

        # --- no buoy contact ------------------------------------------------
        criteria["no_buoy_contact"] = self._score_contact(track, scenario)

        # --- fully autonomous throughout ------------------------------------
        criteria["fully_autonomous_throughout"] = self._score_autonomy(result)

        # --- run fits the 5-minute single take ------------------------------
        dur = result.trace[-1].t if result.trace else 0.0
        fits = dur <= self.max_video_s
        criteria["run_fits_video_limit"] = self._c(
            fits,
            f"run {dur:.0f} s, limit {self.max_video_s:.0f} s "
            "(single continuous take, including your team-name intro)")
        if fits and dur > self.video_warn_s:
            notes.append(
                f"run is {dur:.0f} s — inside the limit but tight once the "
                "team-name intro is added")

        if result.timed_out:
            notes.append("episode timed out — not a valid PoR attempt")

        passed = all(c["passed"] for c in criteria.values())
        return {"passed": passed, "criteria": criteria, "notes": notes}

    # -------------------------------------------------------------- scoring #

    def _score_start(self, track, gates):
        """Handbook: 'the USV must start 3m behind the Gate'."""
        if not gates:
            return self._c(False, "no gate found in scenario")
        mid = self._midpoint(gates[0])
        d = math.hypot(track[0][0] - mid[0], track[0][1] - mid[1])
        lo = self.start_distance_m - self.start_tolerance_m
        hi = self.start_distance_m + self.start_tolerance_m
        return self._c(
            lo <= d <= hi,
            f"started {d:.2f} m from gate 1 centre "
            f"(want {self.start_distance_m} m, accept {lo:.1f}–{hi:.1f})")

    def _score_contact(self, track, scenario):
        """Handbook: 'USV must not strike any buoys'."""
        worst = math.inf
        worst_label = None
        for o in scenario.obstacles:
            d = min(math.hypot(x - o.x, y - o.y) for x, y in track) - o.radius
            if d < worst:
                worst, worst_label = d, o.label
        if worst is math.inf:
            return self._c(True, "no obstacles in scenario")
        return self._c(
            worst > self.contact_margin_m,
            f"closest approach {worst:.2f} m (to {worst_label})")

    def _score_autonomy(self, result):
        """Handbook: 'must demonstrate good autonomous control throughout'.

        Judges look for the visual mode indicator AND hands off the controllers.
        Backends that expose a per-sample mode are checked strictly; those that
        do not (kinematic) are reported as unverifiable rather than silently
        passed — a criterion that always passes is not a criterion.
        """
        modes = [getattr(s, "mode", None) for s in result.trace]
        known = [m for m in modes if m is not None]
        if not known:
            return self._c(
                True,
                "backend does not report mode — NOT VERIFIED here; confirm on "
                "the submission video that the indicator reads AUTO throughout",
                verified=False)
        bad = [m for m in known if m != "AUTO"]
        return self._c(
            not bad,
            f"{len(bad)}/{len(known)} samples not in AUTO"
            + (f" (saw {sorted(set(bad))})" if bad else ""))

    # -------------------------------------------------------------- helpers #

    @staticmethod
    def _c(passed, detail, verified=True):
        return {"passed": bool(passed), "detail": detail, "verified": verified}

    @staticmethod
    def _midpoint(gate):
        a, b = gate
        return ((a.x + b.x) / 2.0, (a.y + b.y) / 2.0)

    @staticmethod
    def _gates(scenario):
        """Pair scenario obstacles into gates by shared y, red/green by label.

        The PoR course is two gates of a red and a green marker (Handbook
        3.1.2 references Taylor Made Sur-Mark 950410 red / 950400 green).
        Returns [(port_obstacle, starboard_obstacle), ...] ordered by y.
        """
        rows = {}
        for o in scenario.obstacles:
            rows.setdefault(round(o.y, 2), []).append(o)
        gates = []
        for y in sorted(rows):
            pair = rows[y]
            if len(pair) == 2:
                gates.append(tuple(sorted(pair, key=lambda o: o.x)))
        return gates

    @staticmethod
    def _gate_crossings(track, a, b):
        """Every pass BETWEEN two gate posts.

        Only true segment intersections count. A boat that rounds the OUTSIDE
        of a post crosses the infinite gate line but not the segment, and is
        correctly not credited — the case a naive line-side test gets wrong.

        Returns:
            list[dict]: {"index", "point", "u"} where u is 0..1 along a->b.
        """
        out = []
        ax, ay, bx, by = a.x, a.y, b.x, b.y
        gx, gy = bx - ax, by - ay
        for i in range(1, len(track)):
            px, py = track[i - 1]
            qx, qy = track[i]
            rx, ry = qx - px, qy - py
            denom = gx * ry - gy * rx
            if abs(denom) < 1e-12:
                continue
            dx, dy = px - ax, py - ay
            u = (dx * ry - dy * rx) / denom
            t = (dx * gy - dy * gx) / denom
            if 0.0 <= u <= 1.0 and 0.0 <= t <= 1.0:
                out.append({"index": i, "point": (px + t * rx, py + t * ry),
                            "u": u})
        return out

    def _crossing_detail(self, xs, label):
        if not xs:
            return f"never passed between the {label} posts"
        u = xs[0]["u"]
        off = abs(u - 0.5) * 2.0
        grazed = off > self.gate_centre_tolerance
        return (f"{label} crossed at u={u:.2f} ({off * 100:.0f}% off centre)"
                + ("  GRAZING — tighten the approach" if grazed else ""))


def score_por_usv(result, scenario, path=None):
    """Convenience wrapper: build the scorer from config and score one episode.

    Args:
        result (EpisodeResult): from EpisodeRunner.
        scenario (Scenario): the PoR gate course.
        path: optional config path override (tests).

    Returns:
        dict: as PorUsvScorer.score().
    """
    from robotx_2026.api.common import config as crsd_config
    return PorUsvScorer(**crsd_config.por_usv_kwargs(path)).score(result, scenario)
