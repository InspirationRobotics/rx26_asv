"""Level 1.5 — outer search-strategy loop (parameter redirection).

CONSTRAINT (CLAUDE.md, hard scope line): this class may only change WHICH
parameters get attention — freeze/unfreeze sets and a guidance string. It never
touches proposal logic, the keep/discard rule, or loop structure. Wanting it to
"just add a recovery behavior" is scope creep into Level 2.

Behavior:
  * every k_inner iterations, freeze any parameter proposed >= freeze_after
    times with zero net improvement (no keeps);
  * unfreeze stale-frozen parameters when the dominant failing objective
    changes (the search has moved to a new failure mode);
  * emit guidance pointed at the currently underperforming objective.
"""


class FreezeRedirect:
    def __init__(self, k_inner: int = 5, freeze_after: int = 3):
        self.k_inner = k_inner
        self.freeze_after = freeze_after
        self.frozen = set()
        self.guidance = ""
        self._frozen_under = {}        # param -> objective focus when frozen
        self._cleared_at = {}          # param -> history index when unfrozen
        self._last_focus = None
        self.events = []               # (iteration, action, detail) audit log

    # ---- objective diagnosis (which objective is underperforming?) ----

    @staticmethod
    def _dominant_failure(agg) -> int:
        if agg is None:
            return 3
        o1, o2, o3 = agg["objective1"], agg["objective2"], agg["objective3"]
        if o1["hard_collisions"] or o1["clearance_violations"] \
                or o1["keepout_violations"] or o1["moving_10m_violations"]:
            return 1
        if o2["stalls"] or o2["flagged_s"] > 0:
            return 2
        if o3["partial_credit"] < 1.0:
            return 3
        return 0                        # nothing failing

    # ---- the update hook (called by Level1Loop after every iteration) ----

    def update(self, history, best_agg):
        n = len(history)
        if n == 0 or n % self.k_inner != 0:
            return

        focus = self._dominant_failure(best_agg)

        # unfreeze stale-frozen params when the failure mode moved on
        if self._last_focus is not None and focus != self._last_focus:
            stale = {p for p, under in self._frozen_under.items()
                     if under == self._last_focus}
            if stale:
                self.frozen -= stale
                for p in stale:
                    self._frozen_under.pop(p, None)
                    self._cleared_at[p] = n       # fresh chance: count from here
                self.events.append((n, "unfreeze",
                                    f"failure mode {self._last_focus}->{focus}: "
                                    f"{sorted(stale)}"))
        self._last_focus = focus

        # freeze params repeatedly proposed with zero net improvement —
        # counting only proposals SINCE the param was last unfrozen, so an
        # unfreeze actually grants a fresh chance instead of instant re-freeze
        counts, keeps = {}, {}
        for idx, r in enumerate(history):
            if idx < self._cleared_at.get(r["param"], 0):
                continue
            counts[r["param"]] = counts.get(r["param"], 0) + 1
            keeps[r["param"]] = keeps.get(r["param"], 0) + (1 if r["kept"] else 0)
        for param, count in counts.items():
            if param in self.frozen:
                continue
            if count >= self.freeze_after and keeps.get(param, 0) == 0:
                self.frozen.add(param)
                self._frozen_under[param] = focus
                self.events.append((n, "freeze",
                                    f"{param}: {count} proposals, 0 kept"))

        self.guidance = {
            0: "all objectives green — explore for headroom without regressing",
            1: "focus obj1: clearance violations present — prioritize avoidance "
               "margins and repulsion-related parameters",
            2: "focus obj2: local-minima flags/stalls present — prioritize "
               "escape/trap-related and speed parameters",
            3: "focus obj3: mission completion below 1.0 — prioritize "
               "progress/goal-seeking parameters without regressing safety",
        }[focus]
