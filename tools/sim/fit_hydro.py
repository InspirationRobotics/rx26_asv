#!/usr/bin/env python3
"""
Fit hydrodynamic coefficients to real trial data.

    THIS IS THE SCRIPT THAT MAKES THE SIM MEAN SOMETHING.

Right now every coefficient in tools/sim/models/crusader_omnix/model.sdf is an
estimate scaled from a comparable hull. That is the right order of magnitude and nothing
more. Until you run this, sim results are RELATIVE ("behaviour B is more robust
than A" — trustworthy) and never ABSOLUTE ("we will dock in 42 s" — not).

Three trials, three afternoons, and the numbers become real. That beats three
weeks of CFD at this timescale, and the logs are direct material for the TDR
Testing Strategy section (25% of 200 points) and the optional Test Plan &
Results appendix, which is eligible for a special judges' award.

-----------------------------------------------------------------------------
THE THREE TRIALS
-----------------------------------------------------------------------------
Run each 3+ times, both directions where it makes sense, in the calmest water
you can find. Log at >= 10 Hz. A MAVLink .tlog or a ROS 2 bag both work.

1. STEP THRUST                                        -> xU, xUU  (surge drag)
   From rest, command a fixed forward thrust. Hold until speed plateaus.
   Record thrust command and forward velocity.
   At terminal velocity acceleration is zero, so thrust exactly balances drag:
       T = xU*u + xUU*u*|u|
   Repeat at 3-4 thrust levels to separate the linear and quadratic terms.
   ONE THRUST LEVEL IS NOT ENOUGH - with a single point the fit is degenerate
   and you can trade xU against xUU freely.

2. TURNING CIRCLE                                     -> nR, nRR  (yaw drag)
   Steady forward speed, constant differential/yaw command, let it settle into
   a circle. Record yaw rate and turn radius.
   At steady state yaw moment balances yaw drag.

3. COAST-DOWN                                         -> xDotU  (added mass)
   Get to a steady speed, cut thrust to zero, log the deceleration curve.
   With thrust removed:  (m + added_mass) * du/dt = -(xU*u + xUU*u*|u|)
   Drag is already known from trial 1, so the decay rate gives added mass.
   THIS IS THE ONLY TRIAL THAT ISOLATES ADDED MASS. Do not skip it - added
   mass dominates the transient response, which is exactly what your controller
   feels during docking and station-keeping.

-----------------------------------------------------------------------------
INPUT FORMAT
-----------------------------------------------------------------------------
CSV per trial. Required columns:

    t,thrust_n,u          step thrust  (u = surge velocity, m/s)
    t,yaw_rate,u          turning circle (yaw_rate rad/s)
    t,u                   coast-down

    trials/usv/step_01.csv, trials/usv/turn_01.csv, trials/usv/coast_01.csv

-----------------------------------------------------------------------------
USAGE
    python3 tools/fit_hydro.py --vehicle usv --trials trials/usv --dry-run
    python3 tools/fit_hydro.py --vehicle usv --trials trials/usv --write
-----------------------------------------------------------------------------
STATUS: fitting maths implemented; NOT yet validated against real logs, because
no trials exist. Treat the first run as something to sanity-check by hand.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
MODEL = REPO / "tools" / "sim" / "models" / "crusader_omnix" / "model.sdf"


def read_csv(path: Path) -> list[dict]:
    with path.open() as fh:
        return [{k: float(v) for k, v in row.items() if v not in ("", None)}
                for row in csv.DictReader(fh)]


def lstsq2(rows: list[tuple[float, float, float]]) -> tuple[float, float]:
    """
    Least squares for  y = a*x1 + b*x2  (2x2 normal equations, no numpy).
    Returns (a, b).
    """
    s11 = sum(x1 * x1 for x1, _, _ in rows)
    s12 = sum(x1 * x2 for x1, x2, _ in rows)
    s22 = sum(x2 * x2 for _, x2, _ in rows)
    sy1 = sum(x1 * y for x1, _, y in rows)
    sy2 = sum(x2 * y for _, x2, y in rows)
    det = s11 * s22 - s12 * s12

    # Check CONDITIONING, not just exact singularity. Two thrust levels that
    # differ by 0.001 N are not singular - the determinant is small but
    # non-zero, so a naive `det == 0` test passes and the fit happily returns
    # coefficients driven entirely by numerical noise. They look plausible.
    # That is the failure mode this guard exists to prevent.
    scale = s11 * s22
    if scale <= 0 or abs(det) / scale < 1e-6:
        raise ValueError(
            "ill-conditioned fit: your thrust levels are too similar to "
            "separate linear from quadratic drag. The solver can trade xU "
            "against xUU almost freely, so the answer would be noise.\n"
            "Run the step-thrust trial at 3-4 CLEARLY different thrust levels "
            "- aim for a 3x spread in terminal velocity between the slowest "
            "and fastest.")
    return ((sy1 * s22 - sy2 * s12) / det,
            (sy2 * s11 - sy1 * s12) / det)


def terminal_velocity(rows: list[dict], tail_frac: float = 0.25) -> float:
    """Mean speed over the last tail_frac of the run, once it has plateaued."""
    us = [r["u"] for r in rows]
    n = max(1, int(len(us) * tail_frac))
    return sum(us[-n:]) / n


def fit_surge_drag(trials: list[Path]) -> tuple[float, float, list]:
    """T = xU*u + xUU*u|u| at terminal velocity, across thrust levels."""
    pts = []
    for p in trials:
        rows = read_csv(p)
        u = terminal_velocity(rows)
        T = sum(r["thrust_n"] for r in rows[-len(rows) // 4:]) / max(1, len(rows) // 4)
        pts.append((u, u * abs(u), T))
    if len(pts) < 2:
        raise ValueError("need >= 2 step-thrust trials at different thrust levels")
    xU, xUU = lstsq2(pts)
    return -abs(xU), -abs(xUU), pts        # Fossen convention: drag is negative


def fit_added_mass(coast: list[Path], mass_kg: float,
                   xU: float, xUU: float) -> tuple[float, list]:
    """
    Effective mass in surge is (m - xDotU), since Fossen writes added mass as a
    NEGATIVE coefficient. With thrust removed:

        (m - xDotU) * du/dt = drag           drag = xU*u + xUU*u|u|   (negative)

    Both drag and du/dt are negative during a coast-down, so their ratio is the
    positive effective mass:

        m - xDotU = drag / (du/dt)
        xDotU     = m - drag/(du/dt)         -> negative, as required

    An earlier version negated `drag` here as well, which double-counted the
    sign and returned an added mass ~7x too large. Caught by fitting synthetic
    trials generated from known coefficients - which is why that test exists.

    Median over the decay, skipping the noisy tail near zero speed.
    """
    ests = []
    for p in coast:
        rows = read_csv(p)
        # CENTRAL differences: second-order accurate, versus first-order for a
        # forward difference.
        #
        # Against synthetic trials with known coefficients (see
        # tools/test_fit_hydro.py) this recovers added mass to 0.6% at a 20 Hz
        # log rate and 0.0% at 200 Hz. So a normal autopilot log rate is fine -
        # sampling is NOT the limiting factor here.
        #
        # What IS limiting is water conditions. The estimator assumes the only
        # force acting is drag; wind, current or residual thrust all get
        # absorbed into the added-mass term. Check the spread the tool reports:
        # a tight spread means a clean decay, a wide one means something else
        # was pushing the vehicle. Retry in calmer water rather than averaging
        # harder.
        for i in range(1, len(rows) - 1):
            dt = rows[i + 1]["t"] - rows[i - 1]["t"]
            if dt <= 0:
                continue
            u = rows[i]["u"]
            if u < 0.15:                    # near-zero: signal is noise
                continue
            dudt = (rows[i + 1]["u"] - rows[i - 1]["u"]) / dt
            if dudt >= 0:                   # not decelerating; skip
                continue
            drag = xU * u + xUU * u * abs(u)
            ests.append(mass_kg - drag / dudt)     # -> xDotU, negative
    if not ests:
        raise ValueError("no usable coast-down samples (need a clean decay "
                         "above 0.15 m/s)")
    ests.sort()
    median = ests[len(ests) // 2]
    return -abs(median), ests


def fit_yaw_drag(turns: list[Path]) -> tuple[float, float, list]:
    """Steady-state yaw: moment balances nR*r + nRR*r|r|. Uses yaw_rate."""
    pts = []
    for p in turns:
        rows = read_csv(p)
        n = max(1, len(rows) // 4)
        r = sum(x["yaw_rate"] for x in rows[-n:]) / n
        m = sum(x.get("yaw_moment_nm", x.get("thrust_n", 0.0)) for x in rows[-n:]) / n
        pts.append((r, r * abs(r), m))
    if len(pts) < 2:
        raise ValueError("need >= 2 turning-circle trials at different rates")
    nR, nRR = lstsq2(pts)
    return -abs(nR), -abs(nRR), pts


def _model_mass() -> float:
    """Hull mass read from the Gazebo model, so the two cannot drift.

    If the model's mass changes, every fit made against the old mass is stale —
    added mass is solved as (m - xDotU) - m, so the mass assumption is baked
    into the answer.

    Returns:
        float: mass in kg from the model's <mass> tag.
    """
    import re
    m = re.search(r"<mass>([\d.]+)</mass>", MODEL.read_text())
    if not m:
        raise SystemExit(f"no <mass> found in {MODEL}")
    return float(m.group(1))


def _write_model(updates: dict, tdir, n_step: int, n_turn: int,
                 n_coast: int) -> None:
    """Patch fitted coefficients into the Hydrodynamics plugin block.

    Rewrites ONLY the tags actually fitted, leaving every other estimate alone,
    and stamps provenance so nobody later mistakes a fitted value for a seed.

    Args:
        updates: {group: {tag: value}} produced by the fits.
        tdir: trial directory, recorded in the provenance comment.
        n_step: number of step-thrust trials used.
        n_turn: number of turning-circle trials used.
        n_coast: number of coast-down trials used.
    """
    import datetime
    import re

    txt = MODEL.read_text(encoding="utf-8")
    for group, vals in updates.items():
        for tag, val in vals.items():
            pat = re.compile(rf"<{tag}>[-\d.eE+]*</{tag}>")
            if not pat.search(txt):
                print(f"  WARNING <{tag}> not present in model.sdf - skipped")
                continue
            txt = pat.sub(f"<{tag}>{val}</{tag}>", txt)

    stamp = (f'<!-- FITTED {datetime.date.today().isoformat()} from {tdir} '
             f'({n_step} step / {n_turn} turn / {n_coast} coast). '
             f'Tags not listed above are still ESTIMATES. -->')
    txt = txt.replace('<plugin filename="gz-sim-hydrodynamics-system"',
                      stamp + '\n    <plugin filename="gz-sim-hydrodynamics-system"',
                      1)
    MODEL.write_text(txt, encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    # ASV only: this repo is the ASV. The UUV and UAV live in separate repos
    # on their own devices and fit their own coefficients there.
    ap.add_argument("--vehicle", default="crusader", choices=["crusader"])
    ap.add_argument("--trials", required=True, help="directory of trial CSVs")
    ap.add_argument("--write", action="store_true",
                    help="update the vehicle YAML in place")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    tdir = Path(args.trials)
    if not tdir.is_dir():
        sys.exit(f"no such directory: {tdir}\n\n"
                 "Expected trial CSVs. See this file's docstring for the three "
                 "trials and the required columns.")

    mass = _model_mass()

    step = sorted(tdir.glob("step_*.csv"))
    turn = sorted(tdir.glob("turn_*.csv"))
    coast = sorted(tdir.glob("coast_*.csv"))

    print(f"vehicle {args.vehicle}   mass {mass} kg")
    print(f"trials: {len(step)} step, {len(turn)} turn, {len(coast)} coast\n")
    if not step:
        sys.exit("no step_*.csv found - surge drag must be fitted first, since "
                 "the coast-down fit depends on it.")

    xU, xUU, pts = fit_surge_drag(step)
    print(f"surge drag   xU  = {xU:9.3f}   xUU = {xUU:9.3f}")
    for u, _, T in pts:
        print(f"               u={u:5.2f} m/s  T={T:7.1f} N")

    updates = {"linear_drag": {"xU": round(xU, 3)},
               "quadratic_drag": {"xUU": round(xUU, 3)}}

    if coast:
        ma, ests = fit_added_mass(coast, mass, xU, xUU)
        spread = (max(ests) - min(ests)) / abs(ma) if ma else 0
        print(f"\nadded mass   xDotU = {ma:9.3f}   "
              f"({len(ests)} samples, spread {spread * 100:.0f}%)")
        if spread > 0.5:
            print("   WARNING wide spread - your coast-down is probably "
                  "contaminated by wind or current. Retry in calmer water.")
        updates["added_mass"] = {"xDotU": round(ma, 3)}
    else:
        print("\nno coast_*.csv - ADDED MASS NOT FITTED. This is the term that "
              "dominates transient response, which is what your controller "
              "feels while docking. Run the trial.")

    if turn:
        nR, nRR, _ = fit_yaw_drag(turn)
        print(f"yaw drag     nR  = {nR:9.3f}   nRR = {nRR:9.3f}")
        updates["linear_drag"]["nR"] = round(nR, 3)
        updates["quadratic_drag"]["nRR"] = round(nRR, 3)
    else:
        print("no turn_*.csv - yaw drag not fitted.")

    if args.write and not args.dry_run:
        _write_model(updates, tdir, len(step), len(turn), len(coast))
        print(f"\nwrote {MODEL.relative_to(REPO)}")
        print("Only the fitted terms changed. Everything else is still an estimate.")
        print("Re-run the gazebo-backend episodes you care about; coefficients")
        print("are part of the physics, so old numbers are not comparable.")
    else:
        print("\n(dry run - pass --write to update the model SDF)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
