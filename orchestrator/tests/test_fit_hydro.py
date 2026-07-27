"""Hydrodynamic coefficient fitter — recovers known coefficients from synthetic
trials.

WHY THIS EXISTS
---------------
A coefficient fitter is the worst kind of code to leave untested: it always
returns a plausible-looking number. No crash, no stack trace, nothing that looks
wrong. You wire the output into the Gazebo model, tune against it for weeks, and
never learn the added mass was 7x too large.

Not hypothetical — the first version of fit_added_mass() double-negated the drag
term and returned -80 against a truth of -12. It printed cleanly.

Method: generate motion from known (xU, xUU, xDotU) with RK4 at a fine step,
decimate to a realistic log rate, feed the trajectories back in, assert we
recover the inputs. The fine-step-then-decimate split matters: an earlier
version integrated with forward Euler AT the log rate, so the generator's own
error swamped the fitter's and made log-rate sensitivity look far worse than it
is.

ASV scope: coefficients here are Crusader's. The UUV and UAV fit their own in
their own repos.
"""
import csv
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "sim"))

from fit_hydro import fit_added_mass, fit_surge_drag        # noqa: E402

TRUTH = {"mass": 35.0, "xU": -35.0, "xUU": -50.0, "xDotU": -12.0}
SOLVER_DT = 0.0002       # fine integration; generator error negligible


def _integrate(u0, thrust, eff, xU, xUU, duration, log_dt):
    """RK4 at SOLVER_DT, sampled every log_dt — separates integration error
    from SAMPLING error. A real vehicle isn't Euler-integrated; it's sampled by
    the autopilot at whatever rate you configured.

    Args:
        u0 (float): initial surge speed, m/s.
        thrust (float): constant thrust, N (0 for coast-down).
        eff (float): effective mass, m - xDotU.
        xU, xUU (float): linear and quadratic drag.
        duration (float): seconds to integrate.
        log_dt (float): sample interval written to the CSV.

    Returns:
        list[dict]: rows with t, thrust_n, u.
    """
    def accel(u):
        return (thrust + xU * u + xUU * u * abs(u)) / eff

    rows, u, t, next_log = [], u0, 0.0, 0.0
    for _ in range(int(duration / SOLVER_DT)):
        if t >= next_log:
            rows.append({"t": round(t, 6), "thrust_n": thrust,
                         "u": round(u, 8)})
            next_log += log_dt
        k1 = accel(u)
        k2 = accel(u + 0.5 * SOLVER_DT * k1)
        k3 = accel(u + 0.5 * SOLVER_DT * k2)
        k4 = accel(u + SOLVER_DT * k3)
        u += (SOLVER_DT / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        t += SOLVER_DT
        if thrust == 0.0 and u <= 0.05:
            break
    return rows


def _synth(dirpath, log_dt):
    """Write step-thrust and coast-down trials for the known coefficients."""
    eff = TRUTH["mass"] - TRUTH["xDotU"]
    dirpath.mkdir(parents=True, exist_ok=True)

    for i, T in enumerate([40.0, 90.0, 160.0, 250.0], 1):
        rows = _integrate(0.0, T, eff, TRUTH["xU"], TRUTH["xUU"], 60.0, log_dt)
        with (dirpath / f"step_{i:02d}.csv").open("w", newline="") as fh:
            w = csv.DictWriter(fh, ["t", "thrust_n", "u"])
            w.writeheader()
            w.writerows(rows)

    for i, u0 in enumerate([2.0, 2.6], 1):
        rows = _integrate(u0, 0.0, eff, TRUTH["xU"], TRUTH["xUU"], 30.0, log_dt)
        with (dirpath / f"coast_{i:02d}.csv").open("w", newline="") as fh:
            w = csv.DictWriter(fh, ["t", "u"])
            w.writeheader()
            w.writerows([{"t": r["t"], "u": r["u"]} for r in rows])
    return dirpath


@pytest.fixture(scope="module")
def trials(tmp_path_factory):
    base = tmp_path_factory.mktemp("trials")
    return {hz: _synth(base / f"hz{hz}", dt)
            for hz, dt in ((20, 0.05), (50, 0.02), (200, 0.005))}


# -------------------------------------------------------------- surge drag #

def test_recovers_surge_drag_at_normal_log_rate(trials):
    xU, xUU, _ = fit_surge_drag(sorted(trials[20].glob("step_*.csv")))
    assert xU == pytest.approx(TRUTH["xU"], abs=0.5)
    assert xUU == pytest.approx(TRUTH["xUU"], abs=0.5)


def test_drag_coefficients_are_negative(trials):
    """Fossen convention: drag opposes motion."""
    xU, xUU, _ = fit_surge_drag(sorted(trials[200].glob("step_*.csv")))
    assert xU < 0 and xUU < 0


# -------------------------------------------------------------- added mass #

@pytest.mark.parametrize("hz,tol_pct", [(20, 3.0), (50, 2.0), (200, 2.0)])
def test_recovers_added_mass(trials, hz, tol_pct):
    """A normal autopilot log rate is enough — sampling is NOT the limiting
    factor, which is worth knowing because it means you needn't fight your
    logging config. Water conditions are the real limit: the estimator assumes
    drag is the only force, so wind and current land in the added-mass term."""
    d = trials[hz]
    xU, xUU, _ = fit_surge_drag(sorted(d.glob("step_*.csv")))
    ma, _ = fit_added_mass(sorted(d.glob("coast_*.csv")), TRUTH["mass"], xU, xUU)
    err = abs(ma - TRUTH["xDotU"]) / abs(TRUTH["xDotU"]) * 100
    assert err < tol_pct, f"{hz} Hz -> {ma:.3f} ({err:.1f}% error)"


def test_added_mass_is_negative_and_not_wildly_overestimated(trials):
    """Regression for the sign bug that motivated this file (-80 vs -12)."""
    ma, _ = fit_added_mass(sorted(trials[200].glob("coast_*.csv")),
                           TRUTH["mass"], TRUTH["xU"], TRUTH["xUU"])
    assert ma < 0
    assert abs(ma) < 2 * abs(TRUTH["xDotU"])


def test_added_mass_error_shrinks_with_log_rate(trials):
    errs = {}
    for hz in (20, 50, 200):
        d = trials[hz]
        xU, xUU, _ = fit_surge_drag(sorted(d.glob("step_*.csv")))
        ma, _ = fit_added_mass(sorted(d.glob("coast_*.csv")),
                               TRUTH["mass"], xU, xUU)
        errs[hz] = abs(ma - TRUTH["xDotU"])
    assert errs[200] <= errs[50] <= errs[20]


# ------------------------------------------------------------- bad input -- #

def test_ill_conditioned_thrust_levels_raise(tmp_path):
    """Two nearly identical thrust levels cannot separate linear from quadratic
    drag. The solver would trade xU against xUU freely and return noise that
    looks like an answer — so it must refuse, not fit."""
    eff = TRUTH["mass"] - TRUTH["xDotU"]
    for i, T in enumerate([100.0, 100.001], 1):
        rows = _integrate(0.0, T, eff, TRUTH["xU"], TRUTH["xUU"], 30.0, 0.05)
        with (tmp_path / f"step_{i:02d}.csv").open("w", newline="") as fh:
            w = csv.DictWriter(fh, ["t", "thrust_n", "u"])
            w.writeheader()
            w.writerows(rows)
    with pytest.raises(ValueError, match="ill-conditioned"):
        fit_surge_drag(sorted(tmp_path.glob("step_*.csv")))


def test_single_trial_raises(tmp_path):
    eff = TRUTH["mass"] - TRUTH["xDotU"]
    rows = _integrate(0.0, 90.0, eff, TRUTH["xU"], TRUTH["xUU"], 30.0, 0.05)
    with (tmp_path / "step_01.csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, ["t", "thrust_n", "u"])
        w.writeheader()
        w.writerows(rows)
    with pytest.raises(ValueError):
        fit_surge_drag(sorted(tmp_path.glob("step_*.csv")))


def test_no_usable_coast_samples_raises(tmp_path):
    """A decay entirely below the noise floor gives nothing to fit."""
    with (tmp_path / "coast_01.csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, ["t", "u"])
        w.writeheader()
        w.writerows([{"t": i * 0.05, "u": 0.05} for i in range(50)])
    with pytest.raises(ValueError, match="no usable coast-down"):
        fit_added_mass(sorted(tmp_path.glob("coast_*.csv")),
                       TRUTH["mass"], TRUTH["xU"], TRUTH["xUU"])


# ----------------------------------------------------------------- model -- #

def test_model_mass_matches_fitter_assumption():
    """The fitter reads hull mass from the Gazebo model so the two cannot
    drift; if the model changes, the fits must be redone against the new mass."""
    import fit_hydro
    assert fit_hydro._model_mass() == pytest.approx(TRUTH["mass"])
