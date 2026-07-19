import json
import subprocess
import sys
import threading
from pathlib import Path

from episodes.backends.kinematic import KinematicBackend
from episodes.runner import EpisodeRunner
from episodes.scenario import Scenario
from evaluator import metrics

ORCH = Path(__file__).parent.parent
SCENARIOS = ORCH / "scenarios"


def run(name, seed=0, noise=0.0):
    sc = Scenario.load(str(SCENARIOS / name))
    backend = KinematicBackend(noise_std=noise)
    res = EpisodeRunner(sc, backend, seed=seed).run()
    return sc, res


def test_mission1_completes_cleanly():
    sc, res = run("mission1_transit.json")
    m = metrics.assemble(res, sc)
    assert m["objective3"]["completed"], m["objective3"]
    assert m["objective1"]["hard_collisions"] == 0
    assert not m["objective1"]["auto_fail"]
    assert m["objective2"]["stalls"] == 0


def test_mission4_events_fire_and_stay_clear():
    sc, res = run("mission4_advanced_keepout.json")
    assert len(res.keepouts) == 1 and res.keepouts[0].zone_id == "K1"
    assert res.keepouts[0].active_until < float("inf")    # all_clear applied
    assert len(res.moving_objects) == 1
    m = metrics.assemble(res, sc)
    assert m["objective3"]["completed"]
    assert m["objective1"]["keepout_violations"] == 0     # zone placed off-path
    assert m["objective1"]["moving_10m_violations"] == 0
    assert not m["objective1"]["auto_fail"]


def test_determinism_same_seed():
    _, a = run("mission1_transit.json", seed=7, noise=0.02)
    _, b = run("mission1_transit.json", seed=7, noise=0.02)
    assert [(s.x, s.y) for s in a.trace] == [(s.x, s.y) for s in b.trace]
    _, c = run("mission1_transit.json", seed=8, noise=0.02)
    assert [(s.x, s.y) for s in a.trace] != [(s.x, s.y) for s in c.trace]


def test_stop_event_teardown():
    sc = Scenario.load(str(SCENARIOS / "mission1_transit.json"))
    stop = threading.Event()
    stop.set()                                            # stop before first step
    res = EpisodeRunner(sc, KinematicBackend(), stop_event=stop).run()
    assert res.stopped_early


def test_cli_emits_metrics_json(tmp_path):
    out = tmp_path / "ep.json"
    proc = subprocess.run(
        [sys.executable, str(ORCH / "run_episode.py"),
         "--scenario", str(SCENARIOS / "mission1_transit.json"),
         "--backend", "kinematic", "--seed", "0", "--out", str(out)],
        capture_output=True, text=True, timeout=120)
    assert proc.returncode == 0, proc.stderr
    m = json.loads(out.read_text())
    assert m["schema"] == "rx26-episode-metrics/1"
    assert {"objective1", "objective2", "objective3"} <= set(m)
