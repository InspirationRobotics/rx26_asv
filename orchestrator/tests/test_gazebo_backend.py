"""Gazebo backend — contract, guards, and world/scenario origin agreement.

These tests deliberately require NO Gazebo, NO SITL and NO pymavlink: they cover
exactly the parts that fail silently on a real run. The dynamics themselves can
only be checked in the container; what CI can check is that a misconfigured run
refuses to start instead of quietly producing plausible, wrong numbers.

The specific failure this guards against: Gazebo is not up, SITL falls back to
its built-in motorboat FDM, the episode completes, the evaluator scores it, and
the strafe results — the entire reason this backend exists — are fiction.
"""
import json
import os
import socket
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "orchestrator"))

from episodes.backends.gazebo import GazeboBackend          # noqa: E402
from episodes.scenario import Scenario                      # noqa: E402

SCENARIOS = REPO / "orchestrator" / "scenarios"
WORLDS = REPO / "tools" / "sim" / "worlds"


# --------------------------------------------------------------------- guards #

def test_refuses_without_sitl_ok(monkeypatch):
    """RX26_SITL_OK is the guard that stops this arming the real boat."""
    monkeypatch.delenv("RX26_SITL_OK", raising=False)
    with pytest.raises(RuntimeError, match="RX26_SITL_OK"):
        GazeboBackend()


@pytest.mark.parametrize("endpoint", [
    "/dev/ttyACM0", "/dev/ttyUSB0", "serial:/dev/ttyACM0", "COM3",
])
def test_refuses_serial_endpoints(monkeypatch, endpoint):
    """Single-Pixhawk-owner rule, enforced structurally like SitlBackend."""
    monkeypatch.setenv("RX26_SITL_OK", "1")
    with pytest.raises(ValueError, match="only udp:/tcp:"):
        GazeboBackend(endpoint=endpoint)


def test_refuses_when_gazebo_absent(monkeypatch):
    """
    THE important one.

    Without this guard a Gazebo-less run silently uses SITL's motorboat FDM,
    which does not model OmniX lateral thrust — so every dp_hold / strafe result
    would look fine and be meaningless.
    """
    monkeypatch.setenv("RX26_SITL_OK", "1")
    with pytest.raises(RuntimeError, match="no Gazebo server"):
        GazeboBackend(endpoint="udp:127.0.0.1:14550")


def test_gazebo_probe_detects_a_bound_port(monkeypatch):
    """Probe returns True only when something actually holds the FDM port."""
    assert GazeboBackend._gazebo_alive(port=9002) is False
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("127.0.0.1", 9002))
        assert GazeboBackend._gazebo_alive(port=9002) is True
    finally:
        s.close()
    assert GazeboBackend._gazebo_alive(port=9002) is False


def test_bypass_is_explicit(monkeypatch):
    """require_gazebo=False exists, but you have to ask for it by name."""
    monkeypatch.setenv("RX26_SITL_OK", "1")
    pytest.importorskip("pymavlink")
    b = GazeboBackend(endpoint="udp:127.0.0.1:14550", require_gazebo=False)
    assert b.speed_scale == 1.0


# ---------------------------------------------------------------- contract --- #

def test_implements_the_backend_contract():
    """Same surface as KinematicBackend/SitlBackend, so the runner is agnostic."""
    for name in ("reset", "set_target", "step", "state", "shutdown",
                 "set_speed_scale"):
        assert callable(getattr(GazeboBackend, name)), f"missing {name}()"


def test_frame_conversion_matches_sitl_backend():
    """
    gazebo and sitl episodes must be directly comparable, which requires the
    SAME lat/lon <-> xy conversion. If these ever diverge, a mechanism that
    looks better under one backend may only look better because the frames
    differ.
    """
    from episodes.backends.sitl import SitlBackend

    class _G(GazeboBackend):
        def __init__(self):
            self.origin = (1.28165, 103.85406)

    class _S(SitlBackend):
        def __init__(self):
            self.origin = (1.28165, 103.85406)

    g, s = _G(), _S()
    for x, y in [(0, 0), (25, -40), (-13.5, 88.2)]:
        assert g._to_latlon(x, y) == s._to_latlon(x, y)
        assert g._to_xy(*g._to_latlon(x, y)) == pytest.approx(
            s._to_xy(*s._to_latlon(x, y)))


# -------------------------------------------------- world/scenario agreement - #

def _scenarios():
    return sorted(SCENARIOS.glob("*.json"))


def test_scenario_to_world_generates_every_scenario(tmp_path):
    r = subprocess.run(
        [sys.executable, str(REPO / "tools/sim/scenario_to_world.py"), "--all"],
        capture_output=True, text=True, cwd=REPO)
    assert r.returncode == 0, r.stderr


@pytest.mark.parametrize("path", _scenarios(), ids=lambda p: p.stem)
def test_world_origin_equals_scenario_origin(path):
    """
    The generated world's <spherical_coordinates> must equal scenario.origin.

    A mismatch puts the vehicle in the right place on screen while its GPS fix
    is somewhere else — every position metric is then wrong and nothing in the
    output says so.
    """
    import re
    sc = json.loads(path.read_text())
    world = WORLDS / f"{sc['name']}.sdf"
    if not world.exists():
        pytest.skip("run tools/sim/scenario_to_world.py --all")
    txt = world.read_text(encoding="utf-8")
    lat = float(re.search(r"<latitude_deg>([-\d.]+)", txt).group(1))
    lon = float(re.search(r"<longitude_deg>([-\d.]+)", txt).group(1))
    assert (lat, lon) == tuple(sc["origin"])


@pytest.mark.parametrize("path", _scenarios(), ids=lambda p: p.stem)
def test_world_obstacles_match_scenario_obstacles(path):
    """
    Physical buoys in the world must match the obstacles the evaluator scores
    collisions against — same count, same position, same radius.

    Otherwise objective-1 is computed against geometry the vehicle is not
    actually hitting.
    """
    import xml.etree.ElementTree as ET
    sc = json.loads(path.read_text())
    world = WORLDS / f"{sc['name']}.sdf"
    if not world.exists():
        pytest.skip("run tools/sim/scenario_to_world.py --all")

    root = ET.parse(world).getroot().find("world")
    models = {m.get("name"): m for m in root.findall("model")}

    for i, obs in enumerate(sc.get("obstacles", [])):
        name = obs.get("label") or f"obstacle_{i}"
        assert name in models, f"{name} missing from generated world"
        pose = [float(v) for v in models[name].find("pose").text.split()]
        assert pose[0] == pytest.approx(obs["x"])
        assert pose[1] == pytest.approx(obs["y"])
        r = models[name].find(".//collision/geometry/cylinder/radius")
        assert float(r.text) == pytest.approx(obs.get("radius", 0.3))


@pytest.mark.parametrize("path", _scenarios(), ids=lambda p: p.stem)
def test_keepouts_are_not_rendered(path):
    """
    Keep-outs are virtual RoboCommand zones with no physical presence on the
    course. Rendering them would let a perception stack "see" something that
    does not exist in the real world.
    """
    import xml.etree.ElementTree as ET
    sc = json.loads(path.read_text())
    world = WORLDS / f"{sc['name']}.sdf"
    if not world.exists():
        pytest.skip("run tools/sim/scenario_to_world.py --all")
    names = {m.get("name") for m in
             ET.parse(world).getroot().find("world").findall("model")}
    for ko in sc.get("keepouts", []):
        assert ko.get("zone_id") not in names


def test_origin_mismatch_is_refused(monkeypatch, tmp_path):
    """A world built for a different origin must abort the run, not drift."""
    monkeypatch.setenv("RX26_SITL_OK", "1")
    pytest.importorskip("pymavlink")

    bad = tmp_path / "wrong_origin.sdf"
    bad.write_text(
        "<sdf><world name='w'><spherical_coordinates>"
        "<latitude_deg>32.7020</latitude_deg>"
        "<longitude_deg>-117.2510</longitude_deg>"
        "</spherical_coordinates></world></sdf>")

    b = GazeboBackend(endpoint="udp:127.0.0.1:14550",
                      gz_world=str(bad), require_gazebo=False)
    b.origin = (1.28165, 103.85406)          # Marina Bay scenario
    with pytest.raises(RuntimeError, match="ORIGIN MISMATCH"):
        b._warn_origin_mismatch()
