"""Source-level wiring checks for the rclpy node layer.

These exist because the unit tier cannot import anything that imports rclpy —
about half the package — so a node attribute that is declared and then never
fed is invisible to every test we can run off-boat. roa_apf_node.speed was
exactly that: initialised to 0.0, passed to ProgressMonitor on every tick, and
never assigned again, which silently satisfied objective-2's speed floor
forever.

This is a stopgap tier, not a substitute for launch_testing against a real
graph. It only asserts wiring that has already been got wrong once.
"""
import ast
from pathlib import Path

import pytest

# <repo>/rx26_asv (package) / rx26_asv (python module) / api
PKG = Path(__file__).parent.parent / "rx26_asv" / "rx26_asv" / "api"


def module_ast(relpath):
    return ast.parse((PKG / relpath).read_text())


def assigned_attrs(tree, classname, methodname):
    """{attr} assigned via `self.<attr> = ...` inside one method."""
    for cls in ast.walk(tree):
        if isinstance(cls, ast.ClassDef) and cls.name == classname:
            for fn in cls.body:
                if isinstance(fn, ast.FunctionDef) and fn.name == methodname:
                    return {
                        t.attr
                        for node in ast.walk(fn)
                        if isinstance(node, (ast.Assign, ast.AugAssign))
                        for t in (node.targets if isinstance(node, ast.Assign)
                                  else [node.target])
                        if isinstance(t, ast.Attribute)
                        and isinstance(t.value, ast.Name) and t.value.id == "self"
                    }
    pytest.fail(f"{classname}.{methodname} not found")


def test_apf_node_feeds_speed_from_the_pose_callback():
    tree = module_ast("navigation/roa_apf_node.py")
    assert "speed" in assigned_attrs(tree, "RoaApfNode", "_pose_cb"), \
        ("roa_apf_node.speed is never updated from /crsd/pose — the "
         "ProgressMonitor speed floor is satisfied on every tick regardless "
         "of how fast the boat is actually moving")


def test_apf_node_still_seeds_speed_in_init():
    tree = module_ast("navigation/roa_apf_node.py")
    assert "speed" in assigned_attrs(tree, "RoaApfNode", "__init__")


def test_bridge_publishes_ground_speed_on_pose():
    """telemetry_bridge is the only producer of /crsd/pose ground_speed."""
    src = (PKG / "navigation/telemetry_bridge.py").read_text()
    assert "ground_speed" in src, \
        "telemetry_bridge no longer populates LatLonHead.ground_speed"
    assert "ground_speed_mps" in src, \
        "ground speed must come from geo.ground_speed_mps (the tested cm/s seam)"


# ---------------------------------------------------------------------------
# RC-loss detection. Measured on Crusader 2026-08-02: with the ELRS receiver's
# failsafe set to "Last Position", ch7 held 1995 through a full transmitter
# power-down. Every RC-loss path in the stack keyed off "PWM drops to 0", so the
# watchdog never force-disarmed and the LED stayed YELLOW on a pilotless boat.
# These pin the second signal (ArduPilot's own SYS_STATUS RC-receiver health bit)
# into place — exactly the class of wiring this file exists to guard.
# ---------------------------------------------------------------------------

def test_bridge_republishes_rc_receiver_health():
    src = (PKG / "navigation/telemetry_bridge.py").read_text()
    assert "MAV_SYS_STATUS_SENSOR_RC_RECEIVER" in src, \
        ("telemetry_bridge no longer reads the SYS_STATUS RC-receiver health "
         "bit — RC-loss detection falls back to a PWM check that a "
         "hold-last-position receiver defeats")
    assert "/crsd/rc_link_health" in src, \
        "telemetry_bridge does not publish /crsd/rc_link_health"


def test_watchdog_gates_link_health_on_the_receiver_bit():
    tree = module_ast("safety/rc_heartbeat_watchdog.py")
    src = (PKG / "safety/rc_heartbeat_watchdog.py").read_text()
    assert "/crsd/rc_link_health" in src, \
        "rc_heartbeat_watchdog does not subscribe to /crsd/rc_link_health"
    assert "rc_link_healthy" in assigned_attrs(
        tree, "RCHeartbeatWatchdog", "_health_cb"), \
        "rc_heartbeat_watchdog._health_cb does not record the health verdict"
    # the verdict has to actually reach the state machine, not just be stored
    assert "_rc_health_ok" in src and "monitored_valid = pwm_ok" in src, \
        ("the RC-receiver health verdict is recorded but never AND-ed into "
         "monitored_valid — the watchdog still cannot see a dead transmitter "
         "whose receiver holds last position")


def test_led_status_ages_out_its_inputs():
    """telemetry_bridge STOPS republishing a stale stream by design, so a node
    that only updates state in callbacks freezes its last value when the gateway
    dies. For the status LED that means holding YELLOW on a dead boat."""
    tree = module_ast("pixhawk/pixhawk_led_status_node.py")
    for method, attr in (("_fcu_cb", "fcu_t"), ("_rc_cb", "rc_t"),
                         ("_health_cb", "health_t")):
        assert attr in assigned_attrs(tree, "PixhawkLEDStatusNode", method), \
            (f"{method} does not stamp {attr} — a stopped stream will hold its "
             "last value instead of failing the LED to RED")
    src = (PKG / "pixhawk/pixhawk_led_status_node.py").read_text()
    assert "input_timeout_s" in src, \
        "pixhawk_led_status_node has no staleness timeout on its inputs"


def test_latlonhead_declares_ground_speed():
    msg = (Path(__file__).parent.parent / "interfaces" / "msg"
           / "LatLonHead.msg").read_text()
    fields = [ln.split("#")[0].split() for ln in msg.splitlines()
              if ln.strip() and not ln.strip().startswith("#")]
    assert any(f[-1] == "ground_speed" for f in fields if len(f) >= 2), \
        "LatLonHead.msg has no ground_speed field for the bridge to publish"
