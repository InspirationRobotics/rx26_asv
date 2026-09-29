"""operator_tools — what a person at the ground station does, for sim tools.

Shared by task1_goal and manual_drive so there is one arming path, not two.
"""
import os
import time

os.environ.setdefault("MAVLINK20", "1")

TOOLING_ENDPOINT = "udpin:127.0.0.1:14550"   # never 14551: telemetry_bridge's


def arm(endpoint=TOOLING_ENDPOINT, timeout_s=60.0):
    """ARM over MAVLink on the tooling port, retrying while pre-arm refuses
    (ArduPilot refuses for ~10-20 s after boot). The bridge has no arm path on
    purpose; on the water the pilot arms with SB. Returns (ok, why)."""
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(endpoint, source_system=255)
    try:
        if m.wait_heartbeat(timeout=20) is None:
            return False, "no HEARTBEAT on " + endpoint
        t_end = time.time() + timeout_s
        last = ""
        while time.time() < t_end:
            m.arducopter_arm()
            t_ack = time.time() + 3
            while time.time() < t_ack:
                msg = m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=0.5)
                if msg is None:
                    continue
                if msg.get_type() == "STATUSTEXT":
                    last = msg.text
                elif msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result == 0:
                        return True, "armed"
                    break
        return False, "arm refused: " + last
    finally:
        m.close()


def set_mode(node, publisher, name, timeout_s=15.0):
    """Ask telemetry_bridge for a mode via /crsd/set_mode (its own path) and
    WAIT until /crsd/fcu_status reports it. Returns True once confirmed.

    Fire-and-forget is not enough, and it failed once (2026-09-29): a fresh
    publisher's first messages can be dropped before DDS has matched it to the
    bridge's subscription, and even a delivered request only shows up in the
    next HEARTBEAT. A goal sent in that gap meets a tree that still sees HOLD
    and refuses (OUTCOME_NOT_AUTONOMOUS). So: re-request every 0.5 s until the
    autopilot says so."""
    import time

    import rclpy
    from std_msgs.msg import String
    from crusader_msgs.msg import FcuStatus

    if not hasattr(node, "_fcu_mode"):
        node._fcu_mode = None

        def _on_status(msg):
            node._fcu_mode = msg.mode
        node.create_subscription(FcuStatus, "/crsd/fcu_status", _on_status, 10)
    t_end = time.time() + timeout_s
    next_req = 0.0
    while time.time() < t_end:
        if node._fcu_mode == name:
            return True
        if time.time() >= next_req:
            publisher.publish(String(data=name))
            next_req = time.time() + 0.5
        rclpy.spin_once(node, timeout_sec=0.1)
    return False
