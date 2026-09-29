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


def set_mode(node, publisher, name, spins=3):
    """Ask telemetry_bridge for a mode via /crsd/set_mode (its own path).
    Nobody latches that topic, so say it a few times."""
    import rclpy
    from std_msgs.msg import String
    for _ in range(spins):
        publisher.publish(String(data=name))
        rclpy.spin_once(node, timeout_sec=0.3)
