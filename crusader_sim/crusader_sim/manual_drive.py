"""manual_drive — drive the sim boat in MANUAL through the boat's own override path.

    python3 -m crusader_sim.manual_drive demo     # scripted surge/sway/yaw, measured
    python3 -m crusader_sim.manual_drive keys     # keyboard teleop

Runs in: the crsd-sim container, after gz_sim_up.sh (needs a real terminal for
`keys`: docker exec -it crsd-sim bash -lc "python3 -m crusader_sim.manual_drive keys").

THIS IS THE REAL BOAT'S DIRECT-CONTROL PATH, not a sim shortcut. It publishes
crusader_msgs/RcChannels on /crsd/rc_override — the same topic the Task 3
fixed-nozzle shot drives — and telemetry_bridge applies its own rules
(override_core.py) before anything reaches the autopilot:
    only in MANUAL (override_modes), only ch1 steer / ch3 throttle / ch4 lateral,
    clamped to 1500 +- override_max_us (150), released after 0.5 s of silence.
ArduRover's OmniX mixer then turns (throttle, steering, lateral) into the four
thruster outputs. So "thruster commands" here are body-frame surge / sway / yaw
demands, exactly as on the water — see the README for per-thruster options.

Keys (hold-to-drive feel: each press sets a level, space zeroes all):
    w/s  surge  +/-      a/d  sway port/starboard     q/e  yaw left/right
    space  all stop      m  MANUAL     h  HOLD     x  quit (releases the sticks)
"""
import argparse
import math
import select
import sys
import termios
import time
import tty

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from crusader_msgs.msg import FcuStatus, RcChannels
from crusader_sim.operator_tools import arm, set_mode

MAX_US = 150            # = telemetry_bridge override_max_us; more is clamped anyway
RATE_HZ = 10.0          # well inside the 0.5 s deadman


class Driver(Node):
    def __init__(self):
        super().__init__("sim_manual_drive")
        self.pub = self.create_publisher(RcChannels, "/crsd/rc_override", 10)
        self.mode_pub = self.create_publisher(String, "/crsd/set_mode", 10)
        self.odom = None
        self.mode = "?"
        self.create_subscription(Odometry, "/sim/crusader/odometry", self._odom, 10)
        self.create_subscription(FcuStatus, "/crsd/fcu_status", self._status, 10)

    def _odom(self, m):
        self.odom = m

    def _status(self, m):
        self.mode = getattr(m, "mode", "?")

    def send(self, surge=0.0, sway=0.0, yaw=0.0):
        """-1..1 each: surge + = forward, sway + = STARBOARD, yaw + = clockwise."""
        ch = [0] * 18
        ch[0] = int(1500 + MAX_US * max(-1.0, min(1.0, yaw)))     # ch1 steer
        ch[2] = int(1500 + MAX_US * max(-1.0, min(1.0, surge)))   # ch3 throttle
        ch[3] = int(1500 + MAX_US * max(-1.0, min(1.0, sway)))    # ch4 lateral
        msg = RcChannels(channels=ch)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def release(self):
        self.pub.publish(RcChannels(channels=[0] * 18))

    def pose(self):
        """(x, y, yaw) true, world ENU."""
        p = self.odom.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return p.position.x, p.position.y, yaw


def spin_for(node, seconds, **cmd):
    t_end = time.time() + seconds
    while time.time() < t_end:
        node.send(**cmd)
        rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)


def demo(node):
    rows = []
    for label, cmd in (("surge +", {"surge": 1.0}), ("sway +(stbd)", {"sway": 1.0}),
                       ("yaw +(cw)", {"yaw": 1.0})):
        spin_for(node, 4.0)                          # settle, sticks centred
        x0, y0, h0 = node.pose()
        turned, last = 0.0, h0
        t_end = time.time() + 6.0
        while time.time() < t_end:
            node.send(**cmd)
            rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)
            h = node.pose()[2]
            turned += (h - last + math.pi) % (2 * math.pi) - math.pi
            last = h
        x1, y1, _ = node.pose()
        dx, dy = x1 - x0, y1 - y0
        fwd = dx * math.cos(h0) + dy * math.sin(h0)
        stbd = dx * math.sin(h0) - dy * math.cos(h0)
        rows.append((label, fwd, stbd, -math.degrees(turned)))   # ENU ccw -> cw
    node.release()
    print(f"{'command':14s} {'fwd m':>7s} {'stbd m':>7s} {'yaw cw deg':>11s}")
    for r in rows:
        print(f"{r[0]:14s} {r[1]:7.2f} {r[2]:7.2f} {r[3]:11.1f}")


def keys(node):
    level = {"surge": 0.0, "sway": 0.0, "yaw": 0.0}
    step = {"w": ("surge", 0.25), "s": ("surge", -0.25), "d": ("sway", 0.25),
            "a": ("sway", -0.25), "e": ("yaw", 0.25), "q": ("yaw", -0.25)}
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print(__doc__.split("Keys")[1])
    try:
        while True:
            if select.select([sys.stdin], [], [], 1.0 / RATE_HZ)[0]:
                k = sys.stdin.read(1).lower()
                if k == "x":
                    break
                if k == " ":
                    level = dict.fromkeys(level, 0.0)
                elif k in step:
                    axis, d = step[k]
                    level[axis] = max(-1.0, min(1.0, level[axis] + d))
                elif k in ("m", "h"):
                    set_mode(node, node.mode_pub, "MANUAL" if k == "m" else "HOLD")
            node.send(**level)
            rclpy.spin_once(node, timeout_sec=0.0)
            sys.stdout.write(f"\r mode {node.mode:8s} surge {level['surge']:+.2f} "
                             f"sway {level['sway']:+.2f} yaw {level['yaw']:+.2f}   ")
            sys.stdout.flush()
    finally:
        node.release()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print("\nsticks released")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("what", choices=["demo", "keys"])
    a = ap.parse_args()
    ok, why = arm()
    print(f"[operator] {why}")
    if not ok:
        return 2
    rclpy.init()
    node = Driver()
    set_mode(node, node.mode_pub, "MANUAL")
    t_end = time.time() + 10
    while node.odom is None and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.odom is None:
        print("no /sim/crusader/odometry — is gz_rig_up.sh running?")
        return 2
    try:
        demo(node) if a.what == "demo" else keys(node)
    finally:
        node.release()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
