"""task3_goal — the operator's Task 3 run in Gazebo: arm, GUIDED, go; MANUAL when the tree asks.

    python3 -m crusader_sim.task3_goal --course task3 [--tier disruptive] [--timeout-s 600]

Runs in: the crsd-sim container, after gz_rig_up.sh brought up a dock course
(task3_cannon.xml, dock_slot_node, cannon_aim_node, task3_world).

What a person does for a Task 3 attempt, in order:
  1. ARM on the tooling port (operator_tools.arm, as task1_goal).
  2. WP_RADIUS 0.3 on SITL: the approach's precondition (task3_part1_approach.xml:
     at the boat's 2.0 ArduRover parks 2 m short of every point). SITL's params
     reset at its next start, so nothing needs putting back.
  3. GUIDED through /crsd/set_mode, and the autonomy-drop latch reset
     (/crsd/autonomy_drop_reset, as go.sh does on the boat): telemetry_bridge
     starts with it TRIPPED until a person says "I have the sticks", and the
     MANUAL part's guard band (NotDropped) ends the run on its first tick if not.
  4. The goal: the course's tier, and an approach point - the course's `approach`
     if it has one, else 7 m out in front of the dock's middle (tools/task3_sim's
     approach_m): "the dock is over there".
  5. THE HAND-OVER: when the tree says "WAITING FOR MANUAL" (AwaitMode, after the
     GUIDED approach), MANUAL through /crsd/set_mode - the pilot's SC flip, done
     here because nobody holds a transmitter in the sim.
Then the result, and the referee's verdict (task3_world, /sim/task3/verdict).
Exit status 0 only for SUCCESS and a PASS.

Ctrl-C / SIGTERM cancels the goal and waits for the tree (goal_client.run_goal).
"""
import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from crusader_msgs.action import SafePassage
from crusader_msgs.msg import FcuStatus
from crusader_sim import course as C
from crusader_sim.goal_client import TIERS, install_stop, run_goal
from crusader_sim.operator_tools import arm, set_mode
from crusader_sim.sitl_param_set import set_param

APPROACH_M = 7.0           # out in front of the deck edge (tools/task3_sim Scenario.approach_m)
DECK_M = 1.0               # gen_world.dock(): the deck's depth, back edge to front


def approach_point(course):
    """(lat, lon) of the goal's approach point."""
    o = course["origin"]
    if "approach" in course:
        return C.enu_to_latlon(course["approach"]["x"], course["approach"]["y"], o)
    d = next(e for e in course["elements"] if e.get("type") == "dock")
    psi = math.radians(float(d.get("facing_deg", 180.0)))
    k = DECK_M + APPROACH_M
    return C.enu_to_latlon(float(d["x"]) + k * math.cos(psi), float(d["y"]) + k * math.sin(psi), o)


class Operator(Node):

    def __init__(self):
        super().__init__("sim_task3_operator")
        self.mode_pub = self.create_publisher(String, "/crsd/set_mode", 10)
        self.client = ActionClient(self, SafePassage, "/crsd/safe_passage")
        self.reset_cli = self.create_client(Trigger, "/crsd/autonomy_drop_reset")
        self.mode = None
        self.verdict = None
        self.status = None
        self.want_manual = False
        self._mode_req_t = 0.0
        self._last_lights = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(FcuStatus, "/crsd/fcu_status", self._on_status, 10)
        self.create_subscription(String, "/sim/task3/verdict", self._on_verdict, latched)
        self.create_subscription(String, "/sim/task3/status", self._on_world, 10)

    def _on_status(self, m):
        self.mode = m.mode
        self._fcu_mode = m.mode          # operator_tools.set_mode reads this too

    def _on_verdict(self, m):
        self.verdict = json.loads(m.data)

    def _on_world(self, m):
        s = json.loads(m.data)
        self.status = s
        key = (s.get("lights"), s.get("colour"))
        if key != self._last_lights:
            self._last_lights = key
            print(f"[world] lights {s.get('lights')} (window {s.get('window')} {s.get('colour')})",
                  flush=True)

    def reset_latch(self, timeout_s=10.0):
        """The operator's 'I have the sticks': (ok, message)."""
        if not self.reset_cli.wait_for_service(timeout_sec=timeout_s):
            return False, "no /crsd/autonomy_drop_reset service"
        fut = self.reset_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
        if not fut.done() or fut.result() is None:
            return False, "no answer"
        return bool(fut.result().success), fut.result().message

    def say(self, line):
        print(line, flush=True)
        if "WAITING FOR MANUAL" in line and not self.want_manual:
            self.want_manual = True
            print("[operator] the tree is waiting for MANUAL: flipping SC (MANUAL on /crsd/set_mode)",
                  flush=True)

    def spin(self, t):
        rclpy.spin_once(self, timeout_sec=t)
        now = time.monotonic()
        if self.want_manual and self.mode != "MANUAL" and now - self._mode_req_t > 0.5:
            self.mode_pub.publish(String(data="MANUAL"))
            self._mode_req_t = now


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--course", default="task3")
    ap.add_argument("--tier", choices=sorted(TIERS), default=None,
                    help="default: the course's `tier`")
    ap.add_argument("--timeout-s", type=float, default=600.0)
    a = ap.parse_args()

    course = C.load(a.course)
    tier = TIERS[a.tier or course.get("tier", "disruptive")]
    lat, lon = approach_point(course)

    ok, why = arm()
    print(f"[operator] {why}", flush=True)
    if not ok:
        return 2
    ok, why, _t = set_param("WP_RADIUS", 0.3)
    print("[operator] WP_RADIUS 0.3 on SITL" + ("" if ok else f" NOT set: {why}"), flush=True)

    rclpy.init()
    op = Operator()
    if not set_mode(op, op.mode_pub, "GUIDED"):
        print("[operator] GUIDED never confirmed on /crsd/fcu_status — is telemetry_bridge up?")
        return 2
    print("[operator] mode GUIDED (confirmed by the autopilot)", flush=True)
    ok, why = op.reset_latch()
    print(f"[operator] autonomy-drop latch reset: {why}", flush=True)
    if not ok:
        return 2
    if not op.client.wait_for_server(timeout_sec=20.0):
        print("[operator] no /crsd/safe_passage server (is bt_runner up?)")
        return 2
    goal = SafePassage.Goal(tier=tier, approach_latitude=lat, approach_longitude=lon,
                            orbit_radius_m=0.0, timeout_s=float(a.timeout_s))
    print(f"[operator] goal: tier {tier}, approach {lat:.7f} {lon:.7f}", flush=True)

    stop = install_stop()
    run = run_goal(op.client, goal, op.spin, stop, say=op.say)
    # the verdict is latched; give the referee a moment to see the last report
    t_end = time.monotonic() + 5.0
    while time.monotonic() < t_end and (op.verdict is None or not op.verdict.get("complete")):
        rclpy.spin_once(op, timeout_sec=0.2)
    v = op.verdict
    if v is None:
        print("[referee] no verdict (is task3_world running?)", flush=True)
        passed = False
    else:
        print("[referee] VERDICT: " + ("PASS" if v["pass"] else "FAIL")
              + ("" if v["complete"] else " (the run did not finish its reports)"), flush=True)
        for line in v["lines"]:
            print(f"[referee]   {line}", flush=True)
        passed = bool(v["pass"])
    op.destroy_node()
    rclpy.shutdown()
    if run.status != "result":
        return 1
    return 0 if (run.result.outcome == SafePassage.Result.OUTCOME_SUCCESS and passed) else 1


if __name__ == "__main__":
    sys.exit(main())
