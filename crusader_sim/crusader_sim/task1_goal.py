"""task1_goal — the operator's "Send goal" for a Gazebo run: arm, GUIDED, go.

    python3 -m crusader_sim.task1_goal --course task1_core [--tier disruptive] [--no-judge]

Runs in: the crsd-sim container, after gz_rig_up.sh.

--no-judge leaves the referee out: the Task 1 panel runs its own, because only
the panel knows when it changed a beacon mid-run.

Does what an operator does before a Task 1 attempt, in order, and then watches:
  1. ARM, over MAVLink on the TOOLING port 14550 (the bridge has no arm path on
     purpose; on the water the pilot arms with SB). Retries while pre-arm
     refuses, as ArduPilot does for ~10-20 s after boot.
  2. GUIDED, through /crsd/set_mode — the bridge's own mode path.
  3. The SafePassage goal to bt_runner (/crsd/safe_passage): tier from the
     course, approach point = the course's `approach` if it has one, else 6 m
     short of the ENTRY buoy on the line from the start — the operator's rough
     "the course is over there".
Then prints the tree's feedback (phase, progress, buoys) until the result.
"""
import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from nav_msgs.msg import Odometry

from crusader_msgs.action import SafePassage
from crusader_sim import course as C
from crusader_sim.operator_tools import arm, set_mode
from crusader_sim.task1_judge import Task1Judge, format_verdict

TIERS = {"core": 0, "advanced": 1, "disruptive": 2}


def approach_point(course):
    """(lat, lon) of the goal's approach point."""
    o = course["origin"]
    if "approach" in course:
        return C.enu_to_latlon(course["approach"]["x"], course["approach"]["y"], o)
    entry = next((b for b in C.buoys(course) if b[3] == "flash_blue"), None)
    if entry is None:
        raise ValueError("course has no flash_blue ENTRY buoy and no `approach`")
    s = course.get("boat_start", {"x": 0.0, "y": 0.0})
    dx, dy = entry[1] - s["x"], entry[2] - s["y"]
    d = math.hypot(dx, dy)
    k = max(0.0, (d - 6.0) / d) if d > 0 else 0.0
    return C.enu_to_latlon(s["x"] + dx * k, s["y"] + dy * k, o)


class Operator(Node):
    """The operator, plus (unless judge is None) an independent referee watching
    the TRUE path (task1_judge) — the tree's own 'passed correctly' is the tree
    grading itself."""

    def __init__(self, judge=None):
        super().__init__("sim_operator")
        self.mode_pub = self.create_publisher(String, "/crsd/set_mode", 10)
        self.client = ActionClient(self, SafePassage, "/crsd/safe_passage")
        if judge is not None:
            self.create_subscription(Odometry, "/sim/crusader/odometry",
                                     lambda m: _feed_judge(judge, m), 10)


def _feed_judge(judge, m):
    """True pose -> referee. The heading lets it test contact against the hull
    rectangle: a bow pressed on a buoy is 0.70 m centre to centre, which a
    0.55 m circle misses (seen 2026-09-30)."""
    p, q = m.pose.pose.position, m.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    judge.update(p.x, p.y, yaw)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--course", default="task1_core")
    ap.add_argument("--tier", choices=sorted(TIERS), default=None,
                    help="default: the course's `tier`")
    ap.add_argument("--timeout-s", type=float, default=400.0)
    ap.add_argument("--no-judge", action="store_true",
                    help="no referee (the Task 1 panel runs its own)")
    a = ap.parse_args()

    course = C.load(a.course)
    tier = TIERS[a.tier or course.get("tier", "core")]
    lat, lon = approach_point(course)

    ok, why = arm()
    print(f"[operator] {why}", flush=True)
    if not ok:
        return 2

    rclpy.init()
    judge = None if a.no_judge else Task1Judge(course)
    op = Operator(judge)
    if not set_mode(op, op.mode_pub, "GUIDED"):
        print("[operator] GUIDED never confirmed on /crsd/fcu_status — is telemetry_bridge up?")
        return 2
    print("[operator] mode GUIDED (confirmed by the autopilot)", flush=True)

    if not op.client.wait_for_server(timeout_sec=20.0):
        print("[operator] no /crsd/safe_passage server (is bt_runner up?)")
        return 2
    goal = SafePassage.Goal(tier=tier, approach_latitude=lat, approach_longitude=lon,
                            orbit_radius_m=0.0, timeout_s=float(a.timeout_s))
    print(f"[operator] goal: tier {tier}, approach {lat:.7f} {lon:.7f}", flush=True)

    last = [None]

    def fb(msg):
        # one line per phase change, per 5 % of progress, or per new warning —
        # the tree sends feedback every tick and 0.1 % steps drown the phases
        f = msg.feedback
        key = (f.phase, int(f.progress * 20), f.buoys_resolved, f.plan_version, f.warning)
        if key != last[0]:
            print(f"[tree] {f.phase:12s} {f.progress * 100:5.1f}%  buoys {f.buoys_resolved}/"
                  f"{f.buoys_known}  plan v{f.plan_version}  {f.warning}", flush=True)
            last[0] = key

    fut = op.client.send_goal_async(goal, feedback_callback=fb)
    rclpy.spin_until_future_complete(op, fut)
    handle = fut.result()
    if not handle.accepted:
        print("[operator] goal REJECTED")
        return 1
    res_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(op, res_fut)
    r = res_fut.result().result
    print(f"[result] outcome {r.outcome}  {r.detail}\n"
          f"         classified {r.buoys_classified}, passed correctly "
          f"{r.buoys_passed_correctly}, {r.elapsed_s:.0f} s", flush=True)
    passed = True
    if judge is not None:
        v = judge.verdict()
        print(format_verdict(v), flush=True)
        passed = v["pass"]
    op.destroy_node()
    rclpy.shutdown()
    return 0 if (r.outcome == SafePassage.Result.OUTCOME_SUCCESS and passed) else 1


if __name__ == "__main__":
    sys.exit(main())
