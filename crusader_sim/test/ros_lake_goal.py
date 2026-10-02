"""ros_lake_goal — lake_goal against a fake boat, with REAL rclpy, in an isolated ROS domain (87).

A manual check, not part of `unittest discover` (it needs ROS). test_lake.py runs the same logic against a fake rclpy; this
is the same cases against the real thing, where the interesting risk is rclpy's own SIGINT handler.

Runs in: the crsd-sim container (or `asv`), ROS sourced. ROS_DOMAIN_ID=87 keeps it off a running sim's graph:

    docker exec -e ROS_DOMAIN_ID=87 crsd-sim bash -lc 'source /opt/ros/humble/setup.bash;
      source /root/robotx_ws/install/setup.bash;
      PYTHONPATH=/root/robotx_ws/src/rx26_asv/crusader_sim:$PYTHONPATH \\
      python3 -u /root/robotx_ws/src/rx26_asv/crusader_sim/test/ros_lake_goal.py'

The fake boat (this file with --server) publishes /crsd/fcu_status and serves /crsd/safe_passage: it accepts a goal, ticks
feedback, and ends on a cancel with OUTCOME_CANCELLED. Cases: SIGINT and SIGTERM cancel the goal (also from a parent that
ignores SIGINT, as a shell's `&` does); HOLD, disarmed and no status are refused and the server never sees a goal.
Checked 2026-10-02: 6/6.
"""
import os
import signal
import subprocess
import sys
import time

EV = "/tmp/ros_lake_goal_events.txt"


def server(mode, armed):
    import rclpy
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    from crusader_msgs.action import SafePassage
    from crusader_msgs.msg import FcuStatus

    def ev(text):
        with open(EV, "a") as f:
            f.write("%s\n" % text)

    class Boat(Node):
        def __init__(self):
            super().__init__("fake_boat")
            self.pub = self.create_publisher(FcuStatus, "/crsd/fcu_status", 10)
            if mode != "none":
                self.create_timer(0.2, self.tick)
            self.srv = ActionServer(self, SafePassage, "/crsd/safe_passage", execute_callback=self.execute,
                                    goal_callback=self.on_goal, cancel_callback=self.on_cancel)

        def tick(self):
            m = FcuStatus()
            m.mode, m.armed, m.system_status = mode, armed == "armed", 4
            self.pub.publish(m)

        def on_goal(self, g):
            ev("goal tier=%d approach=%.6f,%.6f timeout=%.0f" % (g.tier, g.approach_latitude, g.approach_longitude, g.timeout_s))
            return GoalResponse.ACCEPT

        def on_cancel(self, h):
            ev("cancel")
            return CancelResponse.ACCEPT

        def execute(self, h):
            t0 = time.time()
            fb = SafePassage.Feedback()
            while time.time() - t0 < 20.0:
                if h.is_cancel_requested:
                    h.canceled()
                    r = SafePassage.Result()
                    r.outcome, r.detail = SafePassage.Result.OUTCOME_CANCELLED, "cancelled by the client"
                    ev("result CANCELLED")
                    return r
                fb.phase, fb.progress = "TRANSIT", (time.time() - t0) / 20.0
                h.publish_feedback(fb)
                time.sleep(0.5)
            h.succeed()
            r = SafePassage.Result()
            r.outcome, r.detail = SafePassage.Result.OUTCOME_TIMEOUT, "nobody cancelled"
            ev("result TIMEOUT")
            return r

    rclpy.init()
    ex = MultiThreadedExecutor()
    ex.add_node(Boat())
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


def run_case(name, mode, armed, sig, ignore_sigint, want_rc, want_events, want_out):
    try:
        os.remove(EV)
    except OSError:
        pass
    srv = subprocess.Popen([sys.executable, "-u", __file__, "--server", mode, armed], start_new_session=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3)
    pre = (lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)) if ignore_sigint else None
    g = subprocess.Popen([sys.executable, "-u", "-m", "crusader_sim.lake_goal", "--approach", "1.3,103.85", "--fcu-wait-s", "4"],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, preexec_fn=pre)
    if sig is not None:
        time.sleep(6)
        g.send_signal(sig)
    out, _ = g.communicate(timeout=60)
    time.sleep(0.5)
    os.killpg(srv.pid, signal.SIGINT)
    try:
        srv.wait(5)
    except subprocess.TimeoutExpired:
        os.killpg(srv.pid, signal.SIGKILL)
    ev = open(EV).read() if os.path.exists(EV) else ""
    ok = bool(g.returncode == want_rc and all(w in ev for w in want_events) and all(w in out for w in want_out)
              and (want_events or not ev))
    print("  %s %s: exit %s, server saw %r" % ("PASS" if ok else "FAIL", name, g.returncode, ev.strip().replace("\n", " | ")), flush=True)
    if not ok:
        print(out)
    return ok


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--server":
        return server(sys.argv[2], sys.argv[3])
    res = [
        run_case("SIGINT cancels the goal", "GUIDED", "armed", signal.SIGINT, False, 1, ["goal tier=2", "cancel", "result CANCELLED"], ["CANCELLING", "outcome 3"]),
        run_case("SIGINT cancels it from a parent that ignores SIGINT", "GUIDED", "armed", signal.SIGINT, True, 1, ["cancel", "result CANCELLED"], ["CANCELLING"]),
        run_case("SIGTERM cancels the goal", "GUIDED", "armed", signal.SIGTERM, False, 1, ["cancel", "result CANCELLED"], ["CANCELLING", "outcome 3"]),
        run_case("HOLD is refused, no goal sent", "HOLD", "armed", None, False, 3, [], ["REFUSED", "not GUIDED"]),
        run_case("disarmed is refused, no goal sent", "GUIDED", "disarmed", None, False, 3, [], ["REFUSED", "NOT ARMED"]),
        run_case("no FCU status at all is refused", "none", "armed", None, False, 3, [], ["REFUSED", "no /crsd/fcu_status"]),
    ]
    print("RESULT %d/%d" % (sum(res), len(res)))
    return 0 if all(res) else 1


if __name__ == "__main__":
    sys.exit(main())
