"""mission_planner — the one node that decides what state and task the boat is in.

    ros2 run crusader_behavior mission_planner

NOT PROVEN ON THE BOAT. Started by hand or by its own systemd unit; deliberately
absent from core.launch.py until it has carried a run.

WHY THIS NODE EXISTS. `ocs_client` used to derive STATE_AUTO itself from
`armed` plus `/crsd/autonomy_active`. That put the answer to "is this boat
driving itself" in the same node whose job is to transmit it, and it disagreed
with `pixhawk_led_status`, which was answering the same question from the
autopilot mode. Two nodes answering one question is how they eventually answer
it differently — and here the two answers are the status light the safety
officer reads and the state RoboNation scores. One node owns it now, and the
rule lives in mission_core so it can be exercised with no rclpy in the way.

WHAT IT WILL BECOME. This is the seed of the mission executive: it already holds
the shape (state, task, whether a mission is running) that starting and
sequencing missions needs. Today it only reports, which is all the
Communications Proof of Readiness requires.

WHAT IT MUST NEVER DO. It does not change flight mode, arm, or disarm. An OCS
standby-auto directive arrives here as a request and is logged and republished,
nothing more. The standing rule across both vehicle repos holds without
exception: the RC e-stop is the only safety path, and WiFi is never a safety
mechanism. A human puts this boat into GUIDED.
"""
import json
import time

from crusader_common import config as crsd_config
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config
from crusader_common.stream_cache import StreamCache
from crusader_msgs.msg import FcuStatus, MissionState
from rclpy.node import Node
from std_msgs.msg import Bool, String

from crusader_behavior.mission import mission_core

PARAM_SPEC = {
    "rate_hz": dict(read_only=True, lo=0.5, hi=20.0,
                    description="how often mission_state is published; must be "
                                "at least the OCS heartbeat rate or ocs_client "
                                "sees it as stale"),
    "status_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                             description="= shared.pose_timeout_s; stale "
                                         "fcu_status -> publish nothing"),
    "bench_auto": dict(read_only=True,
                       description="report STATE_AUTO with no autopilot, for "
                                   "producing rc-test logs off the water. "
                                   "NEVER true for a scored run."),
}


class MissionPlanner(Node):
    def __init__(self):
        super().__init__("mission_planner")
        p = declare_from_config(
            self, crsd_config.node_params("mission_planner"), PARAM_SPEC)
        self.p = p

        self._status = StreamCache(p["status_timeout_s"])
        self._killed = False
        self._task = mission_core.TASK_NONE
        self._mission_active = False
        self._mission_name = ""
        self._last_published = None
        self._quiet_reason = None

        self.pub = self.create_publisher(MissionState, "/crsd/mission_state", 10)
        self.create_subscription(FcuStatus, "/crsd/fcu_status", self._on_status, 10)
        self.create_subscription(Bool, "/crsd/kill_active", self._on_kill, 10)
        # Advisory input from the OCS, republished by ocs_client. Read, logged,
        # never acted on -- see the module docstring.
        self.create_subscription(String, "/crsd/ocs_directive",
                                 self._on_directive, 10)

        if p["bench_auto"]:
            self.get_logger().warning(
                "bench_auto is ON — this node will report STATE_AUTO regardless "
                "of what the autopilot is doing. Never leave this set for a "
                "scored run.")

        self.create_timer(1.0 / float(p["rate_hz"]), self._tick)
        self.get_logger().info(
            "mission_planner up: %.1f Hz, autonomy modes %s"
            % (p["rate_hz"], ", ".join(mission_core.AUTO_MODES)))

    # ---- inputs -----------------------------------------------------------

    def _on_status(self, msg):
        self._status.set(msg, time.monotonic())

    def _on_kill(self, msg):
        was, self._killed = self._killed, bool(msg.data)
        if was != self._killed:
            self.get_logger().warning(
                "RC kill %s" % ("ACTIVE" if self._killed else "cleared"))

    def _on_directive(self, msg):
        """An OCS directive. We note it; the autopilot is not touched.

        Logged at info so the run log shows the OCS asked and when — which is
        what makes 'the boat never went auto' a question with an answer.
        """
        try:
            d = json.loads(msg.data).get("ocs_directive", {})
        except (ValueError, AttributeError):
            self.get_logger().warning("undecodable OCS directive, ignored")
            return
        action = d.get("action", "?")
        if action == "standby_auto":
            self.get_logger().info(
                "OCS asks us to stand by in auto for declaration_seq=%s. "
                "NOT changing mode — a human puts this boat in GUIDED."
                % d.get("declaration_seq"))
        else:
            self.get_logger().info("OCS directive %r noted, no action" % action)

    # ---- the answer -------------------------------------------------------

    def _tick(self):
        now = time.monotonic()
        status = self._status.get(now)
        state, task, reason, mission_active = mission_core.decide(
            mode=None if status is None else status.mode,
            armed=None if status is None else status.armed,
            killed=self._killed,
            mission_active=self._mission_active,
            mission_name=self._mission_name,
            task=self._task,
            bench_auto=bool(self.p["bench_auto"]))

        if state is None:
            # Publish nothing rather than guess. ocs_client sees mission_state
            # go stale and lets the heartbeat lapse, which is the truth.
            if reason != self._quiet_reason:
                self._quiet_reason = reason
                self.get_logger().warning(
                    "no mission_state: %s — the OCS will show rising silence"
                    % reason)
            return
        self._quiet_reason = None

        if (state, task) != self._last_published:
            self._last_published = (state, task)
            self.get_logger().info("state=%s task=%s (%s)" % (state, task, reason))

        m = MissionState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state = state
        m.task = task
        m.reason = reason
        m.mission_active = bool(mission_active)
        m.mission_name = self._mission_name
        self.pub.publish(m)


def main(args=None):
    run_node(MissionPlanner, args)


if __name__ == "__main__":
    main()
