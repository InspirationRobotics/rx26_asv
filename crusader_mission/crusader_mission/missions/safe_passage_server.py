"""safe_passage_server — Task 1 Safe Passage as a ROS action server.

    ros2 action send_goal /crsd/safe_passage crusader_msgs/action/SafePassage \
      "{tier: 0, timeout_s: 20.0}" --feedback

  in:  /crsd/fcu_status        (FcuStatus)  mode + armed — THE autonomy latch
       /crsd/pose              (LatLonHead) where we are, for the result
  out: /crsd/current_task      (String, latched)  task-state source for the OCS
       /crsd/autonomy_active   (Bool)  heartbeat that turns the mast light GREEN
       /crsd/avoidance_enable  (Bool, latched)  avoidance on for the whole run
       /crsd/guided_setpoint   (GuidedSetpoint)  ONLY when publish_setpoints

THE MISSION IS CURRENTLY EMPTY, ON PURPOSE. Every phase is a dwell (see
safe_passage_core), and `publish_setpoints` defaults FALSE, so this node cannot
move the boat. What it does exercise is the workflow that has to work before any
navigation logic is worth writing — goal accepted, feedback streaming, cancel
honoured promptly, result returned, tree advances — plus the three ways a run
ends badly, on the real vehicle, with real HEARTBEAT.

THREE THINGS THAT ARE NOT SCAFFOLDING, and will not change when the mission
fills in:

1. **The autonomy latch is the flight mode.** `shared.autonomous_modes` — the
   same list telemetry_bridge gates on, the LED shows and the OCS reports. The
   pilot moving SC out of GUIDED ends the goal with OUTCOME_NOT_AUTONOMOUS. We
   do not try to take the mode back; that is the pilot's switch.

2. **Belt and braces on movement.** Even with publish_setpoints true, the boat
   moves only if BOTH we choose to send (this latch) and ArduPilot chooses to
   obey (SET_POSITION_TARGET_* is ignored outside GUIDED). Neither alone.

3. **One goal at a time.** A second goal is REJECTED, not queued and not
   preempting. Two missions steering one boat is not a state anyone can reason
   about at a dock, and a queue would let a goal sit invisible behind a running
   one until long after whoever sent it stopped watching.

CANCEL. Written first, because a cancel that arrives late is a cancel that does
not work. The server runs under a MultiThreadedExecutor with its action server
in a ReentrantCallbackGroup: without both, execute_callback blocks the executor
that would have to service the cancel, and the cancel is only noticed when the
mission ends by itself. The loop checks cancel at the TOP of every tick, so the
worst-case latency is one tick (default 10 Hz), not one phase.
"""
import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from crusader_msgs.action import SafePassage
from crusader_msgs.msg import FcuStatus, GuidedSetpoint, LatLonHead

from crusader_common import config as crsd_config
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config
from crusader_common.stream_cache import StreamCache

from . import safe_passage_core as spc
from .phase_sequencer import (OUTCOME_CANCELLED, OUTCOME_FAULT,
                              OUTCOME_NAMES, OUTCOME_SUCCESS,
                              PhaseSequencer)

PARAM_SPEC = {
    "action_name": dict(read_only=True,
                        description="action name; the tree's leaf must match"),
    "tick_hz": dict(read_only=True, lo=1.0, hi=50.0,
                    description="mission tick rate; also the cancel latency"),
    "default_timeout_s": dict(read_only=True, lo=1.0, hi=3600.0,
                              description="used when the goal sends 0"),
    "mode_grace_s": dict(read_only=True, lo=0.0, hi=30.0,
                         description="how long an unknown mode is tolerated at goal start"),
    "publish_setpoints": dict(read_only=True,
                              description="FALSE = the boat cannot be moved by this node"),
    "orbit_radius_m": dict(read_only=True, lo=1.0, hi=50.0,
                           description="default when the goal sends 0"),
    "task_token": dict(read_only=True,
                       description="what to publish on /crsd/current_task while running"),
    "phase_dwell_s": dict(read_only=True,
                          description="per-phase seconds, as 'PHASE:secs' strings"),
}

# Latched: a subscriber that starts after us must still learn the current value
# rather than sit on a default. avoidance_enable especially — proximity_bridge
# coming up mid-mission and defaulting to "off" would silently disarm avoidance.
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

# The task token for "not attempting a task". The handbook (3.4.11) makes the
# transition INTO a task value the start of the attempt and the transition to
# TASK_NONE its end, so this string is a wire value, not a label.
TASK_NONE = "TASK_NONE"


def parse_phase_dwell(entries):
    """['APPROACH:8', 'OBSERVE:15'] -> {'APPROACH': 8.0, 'OBSERVE': 15.0}.

    A list of strings rather than a nested dict because rcl's parameter types
    are flat — there is no dict parameter, and encoding one as parallel
    name/value arrays lets them drift out of step. Raises on a malformed entry
    instead of skipping it: a typo that silently leaves a phase at its default
    reads as "the parameter had no effect", which is a long afternoon.
    """
    out = {}
    for e in entries:
        name, sep, value = str(e).partition(":")
        if not sep:
            raise ValueError("phase_dwell_s entry {!r} is not 'PHASE:seconds'"
                             .format(e))
        try:
            out[name.strip().upper()] = float(value)
        except ValueError:
            raise ValueError("phase_dwell_s entry {!r} has a non-numeric time"
                             .format(e))
    return out


class SafePassageServer(Node):
    def __init__(self):
        super().__init__("safe_passage_server")
        p = declare_from_config(
            self, crsd_config.node_params("safe_passage_server"), PARAM_SPEC)
        self._tick_dt = 1.0 / float(p["tick_hz"])
        self._default_timeout_s = float(p["default_timeout_s"])
        self._mode_grace_s = float(p["mode_grace_s"])
        self._publish_setpoints = bool(p["publish_setpoints"])
        self._orbit_radius_m = float(p["orbit_radius_m"])
        self._task_token = str(p["task_token"])
        self._dwell = parse_phase_dwell(p["phase_dwell_s"])

        shared = crsd_config.shared_params()
        self._auto_modes = {str(m).upper()
                            for m in shared.get("autonomous_modes", ())}
        if not self._auto_modes:
            raise RuntimeError(
                "shared.autonomous_modes is empty — with no autonomous mode "
                "list this node could never be allowed to drive, and starting "
                "it would only look like it might")

        # Both caches use the shared pose_timeout_s: a mission trusting state
        # for longer than telemetry_bridge vouches for it is the frozen-pose
        # failure that value exists to prevent.
        timeout = float(shared["pose_timeout_s"])
        self._status = StreamCache(timeout)
        self._pose = StreamCache(timeout)

        self.create_subscription(FcuStatus, "/crsd/fcu_status",
                                 self._status_cb, 10)
        self.create_subscription(LatLonHead, "/crsd/pose", self._pose_cb, 10)

        self._task_pub = self.create_publisher(String, "/crsd/current_task",
                                               LATCHED)
        self._avoid_pub = self.create_publisher(Bool, "/crsd/avoidance_enable",
                                                LATCHED)
        self._autonomy_pub = self.create_publisher(Bool,
                                                   "/crsd/autonomy_active", 10)
        self._setpoint_pub = self.create_publisher(GuidedSetpoint,
                                                   "/crsd/guided_setpoint", 10)

        # Publish the idle task state at startup so the latched topic is never
        # empty. A subscriber that finds nothing there cannot tell "no mission"
        # from "the publisher is dead".
        self._publish_task(TASK_NONE)
        self._avoid_pub.publish(Bool(data=True))

        self._busy = False
        self._group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self, SafePassage, str(p["action_name"]),
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._group)

        self.get_logger().info(
            "safe_passage ready on {}: autonomous modes {}, setpoints {}, "
            "phases {}".format(
                p["action_name"], sorted(self._auto_modes),
                "ENABLED" if self._publish_setpoints else "DISABLED (read-only)",
                " ".join("{}={}s".format(n, self._dwell.get(n, 1.0))
                         for n in spc.ORDER)))
        if not self._publish_setpoints:
            self.get_logger().warn(
                "publish_setpoints is FALSE — this node CANNOT move the boat. "
                "That is the read-only posture, not a fault.")

    # ---- inputs ----

    def _status_cb(self, msg):
        self._status.set(msg, time.monotonic())

    def _pose_cb(self, msg):
        self._pose.set(msg, time.monotonic())

    def _autonomous(self):
        """True / False / None — see PhaseSequencer for why None is not False."""
        status = self._status.get(time.monotonic())
        if status is None:
            return None
        return str(status.mode).upper() in self._auto_modes

    def _publish_task(self, token):
        self._task_pub.publish(String(data=token))

    # ---- goal handling ----

    def _on_goal(self, goal_request):
        if self._busy:
            self.get_logger().warn(
                "REJECTING a second goal — one mission at a time. Cancel the "
                "running one first.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        self.get_logger().info("cancel requested; stopping at the next tick")
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        self._busy = True
        req = goal_handle.request
        timeout_s = req.timeout_s if req.timeout_s > 0 else self._default_timeout_s
        started = time.monotonic()
        entry = exit_ = None
        try:
            seq = PhaseSequencer(spc.build_phases(self._dwell),
                                 timeout_s=timeout_s,
                                 mode_grace_s=self._mode_grace_s)
            seq.start(started)
            self._publish_task(self._task_token)
            self.get_logger().info(
                "GOAL tier={} timeout={:.0f}s approach=({:.6f}, {:.6f}) "
                "orbit_r={:.1f}m".format(
                    req.tier, timeout_s, req.approach_latitude,
                    req.approach_longitude,
                    req.orbit_radius_m or self._orbit_radius_m))

            verdict = self._run_loop(goal_handle, seq)

            elapsed = time.monotonic() - started
            # TWO call sites, not one logger picked by a conditional. rclpy
            # caches a logger's severity PER CALL SITE and raises
            #   ValueError: Logger severity cannot be changed between calls
            # the second time one line logs at a different level. `say = info if
            # ok else warn` therefore works for the first result and throws for
            # every result of the other kind afterwards — which lands INSIDE
            # execute_callback, so rclpy returns a DEFAULT result and the caller
            # sees outcome 0 (SUCCESS) on a run that timed out. Observed on the
            # boat 2026-09-05: a TIMEOUT reported as outcome 0, status ABORTED.
            line = "RESULT {} after {:.1f}s in {}: {}".format(
                OUTCOME_NAMES.get(verdict.outcome, verdict.outcome), elapsed,
                verdict.phase, verdict.detail)
            if verdict.outcome == OUTCOME_SUCCESS:
                self.get_logger().info(line)
            else:
                self.get_logger().warn(line)

            # rclpy needs the terminal state set explicitly, and the RIGHT one:
            # a cancelled goal that is `succeed()`-ed reports success to the
            # caller, so the tree carries on as though the mission had worked.
            if verdict.outcome == OUTCOME_CANCELLED:
                goal_handle.canceled()
            elif verdict.outcome == OUTCOME_SUCCESS:
                goal_handle.succeed()
            else:
                goal_handle.abort()

            return self._result(verdict, elapsed, entry, exit_)
        except Exception as e:
            # NEVER let an exception here reach rclpy: it returns a DEFAULT
            # Result, whose outcome field is 0 — SUCCESS. A crash that reports
            # success is worse than a crash.
            self.get_logger().error("mission raised {}: {}".format(
                type(e).__name__, e))
            try:
                goal_handle.abort()
            except Exception:
                pass
            r = SafePassage.Result()
            r.outcome = OUTCOME_FAULT
            r.detail = "mission raised {}: {}".format(type(e).__name__, e)
            r.elapsed_s = time.monotonic() - started
            r.entry_latitude, r.entry_longitude = float("nan"), float("nan")
            r.exit_latitude, r.exit_longitude = float("nan"), float("nan")
            return r
        finally:
            # Everything here must happen on EVERY exit path, including an
            # exception thrown from the loop. Leaving current_task at the task
            # value tells the OCS the attempt is still running, and leaving
            # avoidance disabled leaves the boat blind for the next mission.
            self._publish_task(TASK_NONE)
            self._avoid_pub.publish(Bool(data=True))
            self._autonomy_pub.publish(Bool(data=False))
            self._busy = False

    def _run_loop(self, goal_handle, seq):
        """Tick until the sequencer says stop. Returns the terminal Verdict."""
        fb = SafePassage.Feedback()
        while True:
            now = time.monotonic()
            verdict = seq.poll(now,
                               autonomous=self._autonomous(),
                               cancel_requested=goal_handle.is_cancel_requested,
                               world=None)
            if not verdict.running:
                return verdict

            if verdict.phase_changed:
                self.get_logger().info("phase -> {}".format(verdict.phase))

            # The LED heartbeat, and it is deliberately inside the loop rather
            # than on its own timer: the mast light goes GREEN because a mission
            # is actually ticking, not because a node is merely alive.
            self._autonomy_pub.publish(Bool(data=True))

            fb.phase = verdict.phase
            fb.progress = float(verdict.progress)
            fb.buoys_known = 0
            fb.buoys_resolved = 0
            fb.plan_version = 0
            fb.warning = verdict.detail
            goal_handle.publish_feedback(fb)

            time.sleep(self._tick_dt)

    def _result(self, verdict, elapsed, entry, exit_):
        r = SafePassage.Result()
        r.outcome = int(verdict.outcome)
        r.detail = verdict.detail
        r.buoys_classified = 0
        r.buoys_passed_correctly = 0
        r.elapsed_s = float(elapsed)
        # NaN, not 0.0. (0, 0) is a real place in the Gulf of Guinea, and a
        # result that reports it reads as a measurement. NaN is the blank.
        r.entry_latitude, r.entry_longitude = self._latlon_or_blank(entry)
        r.exit_latitude, r.exit_longitude = self._latlon_or_blank(exit_)
        return r

    @staticmethod
    def _latlon_or_blank(fix):
        if fix is None:
            return float("nan"), float("nan")
        return float(fix[0]), float(fix[1])


def main(args=None):
    # MultiThreadedExecutor is not optional here — see the module docstring.
    # Passed as a FACTORY: an Executor built before rclpy.init() dies with a
    # bare AttributeError that names neither the executor nor init.
    run_node(SafePassageServer, args, executor_factory=MultiThreadedExecutor)


if __name__ == "__main__":
    main()
