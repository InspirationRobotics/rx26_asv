"""ocs_client — publishes Crusader's heartbeat to the Operator Control Station.

NOT PROVEN ON THE BOAT. Deliberately absent from core.launch.py for the same
reason ground_station is: it has never carried a real run. Start it by hand.

    ros2 run crusader_groundstation ocs_client

The OCS holds the team's single connection to RoboNation's RoboCommand and is the
only thing allowed to talk to it. This node's whole job is to keep the OCS
supplied with a 2 Hz heartbeat -- the rate the handbook mandates for every active
vehicle -- over one TCP connection that also carries commands back.

WHAT THIS NODE MUST NEVER DO IS ACT. Inbound commands are republished to
/crsd/ocs_command and nothing else. The mission executive may choose to read them;
this node has no opinion. The standing rule holds unchanged: the RC e-stop is the
only safety path, and WiFi is never a safety mechanism.

THE TASK IT REPORTS IS THE ONE A MISSION CLAIMS. `current_task` comes from
/crsd/current_task, published by whichever mission action server is running, and
it is not a cosmetic field: the handbook (3.4.11) makes the transition INTO a
task value the start of that attempt and the transition to TASK_NONE the stand
down, and above Core the beacons activate on it. Two rules guard it, both in
_on_task/_current_task:

  * a token not in RoboCommand's RxTask enum is REFUSED, not forwarded —
    protobuf's ParseDict rejects the whole frame on an unknown enum name, so a
    typo would cost every heartbeat sent while it was set, not just the field;
  * if the publisher disappears while a task is claimed, we stand down to
    TASK_NONE. The topic is latched and published only on transitions, so a dead
    mission node would otherwise leave us telling RoboCommand a task is under way
    on behalf of a process that no longer exists.

TELEMETRY IT WILL NOT INVENT. If /crsd/pose or /crsd/fcu_status has gone stale,
no heartbeat is sent at all. A fabricated position is worse than a gap: the gap is
visible on the OCS as rising silence, while a made-up fix is indistinguishable
from a real one and scores as though the boat were somewhere it is not. The same
reasoning bans STATE_UNKNOWN -- it is the proto zero value, the OCS validator
refuses it outright, and "the reporter forgot" and "the boat does not know" are
not the same claim.
"""
import json
import math
import time
from collections import deque

from crusader_common import config as crsd_config
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config
from crusader_common.stream_cache import StreamCache
from crusader_msgs.msg import Attitude, FcuStatus, LatLonHead
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from crusader_groundstation.ocs_link import (TASK_NONE, OcsLink,
                                             coerce_task, fake_report,
                                             rfc3339)

PARAM_SPEC = {
    "ocs_host": dict(read_only=True,
                     description="the OCS on the TEAM subnet. No discovery: "
                                 "give the OCS a static address or a DHCP "
                                 "reservation. NOT 127.0.0.1, which is this "
                                 "Jetson."),
    "ocs_port": dict(read_only=True, lo=1024, hi=65535,
                     description="clear of 8090 ground_station and 8080/8081 "
                                 "the viewers"),
    "vehicle_id": dict(read_only=True,
                       description="must match a [[vehicle]] id in the OCS "
                                   "bridge.toml, or every frame is dropped"),
    "team_id": dict(read_only=True, description="= bridge.toml team_id"),
    "rate_hz": dict(read_only=True, lo=0.1, hi=10.0,
                    description="2.0 is the handbook's mandated heartbeat rate"),
    "fake_telemetry": dict(read_only=True,
                           description="synthesize a circle instead of "
                                       "subscribing; for bench use with no "
                                       "Pixhawk. Logs a warning while on."),
    "pose_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                           description="= shared.pose_timeout_s"),
    "attitude_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                               description="stale -> roll/pitch omitted"),
    "status_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                             description="stale -> no heartbeat at all, since "
                                         "state cannot be known"),
}


class OcsClient(Node):
    def __init__(self):
        super().__init__("ocs_client")
        p = declare_from_config(self, crsd_config.node_params("ocs_client"),
                                PARAM_SPEC)
        self.p = p

        self._pose = StreamCache(p["pose_timeout_s"])
        self._att = StreamCache(p["attitude_timeout_s"])
        self._status = StreamCache(p["status_timeout_s"])
        # `_autonomy` (/crsd/autonomy_active) is still subscribed because the
        # ground station shows it, but it no longer decides STATE_AUTO -- the
        # flight mode does. See _build().
        self._autonomy = False
        self._kill = False
        # The task a mission node claims to be attempting. See _current_task().
        self._task = TASK_NONE
        self._warned_tasks = set()
        self._auto_modes = {str(m).upper() for m in
                            crsd_config.shared_params().get(
                                "autonomous_modes", ("AUTO", "GUIDED"))}
        self.get_logger().info(
            f"STATE_AUTO reported for modes: {sorted(self._auto_modes)}")
        self._t0 = time.time()
        self._skipped = 0

        # Commands arrive on the link thread; rclpy publishers are not promised
        # to be thread-safe, so they are parked here and published from the
        # timer. Bounded: if nothing is draining, dropping the oldest advisory
        # is better than growing without limit.
        self._inbox = deque(maxlen=32)
        self._cmd_pub = self.create_publisher(String, "/crsd/ocs_command", 10)

        if p["fake_telemetry"]:
            self.get_logger().warning(
                "fake_telemetry is ON -- this node is publishing invented "
                "positions to the OCS. Never leave this set for a scored run.")
        else:
            self.create_subscription(LatLonHead, "/crsd/pose", self._on_pose, 10)
            self.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)
            self.create_subscription(FcuStatus, "/crsd/fcu_status",
                                     self._on_status, 10)
            self.create_subscription(Bool, "/crsd/autonomy_active",
                                     self._on_autonomy, 10)
            self.create_subscription(Bool, "/crsd/kill_active", self._on_kill, 10)
            # TRANSIENT_LOCAL to match safe_passage_server: current_task is
            # published on TRANSITIONS, not periodically, so a VOLATILE
            # subscription that starts mid-mission would never learn the task
            # was already under way and would report TASK_NONE through the
            # whole attempt.
            self.create_subscription(
                String, "/crsd/current_task", self._on_task,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.link = OcsLink(p["ocs_host"], int(p["ocs_port"]),
                            on_command=self._inbox.append,
                            log=lambda m: self.get_logger().info(str(m)))
        self.link.start()
        self.get_logger().info(
            "ocs_client: %s -> %s:%d at %.1f Hz"
            % (p["vehicle_id"], p["ocs_host"], int(p["ocs_port"]), p["rate_hz"]))

        self.create_timer(1.0 / float(p["rate_hz"]), self._tick)

    def destroy_node(self):
        self.link.stop()
        return super().destroy_node()

    # ---- subscriptions ---------------------------------------------------

    def _on_pose(self, msg):
        self._pose.set(msg, time.monotonic())

    def _on_att(self, msg):
        self._att.set(msg, time.monotonic())

    def _on_status(self, msg):
        self._status.set(msg, time.monotonic())

    def _on_autonomy(self, msg):
        self._autonomy = bool(msg.data)

    def _on_kill(self, msg):
        self._kill = bool(msg.data)

    def _on_task(self, msg):
        """Accept a task token only if RoboCommand's schema has that name.

        A token the OCS cannot parse is not a cosmetic error: ParseDict rejects
        the frame and the heartbeat never reaches RoboCommand at all. So an
        unknown token costs EVERY heartbeat sent while it is set, not just the
        field. Falling back to TASK_NONE keeps the position, speed and state
        flowing, which is the part that cannot be reconstructed later.

        Warned once per distinct bad token — a typo at 2 Hz would otherwise bury
        the log in the same line.
        """
        token, error = coerce_task(msg.data)
        if error and str(msg.data) not in self._warned_tasks:
            self._warned_tasks.add(str(msg.data))
            self.get_logger().error("REFUSING task token: " + error)
        elif not error and token != self._task:
            self.get_logger().info("current_task -> %s" % token)
        self._task = token

    def _current_task(self):
        """The task to report, or TASK_NONE if nothing is claiming one.

        THE LIVENESS CHECK IS THE POINT. current_task is latched and published
        only on transitions, so if the mission node dies mid-attempt the last
        value sits here forever and this node keeps telling RoboCommand a task
        is under way. The handbook makes the transition to TASK_NONE the
        stand-down signal, so a frozen token is not a stale field — it is a
        claim we are still trying, made on behalf of a process that no longer
        exists.

        A publisher count of zero says exactly that, and needs no keepalive from
        the mission node. The normal path is still the mission server's own
        `finally`, which publishes TASK_NONE on every exit; this covers the
        abnormal one.
        """
        if self._task != TASK_NONE and not self.count_publishers(
                "/crsd/current_task"):
            self.get_logger().warning(
                "task source vanished while claiming %s — standing down to %s"
                % (self._task, TASK_NONE))
            self._task = TASK_NONE
        return self._task

    # ---- the heartbeat ---------------------------------------------------

    def _tick(self):
        while self._inbox:
            self._republish(self._inbox.popleft())

        report = self._build(time.monotonic())
        if report is None:
            return
        self.link.publish(report)

    def _republish(self, cmd):
        """Hand an OCS command to whoever wants it. This node does not act."""
        self._cmd_pub.publish(String(data=json.dumps(cmd)))
        self.get_logger().info("OCS command -> /crsd/ocs_command: %s"
                               % sorted(cmd))

    def _build(self, now):
        p = self.p
        if p["fake_telemetry"]:
            return fake_report(p["vehicle_id"], p["team_id"], self._t0)

        pose = self._pose.get(now)
        status = self._status.get(now)
        if pose is None or status is None:
            # Say so once per transition, not twice a second.
            if self._pose.went_stale(now) or self._status.went_stale(now):
                self.get_logger().warning(
                    "no heartbeat: pose or fcu_status is stale -- the OCS will "
                    "show rising silence, which is the truth")
            self._skipped += 1
            return None

        # STATE_AUTO is decided by the SAME list telemetry_bridge gates commands
        # on and pixhawk_led_status lights GREEN for (shared.autonomous_modes).
        # Before this, three places decided "are we autonomous" independently and
        # could disagree: the bridge could be refusing every setpoint while this
        # reported STATE_AUTO to RoboCommand. Reporting a state we are not in is
        # worse than reporting a boring one.
        #
        # `armed` stays in the AND: a disarmed boat is not driving itself
        # whatever mode is selected.
        if self._kill:
            state = "STATE_KILLED"
        elif status.armed and str(status.mode).upper() in self._auto_modes:
            state = "STATE_AUTO"
        else:
            state = "STATE_MANUAL"

        hb = {
            "state": state,
            "position": {"latitude": pose.latitude, "longitude": pose.longitude},
            "spd_mps": pose.ground_speed,
            # Passed through as-is, NaN and all. telemetry_bridge sets it NaN
            # when GPS yaw is unresolved, and the OCS validator exists to catch
            # exactly that -- scrubbing it here would hide a real fault.
            "heading_deg": pose.heading,
            "vehicle_type": "TYPE_USV",
            # From /crsd/current_task, published by whichever mission action
            # server is running. Validated against RoboCommand's own RxTask enum
            # and defaulted to TASK_NONE — see _on_task and _current_task.
            "current_task": self._current_task(),
        }

        att = self._att.get(now)
        if att is not None:
            # Attitude is radians in the autopilot's own NED axes; the report
            # wants degrees.
            hb["roll_deg"] = math.degrees(att.roll)
            hb["pitch_deg"] = math.degrees(att.pitch)

        return {"team_id": p["team_id"], "vehicle_id": p["vehicle_id"],
                "sent_at": rfc3339(), "heartbeat": hb}


def main(args=None):
    run_node(OcsClient, args)


if __name__ == "__main__":
    main()
