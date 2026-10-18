"""ocs_client — publishes Crusader's heartbeat to the Operator Control Station.

NOT PROVEN ON THE BOAT; absent from core.launch.py like ground_station. Start
it by hand:

    ros2 run crusader_groundstation ocs_client

The OCS holds the team's single connection to RoboNation's RoboCommand. This
node keeps it supplied with the handbook's 2 Hz heartbeat over one TCP
connection that also carries commands back.

THIS NODE NEVER ACTS. Inbound commands are republished to /crsd/ocs_command and
nothing else. The RC e-stop is the only safety path; WiFi never is.

`current_task` comes from /crsd/current_task and is not cosmetic: handbook
3.4.11 makes the transition INTO a task value the start of that attempt and
TASK_NONE the stand-down, and above Core the beacons activate on it. Unknown
tokens are refused and a vanished publisher stands us down (_on_task,
_current_task).

FINDING THE OCS. `ocs_host: "auto"` listens for the OCS's UDP beacon (which
must carry this node's `team_id`) on `ocs_discovery_port` and dials the sender,
falling back to `ocs_fallback_hosts`; a fixed IP is dialled and nothing else.
Rules in ocs_link.py; the OCS's half in OCS/rx_bridge/beacon.py.

TELEMETRY IT WILL NOT INVENT. If /crsd/pose or /crsd/fcu_status is stale, no
heartbeat is sent: the gap shows on the OCS as rising silence, while a made-up
fix scores as though the boat were somewhere it is not. STATE_UNKNOWN is banned
for the same reason -- it is the proto zero value and the OCS validator refuses
it.
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

from crusader_groundstation.ocs_link import (DEFAULT_DISCOVERY_PORT,
                                             DEFAULT_FALLBACK_HOSTS, TASK_NONE,
                                             OcsLink, coerce_task, fake_report,
                                             rfc3339)

PARAM_SPEC = {
    "ocs_host": dict(read_only=True,
                     description="'auto' = find the OCS from its UDP beacon, "
                                 "falling back to ocs_fallback_hosts; or a "
                                 "fixed IP of the OCS on the TEAM subnet, "
                                 "dialled and nothing else. NOT 127.0.0.1, "
                                 "which is this Jetson. Never a RoboCommand "
                                 "course address (192.168.65-68.x)."),
    "ocs_port": dict(read_only=True, lo=1024, hi=65535,
                     description="clear of 8090 ground_station and 8080/8081 "
                                 "the viewers. With 'auto' it is the port tried "
                                 "on fallback hosts; a beacon names its own."),
    "ocs_discovery_port": dict(read_only=True, lo=0, hi=65535,
                               description="UDP port this node listens on for "
                                           "the OCS beacon (ocs_host 'auto' "
                                           "only); = [discovery] port in the "
                                           "OCS bridge.toml. 0 = do not "
                                           "listen, use the fallback hosts "
                                           "alone."),
    "ocs_fallback_hosts": dict(read_only=True,
                               description="OCS addresses tried in turn when "
                                           "no beacon is heard (ocs_host "
                                           "'auto' only). IPv4 literals, "
                                           "none on a RoboCommand course "
                                           "network."),
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
        # Recorded but unused: STATE_AUTO comes from the flight mode, see _build().
        self._autonomy = False
        self._kill = False
        # The task a mission node claims to be attempting.
        self._task = TASK_NONE
        self._warned_tasks = set()
        self._auto_modes = {str(m).upper() for m in
                            crsd_config.shared_params().get(
                                "autonomous_modes", ("AUTO", "GUIDED"))}
        self.get_logger().info(
            f"STATE_AUTO reported for modes: {sorted(self._auto_modes)}")
        self._t0 = time.time()
        self._skipped = 0

        # Commands arrive on the link thread and rclpy publishers are not
        # promised thread-safe, so they are parked here for the timer. Bounded:
        # dropping the oldest advisory beats growing without limit.
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
            # TRANSIENT_LOCAL to match bt_runner's latched publisher:
            # current_task is published on transitions only, so a VOLATILE
            # subscriber starting mid-mission would report TASK_NONE throughout.
            self.create_subscription(
                String, "/crsd/current_task", self._on_task,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # .get so a params file without the discovery keys still starts; they
        # matter only when ocs_host is "auto".
        self.link = OcsLink(
            str(p["ocs_host"]).strip(), int(p["ocs_port"]),
            on_command=self._inbox.append,
            log=lambda m: self.get_logger().info(str(m)),
            team=str(p["team_id"]),
            discovery_port=int(p.get("ocs_discovery_port", DEFAULT_DISCOVERY_PORT)),
            fallback_hosts=list(p.get("ocs_fallback_hosts",
                                      DEFAULT_FALLBACK_HOSTS)))
        self.link.start()
        self.get_logger().info(
            "ocs_client: %s -> %s at %.1f Hz"
            % (p["vehicle_id"], self.link.describe(), p["rate_hz"]))

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
        """Accept a task token only if RoboCommand's RxTask enum has that name.

        ParseDict rejects the whole frame on an unknown enum name, so a bad
        token would cost EVERY heartbeat while set, not just the field.
        TASK_NONE keeps position, speed and state flowing. Warned once per
        distinct bad token.
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

        current_task is latched and published only on transitions, so if the
        publisher dies mid-attempt the last token would sit here, still telling
        RoboCommand a task is under way. A publisher count of zero catches that
        with no keepalive; the normal stand-down is the publisher's own
        TASK_NONE on exit.
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

        # STATE_AUTO uses the same shared.autonomous_modes list telemetry_bridge
        # gates commands on and pixhawk_led_status lights GREEN for, so the
        # three cannot disagree. A disarmed boat is not driving itself whatever
        # the mode.
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
            # NaN passed through: telemetry_bridge sets it when GPS yaw is
            # unresolved, and the OCS validator exists to catch exactly that.
            "heading_deg": pose.heading,
            "vehicle_type": "TYPE_USV",
            "current_task": self._current_task(),
        }

        att = self._att.get(now)
        if att is not None:
            # Radians in the autopilot's NED axes; the report wants degrees.
            hb["roll_deg"] = math.degrees(att.roll)
            hb["pitch_deg"] = math.degrees(att.pitch)

        return {"team_id": p["team_id"], "vehicle_id": p["vehicle_id"],
                "sent_at": rfc3339(), "heartbeat": hb}


def main(args=None):
    run_node(OcsClient, args)


if __name__ == "__main__":
    main()
