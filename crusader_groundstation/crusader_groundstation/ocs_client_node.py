"""ocs_client — Crusader's link to the Operator Control Station, which holds the team's one
connection to RoboNation's RoboCommand. Not in core.launch.py: start it from the Nodes tab.

  out to the OCS: the handbook's 2 Hz heartbeat (latest-wins) and every task report from
                  /crsd/robocommand/report (in order, none dropped), each as
                  {team_id, vehicle_id, sent_at, <kind>: body}
  in:             OCS commands -> /crsd/ocs_command. THIS NODE NEVER ACTS on one.
  echo:           /crsd/ocs_traffic, JSON {dir, kind, body, status, stamp}, for the GCS

`current_task` comes from /crsd/current_task: handbook 3.4.11 makes the transition INTO a
task the start of that attempt, and above Core the beacons activate on it. Unknown tokens
are refused and a vanished publisher stands us down.

FINDING THE OCS. `ocs_host: "auto"` listens for the OCS's UDP beacon (which must carry
`team_id`) and dials the sender, falling back to `ocs_fallback_hosts`; a fixed IP is dialled
and nothing else (ocs_link.py).

TELEMETRY IT WILL NOT INVENT. With /crsd/pose or /crsd/fcu_status stale no heartbeat is
sent: the OCS shows rising silence, where a made-up fix would score as a real position.
"""
import json
import math
import time
from collections import deque

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from crusader_common.node_main import declare_params, run_node
from crusader_common.stream_cache import StreamCache
from crusader_msgs.msg import Attitude, FcuStatus, LatLonHead, RoboCommandReport

from crusader_groundstation.ocs_link import (DEFAULT_DISCOVERY_PORT, DEFAULT_FALLBACK_HOSTS,
                                             TASK_NONE, OcsLink, coerce_task, fake_report,
                                             json_safe, rfc3339)

PARAMS = {
    "ocs_host": "auto",                 # or a fixed OCS IP on the TEAM subnet, never a
                                        # RoboCommand course address (192.168.65-68.x)
    "ocs_port": 37564,
    "ocs_discovery_port": DEFAULT_DISCOVERY_PORT,   # = [discovery] port in bridge.toml; 0 = none
    "ocs_fallback_hosts": list(DEFAULT_FALLBACK_HOSTS),
    "vehicle_id": "USV1",               # must match a [[vehicle]] id in bridge.toml
    "team_id": "ASTA",                  # = bridge.toml team_id
    "rate_hz": 2.0,                     # the handbook's mandated heartbeat rate
    "fake_telemetry": False,            # a synthetic circle, for the bench with no Pixhawk
    "autonomous_modes": ["GUIDED", "AUTO", "LOITER", "RTL"],   # = telemetry_bridge's
    "pose_timeout_s": 1.0,
    "attitude_timeout_s": 1.0,          # stale -> roll/pitch omitted
    "status_timeout_s": 1.0,            # stale -> no heartbeat: the state is unknowable
}
HEARTBEAT_ECHO_S = 5.0                  # one heartbeat in the traffic echo per this, and
                                        # every change of state or task


class OcsClient(Node):
    def __init__(self):
        super().__init__("ocs_client")
        p = self.p = declare_params(self, PARAMS)
        self._pose = StreamCache(p["pose_timeout_s"])
        self._att = StreamCache(p["attitude_timeout_s"])
        self._status = StreamCache(p["status_timeout_s"])
        self._kill = False
        self._task = TASK_NONE
        self._warned_tasks = set()
        self._auto_modes = {str(m).upper() for m in p["autonomous_modes"]}
        self._t0 = time.time()
        self._echoed = (None, 0.0)          # (state, task), when

        # The link thread hands frames over through these, drained by the timer:
        # rclpy publishers are not promised thread-safe.
        self._inbox = deque(maxlen=32)      # commands from the OCS
        self._outbox = deque(maxlen=500)    # frames that went out
        self._cmd_pub = self.create_publisher(String, "/crsd/ocs_command", 10)
        self._traffic_pub = self.create_publisher(String, "/crsd/ocs_traffic", 50)
        self.create_subscription(RoboCommandReport, "/crsd/robocommand/report",
                                 self._on_report,
                                 QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))

        if p["fake_telemetry"]:
            self.get_logger().warning("fake_telemetry is ON: this node sends invented "
                                      "positions to the OCS. Never for a scored run.")
        else:
            self.create_subscription(LatLonHead, "/crsd/pose",
                                     lambda m: self._pose.set(m, time.monotonic()), 10)
            self.create_subscription(Attitude, "/crsd/attitude",
                                     lambda m: self._att.set(m, time.monotonic()), 10)
            self.create_subscription(FcuStatus, "/crsd/fcu_status",
                                     lambda m: self._status.set(m, time.monotonic()), 10)
            self.create_subscription(Bool, "/crsd/kill_active", self._on_kill, 10)
            # Latched like bt_runner's publisher: it publishes on transitions only.
            self.create_subscription(
                String, "/crsd/current_task", self._on_task,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.link = OcsLink(
            str(p["ocs_host"]).strip(), int(p["ocs_port"]), on_command=self._inbox.append,
            on_sent=self._outbox.append, log=lambda m: self.get_logger().info(str(m)),
            team=str(p["team_id"]), discovery_port=int(p["ocs_discovery_port"]),
            fallback_hosts=list(p["ocs_fallback_hosts"]))
        self.link.start()
        self.get_logger().info(f"ocs_client: {p['vehicle_id']} -> {self.link.describe()} at "
                               f"{p['rate_hz']:.1f} Hz; STATE_AUTO in {sorted(self._auto_modes)}")
        self.create_timer(1.0 / float(p["rate_hz"]), self._tick)

    def destroy_node(self):
        self.link.stop()
        return super().destroy_node()

    def _envelope(self, kind, body):
        return {"team_id": self.p["team_id"], "vehicle_id": self.p["vehicle_id"],
                "sent_at": rfc3339(), kind: body}

    def _echo(self, direction, kind, body, status):
        self._traffic_pub.publish(String(data=json.dumps(json_safe(
            {"dir": direction, "kind": kind, "body": body, "status": status,
             "stamp": time.time()}), default=str)))

    # ---- inputs ----------------------------------------------------------

    def _on_kill(self, msg):
        self._kill = bool(msg.data)

    def _on_report(self, msg):
        """One task report from a BT leaf, queued for the OCS in arrival order."""
        try:
            body = json.loads(msg.body_json)
        except ValueError as e:
            self.get_logger().error(f"report {msg.kind!r} is not JSON ({e}): not sent")
            self._echo("tx", msg.kind, msg.body_json, f"refused: not JSON ({e})")
            return
        self.link.send_report(self._envelope(msg.kind, body))
        self.get_logger().info(f"report {msg.kind} queued for the OCS")
        self._echo("tx", msg.kind, body, "queued" if self.link.connected else
                   "queued: no OCS link yet")

    def _on_task(self, msg):
        """Accept a task token only if RoboCommand's RxTask enum has it: ParseDict rejects
        the whole frame on an unknown name, so a bad token would cost every heartbeat."""
        token, error = coerce_task(msg.data)
        if error and str(msg.data) not in self._warned_tasks:
            self._warned_tasks.add(str(msg.data))
            self.get_logger().error("REFUSING task token: " + error)
        elif not error and token != self._task:
            self.get_logger().info("current_task -> %s" % token)
        self._task = token

    def _current_task(self):
        """TASK_NONE when the latched publisher has died mid-attempt: its last token would
        otherwise keep telling RoboCommand that a task is under way."""
        if self._task != TASK_NONE and not self.count_publishers("/crsd/current_task"):
            self.get_logger().warning("task source vanished while claiming %s: standing "
                                      "down to %s" % (self._task, TASK_NONE))
            self._task = TASK_NONE
        return self._task

    # ---- the timer ---------------------------------------------------------

    def _tick(self):
        while self._inbox:
            cmd = self._inbox.popleft()
            self._cmd_pub.publish(String(data=json.dumps(cmd)))
            self.get_logger().info("OCS command -> /crsd/ocs_command: %s" % sorted(cmd))
            self._echo("rx", next((k for k in cmd if k not in ("team_id", "vehicle_id",
                                                              "sent_at")), "command"),
                       cmd, "republished")
        while self._outbox:
            self._echo_sent(self._outbox.popleft())
        report = self._heartbeat(time.monotonic())
        if report is not None:
            self.link.publish(report)

    def _echo_sent(self, frame):
        kind = next((k for k in frame if k not in ("team_id", "vehicle_id", "sent_at")), "?")
        if kind == "heartbeat":
            key, now = (frame[kind]["state"], frame[kind]["current_task"]), time.monotonic()
            if key == self._echoed[0] and now - self._echoed[1] < HEARTBEAT_ECHO_S:
                return
            self._echoed = (key, now)
        self._echo("tx", kind, frame[kind], "sent")

    def _heartbeat(self, now):
        p = self.p
        if p["fake_telemetry"]:
            return fake_report(p["vehicle_id"], p["team_id"], self._t0)
        pose = self._pose.get(now)
        status = self._status.get(now)
        if pose is None or status is None:
            if self._pose.went_stale(now) or self._status.went_stale(now):
                self.get_logger().warning("no heartbeat: pose or fcu_status is stale; the "
                                          "OCS will show rising silence, which is the truth")
                self._echo("tx", "heartbeat", None, "withheld: pose or fcu_status stale")
            return None
        # A disarmed boat is not driving itself whatever the mode.
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
            # NaN passed through: unresolved GPS yaw, which the OCS validator catches.
            "heading_deg": pose.heading,
            "vehicle_type": "TYPE_USV",
            "current_task": self._current_task(),
        }
        att = self._att.get(now)
        if att is not None:
            hb["roll_deg"] = math.degrees(att.roll)
            hb["pitch_deg"] = math.degrees(att.pitch)
        return self._envelope("heartbeat", hb)


def main(args=None):
    run_node(OcsClient, args)


if __name__ == "__main__":
    main()
