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

TELEMETRY IT WILL NOT INVENT. If /crsd/pose or /crsd/mission_state has gone
stale, no heartbeat is sent at all. A fabricated position is worse than a gap: the gap is
visible on the OCS as rising silence, while a made-up fix is indistinguishable
from a real one and scores as though the boat were somewhere it is not. The same
reasoning bans STATE_UNKNOWN -- it is the proto zero value, the OCS validator
refuses it outright, and "the reporter forgot" and "the boat does not know" are
not the same claim.

IT DOES NOT DECIDE THE STATE. mission_planner owns STATE_AUTO/MANUAL/KILLED
and the current task; this node carries its answer. Deriving state here as
well is how the status light the safety officer reads and the state
RoboNation scores end up disagreeing.
"""
import json
import math
import time
from collections import deque

from crusader_common import config as crsd_config
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config
from crusader_common.stream_cache import StreamCache
from crusader_msgs.msg import Attitude, LatLonHead, MissionState
from rclpy.node import Node
from std_msgs.msg import Bool, String

from crusader_groundstation.ocs_link import OcsLink, fake_report, rfc3339

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
    "mission_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                              description="stale mission_state -> no "
                                          "heartbeat; the planner owns "
                                          "state and task"),
}


class OcsClient(Node):
    def __init__(self):
        super().__init__("ocs_client")
        p = declare_from_config(self, crsd_config.node_params("ocs_client"),
                                PARAM_SPEC)
        self.p = p

        self._pose = StreamCache(p["pose_timeout_s"])
        self._att = StreamCache(p["attitude_timeout_s"])
        # state and task come from mission_planner, not from here. This
        # node transmits; it does not decide.
        self._mission = StreamCache(p["mission_timeout_s"])
        self._kill = False
        self._t0 = time.time()
        self._skipped = 0

        # Commands arrive on the link thread; rclpy publishers are not promised
        # to be thread-safe, so they are parked here and published from the
        # timer. Bounded: if nothing is draining, dropping the oldest advisory
        # is better than growing without limit.
        self._inbox = deque(maxlen=32)
        self._cmd_pub = self.create_publisher(String, "/crsd/ocs_command", 10)
        # Directives are OURS, not RoboNation's: a separate topic so the
        # mission planner can subscribe to advice from the OCS without
        # also receiving every RxCommand relayed from the course.
        self._dir_pub = self.create_publisher(String, "/crsd/ocs_directive", 10)

        if p["fake_telemetry"]:
            self.get_logger().warning(
                "fake_telemetry is ON -- this node is publishing invented "
                "positions to the OCS. Never leave this set for a scored run.")
        else:
            self.create_subscription(LatLonHead, "/crsd/pose", self._on_pose, 10)
            self.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)
            self.create_subscription(Bool, "/crsd/kill_active", self._on_kill, 10)
            self.create_subscription(MissionState, "/crsd/mission_state",
                                     self._on_mission, 10)

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

    def _on_kill(self, msg):
        self._kill = bool(msg.data)

    def _on_mission(self, msg):
        self._mission.set(msg, time.monotonic())

    # ---- the heartbeat ---------------------------------------------------

    def _tick(self):
        while self._inbox:
            self._republish(self._inbox.popleft())

        report = self._build(time.monotonic())
        if report is None:
            return
        self.link.publish(report)

    def _republish(self, cmd):
        """Hand an OCS message to whoever wants it. This node does not act.

        Two shapes arrive on the link: RxCommand relayed from RoboCommand,
        and our own ocs_directive. They go to different topics because they
        have different authority — one is the course talking to the fleet,
        the other is our operator asking. Neither actuates anything here.
        """
        blob = String(data=json.dumps(cmd))
        if "ocs_directive" in cmd:
            self._dir_pub.publish(blob)
            d = cmd.get("ocs_directive") or {}
            self.get_logger().info(
                "OCS directive %r (declaration_seq=%s) -> "
                "/crsd/ocs_directive" % (d.get("action"),
                                         d.get("declaration_seq")))
            return
        self._cmd_pub.publish(blob)
        self.get_logger().info("OCS command -> /crsd/ocs_command: %s"
                               % sorted(cmd))

    def _build(self, now):
        p = self.p
        if p["fake_telemetry"]:
            return fake_report(p["vehicle_id"], p["team_id"], self._t0)

        pose = self._pose.get(now)
        mission = self._mission.get(now)
        if pose is None or mission is None:
            # Say so once per transition, not twice a second.
            if self._pose.went_stale(now) or self._mission.went_stale(now):
                self.get_logger().warning(
                    "no heartbeat: pose or mission_state is stale -- the OCS "
                    "will show rising silence, which is the truth. Is "
                    "mission_planner running?")
            self._skipped += 1
            return None

        # The planner decided these; we only carry them. Deciding here too
        # is how the status light and the scored state drift apart.
        state = mission.state

        hb = {
            "state": state,
            "position": {"latitude": pose.latitude, "longitude": pose.longitude},
            "spd_mps": pose.ground_speed,
            # Passed through as-is, NaN and all. telemetry_bridge sets it NaN
            # when GPS yaw is unresolved, and the OCS validator exists to catch
            # exactly that -- scrubbing it here would hide a real fault.
            "heading_deg": pose.heading,
            "vehicle_type": "TYPE_USV",
            "current_task": mission.task,
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
