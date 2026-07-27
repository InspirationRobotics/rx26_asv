"""telemetry_bridge — the single ROS-side gateway to MAVProxy's rebroadcast.

Plan §3.1/§3.2. Two jobs, deliberately fused into one node:

1. RX: consume MAVProxy's rebroadcast (pymavlink over UDP — NEVER a serial device;
   the Pixhawk has exactly one owner and it is MAVProxy) and republish as topics:
     /crsd/pose          interfaces/LatLonHead   (GLOBAL_POSITION_INT)
     /crsd/fcu_status    interfaces/FcuStatus    (HEARTBEAT)
     /crsd/rc_channels   interfaces/RcChannels   (RC_CHANNELS)
     /crsd/autonomy_drop std_msgs/Bool           (latched, TRANSIENT_LOCAL)
   Other nodes subscribe to these topics instead of opening their own MAVLink
   connection — this node existing is what keeps the "no second consumer racing
   the ROS graph" rule enforceable.

2. TX: the ONLY sanctioned path for RC overrides. Nodes publish
   interfaces/RcChannels on /crsd/rc_override; this node forwards them to the
   autopilot — UNLESS the autonomy-drop latch (api.common.drop_latch) has
   tripped, in which case it sends release frames (all-zero override) and drops
   every subsequent override until the operator resets via the
   /crsd/autonomy_drop_reset service (std_srvs/Trigger). Because misbehaving
   nodes have no MAVLink connection of their own, a tripped latch cannot be
   bypassed from the ROS graph. (G1 gate: this is the mechanism under test.)

3. TX (safety): the ONLY sanctioned force-disarm path. rc_heartbeat_watchdog
   publishes std_msgs/Bool on /crsd/force_disarm on RC-transmitter link loss;
   this node forwards it as MAV_CMD_COMPONENT_ARM_DISARM (force magic). Unlike
   the RC-override/GUIDED paths it is NOT gated by the autonomy-drop latch — a
   force-disarm must fire even when the latch has already tripped.

The hardware e-stop (SB switch) remains below and independent of all of this.

Parameters:
  mav_endpoint     (str,  default udp:127.0.0.1:14551)  MAVProxy --out for ROS
  drop_channel     (int,  default 7)     RC channel of the autonomy-drop switch
  drop_threshold   (int,  default 1700)  us; >= trips (or <= if drop_invert)
  drop_invert      (bool, default False)
  rc_stale_timeout (float, default 1.0)  s without RC_CHANNELS -> trip
"""
import json
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from interfaces.msg import (LatLonHead, FcuStatus, RcChannels, DetectionArray,
                            GuidedSetpoint)

from rx26_asv.api.common import config as crsd_config
from rx26_asv.api.common.drop_latch import DropLatch
from rx26_asv.api.common.node_main import run_node
from rx26_asv.api.common.param_utils import declare_from_config
from rx26_asv.api.navigation.fence_core import (FenceError, FenceProtocol, MavFenceTransport,
                         items_from_keepouts)

# All bridge params are SAFETY CONFIG -> read_only: `ros2 param set` is
# rejected; the change path is config/crusader_params.yaml + node restart.
PARAM_SPEC = {
    "mav_endpoint": dict(read_only=True,
                         description="MAVProxy rebroadcast (udp/tcp only)"),
    "drop_channel": dict(read_only=True, lo=1, hi=18,
                         description="autonomy-drop RC channel"),
    "drop_threshold": dict(read_only=True, lo=800, hi=2200,
                           description="us; crossing trips the latch"),
    "drop_invert": dict(read_only=True, description="low = drop position"),
    "rc_stale_timeout": dict(read_only=True, lo=0.2, hi=10.0,
                             description="s without RC before trip"),
}

MISSION_MSG_TYPES = ("MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK",
                     "MISSION_COUNT", "MISSION_ITEM", "MISSION_ITEM_INT")

RELEASE_FRAMES = 5          # all-zero override frames sent on trip
PUB_RATE_HZ = 20.0

# Magic value ArduPilot requires in param2 of MAV_CMD_COMPONENT_ARM_DISARM to
# force-disarm even while the vehicle is moving.
FORCE_DISARM_MAGIC = 21196

# ArduPilot's documented position-only SET_POSITION_TARGET_GLOBAL_INT mask —
# the EXACT value the pre-integration gate_navigator used and field-exercised.
# yaw on GuidedSetpoint is accepted but not commanded yet (left to ArduRover);
# adding yaw control means clearing the yaw-ignore bit and is a separate change.
POSITION_ONLY_TYPE_MASK = 0b110111111100  # 3580


class TelemetryBridge(Node):

    def __init__(self):
        super().__init__("telemetry_bridge")
        p = declare_from_config(self, crsd_config.node_params("telemetry_bridge"),
                                PARAM_SPEC)

        endpoint = p["mav_endpoint"]
        if not (endpoint.startswith("udp") or endpoint.startswith("tcp")):
            # never a serial device — single-Pixhawk-owner rule, fail loudly
            raise ValueError(
                f"mav_endpoint {endpoint!r} is not udp/tcp; refusing (MAVProxy "
                "is the sole Pixhawk owner; this node consumes its rebroadcast)")

        self.latch = DropLatch(
            channel=p["drop_channel"],
            threshold=p["drop_threshold"],
            invert=p["drop_invert"],
            stale_timeout=p["rc_stale_timeout"])

        latched_qos = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(LatLonHead, "/crsd/pose", 10)
        self.status_pub = self.create_publisher(FcuStatus, "/crsd/fcu_status", 10)
        self.rc_pub = self.create_publisher(RcChannels, "/crsd/rc_channels", 10)
        self.drop_pub = self.create_publisher(Bool, "/crsd/autonomy_drop", latched_qos)

        self.create_subscription(RcChannels, "/crsd/rc_override",
                                 self._override_cb, 10)
        # Sanctioned GUIDED setpoint TX — task nodes (gate_navigator) publish here
        # instead of opening their own MAVLink connection. Gated by the same latch
        # as RC overrides, so an autonomy drop stops GUIDED motion too.
        self.create_subscription(GuidedSetpoint, "/crsd/guided_setpoint",
                                 self._guided_cb, 10)
        # Sanctioned force-disarm TX — rc_heartbeat_watchdog publishes here on RC
        # link loss instead of opening its own MAVLink connection. Deliberately
        # NOT gated by the autonomy-drop latch: a force-disarm must fire even
        # (especially) when the latch has already tripped. This is the ONLY
        # sanctioned disarm path from the ROS graph.
        self.create_subscription(Bool, "/crsd/force_disarm",
                                 self._force_disarm_cb, 10)
        self.create_service(Trigger, "/crsd/autonomy_drop_reset", self._reset_cb)

        # --- keep-out -> exclusion-fence path (plan §3.2: AVOID_* is the hard
        # backstop; a fence the autopilot doesn't echo back does not exist) ---
        # /crsd/keepouts semantics: DetectionArray frame="world"; each Detection
        # is a circular zone, label = zone_id; radius <= 0 = All Clear for that
        # zone. Publisher is the Phase-4 RoboCommand comms node (mock until then).
        self.fence_pub = self.create_publisher(String, "/crsd/fence_state", latched_qos)
        self.create_subscription(DetectionArray, "/crsd/keepouts",
                                 self._keepouts_cb, 10)
        self.create_subscription(LatLonHead, "/crsd/world_origin",
                                 self._origin_cb, latched_qos)
        self._origin = None
        self._zones = {}                     # zone_id -> (x, y, radius) world m
        self._zones_lock = threading.Lock()
        self._mission_q = queue.Queue()
        self._fence_dirty = threading.Event()
        self._fence_thread = threading.Thread(target=self._fence_worker, daemon=True)

        self._lock = threading.Lock()
        self._pose = None            # (lat, lon, heading_deg)
        self._status = None          # (mode_str, armed, system_status)
        self._rc = None              # list[int] 18

        from pymavlink import mavutil
        self._mavutil = mavutil
        self.conn = mavutil.mavlink_connection(endpoint)
        self.get_logger().info(f"waiting for heartbeat on {endpoint} ...")

        self._stop = threading.Event()          # deterministic teardown
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self._fence_thread.start()

        self.create_timer(1.0 / PUB_RATE_HZ, self._publish_tick)
        self._publish_drop_state()               # initial latched state (STARTUP=blocked)
        self.get_logger().info(
            f"autonomy-drop: ch{self.latch.channel} thr={self.latch.threshold} "
            f"invert={self.latch.invert} — overrides BLOCKED until safe RC seen")

    # ---------- MAVLink RX ----------

    def _rx_loop(self):
        # interruptible heartbeat wait: 1 s slices so the stop Event works even
        # before MAVProxy is up, with a periodic loud reminder (never silent)
        waited = 0
        while not self._stop.is_set():
            if self.conn.wait_heartbeat(timeout=1.0):
                break
            waited += 1
            if waited % 10 == 0:
                self.get_logger().warn(
                    f"still no heartbeat after {waited}s — is MAVProxy running?")
        else:
            return
        self.get_logger().info("heartbeat OK")
        while not self._stop.is_set():
            msg = self.conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            t = time.monotonic()
            mtype = msg.get_type()
            if mtype in MISSION_MSG_TYPES:
                self._mission_q.put(msg)     # fence dialog msgs -> uploader
                continue
            with self._lock:
                if mtype == "GLOBAL_POSITION_INT":
                    hdg = msg.hdg / 100.0 if msg.hdg != 65535 else float("nan")
                    self._pose = (msg.lat / 1e7, msg.lon / 1e7, hdg)
                elif mtype == "HEARTBEAT" and msg.get_srcComponent() == 1:
                    mode = self._mavutil.mode_string_v10(msg)
                    armed = bool(msg.base_mode &
                                 self._mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self._status = (mode, armed, msg.system_status)
                elif mtype == "RC_CHANNELS":
                    self._rc = [getattr(msg, f"chan{i}_raw", 0) or 0
                                for i in range(1, 19)]
                    if self.latch.rc_sample(self._rc, t):
                        self._handle_trip()

    # ---------- publishing ----------

    def _publish_tick(self):
        now = self.get_clock().now().to_msg()
        with self._lock:
            pose, status, rc = self._pose, self._status, self._rc
            if self.latch.tick(time.monotonic()):
                self._handle_trip()
        if pose:
            m = LatLonHead()
            m.header.stamp = now
            m.latitude, m.longitude, m.heading = pose
            self.pose_pub.publish(m)
        if status:
            m = FcuStatus()
            m.header.stamp = now
            m.mode, m.armed, m.system_status = status
            self.status_pub.publish(m)
        if rc:
            m = RcChannels()
            m.header.stamp = now
            m.channels = rc
            self.rc_pub.publish(m)

    def _publish_drop_state(self):
        self.drop_pub.publish(Bool(data=not self.latch.allowed))

    # ---------- override TX (the enforcement point) ----------

    def _override_cb(self, msg: RcChannels):
        if not self.latch.allowed:
            return                   # dropped/startup: overrides die here
        self._send_override(list(msg.channels[:8]))

    def _send_override(self, ch8):
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system, self.conn.target_component, *ch8)

    # ---------- GUIDED setpoint TX (sanctioned, latch-gated) ----------

    def _guided_cb(self, msg: GuidedSetpoint):
        if not self.latch.allowed:
            return                   # dropped/startup: setpoints die here too
        time_boot_ms = int(time.monotonic() * 1000.0) & 0xFFFFFFFF
        self.conn.mav.set_position_target_global_int_send(
            time_boot_ms,
            self.conn.target_system, self.conn.target_component,
            self._mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            POSITION_ONLY_TYPE_MASK,
            int(msg.latitude * 1e7), int(msg.longitude * 1e7),
            0, 0, 0, 0, 0, 0, 0, 0, 0)

    # ---------- force-disarm TX (safety watchdog, latch-INDEPENDENT) ----------

    def _force_disarm_cb(self, msg: Bool):
        if not msg.data:
            return
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            self._mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0,                        # param1: 0 = disarm
            FORCE_DISARM_MAGIC,       # param2: force even while moving
            0, 0, 0, 0, 0)
        self.get_logger().warn(
            "force-disarm forwarded (rc_heartbeat_watchdog)",
            throttle_duration_sec=1.0)

    def _handle_trip(self):
        # called with latch already DROPPED; release every channel to the pilot
        self.get_logger().error(
            f"AUTONOMY DROP: {self.latch.trip_reason} — releasing RC overrides")
        for _ in range(RELEASE_FRAMES):
            self._send_override([0] * 8)
        self._publish_drop_state()

    # ---------- keep-out fence path ----------

    def _origin_cb(self, msg: LatLonHead):
        self._origin = (msg.latitude, msg.longitude)

    def _keepouts_cb(self, msg: DetectionArray):
        if msg.frame != "world":
            self.get_logger().warn(f"keepouts ignored: frame={msg.frame!r}")
            return
        with self._zones_lock:
            for d in msg.detections:
                if d.radius <= 0:
                    if self._zones.pop(d.label, None) is not None:
                        self.get_logger().info(f"All Clear: zone {d.label}")
                else:
                    self._zones[d.label] = (float(d.x), float(d.y), float(d.radius))
        self._fence_dirty.set()

    def _fence_worker(self):
        """Single-flight uploader: coalesces bursts, always uploads the latest
        zone set, verifies by readback, publishes /crsd/fence_state."""
        while not self._stop.is_set():
            if not self._fence_dirty.wait(timeout=0.5):
                continue
            self._fence_dirty.clear()
            if self._origin is None:
                self.get_logger().error(
                    "keep-outs received but no world origin yet — fence NOT "
                    "uploaded (will retry)")
                self._fence_dirty.set()
                time.sleep(1.0)
                continue
            with self._zones_lock:
                zones = [(zid, x, y, r) for zid, (x, y, r) in self._zones.items()]
            items = items_from_keepouts(zones, self._origin)
            state = {"zones": sorted(z[0] for z in zones),
                     "verified": False, "error": None, "t": time.time()}
            try:
                while not self._mission_q.empty():   # drain stale dialog msgs
                    self._mission_q.get_nowait()
                transport = MavFenceTransport(self.conn, self._mission_q,
                                              self._mavutil.mavlink)
                FenceProtocol(transport).upload_and_verify(items)
                state["verified"] = True
                self.get_logger().info(f"fence verified: {state['zones']}")
            except FenceError as e:
                state["error"] = str(e)
                self.get_logger().error(f"FENCE UPLOAD FAILED: {e} — ArduRover "
                                        "is NOT enforcing the keep-out set")
                self._fence_dirty.set()              # retry
                time.sleep(2.0)
            self.fence_pub.publish(String(data=json.dumps(state)))

    # ---------- reset service ----------

    def _reset_cb(self, request, response):
        ok, reason = self.latch.reset(time.monotonic())
        response.success = ok
        response.message = reason
        (self.get_logger().warn if ok else self.get_logger().error)(
            f"autonomy-drop reset: {reason}")
        self._publish_drop_state()
        return response

    # ---------- teardown ----------

    def destroy_node(self):
        self._stop.set()
        self._rx_thread.join(timeout=2.0)
        self._fence_thread.join(timeout=2.0)
        try:
            self.conn.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    run_node(TelemetryBridge, args=args)


if __name__ == "__main__":
    main()
