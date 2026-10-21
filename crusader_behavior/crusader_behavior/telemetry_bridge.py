"""telemetry_bridge: the only ROS node that talks to the autopilot (through MAVProxy's UDP rebroadcast).

RX: /crsd/pose, /crsd/attitude, /crsd/fcu_status, /crsd/rc_channels, /crsd/autonomy_drop,
    /crsd/pump_state, /crsd/fcu_health. A stream is republished only while it is fresh,
    stamped at receipt.
TX: /crsd/force_disarm (never gated), /crsd/guided_setpoint and /crsd/guided_heading_speed
    (autonomous modes only; heading+speed also needs the drop latch), /crsd/rc_override
    (override_core), /crsd/pump_cmd (pump_core), /crsd/set_mode, crsd/obstacle_distance.
"""
import os
import threading
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from crusader_msgs.msg import (Attitude, FcuHealth, FcuStatus, GuidedHeadingSpeed,
                               GuidedSetpoint, LatLonHead, ObstacleDistance, PumpCommand,
                               PumpState, RcChannels)

from crusader_common import geo
from crusader_common.node_main import declare_params, run_node
from crusader_common.stream_cache import StreamCache

from crusader_behavior import fcu_health, guided_hs_core, override_core, pump_core
from crusader_behavior.drop_latch import DropLatch

# MAVLink 2 before pymavlink is first imported: the v1 dialect has no OBSTACLE_DISTANCE.
os.environ["MAVLINK20"] = "1"

PARAMS = {
    "mav_endpoint": "udp:127.0.0.1:14551",
    "fcu_sysid": 2,
    "autonomous_modes": ["GUIDED", "AUTO", "LOITER", "RTL"],
    "mode_timeout_s": 1.0,
    "drop_channel": 9,
    "drop_threshold": 1700,
    "drop_invert": False,
    "rc_stale_timeout": 1.0,
    "stream_timeout_s": 1.0,
    "pump_servo_channel": 0,
    "pump_rc_channel": 10,
    "pump_on_pwm": 2000,
    "pump_off_pwm": 1000,
    "pump_max_burst_s": 1.0,
    "pump_min_gap_s": 1.0,
    "pump_allow_disarmed": False,
    "hs_max_speed_mps": 0.4,
    "hs_wp_speed_mps": 1.0,
    "hs_deadman_s": 0.5,
    "override_channels": [1, 3, 4],
    "override_max_us": 150,
    "override_deadman_s": 0.5,
    "override_modes": ["MANUAL"],
    "estop_channel": 7,
    "estop_threshold": 1200,
}

REQUIRED_SENDERS = ("rc_channels_override_send", "set_position_target_global_int_send",
                    "obstacle_distance_send", "command_long_send", "set_attitude_target_send")
RELEASE_FRAMES = 5
PUB_RATE_HZ = 20.0
HEALTH_PERIOD_S = 1.0
FORCE_DISARM_MAGIC = 21196                 # param2 of ARM_DISARM: disarm even while moving
POSITION_ONLY_TYPE_MASK = 0b110111111100   # SET_POSITION_TARGET_GLOBAL_INT, position only
OBSTACLE_SECTORS = 72
AUTOPILOT_COMPONENT = 1


def guarded(fn):
    """Callbacks share the executor with force-disarm: log a failure, never raise."""
    def wrapper(self, *args):
        try:
            return fn(self, *args)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"{fn.__name__} failed: {type(e).__name__}: {e}",
                                    throttle_duration_sec=2.0)
    return wrapper


class TelemetryBridge(Node):

    def __init__(self):
        super().__init__("telemetry_bridge")
        p = declare_params(self, PARAMS)

        endpoint = p["mav_endpoint"]
        if not endpoint.startswith(("udp", "tcp")):
            raise ValueError(f"mav_endpoint {endpoint!r} is not udp/tcp: MAVProxy owns the serial port")
        self.fcu_sysid = p["fcu_sysid"]
        self.estop = (p["estop_channel"], p["estop_threshold"])
        self.latch = DropLatch(channel=p["drop_channel"], threshold=p["drop_threshold"],
                               invert=p["drop_invert"], stale_timeout=p["rc_stale_timeout"])
        # The flight mode is the autonomy interlock: anything else means the pilot is flying.
        self.autonomous_modes = {str(m).upper() for m in p["autonomous_modes"]}

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(LatLonHead, "/crsd/pose", 10)
        self.att_pub = self.create_publisher(Attitude, "/crsd/attitude", 10)
        self.status_pub = self.create_publisher(FcuStatus, "/crsd/fcu_status", 10)
        self.rc_pub = self.create_publisher(RcChannels, "/crsd/rc_channels", 10)
        self.drop_pub = self.create_publisher(Bool, "/crsd/autonomy_drop", latched)
        self.pump_pub = self.create_publisher(PumpState, "/crsd/pump_state", 10)
        self.health_pub = self.create_publisher(FcuHealth, "/crsd/fcu_health", 10)

        self.create_subscription(RcChannels, "/crsd/rc_override", self._override_cb, 10)
        self.create_subscription(Bool, "/crsd/force_disarm", self._force_disarm_cb, 10)
        self.create_subscription(GuidedSetpoint, "/crsd/guided_setpoint", self._guided_cb, 10)
        self.create_subscription(String, "/crsd/set_mode", self._set_mode_cb, 10)
        # Not mode-gated: obstacle data feeds the autopilot's own avoidance in every mode.
        self.create_subscription(ObstacleDistance, "crsd/obstacle_distance", self._obstacle_cb, 10)
        self.create_service(Trigger, "/crsd/autonomy_drop_reset", self._reset_cb)
        # A topic, not a service: waiting on COMMAND_ACK would block force-disarm.
        self.create_subscription(PumpCommand, "/crsd/pump_cmd", self._pump_cb, 10)
        self.create_subscription(GuidedHeadingSpeed, "/crsd/guided_heading_speed", self._hs_cb, 10)

        t_out = p["stream_timeout_s"]
        self._lock = threading.Lock()      # the caches, written by the RX thread
        self._tx = threading.Lock()        # pymavlink's sender is not thread-safe; always taken last
        self._pose = StreamCache(t_out)
        self._att = StreamCache(t_out)
        self._status = StreamCache(p["mode_timeout_s"])
        self._rc = StreamCache(t_out)
        self._servo = StreamCache(t_out)
        self._sys = StreamCache(t_out * 3)     # 1-2 Hz streams: three of their periods
        self._batt = StreamCache(t_out * 3)
        self._gps = StreamCache(t_out * 3)
        self._ekf = StreamCache(t_out * 3)
        self._statustext = ("", 0)
        self._next_health = 0.0

        self.pump = pump_core.PumpGate(pump_core.PumpParams.from_dict(p))
        self._hs_lock = threading.Lock()   # _handle_trip runs with _lock held; Lock is not re-entrant
        self.hs = guided_hs_core.HsGate(guided_hs_core.HsParams.from_dict(p))
        self.ov = override_core.OverrideGate(override_core.OverrideParams.from_dict(p))
        self._pump_seq = 0
        self._pump_result = pump_core.RESULT_NONE
        self._pump_reason = ""
        self._pump_ack_until = 0.0

        from pymavlink import mavutil
        self._mavutil = mavutil
        self.conn = mavutil.mavlink_connection(endpoint)
        # Pinned: pymavlink would otherwise lock onto the first vehicle heartbeat it hears,
        # and Ekko (sysid 1) shares the radio mesh.
        self.conn.target_system = self.fcu_sysid
        missing = [n for n in REQUIRED_SENDERS if not hasattr(self.conn.mav, n)]
        if missing:
            raise RuntimeError(f"pymavlink dialect {mavutil.mavlink.__name__} cannot send "
                               f"{', '.join(missing)}: MAVLINK20 was not set before the import")

        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self.create_timer(1.0 / PUB_RATE_HZ, self._publish_tick)
        self._publish_drop_state()
        self.get_logger().info(
            f"autopilot sysid {self.fcu_sysid} on {endpoint}; autonomous modes "
            f"{sorted(self.autonomous_modes)}; pump "
            f"{'SERVO%d' % self.pump.p.servo_channel if self.pump.p.servo_channel else 'none'}")

    # ---------- MAVLink TX ----------

    def _send(self, name, *args):
        with self._tx:
            getattr(self.conn.mav, name)(*args)

    def _command_long(self, command, params):
        self._send("command_long_send", self.fcu_sysid, AUTOPILOT_COMPONENT, command, 0, *params)

    def _send_override(self, ch8):
        self._send("rc_channels_override_send", self.fcu_sysid, AUTOPILOT_COMPONENT, *ch8)

    def _hs_send(self, fields):
        mask, q, thrust = fields
        self._send("set_attitude_target_send", int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
                   self.fcu_sysid, AUTOPILOT_COMPONENT, mask, q, 0.0, 0.0, 0.0, thrust)

    # ---------- MAVLink RX ----------

    def _rx_loop(self):
        waited = 0
        while not self._stop.is_set():
            msg = self.conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                waited += 1
                if waited % 10 == 0:
                    self.get_logger().warn(f"no MAVLink for {waited}s: is MAVProxy running?")
                continue
            waited = 0
            if msg.get_srcSystem() != self.fcu_sysid:
                continue
            self._ingest(msg)

    def _ingest(self, msg):
        t = time.monotonic()
        mtype = msg.get_type()
        stamp = self.get_clock().now().to_msg()     # at receipt, so a republish carries its age
        att = None
        with self._lock:
            if mtype == "GLOBAL_POSITION_INT":
                hdg = msg.hdg / 100.0 if msg.hdg != 65535 else float("nan")
                self._pose.set((msg.lat / 1e7, msg.lon / 1e7, hdg,
                                geo.ground_speed_mps(msg.vx, msg.vy)), t, stamp)
            elif mtype == "ATTITUDE":
                # The autopilot's own axes (rad, NED body); published on arrival, not resampled.
                att = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)
                self._att.set(att, t, stamp)
            elif mtype == "HEARTBEAT" and msg.get_srcComponent() == AUTOPILOT_COMPONENT:
                armed = bool(msg.base_mode & self._mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._status.set((self._mavutil.mode_string_v10(msg), armed, msg.system_status),
                                 t, stamp)
            elif mtype == "RC_CHANNELS":
                rc = [getattr(msg, f"chan{i}_raw", 0) or 0 for i in range(1, 19)]
                self._rc.set(rc, t, stamp)
                if self.latch.rc_sample(rc, t):
                    self._handle_trip()
            elif mtype == "SERVO_OUTPUT_RAW" and self.pump.p.servo_channel:
                pwm = pump_core.servo_raw(msg, self.pump.p.servo_channel)
                if pwm is not None:
                    self._servo.set(pwm, t, stamp)
            elif mtype == "SYS_STATUS":
                self._sys.set({"voltage_mv": msg.voltage_battery, "remaining_pct": msg.battery_remaining,
                               "present": msg.onboard_control_sensors_present,
                               "health": msg.onboard_control_sensors_health}, t)
            elif mtype == "BATTERY_STATUS" and msg.id == 0:
                self._batt.set({"voltage_mv": msg.voltages[0], "remaining_pct": msg.battery_remaining}, t)
            elif mtype == "GPS_RAW_INT":
                self._gps.set({"fix_type": msg.fix_type, "sats": msg.satellites_visible,
                               "eph": msg.eph}, t)
            elif mtype == "EKF_STATUS_REPORT":
                self._ekf.set(msg.flags, t)
            elif mtype == "COMMAND_ACK" and t < self._pump_ack_until and \
                    msg.command in (pump_core.MAV_CMD_DO_REPEAT_SERVO, pump_core.MAV_CMD_DO_SET_SERVO):
                # MAVProxy hands every output every ACK; one pump command is in flight at a time.
                accepted = msg.result == 0
                self._pump_result = pump_core.RESULT_ACCEPTED if accepted else pump_core.RESULT_REJECTED
                if not accepted:
                    self._pump_reason = f"autopilot rejected it (MAV_RESULT {msg.result})"
                self._pump_ack_until = 0.0
            elif mtype == "STATUSTEXT":
                text = str(msg.text)
                self._statustext = (text, msg.severity)
                if "ServoRelayEvent" in text:          # the autopilot refusing the pump output
                    self._pump_reason = text
                    self.get_logger().error(f"pump: autopilot says {text!r}")
        if att is not None:
            m = Attitude()
            m.header.stamp = stamp
            m.roll, m.pitch, m.yaw, m.rollspeed, m.pitchspeed, m.yawspeed = att
            self.att_pub.publish(m)

    # ---------- publishing ----------

    def _publish_tick(self):
        t = time.monotonic()
        with self._lock:
            pose, status, rc = self._pose.get(t), self._status.get(t), self._rc.get(t)
            stale = [name for name, c in (("pose", self._pose), ("attitude", self._att),
                                          ("fcu_status", self._status), ("rc_channels", self._rc))
                     if c.went_stale(t)]
            if self.latch.tick(t):
                self._handle_trip()
        for name in stale:
            self.get_logger().error(f"MAVLink stream {name!r} stale: not republishing; is MAVProxy up?")
        if pose is not None:
            m = LatLonHead()
            m.header.stamp = self._pose.stamp
            m.latitude, m.longitude, m.heading, m.ground_speed = pose
            self.pose_pub.publish(m)
        if status is not None:
            m = FcuStatus()
            m.header.stamp = self._status.stamp
            m.mode, m.armed, m.system_status = status
            self.status_pub.publish(m)
        if rc is not None:
            m = RcChannels()
            m.header.stamp = self._rc.stamp
            m.channels = rc
            self.rc_pub.publish(m)
        if t >= self._next_health:
            self._next_health = t + HEALTH_PERIOD_S
            self._publish_health(t, rc)
        try:
            self._pump_tick(t)
            with self._hs_lock:
                stop = self.hs.tick(t)
            if stop is not None:
                self._hs_send(stop)
                self.get_logger().warn("guided heading+speed: commands stopped, stop sent")
            mode = "UNKNOWN" if status is None else str(status[0])
            with self._hs_lock:
                rel = self.ov.tick(t, mode)
            if rel is not None:
                self._send_override(rel)
                self.get_logger().warn(f"rc override: quiet or mode {mode}: sticks released")
        except Exception as e:  # noqa: BLE001 - the pump must not take telemetry down
            self.get_logger().error(f"pump/motion tick failed: {e}", throttle_duration_sec=5.0)

    def _publish_health(self, t, rc):
        with self._lock:
            fields = fcu_health.build(self._sys.get(t), self._batt.get(t), self._gps.get(t),
                                      self._ekf.get(t), rc, *self.estop)
            text, severity = self._statustext
        m = FcuHealth()
        m.header.stamp = self.get_clock().now().to_msg()
        for name, value in fields.items():
            setattr(m, name, value)
        m.last_statustext = text
        m.last_statustext_severity = int(severity)
        self.health_pub.publish(m)

    def _publish_drop_state(self):
        self.drop_pub.publish(Bool(data=not self.latch.allowed))

    # ---------- the water pump (pump_core) ----------

    def _pump_inputs(self, t):
        with self._lock:
            rc, status, pwm = self._rc.get(t), self._status.get(t), self._servo.get(t)
        return pump_core.PumpInputs(now=t, rc=rc, armed=None if status is None else bool(status[1]),
                                    output_pwm=pwm)

    def _pump_off(self):
        self._command_long(pump_core.MAV_CMD_DO_SET_SERVO,
                           pump_core.set_servo_params(self.pump.p.servo_channel, self.pump.p.off_pwm))

    @guarded
    def _pump_cb(self, msg: PumpCommand):
        """A burst of duration_s, or OFF (duration <= 0)."""
        t = time.monotonic()
        p = self.pump.p
        self._pump_seq = int(msg.seq)
        source = msg.source or "?"
        duration = float(msg.duration_s)
        if p.servo_channel <= 0:
            self._pump_result, self._pump_reason = pump_core.RESULT_REFUSED, "no pump path"
            return
        if duration <= 0.0:
            self._pump_off()
            self._pump_result, self._pump_reason = pump_core.RESULT_SENT, f"OFF ({source})"
            self._pump_ack_until = t + 1.5
            return
        ok, why = self.pump.check_burst(duration, self._pump_inputs(t))
        if not ok:
            self._pump_result, self._pump_reason = pump_core.RESULT_REFUSED, why
            self.get_logger().warn(f"pump burst from {source} refused: {why}", throttle_duration_sec=1.0)
            return
        self.pump.start_burst(duration, t)
        self._command_long(pump_core.MAV_CMD_DO_REPEAT_SERVO,
                           pump_core.repeat_servo_params(p.servo_channel, p.on_pwm, duration))
        self._pump_result = pump_core.RESULT_SENT
        self._pump_reason = f"burst {duration:.2f} s ({source})"
        self._pump_ack_until = t + 1.5
        self.get_logger().info(f"pump: {self._pump_reason}")

    def _pump_tick(self, t):
        p = self.pump.p
        if p.servo_channel <= 0:
            return
        inp = self._pump_inputs(t)
        why = self.pump.watchdog(inp)
        if why:
            self._pump_off()
            self.get_logger().error(f"pump watchdog: {why}: OFF sent")
            self._pump_reason = f"watchdog: {why}"
        m = PumpState()
        if self._servo.stamp is not None:
            m.header.stamp = self._servo.stamp
        m.enabled = self.pump.enabled
        m.output_fresh = inp.output_pwm is not None
        m.output_pwm = int(inp.output_pwm or 0)
        m.on = p.is_on(inp.output_pwm)
        m.pilot_pwm = 0 if inp.rc is None else pump_core.rc_value(inp.rc, p.rc_channel)
        m.last_seq = self._pump_seq
        m.last_result = self._pump_result
        m.last_reason = self.pump.latched or self._pump_reason
        self.pump_pub.publish(m)

    # ---------- motion TX ----------

    def _mode_and_latch(self):
        with self._lock:
            status = self._status.get(time.monotonic())
            allowed = self.latch.allowed
        return ("UNKNOWN" if status is None else str(status[0])), allowed

    @guarded
    def _hs_cb(self, msg: GuidedHeadingSpeed):
        """Heading + signed speed: GUIDED only, and only while the drop latch allows it."""
        mode, allowed = self._mode_and_latch()
        with self._hs_lock:
            ok, why, fields = self.hs.on_command(time.monotonic(), float(msg.heading_deg),
                                                 float(msg.speed_mps), mode, allowed)
        if not ok:
            self.get_logger().warning(f"guided heading+speed dropped: {why}", throttle_duration_sec=5.0)
            return
        self._hs_send(fields)
        if why != "ok":
            self.get_logger().warn(f"guided heading+speed: {why}", throttle_duration_sec=2.0)

    @guarded
    def _override_cb(self, msg: RcChannels):
        """Software on the sticks: override_modes only, override_channels only, clamped."""
        mode, allowed = self._mode_and_latch()
        with self._hs_lock:
            ok, why, out = self.ov.on_command(time.monotonic(), list(msg.channels), mode, allowed)
        if out is not None:
            self._send_override(out)
        if not ok:
            self.get_logger().warning(f"rc override dropped: {why}", throttle_duration_sec=5.0)
        elif why not in ("ok", "release"):
            self.get_logger().warn(f"rc override: {why}", throttle_duration_sec=2.0)

    @guarded
    def _guided_cb(self, msg: GuidedSetpoint):
        """A position setpoint, only in an autonomous mode. Not latch-gated: the mode is the
        interlock, and ArduPilot ignores position targets outside its guided modes anyway."""
        mode, _ = self._mode_and_latch()
        if mode.upper() not in self.autonomous_modes:
            self.get_logger().warning(f"guided setpoint dropped: mode {mode} is not autonomous",
                                      throttle_duration_sec=5.0)
            return
        bad = guided_hs_core.setpoint_error(msg.latitude, msg.longitude)
        if bad:
            self.get_logger().error(f"guided setpoint refused: {bad}", throttle_duration_sec=2.0)
            return
        self._send("set_position_target_global_int_send", int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
                   self.fcu_sysid, AUTOPILOT_COMPONENT,
                   self._mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, POSITION_ONLY_TYPE_MASK,
                   int(round(msg.latitude * 1e7)), int(round(msg.longitude * 1e7)),
                   0, 0, 0, 0, 0, 0, 0, 0, 0)

    @guarded
    def _obstacle_cb(self, msg: ObstacleDistance):
        """OBSTACLE_DISTANCE for the autopilot's proximity (needs PRX1_TYPE=2)."""
        distances = list(msg.distances)
        if len(distances) != OBSTACLE_SECTORS:
            self.get_logger().error(f"obstacle_distance has {len(distances)} sectors, not "
                                    f"{OBSTACLE_SECTORS}: dropped", throttle_duration_sec=5.0)
            return
        inc = float(msg.increment_deg)
        # Both increment fields: ArduPilot has read the integer one when the float was meant.
        self._send("obstacle_distance_send", int(time.time() * 1e6),
                   self._mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, distances,
                   int(round(inc)) & 0xFF, int(msg.min_distance_cm), int(msg.max_distance_cm),
                   inc, float(msg.angle_offset_deg), self._mavutil.mavlink.MAV_FRAME_BODY_FRD)

    @guarded
    def _set_mode_cb(self, msg: String):
        """Not gated: this is how a mission enters GUIDED. The pilot's SC switch outranks it."""
        name = str(msg.data).strip().upper()
        if not name:
            return
        mapping = self.conn.mode_mapping() or {}
        if name not in mapping:
            self.get_logger().error(f"set_mode {name!r} refused: not a mode this vehicle offers "
                                    f"({', '.join(sorted(mapping)) or 'mode map not yet known'})")
            return
        self.get_logger().info(f"set_mode -> {name}")
        with self._tx:
            self.conn.set_mode(name)

    @guarded
    def _force_disarm_cb(self, msg: Bool):
        if not msg.data:
            return
        self._command_long(self._mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                           (0, FORCE_DISARM_MAGIC, 0, 0, 0, 0, 0))
        self.get_logger().warn("force-disarm forwarded (rc_watchdog)", throttle_duration_sec=1.0)

    def _handle_trip(self):
        """The drop latch tripped (called with _lock held): release the sticks, stop our motion."""
        self.get_logger().error(f"AUTONOMY DROP: {self.latch.trip_reason}: releasing RC overrides")
        for _ in range(RELEASE_FRAMES):
            self._send_override([0] * 8)
        with self._hs_lock:
            self.ov.on_trip()
            stop = self.hs.on_trip()
        if stop is not None:
            self._hs_send(stop)
        self._publish_drop_state()

    def _reset_cb(self, request, response):
        # One severity per call site: rclpy refuses a call site that switches between them.
        try:
            ok, reason = self.latch.reset(time.monotonic())
            response.success, response.message = ok, reason
            if ok:
                self.get_logger().warn(f"autonomy-drop reset: {reason}")
            else:
                self.get_logger().error(f"autonomy-drop reset refused: {reason}")
            self._publish_drop_state()
        except Exception as e:  # noqa: BLE001
            response.success, response.message = False, f"reset failed: {e}"
        return response

    def destroy_node(self):
        if self.pump.p.servo_channel > 0:
            try:
                self._pump_off()       # the autopilot's own timer ends a burst anyway
            except Exception:  # noqa: BLE001
                pass
        self._stop.set()
        self._rx_thread.join(timeout=2.0)
        try:
            self.conn.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    run_node(TelemetryBridge, args=args)


if __name__ == "__main__":
    main()
