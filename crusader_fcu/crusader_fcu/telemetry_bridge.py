"""telemetry_bridge — the single ROS-side gateway to MAVProxy's rebroadcast.

Three jobs, deliberately fused into one node:

1. RX: consume MAVProxy's rebroadcast (pymavlink over UDP — NEVER a serial device;
   the Pixhawk has exactly one owner and it is MAVProxy) and republish as topics:
     /crsd/pose          interfaces/LatLonHead   (GLOBAL_POSITION_INT)
     /crsd/attitude      interfaces/Attitude     (ATTITUDE, on arrival)
     /crsd/fcu_status    interfaces/FcuStatus    (HEARTBEAT)
     /crsd/rc_channels   interfaces/RcChannels   (RC_CHANNELS)
     /crsd/autonomy_drop std_msgs/Bool           (latched, TRANSIENT_LOCAL)
   Other nodes subscribe to these topics instead of opening their own MAVLink
   connection — this node existing is what keeps the "no second consumer racing
   the ROS graph" rule enforceable.

2. TX (safety): the ONLY sanctioned force-disarm path. rc_heartbeat_watchdog
   publishes std_msgs/Bool on /crsd/force_disarm on RC-transmitter link loss;
   this node forwards it as MAV_CMD_COMPONENT_ARM_DISARM (force magic). It is
   NOT gated by the autonomy-drop latch — a force-disarm must fire even
   (especially) when the latch has already tripped.

3. TX (autonomy): the ONLY sanctioned path for RC overrides. Nodes publish
   interfaces/RcChannels on /crsd/rc_override; this node forwards them to the
   autopilot — UNLESS the autonomy-drop latch (crusader_common.drop_latch) has
   tripped, in which case it sends release frames (all-zero override) and drops
   every subsequent override until the operator resets via the
   /crsd/autonomy_drop_reset service (std_srvs/Trigger). Because misbehaving
   nodes have no MAVLink connection of their own, a tripped latch cannot be
   bypassed from the ROS graph.

   NOTE: this repo currently ships NO publisher on /crsd/rc_override — the
   override path and its latch are the Gate G1 mechanism, kept here so the
   enforcement point exists, but nothing exercises them yet. G1 sign-off needs a
   bench node that drives this topic (docs/G1_bench_procedure.md).

4. TX (payload): the ONLY sanctioned path to the water pump. Nodes publish
   crusader_msgs/PumpCommand on /crsd/pump_cmd; this node checks it against
   crusader_fcu.pump_core (pump path configured, RC fresh, SB not in e-stop, the
   pilot's pump switch OFF, armed, output reading OFF, burst length and gap) and
   sends ONE-cycle MAV_CMD_DO_REPEAT_SERVO, so the autopilot itself ends the
   burst. A watchdog sends the pump OFF if it is on under e-stop or RC loss (the
   e-stop stops motors, not a pass-through output), and latches the path off if
   a burst does not end. State on /crsd/pump_state. Disabled unless
   pump_servo_channel is set; NOT mode-gated and NOT latch-gated (it moves water,
   not the boat), which is a team decision recorded in docs/G7_pump_bench.md.

The hardware e-stop (SB switch) remains below and independent of all of this.

Why attitude is its OWN topic and not three more fields on LatLonHead: ATTITUDE
and GLOBAL_POSITION_INT are separate MAVLink streams (EXTRA1 and POSITION) that
can die independently, and a consumer mapping a detection needs to know WHICH
one went quiet. Folding them together would make one stream's staleness silently
gate the other's — the exact frozen-value failure StreamCache exists to stop —
and LatLonHead is also used as a bare position carrier (goals, origins) where
attitude has no meaning. The cost is that a mapping consumer must check two
topics; that cost is the point.

And why attitude alone is published FROM THE RX THREAD rather than on the 20 Hz
tick: it is the one stream whose value is the instantaneous number, not the
latest known state. ATTITUDE is set to 30 Hz (SR0_EXTRA1) to match the camera,
and resampling 30 Hz onto a 20 Hz tick drops one frame in three and time-shifts
the rest by up to 50 ms — at a 2 rad/s roll that is ~6 degrees of attitude
error, which is most of what roll/pitch compensation was added to remove.
Publishing on arrival also makes the staleness rule below automatic for this
topic: there is no cached value to replay, so silence is silence for free. The
StreamCache is kept anyway, purely so _publish_tick still logs the stale edge.

Parameters:
  mav_endpoint     (str,  default udp:127.0.0.1:14551)  MAVProxy --out for ROS
  drop_channel     (int,  default 7)     RC channel of the autonomy-drop switch
  drop_threshold   (int,  default 1700)  us; >= trips (or <= if drop_invert)
  drop_invert      (bool, default False)
  rc_stale_timeout (float, default 1.0)  s without RC_CHANNELS -> trip
  stream_timeout_s (float, default 1.0)  s without a frame on a stream before
                                         that stream stops being republished

Staleness rule (safety-relevant): each RX stream is republished ONLY while it
is fresh, and its header carries the stamp captured at RECEIPT. Rebroadcasting
the last cached frame with a fresh stamp — as this node originally did —
makes a dead MAVProxy indistinguishable from a healthy one, which silently
disables rc_heartbeat_watchdog (both its RC-loss and its gateway-down paths).
Silence must stay silent.
"""
import os
import threading
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from crusader_msgs.msg import (Attitude, FcuStatus, GuidedSetpoint, LatLonHead,
                               ObstacleDistance, PumpCommand, PumpState, RcChannels)

from crusader_common import config as crsd_config
from crusader_common import geo
from crusader_common.drop_latch import DropLatch
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config
from crusader_common.stream_cache import StreamCache

from crusader_fcu import pump_core

# MAVLink 2 is REQUIRED, and must be selected before pymavlink is first
# imported anywhere in this process: mavutil binds its dialect module at
# import time, and tests only for the PRESENCE of this variable, not its
# value. The v1.0 dialect has no OBSTACLE_DISTANCE at all - msgid 330 is
# above v1's 255 ceiling - so on v1 the proximity TX path does not fail at
# startup. It fails on the FIRST crsd/obstacle_distance message and takes
# the whole FCU gateway down with it, which is how a LiDAR feature became a
# total telemetry and arming outage on 2026-09-05.
os.environ["MAVLINK20"] = "1"

# Every MAVLink sender this node uses, verified once against the dialect we
# actually got (see _require_senders). Keep in step with the *_send calls.
REQUIRED_SENDERS = (
    "rc_channels_override_send",
    "set_position_target_global_int_send",
    "obstacle_distance_send",
    "command_long_send",
)

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
    "stream_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                             description="s without a MAVLink frame before "
                                         "that stream stops being republished"),
    "autonomous_modes": dict(read_only=True,
                             description="flight modes in which this bridge will "
                                         "forward autonomous commands; anything "
                                         "else is the pilot flying"),
    "mode_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                           description="s without HEARTBEAT before the mode is "
                                       "treated as unknown (= not autonomous)"),
    # --- the water pump (pump_core) ---
    "pump_servo_channel": dict(read_only=True, lo=0, hi=16,
                               description="Pixhawk output driving the pump; "
                                           "0 = no pump path"),
    "pump_rc_channel": dict(read_only=True, lo=1, hi=18,
                            description="the pilot's pump switch"),
    "pump_on_pwm": dict(read_only=True, lo=800, hi=2200,
                        description="us that run the pump"),
    "pump_off_pwm": dict(read_only=True, lo=800, hi=2200,
                         description="us that stop it; MUST equal SERVOn_TRIM"),
    "pump_max_burst_s": dict(read_only=True, lo=0.05, hi=2.0,
                             description="longest burst accepted"),
    "pump_min_gap_s": dict(read_only=True, lo=0.0, hi=30.0,
                           description="s from one burst's end to the next"),
    "pump_allow_disarmed": dict(read_only=True,
                                description="bench only: bursts while disarmed"),
    "estop_channel": dict(read_only=True, lo=1, hi=18,
                          description="SB e-stop RC channel"),
    "estop_threshold": dict(read_only=True, lo=800, hi=2200,
                            description="us; below = e-stop (or RC lost)"),
}

RELEASE_FRAMES = 5          # all-zero override frames sent on trip
PUB_RATE_HZ = 20.0

# Magic value ArduPilot requires in param2 of MAV_CMD_COMPONENT_ARM_DISARM to
# force-disarm even while the vehicle is moving.
FORCE_DISARM_MAGIC = 21196

# ArduPilot's documented position-only SET_POSITION_TARGET_GLOBAL_INT mask —
# the EXACT value the pre-integration gate_navigator used and field-exercised.
# yaw on GuidedSetpoint is accepted but not commanded (left to ArduRover);
# commanding yaw means clearing the yaw-ignore bit and is a separate change.
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

        # THE AUTONOMY INTERLOCK IS THE FLIGHT MODE, not an RC channel.
        #
        # The pilot selects an autonomous mode on SC; anything else means they
        # are flying it. That gives two independent things which must BOTH agree
        # before the boat moves under our command:
        #   * we choose to send        — this gate
        #   * ArduPilot chooses to obey — SET_POSITION_TARGET_* is acted on only
        #                                 in GUIDED/AUTO/RTL and ignored elsewhere
        # so leaving the mode stops both at once, with no software in the path
        # and nothing for us to get wrong. It also needs no spare RC channel,
        # which matters because the drop_channel default (7) is the SB arm/e-stop
        # switch — arming drives it to ~1995 and trips the RC latch.
        #
        # Uppercased once here: mode_string_v10 returns e.g. "GUIDED", and a YAML
        # typo of "guided" silently meaning "never autonomous" is exactly the
        # class of failure this repo keeps getting bitten by.
        self.autonomous_modes = {str(m).upper() for m in p["autonomous_modes"]}
        self._mode_timeout_s = p["mode_timeout_s"]
        self.get_logger().info(
            f"autonomy interlock: modes {sorted(self.autonomous_modes)} "
            f"(mode staler than {self._mode_timeout_s:.1f}s counts as NOT autonomous)")

        latched_qos = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(LatLonHead, "/crsd/pose", 10)
        self.att_pub = self.create_publisher(Attitude, "/crsd/attitude", 10)
        self.status_pub = self.create_publisher(FcuStatus, "/crsd/fcu_status", 10)
        self.rc_pub = self.create_publisher(RcChannels, "/crsd/rc_channels", 10)
        self.drop_pub = self.create_publisher(Bool, "/crsd/autonomy_drop", latched_qos)
        self.pump_pub = self.create_publisher(PumpState, "/crsd/pump_state", 10)

        self.create_subscription(RcChannels, "/crsd/rc_override",
                                 self._override_cb, 10)
        # Sanctioned force-disarm TX — rc_heartbeat_watchdog publishes here on RC
        # link loss instead of opening its own MAVLink connection. Deliberately
        # NOT gated by the autonomy-drop latch: a force-disarm must fire even
        # (especially) when the latch has already tripped. This is the ONLY
        # sanctioned disarm path from the ROS graph.
        self.create_subscription(Bool, "/crsd/force_disarm",
                                 self._force_disarm_cb, 10)
        # Sanctioned GUIDED setpoint TX. Gated by the MODE interlock above, not
        # by the RC drop latch — see _guided_cb.
        self.create_subscription(GuidedSetpoint, "/crsd/guided_setpoint",
                                 self._guided_cb, 10)
        # Sanctioned mode-change TX. Fire-and-forget by design: there is no ack
        # to wait on that is worth more than reading the mode back off
        # /crsd/fcu_status, which is the autopilot's own answer rather than an
        # acknowledgement of our request.
        self.create_subscription(String, "/crsd/set_mode", self._set_mode_cb, 10)
        # Proximity TX. Deliberately NOT mode-gated: obstacle data is an input to
        # the autopilot's own avoidance, not a command from us. The pilot flying
        # in MANUAL benefits from ArduPilot knowing where things are just as much
        # as an autonomous run does, and withholding it in MANUAL would make the
        # boat's behaviour change with the mode switch for no good reason.
        self.create_subscription(ObstacleDistance, "crsd/obstacle_distance",
                                 self._obstacle_cb, 10)
        self.create_service(Trigger, "/crsd/autonomy_drop_reset", self._reset_cb)
        # Sanctioned pump TX (job 4 above). Checked by pump_core, timed by the
        # autopilot. A topic, not a service: a service waiting on COMMAND_ACK
        # would block this single-threaded executor, force-disarm included.
        self.create_subscription(PumpCommand, "/crsd/pump_cmd", self._pump_cb, 10)

        # Each stream is republished ONLY while it is fresh. Rebroadcasting the
        # last cached frame with a fresh stamp after MAVProxy dies makes a dead
        # gateway indistinguishable from a healthy one — it defeats
        # rc_heartbeat_watchdog's RC-loss AND gateway-down detection. See
        # stream_cache.py.
        self._lock = threading.Lock()
        t_out = p["stream_timeout_s"]
        self._pose = StreamCache(t_out)    # (lat, lon, heading_deg, speed_mps)
        self._att = StreamCache(t_out)     # (r, p, y, rspd, pspd, yspd) [rad, rad/s]
        self._status = StreamCache(t_out)  # (mode_str, armed, system_status)
        self._rc = StreamCache(t_out)      # list[int] 18
        self._servo = StreamCache(t_out)   # the pump output's PWM (SERVO_OUTPUT_RAW)
        self.pump = pump_core.PumpGate(pump_core.PumpParams.from_dict(p))
        self._pump_seq = 0
        self._pump_result = pump_core.RESULT_NONE
        self._pump_reason = ""
        self._pump_ack_until = 0.0         # an ACK before this is ours
        if self.pump.p.servo_channel:
            self.get_logger().info(
                f"pump path: SERVO{self.pump.p.servo_channel} "
                f"(pilot ch{self.pump.p.rc_channel}), on {self.pump.p.on_pwm} / "
                f"off {self.pump.p.off_pwm}, bursts <= {self.pump.p.max_burst_s:.2f} s"
                + (" -- DISARMED BURSTS ALLOWED (bench)" if self.pump.p.allow_disarmed
                   else ""))
        else:
            self.get_logger().info("pump path: none (pump_servo_channel 0)")

        from pymavlink import mavutil
        self._mavutil = mavutil
        self.conn = mavutil.mavlink_connection(endpoint)
        self._require_senders()
        self.get_logger().info(f"waiting for heartbeat on {endpoint} ...")

        self._stop = threading.Event()          # deterministic teardown
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.create_timer(1.0 / PUB_RATE_HZ, self._publish_tick)
        self._publish_drop_state()               # initial latched state (STARTUP=blocked)
        self.get_logger().info(
            f"autonomy-drop: ch{self.latch.channel} thr={self.latch.threshold} "
            f"invert={self.latch.invert} — overrides BLOCKED until safe RC seen")

    def _require_senders(self):
        """Abort at STARTUP if the dialect cannot send what this node sends.

        Raises:
            RuntimeError: if any name in REQUIRED_SENDERS is missing from the
                connection's MAVLink object.

        A missing sender means MAVLINK20 was not in effect when pymavlink was
        first imported - something imported it ahead of this module. Checked
        here rather than left to the call site because the call site is a
        subscriber callback: the failure would otherwise surface a second
        after a clean-looking start, as an AttributeError with no hint that
        the dialect is the cause.
        """
        missing = [n for n in REQUIRED_SENDERS
                   if not hasattr(self.conn.mav, n)]
        if missing:
            raise RuntimeError(
                f"pymavlink dialect is {self._mavutil.mavlink.__name__}, which "
                f"cannot send: {', '.join(missing)}. MAVLINK20 was not set "
                f"before pymavlink was first imported in this process.")

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
            # captured at RECEIPT, not at publish, so a republished frame
            # carries the age it actually has
            stamp = self.get_clock().now().to_msg()
            att_now = None
            with self._lock:
                if mtype == "GLOBAL_POSITION_INT":
                    hdg = msg.hdg / 100.0 if msg.hdg != 65535 else float("nan")
                    # vx/vy (cm/s NED) are already in this message — republish
                    # them as ground speed so consumers do not have to
                    # finite-difference position.
                    self._pose.set((msg.lat / 1e7, msg.lon / 1e7, hdg,
                                    geo.ground_speed_mps(msg.vx, msg.vy)),
                                   t, stamp)
                elif mtype == "ATTITUDE":
                    # Republished in the autopilot's own axes/units (rad, NED
                    # body) — see Attitude.msg. Converting here would put a
                    # frame convention in the gateway, where nothing can check
                    # it; crusader_common.geo owns that instead.
                    att_now = (msg.roll, msg.pitch, msg.yaw,
                               msg.rollspeed, msg.pitchspeed, msg.yawspeed)
                    # cached ONLY so _publish_tick can log the stale edge; the
                    # publish itself happens below, not on the tick
                    self._att.set(att_now, t, stamp)
                elif mtype == "HEARTBEAT" and msg.get_srcComponent() == 1:
                    mode = self._mavutil.mode_string_v10(msg)
                    armed = bool(msg.base_mode &
                                 self._mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self._status.set((mode, armed, msg.system_status), t, stamp)
                elif mtype == "RC_CHANNELS":
                    rc = [getattr(msg, f"chan{i}_raw", 0) or 0
                          for i in range(1, 19)]
                    self._rc.set(rc, t, stamp)
                    if self.latch.rc_sample(rc, t):
                        self._handle_trip()
                elif mtype == "SERVO_OUTPUT_RAW" and self.pump.p.servo_channel:
                    pwm = pump_core.servo_raw(msg, self.pump.p.servo_channel)
                    if pwm is not None:
                        self._servo.set(pwm, t, stamp)
                elif mtype == "COMMAND_ACK" and t < self._pump_ack_until and \
                        msg.command in (pump_core.MAV_CMD_DO_REPEAT_SERVO,
                                        pump_core.MAV_CMD_DO_SET_SERVO):
                    # MAVProxy hands every output every ACK, and pymavlink here
                    # shares sysid 255 with it, so "ours" means "a servo ACK
                    # inside the window after we sent one". pump_min_gap_s keeps
                    # it to one pump command in flight.
                    accepted = msg.result == 0      # MAV_RESULT_ACCEPTED
                    self._pump_result = (pump_core.RESULT_ACCEPTED if accepted
                                         else pump_core.RESULT_REJECTED)
                    if not accepted:
                        self._pump_reason = f"autopilot rejected it (MAV_RESULT {msg.result})"
                    self._pump_ack_until = 0.0
                elif mtype == "STATUSTEXT" and "ServoRelayEvent" in str(msg.text):
                    # ArduPilot's own words when it will not drive an output,
                    # e.g. "Channel 10 is already in use": the pump output's
                    # SERVOn_FUNCTION is not one it may override.
                    self._pump_reason = str(msg.text)
                    self.get_logger().error(f"pump: autopilot says {msg.text!r}")
            # Outside the lock: a publish must never be held up by, or hold up,
            # the RC path that force-disarm depends on.
            if att_now is not None:
                m = Attitude()
                m.header.stamp = stamp
                (m.roll, m.pitch, m.yaw,
                 m.rollspeed, m.pitchspeed, m.yawspeed) = att_now
                self.att_pub.publish(m)

    # ---------- publishing ----------

    def _publish_tick(self):
        t = time.monotonic()
        with self._lock:
            pose = self._pose.get(t)
            status = self._status.get(t)
            rc = self._rc.get(t)
            # one loud line per stream the moment it goes stale — a silent
            # gateway must be diagnosable from the log, and consumers that
            # judge health by arrival need the silence to be real
            stale = [name for name, c in (("pose", self._pose),
                                          ("attitude", self._att),
                                          ("fcu_status", self._status),
                                          ("rc_channels", self._rc))
                     if c.went_stale(t)]
            if self.latch.tick(t):
                self._handle_trip()
        for name in stale:
            self.get_logger().error(
                f"MAVLink stream {name!r} stale (> {self._pose.timeout_s:.1f}s) "
                "— NOT republishing; is MAVProxy still up?")
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
        try:
            self._pump_tick(t)
        except Exception as e:          # never let the pump take telemetry down
            self.get_logger().error(f"pump tick failed: {e}",
                                    throttle_duration_sec=5.0)

    # ---------- the water pump (sanctioned, pump_core-gated) ----------

    def _pump_inputs(self, t):
        with self._lock:
            rc = self._rc.get(t)
            status = self._status.get(t)
            pwm = self._servo.get(t)
        return pump_core.PumpInputs(now=t, rc=rc,
                                    armed=None if status is None else bool(status[1]),
                                    output_pwm=pwm)

    def _pump_send(self, params, command):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            command, 0, *params)

    def _pump_cb(self, msg: PumpCommand):
        """A burst, or OFF. Never raises: this runs on the executor that also
        carries force-disarm."""
        try:
            t = time.monotonic()
            p = self.pump.p
            self._pump_seq = int(msg.seq)
            duration = float(msg.duration_s)
            if duration <= 0.0:
                if p.servo_channel <= 0:
                    self._pump_result = pump_core.RESULT_REFUSED
                    self._pump_reason = "no pump path"
                    return
                self._pump_send(pump_core.set_servo_params(p.servo_channel, p.off_pwm),
                                pump_core.MAV_CMD_DO_SET_SERVO)
                self._pump_result = pump_core.RESULT_SENT
                self._pump_reason = f"OFF ({msg.source or '?'})"
                self._pump_ack_until = t + 1.5
                return
            ok, why = self.pump.check_burst(duration, self._pump_inputs(t))
            if not ok:
                self._pump_result = pump_core.RESULT_REFUSED
                self._pump_reason = why
                self.get_logger().warn(
                    f"pump burst from {msg.source or '?'} refused: {why}",
                    throttle_duration_sec=1.0)
                return
            self.pump.start_burst(duration, t)
            self._pump_send(pump_core.repeat_servo_params(p.servo_channel, p.on_pwm,
                                                          duration),
                            pump_core.MAV_CMD_DO_REPEAT_SERVO)
            self._pump_result = pump_core.RESULT_SENT
            self._pump_reason = f"burst {duration:.2f} s ({msg.source or '?'})"
            self._pump_ack_until = t + 1.5
            self.get_logger().info(f"pump: {self._pump_reason}")
        except Exception as e:
            self._pump_result = pump_core.RESULT_REFUSED
            self._pump_reason = f"error: {e}"
            self.get_logger().error(f"pump command failed: {e}")

    def _pump_tick(self, t):
        p = self.pump.p
        if p.servo_channel <= 0:
            return
        inp = self._pump_inputs(t)
        why = self.pump.watchdog(inp)
        if why:
            self._pump_send(pump_core.set_servo_params(p.servo_channel, p.off_pwm),
                            pump_core.MAV_CMD_DO_SET_SERVO)
            self.get_logger().error(f"pump watchdog: {why} -- OFF sent")
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

    # ---------- force-disarm TX (safety watchdog, latch-INDEPENDENT) ----------

    # ---------- the mode interlock ----------

    def _autonomy_allowed(self):
        """(allowed, reason) — may we send an autonomous command right now?

        Fails CLOSED on every uncertainty. An unknown mode is not autonomy: if
        HEARTBEAT has gone stale we do not know what the pilot has selected, and
        assuming the last known mode still holds is exactly the stale-value
        failure StreamCache exists to prevent.
        """
        status = self._status.get(time.monotonic())
        if status is None:
            return False, f"mode unknown (no HEARTBEAT for >{self._mode_timeout_s:.1f}s)"
        mode = str(status[0]).upper()
        if mode not in self.autonomous_modes:
            return False, f"mode {mode} is not autonomous"
        return True, mode

    # ---------- GUIDED setpoint TX (sanctioned, mode-gated) ----------

    def _guided_cb(self, msg: GuidedSetpoint):
        """Forward a position setpoint, but only while the pilot has given us the mode.

        Deliberately NOT gated by the RC drop latch. The latch keys off
        drop_channel, whose default (7) is the SB arm/e-stop switch — arming
        drives it to ~1995 and trips the latch, which would block autonomy at
        exactly the moment autonomy becomes possible. The mode is the honest
        interlock and ArduPilot enforces the same decision independently.
        """
        allowed, reason = self._autonomy_allowed()
        if not allowed:
            # Throttled: a mission publishing at 1-20 Hz into a non-autonomous
            # mode is the NORMAL state on the bench, not an error worth a line
            # per message.
            self.get_logger().warning(
                f"guided setpoint DROPPED — {reason}", throttle_duration_sec=5.0)
            return
        time_boot_ms = int(time.monotonic() * 1000.0) & 0xFFFFFFFF
        self.conn.mav.set_position_target_global_int_send(
            time_boot_ms,
            self.conn.target_system, self.conn.target_component,
            self._mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            POSITION_ONLY_TYPE_MASK,
            int(msg.latitude * 1e7), int(msg.longitude * 1e7),
            0, 0, 0, 0, 0, 0, 0, 0, 0)

    # ---------- proximity TX (sanctioned, ungated) ----------

    def _obstacle_cb(self, msg: ObstacleDistance):
        """Forward a sector map as MAVLink OBSTACLE_DISTANCE.

        Requires PRX1_TYPE=2 (MAVLink) on the autopilot or ArduPilot ignores it
        entirely and the proximity viewer stays empty — which looks exactly like
        this node being broken.

        Both `increment` (uint8 degrees) and `increment_f` (float) are set.
        ArduPilot has historically read the integer field when the float was
        intended, rendering obstacles at wrong bearings; setting both makes the
        message correct under either behaviour.
        """
        time_usec = int(time.time() * 1e6)
        inc = float(msg.increment_deg)
        self.conn.mav.obstacle_distance_send(
            time_usec,
            self._mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            list(msg.distances),
            int(round(inc)) & 0xFF,          # legacy integer field
            int(msg.min_distance_cm),
            int(msg.max_distance_cm),
            inc,                             # increment_f, the one to trust
            float(msg.angle_offset_deg),
            self._mavutil.mavlink.MAV_FRAME_BODY_FRD)

    # ---------- mode-change TX (sanctioned) ----------

    def _set_mode_cb(self, msg: String):
        """Ask the autopilot to change flight mode.

        NOT gated: this is how a mission ENTERS an autonomous mode, so gating it
        on already being in one is circular. It is also how a mission can put the
        boat somewhere safe. The pilot always outranks it — moving SC re-applies
        the switch position and takes the mode back.

        Refuses a mode this vehicle does not have rather than sending a number
        the autopilot will silently ignore.
        """
        name = str(msg.data).strip().upper()
        if not name:
            return
        try:
            mapping = self.conn.mode_mapping() or {}
        except Exception:                        # link not up yet
            mapping = {}
        if name not in mapping:
            self.get_logger().error(
                f"set_mode {name!r} refused — not a mode this vehicle offers "
                f"({', '.join(sorted(mapping)) or 'mode map not yet known'})")
            return
        self.get_logger().info(f"set_mode -> {name}")
        self.conn.set_mode(name)

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
        # Best effort: a bridge going down mid-burst still has the autopilot's
        # own timer ending it, but an explicit OFF costs nothing.
        if self.pump.p.servo_channel > 0:
            try:
                self._pump_send(pump_core.set_servo_params(self.pump.p.servo_channel,
                                                           self.pump.p.off_pwm),
                                pump_core.MAV_CMD_DO_SET_SERVO)
            except Exception:
                pass
        self._stop.set()
        self._rx_thread.join(timeout=2.0)
        try:
            self.conn.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    run_node(TelemetryBridge, args=args)


if __name__ == "__main__":
    main()
