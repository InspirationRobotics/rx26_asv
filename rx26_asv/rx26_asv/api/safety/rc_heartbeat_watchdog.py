"""rc_heartbeat_watchdog — force-disarm on RC-transmitter link loss.

Independent software failsafe that stops the boat when the RC transmitter link
dies, WITHOUT anyone pressing the physical SB kill switch. It complements, and
does not replace, ArduPilot's own FS_THR / FS_GCS failsafes and the hardware
e-stop — run all of them for defense in depth.

REWIRED for this repo (like `pixhawk_led_status_node`): the boat-repo original
opened its own `udpin:127.0.0.1:14552` MAVLink connection, making it a SECOND
consumer of the Pixhawk stream. Here it consumes `telemetry_bridge`'s republished
topics and routes its one command — a force-disarm — back through the bridge, so
it adds no MAVLink connection and cannot race the graph (single-gateway rule,
rx26_asv/README.md).

  in:  /crsd/rc_channels    (RcChannels) monitored-channel PWM = the RC heartbeat
       /crsd/fcu_status     (FcuStatus)  armed state
       /crsd/rc_link_health (Bool)       ArduPilot's RC-receiver failsafe verdict
  out: /crsd/force_disarm  (Bool)        request bridge to force-disarm (UN-gated
                                         by the autonomy-drop latch on purpose)
       /crsd/kill_active   (Bool)        watchdog kill state, for observability

How link health is judged — TWO independent signals, OR'd:

  1. `rc_channel` PWM below `min_valid_pwm` on /crsd/rc_channels.
  2. ArduPilot's SYS_STATUS RC-receiver health bit, republished by
     telemetry_bridge on /crsd/rc_link_health.

Signal 2 is not optional belt-and-braces; signal 1 alone is BROKEN on this boat.
The original code (and the comment that used to sit here) asserted "ArduPilot
reports 0 for that channel when the transmitter is out of range or powered off",
and dropped the SYS_STATUS path the boat-repo original had. Measured on Crusader
2026-08-02: with the ELRS receiver's failsafe holding last position, ch7 read
1995 continuously through a full transmitter power-down. `monitored_valid` never
went false, `_link_lost` never fired, and this watchdog did not force-disarm a
boat with no pilot. Whether a receiver zeroes its outputs on link loss is a
RECEIVER CONFIG choice (ELRS "No Pulses" vs "Last Position"), so it can never be
the sole basis for a safety interlock. The SYS_STATUS bit is ArduPilot's own
failsafe state and is independent of what the receiver puts on the wire.

Signal 2 is strictly ADDITIVE: unknown (autopilot does not advertise the bit) and
stale both fall back to the PWM check, so it can only ever add a way to notice RC
loss, never mask one. Set the receiver to "No Pulses" as well — defense in depth
means both signals working, not one covering for the other.

If nothing confirms the link for `heartbeat_timeout` seconds while the vehicle is
ARMED, the watchdog requests a force-disarm (motors off, same as the kill button).
With `enable_latch` True it also LATCHES: the vehicle is held disarmed until the
operator clears the latch from the transmitter itself — a deliberate LOW->HIGH
toggle of `reset_channel` (e.g. cycling the arm/e-stop switch off and back on).
The toggle only counts once a genuine LOW is seen with the RC link alive, so
simply reconnecting with the switch already HIGH does not clear the latch. With
`enable_latch` False the boat is disarmed while the link is down and recovers on
its own when it returns.

Relationship to telemetry_bridge's autonomy-drop latch: that latch trips on RC
staleness too, but only RELEASES RC overrides (returns control to the pilot). This
watchdog is the layer that actually force-disarms on RC loss — complementary, not
redundant. Both share RC channel 7 (the SB switch) as a read-only input, which is
fine: they are independent consumers of the same republished RC_CHANNELS.
"""
import time

from rclpy.node import Node
from std_msgs.msg import Bool

from interfaces.msg import FcuStatus, RcChannels

from rx26_asv.api.common import config as crsd_config
from rx26_asv.api.common.node_main import run_node
from rx26_asv.api.common.param_utils import declare_from_config
from rx26_asv.api.safety.rc_heartbeat_core import RcHeartbeatCore, WatchdogConfig

# All watchdog params are SAFETY CONFIG -> read_only: `ros2 param set` is
# rejected; the change path is config/crusader_params.yaml + node restart.
PARAM_SPEC = {
    "heartbeat_timeout": dict(read_only=True, lo=0.2, hi=10.0,
                              description="s of stale monitored-channel PWM before RC link declared lost"),
    "rc_channel": dict(read_only=True, lo=1, hi=18,
                       description="RC channel whose valid PWM is the link heartbeat"),
    "min_valid_pwm": dict(read_only=True, lo=800, hi=2200,
                          description="us; below this = no pulses (RC loss reads 0)"),
    "require_armed": dict(read_only=True,
                          description="only force-disarm when the vehicle is armed"),
    "link_timeout": dict(read_only=True, lo=0.2, hi=10.0,
                         description="s without any telemetry_bridge topic before the gateway is deemed down"),
    "enable_latch": dict(read_only=True,
                         description="hold the kill until an RC reset toggle clears it"),
    "reset_channel": dict(read_only=True, lo=1, hi=18,
                          description="RC channel whose low->high toggle clears the latch"),
    "reset_low_pwm": dict(read_only=True, lo=800, hi=2200,
                          description="us; reset switch 'low' below this"),
    "reset_high_pwm": dict(read_only=True, lo=800, hi=2200,
                           description="us; reset switch 'high' above this"),
}

TICK_HZ = 10.0            # watchdog evaluation rate
DISARM_MIN_GAP_S = 0.4    # min seconds between force-disarm requests


class RCHeartbeatWatchdog(Node):

    def __init__(self):
        super().__init__("rc_heartbeat_watchdog")
        p = declare_from_config(
            self, crsd_config.node_params("rc_heartbeat_watchdog"), PARAM_SPEC)
        self.rc_channel = p["rc_channel"]
        self.reset_channel = p["reset_channel"]
        self.min_valid_pwm = p["min_valid_pwm"]

        # All safety logic lives in the ROS-free core (unit-tested in
        # tests/test_rc_heartbeat_core.py); this node only marshals topics.
        self.core = RcHeartbeatCore(
            WatchdogConfig(
                heartbeat_timeout=p["heartbeat_timeout"],
                min_valid_pwm=p["min_valid_pwm"],
                require_armed=p["require_armed"],
                link_timeout=p["link_timeout"],
                enable_latch=p["enable_latch"],
                reset_low=p["reset_low_pwm"],
                reset_high=p["reset_high_pwm"],
            ),
            now=time.monotonic())

        self.disarm_pub = self.create_publisher(Bool, "/crsd/force_disarm", 10)
        self.kill_pub = self.create_publisher(Bool, "/crsd/kill_active", 10)
        self.create_subscription(RcChannels, "/crsd/rc_channels", self._rc_cb, 10)
        self.create_subscription(FcuStatus, "/crsd/fcu_status", self._fcu_cb, 10)
        self.create_subscription(Bool, "/crsd/rc_link_health", self._health_cb, 10)

        self.rc_link_healthy = None    # None = autopilot does not report the bit
        self.health_t = 0.0
        self.last_disarm_req = 0.0
        self.last_kill_published = None
        self.last_status_log = 0.0

        self.create_timer(1.0 / TICK_HZ, self.tick)
        self.get_logger().info(
            f"RC watchdog active: ch{self.rc_channel} heartbeat_timeout="
            f"{self.core.cfg.heartbeat_timeout:.1f}s latch={self.core.cfg.enable_latch}")

    # ---------------- topic intake ----------------
    def _health_cb(self, msg: Bool):
        self.rc_link_healthy = msg.data
        self.health_t = time.monotonic()

    def _rc_health_ok(self, now: float) -> bool:
        """The SYS_STATUS RC-receiver verdict, or True when we do not have one.

        False ONLY on a fresh, explicit unhealthy report. Unknown (the autopilot
        never advertised the bit — telemetry_bridge warns loudly about that) and
        stale both return True so the PWM check remains the decider. This signal
        may only ever ADD a detection, never suppress one.
        """
        if self.rc_link_healthy is None:
            return True
        if (now - self.health_t) > self.core.cfg.link_timeout:
            return True
        return self.rc_link_healthy

    def _rc_cb(self, msg: RcChannels):
        now = time.monotonic()
        pwm_ok = (len(msg.channels) >= self.rc_channel
                  and msg.channels[self.rc_channel - 1] >= self.min_valid_pwm)
        # A receiver whose failsafe holds last position keeps pwm_ok True through
        # a dead transmitter — see the module docstring. The autopilot's own
        # RC-receiver health bit is the signal that does not lie about that.
        health_ok = self._rc_health_ok(now)
        if pwm_ok and not health_ok:
            self.get_logger().error(
                f"RC link declared LOST by ArduPilot (SYS_STATUS RC-receiver "
                f"unhealthy) while ch{self.rc_channel} still reads valid PWM — "
                "receiver failsafe is holding last position; set it to No Pulses",
                throttle_duration_sec=2.0)
        monitored_valid = pwm_ok and health_ok
        reset_pwm = (msg.channels[self.reset_channel - 1]
                     if len(msg.channels) >= self.reset_channel else 0)
        self.core.note_rc(now, monitored_valid, reset_pwm)

    def _fcu_cb(self, msg: FcuStatus):
        self.core.note_fcu(time.monotonic(), msg.armed)

    # ---------------- watchdog loop ----------------
    def tick(self):
        now = time.monotonic()
        d = self.core.tick(now)

        if d.latch_cleared:
            self.get_logger().info(
                "Kill latch cleared by RC reset toggle on channel "
                f"{self.reset_channel}. Re-arm from the transmitter/GCS.")
        if d.gateway_down_in_loss:
            self.get_logger().error(
                "RC link lost AND no telemetry from telemetry_bridge - cannot "
                "force-disarm via the gateway; relying on ArduPilot failsafe",
                throttle_duration_sec=2.0)
        if d.engaged:
            self.get_logger().error(
                f"RC heartbeat lost for >{self.core.cfg.heartbeat_timeout:.1f}s "
                "-> FORCE-DISARMING (latched). Toggle RC channel "
                f"{self.reset_channel} low->high to clear.")
        elif d.disarm_request and not self.core.cfg.enable_latch:
            self.get_logger().error(
                "RC heartbeat lost -> FORCE-DISARMING "
                "(auto-recovers when the link returns)",
                throttle_duration_sec=1.0)

        if d.disarm_request:
            self._request_disarm(now)
        self._publish_kill(d.kill_active)
        self._status_log(now, self.core._link_lost(now), self.core._bridge_ok(now))

    def _request_disarm(self, now):
        if now - self.last_disarm_req < DISARM_MIN_GAP_S:
            return
        self.last_disarm_req = now
        self.disarm_pub.publish(Bool(data=True))
        self.get_logger().warn(
            "force-disarm requested (RC heartbeat lost)", throttle_duration_sec=1.0)

    def _publish_kill(self, active):
        if active == self.last_kill_published:
            return
        self.kill_pub.publish(Bool(data=active))
        self.last_kill_published = active

    def _status_log(self, now, link_lost, bridge_ok):
        if now - self.last_status_log < 5.0:
            return
        self.last_status_log = now
        health = ("unknown" if self.rc_link_healthy is None
                  else ("ok" if self._rc_health_ok(now) else "LOST"))
        self.get_logger().info(
            f"armed={self.core.armed} rc_lost={link_lost} killed={self.core.killed} "
            f"bridge_ok={bridge_ok} rc_health={health} "
            f"rc_age={now - self.core.last_rc_ok:.1f}s")


def main(args=None):
    run_node(RCHeartbeatWatchdog, args=args)


if __name__ == "__main__":
    main()
