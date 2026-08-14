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

  in:  /crsd/rc_channels   (RcChannels)  monitored-channel PWM = the RC heartbeat
       /crsd/fcu_status    (FcuStatus)   armed state
  out: /crsd/force_disarm  (Bool)        request bridge to force-disarm (UN-gated
                                         by the autonomy-drop latch on purpose)
       /crsd/kill_active   (Bool)        watchdog kill state, for observability

How link health is judged: the RC link is alive as long as the monitored channel
(`rc_channel`, the SB arm/e-stop switch by default) carries a valid PWM. On this
hardware ArduPilot reports 0 for that channel when the transmitter is out of range
or powered off — the same "RC loss reads 0" behaviour `pixhawk_led_status_node`
relies on — so a dropout shows up as `pwm < min_valid_pwm`, republished by
telemetry_bridge on `/crsd/rc_channels`. (The boat-repo original also consulted
the SYS_STATUS RC-receiver health bit; telemetry_bridge does not republish
SYS_STATUS, so that path is dropped. The PWM-zero signal is this repo's canonical
RC-loss indicator; add SYS_STATUS republishing to the bridge if a second signal
is ever wanted.)

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

        # All safety logic lives in the ROS-free core (rc_heartbeat_core.py);
        # this node only marshals topics.
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

        self.last_disarm_req = 0.0
        self.last_kill_published = None
        self.last_status_log = 0.0

        self.create_timer(1.0 / TICK_HZ, self.tick)
        self.get_logger().info(
            f"RC watchdog active: ch{self.rc_channel} heartbeat_timeout="
            f"{self.core.cfg.heartbeat_timeout:.1f}s latch={self.core.cfg.enable_latch}")

    # ---------------- topic intake ----------------
    def _rc_cb(self, msg: RcChannels):
        monitored_valid = (len(msg.channels) >= self.rc_channel
                           and msg.channels[self.rc_channel - 1] >= self.min_valid_pwm)
        reset_pwm = (msg.channels[self.reset_channel - 1]
                     if len(msg.channels) >= self.reset_channel else 0)
        self.core.note_rc(time.monotonic(), monitored_valid, reset_pwm)

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
        self.get_logger().info(
            f"armed={self.core.armed} rc_lost={link_lost} killed={self.core.killed} "
            f"bridge_ok={bridge_ok} rc_age={now - self.core.last_rc_ok:.1f}s")


def main(args=None):
    run_node(RCHeartbeatWatchdog, args=args)


if __name__ == "__main__":
    main()
