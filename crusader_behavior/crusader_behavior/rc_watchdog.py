"""rc_watchdog: force-disarm through telemetry_bridge when the RC transmitter link is lost.

A software layer beside ArduPilot's own RC failsafe and the SB e-stop, never instead of them.
The link is alive while `rc_channel` carries a valid PWM (ArduPilot reports 0 on RC loss).
The machine itself is rc_heartbeat_core.

  in:  /crsd/rc_channels, /crsd/fcu_status
  out: /crsd/force_disarm (Bool), /crsd/kill_active (Bool)
"""
import time

from rclpy.node import Node
from std_msgs.msg import Bool

from crusader_msgs.msg import FcuStatus, RcChannels

from crusader_common.node_main import declare_params, run_node
from crusader_behavior.rc_heartbeat_core import RcHeartbeatCore, WatchdogConfig

PARAMS = {
    "heartbeat_timeout": 1.0,
    "rc_channel": 7,
    "min_valid_pwm": 900,
    "require_armed": True,
    "link_timeout": 3.0,
    "enable_latch": False,
    "reset_channel": 7,
    "reset_low_pwm": 1300,
    "reset_high_pwm": 1700,
}

TICK_HZ = 10.0
DISARM_MIN_GAP_S = 0.4


def channel(msg, ch):
    """1-based RC channel value, 0 when the message is too short."""
    return msg.channels[ch - 1] if len(msg.channels) >= ch else 0


class RcWatchdog(Node):

    def __init__(self):
        super().__init__("rc_watchdog")
        p = declare_params(self, PARAMS)
        self.rc_channel = p["rc_channel"]
        self.reset_channel = p["reset_channel"]
        self.min_valid_pwm = p["min_valid_pwm"]
        self.core = RcHeartbeatCore(
            WatchdogConfig(heartbeat_timeout=p["heartbeat_timeout"], min_valid_pwm=p["min_valid_pwm"],
                           require_armed=p["require_armed"], link_timeout=p["link_timeout"],
                           enable_latch=p["enable_latch"], reset_low=p["reset_low_pwm"],
                           reset_high=p["reset_high_pwm"]),
            now=time.monotonic())

        self.disarm_pub = self.create_publisher(Bool, "/crsd/force_disarm", 10)
        self.kill_pub = self.create_publisher(Bool, "/crsd/kill_active", 10)
        self.create_subscription(RcChannels, "/crsd/rc_channels", self._rc_cb, 10)
        self.create_subscription(FcuStatus, "/crsd/fcu_status", self._fcu_cb, 10)
        self.last_disarm_req = 0.0
        self.last_kill_published = None
        self.last_status_log = 0.0
        self.create_timer(1.0 / TICK_HZ, self.tick)
        self.get_logger().info(f"RC watchdog: ch{self.rc_channel}, heartbeat_timeout "
                               f"{p['heartbeat_timeout']:.1f}s, latch {p['enable_latch']}")

    def _rc_cb(self, msg: RcChannels):
        self.core.note_rc(time.monotonic(), channel(msg, self.rc_channel) >= self.min_valid_pwm,
                          channel(msg, self.reset_channel))

    def _fcu_cb(self, msg: FcuStatus):
        self.core.note_fcu(time.monotonic(), msg.armed)

    def tick(self):
        now = time.monotonic()
        d = self.core.tick(now)
        log = self.get_logger()
        if d.latch_cleared:
            log.info(f"kill latch cleared by the RC reset toggle on ch{self.reset_channel}")
        if d.gateway_down_in_loss:
            log.error("RC link lost AND telemetry_bridge silent: cannot disarm through it; "
                      "relying on ArduPilot's failsafe", throttle_duration_sec=2.0)
        if d.engaged:
            log.error(f"RC heartbeat lost: FORCE-DISARMING (latched; toggle ch{self.reset_channel} "
                      "low->high to clear)")
        elif d.disarm_request and not self.core.cfg.enable_latch:
            log.error("RC heartbeat lost: FORCE-DISARMING (recovers with the link)",
                      throttle_duration_sec=1.0)
        if d.disarm_request and now - self.last_disarm_req >= DISARM_MIN_GAP_S:
            self.last_disarm_req = now
            self.disarm_pub.publish(Bool(data=True))
        if d.kill_active != self.last_kill_published:
            self.kill_pub.publish(Bool(data=d.kill_active))
            self.last_kill_published = d.kill_active
        if now - self.last_status_log >= 5.0:
            self.last_status_log = now
            log.info(f"armed={self.core.armed} rc_lost={self.core._link_lost(now)} "
                     f"killed={self.core.killed} bridge_ok={self.core._bridge_ok(now)} "
                     f"rc_age={now - self.core.last_rc_ok:.1f}s")


def main(args=None):
    run_node(RcWatchdog, args=args)


if __name__ == "__main__":
    main()
