"""pixhawk_led_status_node — maps autopilot + RC state to /crsd/led_state.

Ported and REWIRED for this repo: the operational version opened its own
`udpin:127.0.0.1:14550` MAVLink connection, making it a second consumer of the
Pixhawk stream. Here it subscribes to telemetry_bridge's republished topics
instead — telemetry_bridge is the single ROS-side gateway, so this node adds no
new MAVLink connection and cannot race the graph.

  in:  /crsd/fcu_status     (FcuStatus)   mode + armed
       /crsd/rc_channels    (RcChannels)  e-stop switch position
       /crsd/rc_link_health (Bool)        ArduPilot's RC-receiver failsafe verdict
       /crsd/autonomy_active (Bool)       heartbeat from an active autonomy node
  out: /crsd/led_state      (Int32)       1=RED 2=YELLOW 3=GREEN (led_node input)

Why RC for the e-stop: the SB e-stop keeps the vehicle ARMED (RC option 165), so
it is invisible in HEARTBEAT/armed — the switch position must be read from
RC_CHANNELS. NOTE: estop_channel defaults to the SB switch (ch7) per the RC
reference; this is the same channel telemetry_bridge uses for the autonomy-drop
latch — the failsafe pass reconciles RC channel ownership.

TWO ways this node used to show YELLOW on a boat that had lost its pilot, both
found on Crusader 2026-08-02 and both fixed here:

  1. "RC loss reads 0" is FALSE on this airframe. The original code took a lost
     link to mean ch7 drops below estop_threshold. With the ELRS receiver holding
     last position, ch7 sat at 1995 through a full transmitter power-down, so
     estop_active stayed False. /crsd/rc_link_health carries ArduPilot's own
     RC-receiver failsafe bit, which does not care what the receiver puts on the
     wire, and a fresh unhealthy report now forces RED on its own.

  2. Silence was read as "no change". Every input was cached in a callback and
     never expired, while telemetry_bridge STOPS republishing a stale stream by
     design. A dead gateway therefore froze the last colour on the strip instead
     of failing to RED. Inputs now age out after input_timeout_s.

Both directions of the same rule: for a safety indicator, "I do not know" must
render as "not safe". RED is the only colour that is ever safe to guess.
"""
import time

from rclpy.node import Node
from std_msgs.msg import Int32, Bool

from interfaces.msg import FcuStatus, RcChannels

from rx26_asv.api.common import config as crsd_config
from rx26_asv.api.common.node_main import run_node
from rx26_asv.api.common.param_utils import declare_from_config

PARAM_SPEC = {
    "estop_channel": dict(read_only=True, lo=1, hi=18,
                          description="RC channel of the SB e-stop switch"),
    "estop_threshold": dict(read_only=True, lo=800, hi=2200,
                            description="us; below this = e-stopped (RC loss=0)"),
    "autonomy_hold_s": dict(read_only=True, lo=0.2, hi=10.0,
                            description="how long after last autonomy heartbeat to stay GREEN"),
    "input_timeout_s": dict(read_only=True, lo=0.5, hi=10.0,
                            description="s without a bridge topic before the LED fails to RED"),
    "rate_hz": dict(read_only=True, lo=1.0, hi=50.0,
                    description="LED state evaluation rate"),
}

RED, YELLOW, GREEN = 1, 2, 3
AUTO_MODES = ("AUTO", "GUIDED")


class PixhawkLEDStatusNode(Node):
    def __init__(self):
        super().__init__("pixhawk_led_status")
        p = declare_from_config(
            self, crsd_config.node_params("pixhawk_led_status_node"), PARAM_SPEC)
        self.estop_channel = p["estop_channel"]
        self.estop_threshold = p["estop_threshold"]
        self.autonomy_hold_s = p["autonomy_hold_s"]
        self.input_timeout_s = p["input_timeout_s"]

        self.led_pub = self.create_publisher(Int32, "/crsd/led_state", 10)
        self.create_subscription(FcuStatus, "/crsd/fcu_status", self._fcu_cb, 10)
        self.create_subscription(RcChannels, "/crsd/rc_channels", self._rc_cb, 10)
        self.create_subscription(Bool, "/crsd/rc_link_health", self._health_cb, 10)
        self.create_subscription(Bool, "/crsd/autonomy_active", self._auto_cb, 10)

        self.mode = None
        self.armed = False
        self.estop_active = True         # assume killed until real RC arrives
        self.rc_link_healthy = None      # None = autopilot reports no RC bit
        self.autonomy_t = 0.0
        # Receipt times: an input that stops arriving must age out, not persist.
        self.fcu_t = 0.0
        self.rc_t = 0.0
        self.health_t = 0.0
        self.last_state = None
        self.last_reason = None

        self.create_timer(1.0 / p["rate_hz"], self._evaluate)

    def _fcu_cb(self, msg: FcuStatus):
        self.mode = msg.mode
        self.armed = msg.armed
        self.fcu_t = time.time()

    def _rc_cb(self, msg: RcChannels):
        if len(msg.channels) >= self.estop_channel:
            pwm = msg.channels[self.estop_channel - 1]
            self.estop_active = pwm < self.estop_threshold
            self.rc_t = time.time()

    def _health_cb(self, msg: Bool):
        self.rc_link_healthy = msg.data
        self.health_t = time.time()

    def _auto_cb(self, msg: Bool):
        if msg.data:
            self.autonomy_t = time.time()

    def _red_reason(self, now):
        """Why the strip must be RED, or None if it need not be.

        Ordered most-specific first so the logged reason names the actual fault
        rather than a downstream symptom of it.
        """
        if now - self.fcu_t > self.input_timeout_s:
            return "no /crsd/fcu_status (bridge or MAVProxy down)"
        if now - self.rc_t > self.input_timeout_s:
            return "no /crsd/rc_channels (bridge or MAVProxy down)"
        # Only an input we have actually seen can go stale; a graph with no
        # rc_link_health publisher at all degrades to the PWM check, loudly
        # warned about by telemetry_bridge, rather than latching RED forever.
        if self.health_t and now - self.health_t > self.input_timeout_s:
            return "no /crsd/rc_link_health (was present, stopped)"
        if self.rc_link_healthy is False:
            return "ArduPilot reports RC receiver UNHEALTHY (link lost)"
        if not self.armed:
            return "disarmed"
        if self.estop_active:
            return f"e-stop: ch{self.estop_channel} < {self.estop_threshold}us"
        return None

    def _evaluate(self):
        now = time.time()
        if self.mode is None:
            return                       # no autopilot status yet
        reason = self._red_reason(now)
        if reason is not None:
            state = RED
        elif (self.mode in AUTO_MODES
              or (now - self.autonomy_t) < self.autonomy_hold_s):
            state = GREEN
        else:
            state = YELLOW
        self._publish(state, reason)

    def _publish(self, state, reason=None):
        if state == self.last_state and reason == self.last_reason:
            return
        if state != self.last_state:
            self.led_pub.publish(Int32(data=state))
        self.last_state = state
        self.last_reason = reason
        labels = {RED: "RED disarmed/e-stop", YELLOW: "YELLOW manual",
                  GREEN: "GREEN auto"}
        suffix = f" ({reason})" if reason else ""
        self.get_logger().info(f"LED: {labels[state]}{suffix}")


def main(args=None):
    run_node(PixhawkLEDStatusNode, args=args)


if __name__ == "__main__":
    main()
