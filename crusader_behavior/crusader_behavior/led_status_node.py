"""led_status_node: autopilot + RC state -> /crsd/led_state (1 RED, 2 YELLOW, 3 GREEN).

RED = disarmed, e-stopped or RC lost (the SB e-stop keeps the boat armed, so it is read from
RC_CHANNELS; a lost link reads 0). GREEN = an autonomous mode, or a mission ticking
(/crsd/autonomy_active). YELLOW otherwise.
"""
import time

from rclpy.node import Node
from std_msgs.msg import Bool, Int32

from crusader_msgs.msg import FcuStatus, RcChannels

from crusader_common.node_main import declare_params, run_node

PARAMS = {
    "estop_channel": 7,
    "estop_threshold": 1200,
    "autonomy_hold_s": 1.0,
    "rate_hz": 5.0,
    # The same list telemetry_bridge obeys, so the light cannot say GREEN while it refuses.
    "autonomous_modes": ["GUIDED", "AUTO", "LOITER", "RTL"],
}

RED, YELLOW, GREEN = 1, 2, 3
LABELS = {RED: "RED disarmed/e-stop", YELLOW: "YELLOW manual", GREEN: "GREEN auto"}


class LedStatusNode(Node):

    def __init__(self):
        super().__init__("led_status_node")
        p = declare_params(self, PARAMS)
        self.estop_channel = p["estop_channel"]
        self.estop_threshold = p["estop_threshold"]
        self.autonomy_hold_s = p["autonomy_hold_s"]
        self.auto_modes = {str(m).upper() for m in p["autonomous_modes"]}

        self.led_pub = self.create_publisher(Int32, "/crsd/led_state", 10)
        self.create_subscription(FcuStatus, "/crsd/fcu_status", self._fcu_cb, 10)
        self.create_subscription(RcChannels, "/crsd/rc_channels", self._rc_cb, 10)
        self.create_subscription(Bool, "/crsd/autonomy_active", self._auto_cb, 10)
        self.mode = None
        self.armed = False
        self.estop_active = True          # killed until real RC arrives
        self.autonomy_t = 0.0
        self.last_state = None
        self.create_timer(1.0 / p["rate_hz"], self._evaluate)

    def _fcu_cb(self, msg: FcuStatus):
        self.mode = msg.mode
        self.armed = msg.armed

    def _rc_cb(self, msg: RcChannels):
        if len(msg.channels) >= self.estop_channel:
            self.estop_active = msg.channels[self.estop_channel - 1] < self.estop_threshold

    def _auto_cb(self, msg: Bool):
        if msg.data:
            self.autonomy_t = time.time()

    def _evaluate(self):
        if self.mode is None:
            return
        if not self.armed or self.estop_active:
            state = RED
        elif (str(self.mode).upper() in self.auto_modes
              or time.time() - self.autonomy_t < self.autonomy_hold_s):
            state = GREEN
        else:
            state = YELLOW
        if state != self.last_state:
            self.led_pub.publish(Int32(data=state))
            self.last_state = state
            self.get_logger().info(f"LED: {LABELS[state]}")


def main(args=None):
    run_node(LedStatusNode, args=args)


if __name__ == "__main__":
    main()
