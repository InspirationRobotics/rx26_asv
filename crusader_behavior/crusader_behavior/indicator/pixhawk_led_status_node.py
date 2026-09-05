"""pixhawk_led_status_node — maps autopilot + RC state to /crsd/led_state.

Ported and REWIRED for this repo: the operational version opened its own
`udpin:127.0.0.1:14550` MAVLink connection, making it a second consumer of the
Pixhawk stream. Here it subscribes to telemetry_bridge's republished topics
instead — telemetry_bridge is the single ROS-side gateway, so this node adds no
new MAVLink connection and cannot race the graph.

  in:  /crsd/fcu_status     (FcuStatus)   mode + armed
       /crsd/rc_channels    (RcChannels)  e-stop switch position
       /crsd/autonomy_active (Bool)       heartbeat from an active autonomy node
  out: /crsd/led_state      (Int32)       1=RED 2=YELLOW 3=GREEN (led_node input)

Why RC for the e-stop: the SB e-stop keeps the vehicle ARMED (RC option 165), so
it is invisible in HEARTBEAT/armed — the switch position must be read from
RC_CHANNELS. A lost RC link reads 0, which is below the e-stop threshold, so RC
loss also shows RED (fail-safe colour). NOTE: estop_channel defaults to the SB
switch (ch7) per the RC reference; this is the same channel telemetry_bridge uses
for the autonomy-drop latch — the failsafe pass reconciles RC channel ownership.
"""
import time

from rclpy.node import Node
from std_msgs.msg import Int32, Bool

from crusader_msgs.msg import FcuStatus, RcChannels

from crusader_common import config as crsd_config
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config

PARAM_SPEC = {
    "estop_channel": dict(read_only=True, lo=1, hi=18,
                          description="RC channel of the SB e-stop switch"),
    "estop_threshold": dict(read_only=True, lo=800, hi=2200,
                            description="us; below this = e-stopped (RC loss=0)"),
    "autonomy_hold_s": dict(read_only=True, lo=0.2, hi=10.0,
                            description="how long after last autonomy heartbeat to stay GREEN"),
    "rate_hz": dict(read_only=True, lo=1.0, hi=50.0,
                    description="LED state evaluation rate"),
}

RED, YELLOW, GREEN = 1, 2, 3
# Which modes light the mast GREEN. Loaded from shared.autonomous_modes at
# construction (see __init__) rather than hardcoded here, because the SAME list
# decides three things that must never disagree: what telemetry_bridge will
# obey, what this light shows, and what ocs_client reports to RoboCommand as
# STATE_AUTO. handbook 5.3.1 makes the visual state a requirement, so a light
# saying GREEN while the bridge refuses commands is a compliance problem, not
# just a cosmetic one.
#
# This tuple is only the fallback for a config that predates the shared key.
AUTO_MODES_FALLBACK = ("AUTO", "GUIDED")


class PixhawkLEDStatusNode(Node):
    def __init__(self):
        super().__init__("pixhawk_led_status")
        p = declare_from_config(
            self, crsd_config.node_params("pixhawk_led_status_node"), PARAM_SPEC)
        self.estop_channel = p["estop_channel"]
        self.estop_threshold = p["estop_threshold"]
        self.autonomy_hold_s = p["autonomy_hold_s"]

        # Same list telemetry_bridge gates commands on, so the light cannot say
        # GREEN while the bridge is refusing to drive (or vice versa).
        shared = crsd_config.shared_params()
        self.auto_modes = {str(m).upper() for m in
                           shared.get("autonomous_modes", AUTO_MODES_FALLBACK)}
        self.get_logger().info(f"GREEN in modes: {sorted(self.auto_modes)}")

        self.led_pub = self.create_publisher(Int32, "/crsd/led_state", 10)
        self.create_subscription(FcuStatus, "/crsd/fcu_status", self._fcu_cb, 10)
        self.create_subscription(RcChannels, "/crsd/rc_channels", self._rc_cb, 10)
        self.create_subscription(Bool, "/crsd/autonomy_active", self._auto_cb, 10)

        self.mode = None
        self.armed = False
        self.estop_active = True         # assume killed until real RC arrives
        self.autonomy_t = 0.0
        self.last_state = None

        self.create_timer(1.0 / p["rate_hz"], self._evaluate)

    def _fcu_cb(self, msg: FcuStatus):
        self.mode = msg.mode
        self.armed = msg.armed

    def _rc_cb(self, msg: RcChannels):
        if len(msg.channels) >= self.estop_channel:
            pwm = msg.channels[self.estop_channel - 1]
            # RC loss reads 0 -> below threshold -> RED (fail-safe colour)
            self.estop_active = pwm < self.estop_threshold

    def _auto_cb(self, msg: Bool):
        if msg.data:
            self.autonomy_t = time.time()

    def _evaluate(self):
        if self.mode is None:
            return                       # no autopilot status yet
        if not self.armed or self.estop_active:
            state = RED
        elif (str(self.mode).upper() in self.auto_modes
              or (time.time() - self.autonomy_t) < self.autonomy_hold_s):
            state = GREEN
        else:
            state = YELLOW
        self._publish(state)

    def _publish(self, state):
        if state == self.last_state:
            return
        self.led_pub.publish(Int32(data=state))
        self.last_state = state
        labels = {RED: "RED disarmed/e-stop", YELLOW: "YELLOW manual",
                  GREEN: "GREEN auto"}
        self.get_logger().info(f"LED: {labels[state]}")


def main(args=None):
    run_node(PixhawkLEDStatusNode, args=args)


if __name__ == "__main__":
    main()
