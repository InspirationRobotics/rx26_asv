"""led_node — forwards /crsd/led_state (Int32 0..3) to the LED Arduino over serial.

Ported from the operational Crusader stack into this repo's conventions:
  * run_node lifecycle (deterministic teardown: strip OFF + serial close on exit);
  * declare_from_config against config/crusader_params.yaml (no drifting code
    defaults) — port is [RO] (a udev symlink, structural) and baud is [RO];
  * /crsd/ topic namespace like every other node here.

State contract (matches firmware/LED.ino):
  0 = off, 1 = RED (disarmed / e-stop), 2 = YELLOW (armed/manual), 3 = GREEN (auto).

The link auto-reconnects: EMI can knock the CH340/Arduino off the bus and it
re-enumerates; the node retries every `reconnect_s` and re-applies the last
commanded color so the strip never lies about boat state after a glitch.

Manual test (only led_node needs to be running):
  ros2 topic pub /crsd/led_state std_msgs/msg/Int32 "{data: 2}" --once
"""
import time
from threading import Lock

from rclpy.node import Node
from std_msgs.msg import Int32

from robotx_2026.api.common import config as crsd_config
from robotx_2026.api.common.node_main import run_node
from robotx_2026.api.common.param_utils import declare_from_config

PARAM_SPEC = {
    "port": dict(read_only=True,
                 description="LED Arduino serial device (udev symlink)"),
    "baud": dict(read_only=True, lo=9600, hi=1000000,
                 description="serial baud (must match LED.ino Serial.begin)"),
    "reconnect_s": dict(read_only=True, lo=0.5, hi=30.0,
                        description="serial reconnect retry period [s]"),
}

VALID_STATES = {0: "off", 1: "RED disarmed/e-stop", 2: "YELLOW manual",
                3: "GREEN auto"}


class ArduinoLink:
    """Thin serial writer. Import of pyserial is local so the module imports on
    machines without the LED hardware (unit tests, dev laptop)."""

    def __init__(self, port, baud):
        import serial
        self.conn = serial.Serial(port, baud, timeout=1)
        time.sleep(2)          # Arduino resets on connect; wait for boot

    def send_state(self, state: int):
        self.conn.write(f"{state}\n".encode("ascii"))
        self.conn.flush()

    def close(self):
        if self.conn.is_open:
            self.conn.close()


class LEDNode(Node):
    def __init__(self):
        super().__init__("led_node")
        p = declare_from_config(self, crsd_config.node_params("led_node"),
                                PARAM_SPEC)
        self.port = p["port"]
        self.baud = p["baud"]

        self.link = None
        self.last_state = None
        self.lock = Lock()

        self._connect()
        self.create_subscription(Int32, "/crsd/led_state",
                                 self._state_cb, 10)
        self.create_timer(p["reconnect_s"], self._reconnect_check)

    def _connect(self) -> bool:
        try:
            self.link = ArduinoLink(self.port, self.baud)
            self.get_logger().info(f"LED Arduino connected on {self.port}")
            return True
        except Exception as e:
            self.link = None
            self.get_logger().warn(f"LED serial connect failed: {e}")
            return False

    def _mark_dead(self):
        try:
            self.link.close()
        except Exception:
            pass
        self.link = None
        self.get_logger().warn("LED serial link lost — retrying")

    def _reconnect_check(self):
        if self.link is not None:
            return
        if self._connect() and self.last_state is not None:
            self._send(self.last_state)      # restore the color that should show

    def _send(self, state) -> bool:
        try:
            with self.lock:
                self.link.send_state(state)
            return True
        except Exception:
            self._mark_dead()
            return False

    def _state_cb(self, msg: Int32):
        state = msg.data
        if state not in VALID_STATES:
            self.get_logger().warn(f"invalid LED state: {state}")
            return
        self.last_state = state
        self.get_logger().info(f"LED -> {VALID_STATES[state]}")
        if self.link is None:
            self.get_logger().warn("LED not connected — saved, applies on reconnect")
            return
        self._send(state)

    def destroy_node(self):
        if self.link is not None:
            try:
                self.link.send_state(0)      # strip OFF on clean shutdown
                self.link.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    run_node(LEDNode, args=args)


if __name__ == "__main__":
    main()
