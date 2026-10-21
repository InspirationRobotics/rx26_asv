"""led_node: /crsd/led_state (0 off, 1 RED, 2 YELLOW, 3 GREEN) to the LED Arduino over serial.

Reconnects every reconnect_s (EMI knocks the CH340 off the bus) and re-sends the last state.
Test: ros2 topic pub /crsd/led_state std_msgs/msg/Int32 "{data: 2}" --once
"""
import time
from threading import Lock

from rclpy.node import Node
from std_msgs.msg import Int32

from crusader_common.node_main import declare_params, run_node

PARAMS = {
    "port": "/dev/crsd-led",
    "baud": 115200,
    "reconnect_s": 3.0,
}

VALID_STATES = {0: "off", 1: "RED disarmed/e-stop", 2: "YELLOW manual", 3: "GREEN auto"}


class ArduinoLink:

    def __init__(self, port, baud):
        import serial
        self.conn = serial.Serial(port, baud, timeout=1)
        time.sleep(2)          # the Arduino resets on connect

    def send_state(self, state: int):
        self.conn.write(f"{state}\n".encode("ascii"))
        self.conn.flush()

    def close(self):
        if self.conn.is_open:
            self.conn.close()


class LEDNode(Node):

    def __init__(self):
        super().__init__("led_node")
        p = declare_params(self, PARAMS)
        self.port, self.baud = p["port"], p["baud"]
        self.link = None
        self.last_state = None
        self.lock = Lock()
        self._connect()
        self.create_subscription(Int32, "/crsd/led_state", self._state_cb, 10)
        self.create_timer(p["reconnect_s"], self._reconnect_check)

    def _connect(self) -> bool:
        try:
            self.link = ArduinoLink(self.port, self.baud)
            self.get_logger().info(f"LED Arduino connected on {self.port}")
            return True
        except Exception as e:  # noqa: BLE001
            self.link = None
            self.get_logger().warn(f"LED serial connect failed: {e}")
            return False

    def _reconnect_check(self):
        if self.link is None and self._connect() and self.last_state is not None:
            self._send(self.last_state)

    def _send(self, state):
        try:
            with self.lock:
                self.link.send_state(state)
        except Exception:  # noqa: BLE001
            try:
                self.link.close()
            except Exception:  # noqa: BLE001
                pass
            self.link = None
            self.get_logger().warn("LED serial link lost: retrying")

    def _state_cb(self, msg: Int32):
        state = msg.data
        if state not in VALID_STATES:
            self.get_logger().warn(f"invalid LED state: {state}")
            return
        self.last_state = state
        self.get_logger().info(f"LED -> {VALID_STATES[state]}")
        if self.link is None:
            self.get_logger().warn("LED not connected: applies on reconnect")
            return
        self._send(state)

    def destroy_node(self):
        if self.link is not None:
            try:
                self.link.send_state(0)
                self.link.close()
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    run_node(LEDNode, args=args)


if __name__ == "__main__":
    main()
