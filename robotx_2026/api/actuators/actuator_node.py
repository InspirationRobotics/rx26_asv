"""actuator_node — serial-driven mission actuators (Mission 3).

Ported from RoboBoat's servos/ (ArdiunoCompound + ball_launcher_node) and adapted
to this repo's conventions. Mission 3 needs two effectors:
  * a resource-delivery launcher (RoboBoat's racquetball launcher) — Crusader has a
    `/dev/crsd-ball-launcher` device in crusader_devices.json (9600 baud, a Pololu
    Mini Maestro / Arduino compound);
  * a firefighting water cannon/pump (extinguish the RED window until it turns GREEN).

Both hang off one serial controller. Exposed as ROS services so the mission
planner (or a human at the CLI) triggers them explicitly:
  /crsd/actuator/launch  std_srvs/Trigger  -> fire sequence (launch_char, reload_char)
  /crsd/actuator/release std_srvs/Trigger  -> release spring (launch_char only)
  /crsd/actuator/pump    std_srvs/SetBool  -> water cannon on/off (Maestro PWM)

  ros2 service call /crsd/actuator/launch std_srvs/srv/Trigger "{}"
  ros2 service call /crsd/actuator/pump std_srvs/srv/SetBool "{data: true}"

FIRMWARE NOTE: the char/PWM protocol below mirrors the RoboBoat Maestro controller
('g' then 'a' to fire; set_pwm for servos). Confirm it against Crusader's actual
actuator controller firmware on the bench before trusting it — a wrong protocol
fails at the effector, so this node logs every command it sends.
"""
import time
from threading import Lock

from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool

from ..common import config as crsd_config
from ..common.node_main import run_node
from ..common.param_utils import declare_from_config

PARAM_SPEC = {
    "port": dict(read_only=True,
                 description="actuator controller serial device (udev symlink)"),
    "baud": dict(read_only=True, lo=1200, hi=1000000,
                 description="serial baud (Maestro default 9600)"),
    "launch_char": dict(read_only=True, description="fire command char"),
    "reload_char": dict(read_only=True, description="reload command char"),
    "fire_gap_s": dict(read_only=True, lo=0.0, hi=5.0,
                       description="delay between fire and reload chars [s]"),
    "pump_channel": dict(read_only=True, lo=0, hi=17,
                         description="Maestro servo channel for the water pump"),
    "pump_on_us": dict(read_only=True, lo=500, hi=2500,
                       description="pump-ON pulse width [us]"),
    "pump_off_us": dict(read_only=True, lo=500, hi=2500,
                        description="pump-OFF pulse width [us]"),
}


class MaestroLink:
    """Serial writer for a Pololu Mini Maestro / Arduino compound controller.
    pyserial import is local so the module imports without the hardware."""

    def __init__(self, port, baud):
        import serial
        self.conn = serial.Serial(port, baud, timeout=1)
        time.sleep(2)                      # controller resets on connect; wait

    def send_char(self, ch: str):
        self.conn.write(ch.encode("ascii"))
        self.conn.flush()

    def set_pwm(self, channel: int, target_us: int):
        # Pololu compact protocol (0x84): target is in quarter-microseconds.
        target = int(target_us) * 4
        self.conn.write(bytes([0x84, channel, target & 0x7F, (target >> 7) & 0x7F]))
        self.conn.flush()

    def close(self):
        if self.conn.is_open:
            self.conn.close()


class ActuatorNode(Node):
    def __init__(self):
        super().__init__("actuator_node")
        self.p = declare_from_config(
            self, crsd_config.node_params("actuator_node"), PARAM_SPEC)

        self.lock = Lock()
        self.link = None
        self._connect()

        self.create_service(Trigger, "/crsd/actuator/launch", self._launch_cb)
        self.create_service(Trigger, "/crsd/actuator/release", self._release_cb)
        self.create_service(SetBool, "/crsd/actuator/pump", self._pump_cb)
        self.get_logger().info("actuator node up: launch/release/pump services ready")

    def _connect(self) -> bool:
        try:
            self.link = MaestroLink(self.p["port"], self.p["baud"])
            self.get_logger().info(f"actuator controller connected on {self.p['port']}")
            return True
        except Exception as e:
            self.link = None
            self.get_logger().error(f"actuator serial connect failed: {e}")
            return False

    def _ensure_link(self):
        return self.link is not None or self._connect()

    # ---------- services ----------

    def _launch_cb(self, request, response):
        if not self._ensure_link():
            response.success, response.message = False, "no actuator serial link"
            return response
        try:
            with self.lock:
                self.link.send_char(self.p["launch_char"])
                time.sleep(self.p["fire_gap_s"])
                self.link.send_char(self.p["reload_char"])
            self.get_logger().info("launcher fired")
            response.success, response.message = True, "launched"
        except Exception as e:
            self.link = None
            response.success, response.message = False, f"launch failed: {e}"
            self.get_logger().error(response.message)
        return response

    def _release_cb(self, request, response):
        if not self._ensure_link():
            response.success, response.message = False, "no actuator serial link"
            return response
        try:
            with self.lock:
                self.link.send_char(self.p["launch_char"])
            self.get_logger().warning("launcher spring released")
            response.success, response.message = True, "released"
        except Exception as e:
            self.link = None
            response.success, response.message = False, f"release failed: {e}"
            self.get_logger().error(response.message)
        return response

    def _pump_cb(self, request, response):
        if not self._ensure_link():
            response.success, response.message = False, "no actuator serial link"
            return response
        target = self.p["pump_on_us"] if request.data else self.p["pump_off_us"]
        try:
            with self.lock:
                self.link.set_pwm(self.p["pump_channel"], target)
            state = "ON" if request.data else "OFF"
            self.get_logger().info(f"water pump {state} (ch{self.p['pump_channel']} "
                                   f"-> {target}us)")
            response.success, response.message = True, f"pump {state}"
        except Exception as e:
            self.link = None
            response.success, response.message = False, f"pump failed: {e}"
            self.get_logger().error(response.message)
        return response

    def destroy_node(self):
        if self.link is not None:
            try:
                # fail safe: pump OFF on shutdown
                self.link.set_pwm(self.p["pump_channel"], self.p["pump_off_us"])
                self.link.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    run_node(ActuatorNode, args=args)


if __name__ == "__main__":
    main()
