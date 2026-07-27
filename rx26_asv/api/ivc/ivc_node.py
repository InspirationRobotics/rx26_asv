"""ivc_node — inter-vehicle comms over the team WiFi (Bullet AC link).

Bridges the IvcLink peer connection (api.ivc.ivc_link) to ROS topics so mission
code exchanges messages with a partner vehicle (another ASV, or the UAV/UUV relay
in Missions 1/2/3) without touching sockets:

  in:  /crsd/ivc/send    (std_msgs/String)  -> sent to the peer
  out: /crsd/ivc/receive (std_msgs/String)  <- received from the peer
       /crsd/ivc/health  (std_msgs/String)  1 Hz JSON: alive / dead_reason / age

  ros2 topic pub /crsd/ivc/send std_msgs/msg/String "{data: 'hello'}"
  ros2 topic echo /crsd/ivc/receive

Connection is managed on a background thread (accept()/connect() must not block the
ROS executor); it retries every `reconnect_s` until the peer is up and after any
drop. Role/partner are config:
  role="server" -> this vehicle binds and waits (the "leader"/rendezvous host);
  role="client" -> this vehicle connects to `partner_ip`.
Exactly one peer per pairing runs as server. This is the TEAM WiFi link (Bullet AC),
separate from the RJ-45 RoboCommand link (mission/robocomms.py) and the Pixhawk link.
"""
import json
import threading
import time

from rclpy.node import Node
from std_msgs.msg import String

from rx26_asv.api.common import config as crsd_config
from rx26_asv.api.common.node_main import run_node
from rx26_asv.api.common.param_utils import declare_from_config
from rx26_asv.api.ivc.ivc_link import IvcServer, IvcClient

PARAM_SPEC = {
    "role": dict(read_only=True, description='"server" (bind/wait) or "client" (connect)'),
    "partner_ip": dict(read_only=True, description="peer IP (client role only)"),
    "port": dict(read_only=True, lo=1, hi=65535),
    "reconnect_s": dict(read_only=True, lo=0.5, hi=30.0,
                        description="retry period for (re)connect [s]"),
    "poll_hz": dict(read_only=True, lo=1.0, hi=100.0,
                    description="inbound-queue drain rate"),
}


class IvcNode(Node):
    def __init__(self):
        super().__init__("ivc_node")
        p = declare_from_config(self, crsd_config.node_params("ivc_node"), PARAM_SPEC)
        role = str(p["role"]).lower()
        if role == "server":
            self.link = IvcServer(port=p["port"])
            self._connect = lambda: self.link.connect_or_serve(accept_timeout_s=1.0)
            where = f"bind :{p['port']}"
        elif role == "client":
            self.link = IvcClient(server_ip=p["partner_ip"], port=p["port"])
            self._connect = self.link.connect_or_serve
            where = f"connect {p['partner_ip']}:{p['port']}"
        else:
            raise ValueError(f"ivc role must be 'server' or 'client', got {p['role']!r}")
        self.reconnect_s = p["reconnect_s"]

        self.rx_pub = self.create_publisher(String, "/crsd/ivc/receive", 10)
        self.health_pub = self.create_publisher(String, "/crsd/ivc/health", 10)
        self.create_subscription(String, "/crsd/ivc/send", self._send_cb, 10)

        self._stop = threading.Event()
        self._conn_thread = threading.Thread(target=self._conn_manager, daemon=True)
        self._conn_thread.start()

        self.create_timer(1.0 / p["poll_hz"], self._poll)
        self.create_timer(1.0, self._publish_health)
        self.get_logger().info(f"IVC up ({role}, {where}) — team WiFi / Bullet AC link")

    # ---------- connection management (off the ROS executor) ----------

    def _conn_manager(self):
        while not self._stop.is_set():
            if not self.link.alive:
                if self.link.dead_reason:
                    self.get_logger().warn(f"IVC link down: {self.link.dead_reason}")
                    self.link.dead_reason = None
                if self._connect():
                    self.get_logger().info("IVC peer connected")
                else:
                    self._stop.wait(self.reconnect_s)
            else:
                self._stop.wait(0.5)

    # ---------- ROS bridge ----------

    def _send_cb(self, msg: String):
        if not self.link.send(msg.data):
            self.get_logger().warn(f"IVC send dropped (link down): {msg.data!r}")

    def _poll(self):
        for _ in range(100):               # bounded drain per tick
            text = self.link.get_next()
            if text is None:
                break
            self.rx_pub.publish(String(data=text))
            self.get_logger().info(f"IVC recv: {text}", throttle_duration_sec=1.0)

    def _publish_health(self):
        self.health_pub.publish(String(data=json.dumps(self.link.health())))

    def destroy_node(self):
        self._stop.set()
        self._conn_thread.join(timeout=2.0)
        self.link.close()
        super().destroy_node()


def main(args=None):
    run_node(IvcNode, args=args)


if __name__ == "__main__":
    main()
