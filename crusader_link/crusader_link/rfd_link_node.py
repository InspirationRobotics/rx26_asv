"""rfd_link_node: the boat's end of the RFD900 mesh to Ekko (TUNNEL v3, stock pymavlink).

  out: /crsd/radio/traffic (RadioFrame, every frame sent or heard: a record nothing acts on),
       /crsd/passage_plan (PassagePlan, on each of Ekko's LIGHTS frames; version moves on change)
  in:  /crsd/pose (the BOAT packet, 1 Hz once Ekko is heard and while the pose is fresh),
       /crsd/radio/send (RadioSend), /crsd/radio/send_test (Trigger)

It talks to the radio, never the Pixhawk: no command it sends can arm or move the boat.
When Ekko goes quiet it publishes nothing; downstream ages the last plan out.
"""
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from crusader_msgs.msg import LatLonHead, PassageBuoy, PassagePlan, RadioFrame, RadioSend

from crusader_common.node_main import declare_params, run_node
from crusader_common.stream_cache import StreamCache
from crusader_link import tunnel_link as tl
from crusader_link.ekko_map import EkkoMap, PlanVersion

os.environ["MAVLINK20"] = "1"     # TUNNEL is a MAVLink 2 message

PARAMS = {
    "rfd_endpoint": "/dev/crsd-rfd",    # serial device, or udpin:127.0.0.1:14555 for the sim
    "rfd_baud": 115200,
    "uav_sysid": 200,                   # Ekko's companion, which sends the TUNNELs
    "source_system": 42,                # the boat on the mesh (not its autopilot)
    "pose_timeout_s": 1.0,
    "quiet_warn_s": 15.0,
    "reconnect_s": 5.0,
}

BOAT_PERIOD_S = 1.0
SOURCE_COMPONENT = 191              # MAV_COMP_ID_ONBOARD_COMPUTER
NO_PEER = "nothing heard on this link yet, so there is no address to send to"


def has_peer(conn):
    """A udpin socket can send only to addresses it has heard; a serial radio always can."""
    from pymavlink import mavutil
    if getattr(conn, "udp_server", False):
        return bool(getattr(conn, "clients", None))
    if isinstance(conn, mavutil.mavudp):
        return (getattr(conn, "last_address", None) is not None
                or getattr(conn, "destination_addr", None) is not None)
    return True


def describe(msg):
    """(name, payload_type, one line) for any frame on the mesh. Never raises."""
    t = msg.get_type()
    try:
        if t == "TUNNEL":
            ptype = int(msg.payload_type)
            name, summary = tl.describe(ptype, tl.body(msg))
            return name, ptype, summary
        if t == "HEARTBEAT":
            from pymavlink import mavutil
            entry = mavutil.mavlink.enums["MAV_TYPE"].get(msg.type)
            return t, 0, entry.name[len("MAV_TYPE_"):] if entry else "type %d" % msg.type
        return t, 0, ""
    except Exception as e:  # noqa: BLE001
        return t, 0, "does not decode: %s" % e


class RfdLinkNode(Node):

    def __init__(self):
        super().__init__("rfd_link_node")
        self.p = declare_params(self, PARAMS, dynamic=("quiet_warn_s",))
        self.plan_pub = self.create_publisher(PassagePlan, "/crsd/passage_plan", 10)
        self.radio_pub = self.create_publisher(RadioFrame, "/crsd/radio/traffic", 50)
        self.create_subscription(LatLonHead, "/crsd/pose", self._on_pose, 10)
        self.create_subscription(RadioSend, "/crsd/radio/send", self._on_send, 10)
        self.create_service(Trigger, "/crsd/radio/send_test", self._send_test_cb)

        self._ekko = EkkoMap()
        self._version = PlanVersion()
        self._pose = StreamCache(self.p["pose_timeout_s"])
        self._conn = None
        self._conn_lock = threading.Lock()   # pymavlink connections are not thread-safe
        self._stop = threading.Event()
        self._last_rx = None
        self._quiet_warned = False
        self._said = {}
        self._test_n = 0
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx.start()
        self.create_timer(BOAT_PERIOD_S, self._send_boat)
        self.create_timer(5.0, self._quiet_check)
        self.get_logger().info("rfd_link on %s as system %d, Ekko = system %d" % (
            self.p["rfd_endpoint"], self.p["source_system"], self.p["uav_sysid"]))

    def _once(self, key, value):
        """True when `value` differs from what `key` last held: log on change, not every second."""
        changed = self._said.get(key) != value
        self._said[key] = value
        return changed

    # ------------------------------------------------------------------ the radio

    def _connect(self):
        """Open the radio, or None (a radio plugged in later is picked up on the next try)."""
        from pymavlink import mavutil
        endpoint = self.p["rfd_endpoint"]
        try:
            extra = {} if endpoint.startswith(("udp", "tcp")) else {"baud": int(self.p["rfd_baud"])}
            conn = mavutil.mavlink_connection(endpoint, source_system=self.p["source_system"],
                                              source_component=SOURCE_COMPONENT, **extra)
            conn.mav.robust_parsing = True
            self.get_logger().info("radio open: %s" % endpoint)
            self._said.pop("no_radio", None)
            return conn
        except Exception as e:  # noqa: BLE001
            if self._once("no_radio", str(e)):
                self.get_logger().warn("no radio on %s (%s): retrying every %gs" % (
                    endpoint, e, self.p["reconnect_s"]))
            return None

    def _rx_loop(self):
        while not self._stop.is_set() and rclpy.ok():
            if self._conn is None:
                conn = self._connect()
                with self._conn_lock:
                    self._conn = conn
                if conn is None:
                    self._stop.wait(self.p["reconnect_s"])
                continue
            try:
                with self._conn_lock:
                    msg = self._conn.recv_match(blocking=False)
            except Exception as e:  # noqa: BLE001 - an unplugged radio raises here
                self.get_logger().warn("radio lost (%s): reopening" % e)
                with self._conn_lock:
                    self._conn = None
                continue
            if msg is None:
                self._stop.wait(0.02)
                continue
            if msg.get_type() == "BAD_DATA":
                continue
            try:
                self._heard(msg)
            except Exception as e:  # noqa: BLE001 - one bad frame must not take the link down
                self.get_logger().warn("dropped a frame: %s" % e)

    def _heard(self, msg):
        src = msg.get_srcSystem()
        if src == self.p["source_system"]:
            return                                   # our own frame echoed by the mesh
        self._record(RadioFrame.DIR_RX, msg)
        if msg.get_type() != "TUNNEL" or src != self.p["uav_sysid"]:
            return
        self._last_rx = time.monotonic()
        self._quiet_warned = False
        if self._ekko.hear(int(msg.payload_type), tl.body(msg)) != "lights":
            return
        epoch = self._ekko.epoch()
        if self._once("epoch", epoch):
            self.get_logger().info("Ekko's map is number %s: older maps discarded" % epoch)
        plan = self._ekko.plan()
        if self._once("empty", plan is None) and plan is None:
            self.get_logger().warn("Ekko's map has no buoy with a position and a light yet "
                                   "(%s): no plan published" % self._ekko.summary())
        if plan is not None:
            self._publish_plan(plan)

    def _publish_plan(self, plan):
        changed = self._version.update(plan)
        m = PassagePlan()
        m.header.stamp = self.get_clock().now().to_msg()   # receipt: the vehicles share no clock
        m.plan_version = self._version.version
        m.entry_latitude, m.entry_longitude = plan["entry"]
        m.exit_latitude, m.exit_longitude = plan["exit"]
        for b in plan["buoys"]:
            pb = PassageBuoy()
            pb.id, pb.latitude, pb.longitude, pb.beacon = b["id"], b["lat"], b["lon"], b["beacon"]
            m.buoys.append(pb)
        self.plan_pub.publish(m)
        if changed:
            self.get_logger().info("plan v%d: %d buoys, entry %.7f %.7f, exit %.7f %.7f" % (
                m.plan_version, len(m.buoys), m.entry_latitude, m.entry_longitude,
                m.exit_latitude, m.exit_longitude))

    # ------------------------------------------------------------------ TX

    def _send_tunnel(self, target, payload_type, payload):
        """Put one TUNNEL on the air and record it. -> None when sent, else why not."""
        with self._conn_lock:
            conn = self._conn
            if conn is None:
                return "no radio"
            if not has_peer(conn):
                return NO_PEER
            try:
                data = tl.pad(bytes(payload))
                sent = conn.mav.tunnel_encode(target, 0, payload_type, len(payload), data)
                conn.mav.send(sent)
            except Exception as e:  # noqa: BLE001
                return str(e) or type(e).__name__
        self._record(RadioFrame.DIR_TX, sent, dst=target)
        return None

    def _on_pose(self, msg: LatLonHead):
        self._pose.set((msg.latitude, msg.longitude), time.monotonic())

    def _send_boat(self):
        """Once a second, once Ekko is heard: where the boat is and which map it holds. Never
        sent from a stale pose. Addressed to 0 like Ekko's own: an id its autopilot never heard
        is not forwarded."""
        if not self._ekko.heard():
            return
        pose = self._pose.get(time.monotonic())
        why = "no fresh /crsd/pose" if pose is None else \
            self._send_tunnel(0, tl.PAYLOAD_BOAT, self._ekko.boat_payload(*pose))
        if self._once("boat", why):
            if why is None:
                self.get_logger().info("sending the BOAT packet to Ekko (%s)" % self._ekko.summary())
            else:
                self.get_logger().warn("NOT sending the BOAT packet: %s" % why)

    def _on_send(self, msg: RadioSend):
        payload = bytes(msg.payload)
        if len(payload) > tl.MAX_PAYLOAD:
            self.get_logger().error("radio/send: %d bytes is over %d: dropped" % (
                len(payload), tl.MAX_PAYLOAD))
            return
        why = self._send_tunnel(msg.target_system, msg.payload_type, payload)
        if why is not None:
            self.get_logger().warn("radio/send not sent: %s" % why, throttle_duration_sec=2.0)

    def _send_test_cb(self, request, response):
        """One text TUNNEL to Ekko for the ground station's Radio tab; nobody acts on it."""
        text = "test %d from crusader" % (self._test_n + 1)
        why = self._send_tunnel(self.p["uav_sysid"], tl.PAYLOAD_TEST, tl.pack_test(text))
        if why is None:
            self._test_n += 1
        response.success = why is None
        response.message = ("sent %r to system %d" % (text, self.p["uav_sysid"]) if why is None
                            else "not sent: %s" % why)
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------ the record

    def _record(self, direction, msg, dst=None):
        """One RadioFrame for a frame that crossed the radio. Never raises."""
        try:
            name, ptype, summary = describe(msg)
            m = RadioFrame()
            m.header.stamp = self.get_clock().now().to_msg()
            m.direction = direction
            m.src_system = int(msg.get_srcSystem()) & 0xFF
            m.src_component = int(msg.get_srcComponent()) & 0xFF
            m.dst_system = int(getattr(msg, "target_system", 0) if dst is None else dst) & 0xFF
            m.msg_name = name
            m.payload_type = int(ptype) & 0xFFFF
            m.summary = summary
            buf = msg.get_msgbuf()
            m.frame_bytes = min(len(buf) if buf else 0, 0xFFFF)
            self.radio_pub.publish(m)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn("could not record a frame: %s" % e, throttle_duration_sec=5.0)

    def _quiet_check(self):
        quiet_s = self.get_parameter("quiet_warn_s").value
        if self._quiet_warned:
            return
        if self._last_rx is None or time.monotonic() - self._last_rx > quiet_s:
            self._quiet_warned = True
            self.get_logger().warn("nothing from Ekko (system %d) for %.0fs: not republishing "
                                   "the last plan" % (self.p["uav_sysid"], quiet_s))

    def destroy_node(self):
        self._stop.set()
        if self._rx.is_alive():
            self._rx.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    run_node(RfdLinkNode, args=args)


if __name__ == "__main__":
    main()
