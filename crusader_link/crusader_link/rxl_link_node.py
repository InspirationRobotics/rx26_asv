"""rxl_link_node — the boat's end of the UAV link.

    crsd/passage_plan    crusader_msgs/PassagePlan   on arrival, ~0.2 Hz
    crsd/next_gate       crusader_msgs/GatePair      on arrival
    crsd/gate_reached    std_msgs/UInt8   (IN)  -> RXL_USV_REACHED_GATE on the air

THIS IS NOT A SECOND AUTOPILOT GATEWAY. It speaks MAVLink, but to the RADIO,
never to the Pixhawk: a different endpoint, a different dialect, and a message
set with no command in it. It cannot arm, set a mode, or move the boat, and it
must never gain the ability -- crusader_fcu/telemetry_bridge stays the single
path to the autopilot. The two never share a socket.

WHAT IT DOES NOT DO. It does not act on anything it receives, and it does not
interpret the passage. A plan arrives, it becomes a ROS message, and that is
the end of this node's opinion. Deciding what to do about a gate is the
behaviour tree's job; putting any of that here would put mission logic on the
far side of a lossy radio from the thing that ticks it.

SILENCE IS THE FAILURE MODE, ON PURPOSE. When the link dies this node publishes
NOTHING -- it never repeats the last plan to keep a topic alive. Downstream ages
what it last heard and stops the boat; a republished stale plan would defeat
that completely, and the boat would keep driving a passage nobody can still
confirm. Same rule as the StreamCaches in telemetry_bridge: a blank is a fact, a
stale number wearing a fresh timestamp is a guess.
"""
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

from crusader_msgs.msg import GatePair, PassageBuoy, PassagePlan

from crusader_link import rxl_codec


class RxlLinkNode(Node):

    def __init__(self):
        super().__init__("rxl_link_node")

        # udpin: BIND and listen. Not "udp:", which pymavlink resolves to an
        # outbound client that connects nowhere and silently receives nothing.
        #
        # 14555 and the boat's port map: 14551 belongs to telemetry_bridge and
        # is never to be taken -- a udpin bind STEALS datagrams and the
        # displaced consumer sees silence, not an error. 14550 stays free for
        # ad-hoc tooling, and SITL's rover MAVProxy fans out to 14552.
        self.declare_parameter("rxl_endpoint", "udpin:127.0.0.1:14555")
        self.declare_parameter("source_system", 42)
        self.declare_parameter("plan_topic", "crsd/passage_plan")
        self.declare_parameter("gate_topic", "crsd/next_gate")
        self.declare_parameter("gate_reached_topic", "crsd/gate_reached")
        self.declare_parameter("quiet_warn_s", 15.0)

        p = self.get_parameter
        endpoint = p("rxl_endpoint").value

        self.plan_pub = self.create_publisher(PassagePlan, p("plan_topic").value, 10)
        self.gate_pub = self.create_publisher(GatePair, p("gate_topic").value, 10)
        self.create_subscription(
            UInt8, p("gate_reached_topic").value, self._on_gate_reached, 10)

        # pymavlink connections are NOT thread-safe and this one is used from
        # two threads: the RX loop below, and the gate_reached callback on the
        # executor thread. One lock around every use of it.
        self._conn_lock = threading.Lock()
        self._stop = threading.Event()
        self._plan_version = 0
        self._last_rx = None
        self._warned_quiet = False

        self.conn = rxl_codec.connect(endpoint, p("source_system").value)
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx.start()
        self.create_timer(5.0, self._quiet_check)

        self.get_logger().info(
            "rxl_link listening on %s as system %d -> %s, %s; replies "
            "RXL_USV_REACHED_GATE to whoever sent the last frame"
            % (endpoint, p("source_system").value,
               p("plan_topic").value, p("gate_topic").value))

    # ------------------------------------------------------------------ RX

    def _rx_loop(self):
        """Read the radio forever. One wire frame in, at most one ROS message out.

        Published straight from this thread rather than cached for a timer: a
        plan is an EVENT at 0.2 Hz, not a stream to resample, and a timer that
        republished the last one would be exactly the stale-republish this node
        exists to avoid. rclpy publishers are safe to call from here.
        """
        while not self._stop.is_set() and rclpy.ok():
            try:
                with self._conn_lock:
                    msg = self.conn.recv_match(blocking=False)
                if msg is None:
                    self._stop.wait(0.02)
                    continue
                decoded = rxl_codec.decode(msg)
            except Exception as exc:                     # noqa: BLE001
                # A radio delivers corruption as confidently as data. One bad
                # frame must not take the link down with it.
                self.get_logger().warn("dropped a frame: %s" % exc)
                continue

            if decoded is None:
                continue                                 # someone else's traffic
            self._last_rx = self.get_clock().now()
            self._warned_quiet = False

            if decoded["msg"] == "SAFE_PASSAGE":
                self._publish_plan(decoded)
            elif decoded["msg"] == "NEXT_BUOY_SET":
                self._publish_gate(decoded)

    def _publish_plan(self, d):
        m = PassagePlan()
        # RECEIPT time, not the aircraft's clock: the two vehicles share no
        # time base, so a UAV stamp cannot be aged against the boat's clock.
        m.header.stamp = self.get_clock().now().to_msg()
        m.entry_latitude, m.entry_longitude = d["entry"]
        m.exit_latitude, m.exit_longitude = d["exit"]

        # The version is counted HERE, by arrivals, because RXL_SAFE_PASSAGE
        # carries no version field of its own. Every received plan supersedes
        # the last one, which is exactly what the tree needs to react to.
        self._plan_version += 1
        m.plan_version = self._plan_version

        for b in d["buoys"]:
            pb = PassageBuoy()
            pb.id = b["id"]
            pb.latitude, pb.longitude = b["lat"], b["lon"]
            pb.beacon = b["beacon"]
            m.buoys.append(pb)
        self.plan_pub.publish(m)

        if d.get("truncated"):
            self.get_logger().warn(
                "a plan claimed more than %d buoys; took the first %d"
                % (rxl_codec.MAX_BUOYS, rxl_codec.MAX_BUOYS))
        self.get_logger().info(
            "plan v%d: %d buoys, entry %.7f %.7f, exit %.7f %.7f"
            % (m.plan_version, len(m.buoys), m.entry_latitude,
               m.entry_longitude, m.exit_latitude, m.exit_longitude))

    def _publish_gate(self, d):
        m = GatePair()
        m.header.stamp = self.get_clock().now().to_msg()
        m.gate_seq = d["gate_seq"]
        m.red_id = d["red_id"]
        m.green_id = d["green_id"]
        self.gate_pub.publish(m)

        done = (d["red_id"] == rxl_codec.NO_BUOY and
                d["green_id"] == rxl_codec.NO_BUOY)
        self.get_logger().info(
            "gate %d: %s" % (m.gate_seq, "PASSAGE COMPLETE (255/255)" if done
                             else "red %d, green %d" % (m.red_id, m.green_id)))

    # ------------------------------------------------------------------ TX

    def _have_peer(self):
        """Is there anywhere to send to yet?

        Two cases, and getting this wrong costs the whole reverse leg:

        * udpin: pymavlink builds a mavudp with udp_server True, whose write()
          fans out to `clients` -- the set of addresses it has HEARD from.
          `last_address` is never set on a server socket, so testing it means
          the boat silently refuses to transmit for the entire mission. That is
          exactly what happened on 2026-09-10 until bench_rxl_link caught it.
        * anything else (udpout, a serial radio): `last_address`, or a
          destination fixed at construction, so there is always a peer.

        Before anything has arrived there is genuinely nowhere to reply, and
        that is worth saying out loud rather than letting pymavlink swallow the
        socket error -- at that point the boat has not heard a plan either, so
        the tree should not have been transiting in the first place.
        """
        conn = self.conn
        if getattr(conn, "udp_server", False):
            return bool(getattr(conn, "clients", None))
        if getattr(conn, "last_address", None) is not None:
            return True
        return getattr(conn, "destination_addr", None) is not None

    def _on_gate_reached(self, msg: UInt8):
        """The boat cleared a gate. Tell the aircraft and ask for the next pair."""
        with self._conn_lock:
            reachable = self._have_peer()
            if reachable:
                try:
                    rxl_codec.send_usv_reached_gate(self.conn, msg.data)
                except Exception as exc:                 # noqa: BLE001
                    self.get_logger().error(
                        "could not send gate %d: %s" % (msg.data, exc))
                    return
        if reachable:
            self.get_logger().info("sent gate_reached(%d)" % msg.data)
        else:
            self.get_logger().warn(
                "gate_reached(%d) NOT sent: nothing has been received on this "
                "link yet, so there is no address to reply to" % msg.data)

    # --------------------------------------------------------------- health

    def _quiet_check(self):
        """Say once when the link goes quiet. Publish nothing either way."""
        quiet_s = self.get_parameter("quiet_warn_s").value
        if self._last_rx is None:
            if not self._warned_quiet:
                self._warned_quiet = True
                self.get_logger().warn(
                    "nothing received on the RXL link yet -- is the UAV (or the "
                    "bench standing in for it) transmitting?")
            return
        age = (self.get_clock().now() - self._last_rx).nanoseconds / 1e9
        if age > quiet_s and not self._warned_quiet:
            self._warned_quiet = True
            self.get_logger().warn(
                "RXL link quiet for %.0fs. NOT republishing the last plan; "
                "downstream ages it out and stops the boat." % age)

    def destroy_node(self):
        self._stop.set()
        if self._rx.is_alive():
            self._rx.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = RxlLinkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
