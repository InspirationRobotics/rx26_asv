"""rxl_link_node — the boat's end of the UAV link.

    crsd/passage_plan    crusader_msgs/PassagePlan   on arrival, ~0.2 Hz (native)
                                                     or ~1 Hz (Ekko's LIGHTS)
    crsd/next_gate       crusader_msgs/GatePair      on arrival, or answered locally
    crsd/gate_reached    std_msgs/UInt8   (IN)  -> RXL_USV_REACHED_GATE on the air
    crsd/pose            crusader_msgs/LatLonHead (IN)  -> the BOAT TUNNEL, 1 Hz
    crsd/radio/traffic   crusader_msgs/RadioFrame    every frame, either way
    crsd/radio/send_test std_srvs/Trigger (IN)  -> one text TUNNEL on the air

TWO WAYS A PLAN ARRIVES, ONE PLAN OUT. The native RXL_SAFE_PASSAGE (the Fleet ICD
design, what sim_uav and the bench transmit) and Ekko's own TUNNEL format (what
the real aircraft transmits: POSITIONS + LIGHTS, decoded by ekko_map through
tunnel_link). Both become the same PassagePlan on the same topic through
_publish_plan, whichever arrived last wins, and the log says when the source
changes. Against Ekko the boat also (a) sends its BOAT packet once a second --
its position and the map it holds, which is the acknowledgement Ekko waits for --
only while the pose is fresh, and (b) answers its OWN gate checkpoints while
Ekko's LIGHTS are fresh, because Ekko never answers RXL_USV_REACHED_GATE.

THE RADIO RECORD IS A RECORD. /crsd/radio/traffic carries every frame this node
sent or heard, published after the fact for the ground station's Radio tab.
Nothing subscribes to it to make a decision, and nothing may: it exists so an
operator can see the link, including frames in formats this boat does not speak.
The test frame is the same idea pointed the other way -- one known thing on the
air, which neither vehicle acts on, to prove the path end to end.

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
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

from crusader_msgs.msg import (GatePair, LatLonHead, PassageBuoy, PassagePlan,
                               RadioFrame)

from crusader_common import config as crsd_config
from crusader_common.stream_cache import StreamCache

from crusader_link import ekko_map
from crusader_link import rxl_codec
from crusader_link import tunnel_link

#: Not in crusader_params.yaml on purpose: the Ekko link's tuning is code, so a
#: params file written before it existed still starts this node.
POSE_TOPIC = "/crsd/pose"
BOAT_PERIOD_S = 1.0              # Ekko's own cadence for the packets it sends
POSE_TIMEOUT_FALLBACK_S = 1.0    # = shared.pose_timeout_s, if the YAML is unreadable
NO_PEER = ("nothing has been received on this link yet, so there is no address "
           "to send to")

#: Where a plan came from, for the log line when it changes.
SRC_NATIVE = "native RXL_SAFE_PASSAGE"
SRC_EKKO = "Ekko's TUNNEL format (POSITIONS + LIGHTS)"

#: Fallbacks, used ONLY when crusader_params.yaml cannot be read at all (a
#: workspace with crusader_link built but no crusader_bringup installed -- a
#: bench, a test rig). The file is the source of truth everywhere else; these
#: exist so the node still starts and says why, rather than dying on import.
FALLBACKS = {
    "rxl_endpoint": "udpin:127.0.0.1:14555",
    "radio_traffic_topic": "crsd/radio/traffic",
    "uav_sysid": 1,
    "rxl_baud": 115200,
    "source_system": 42,
    "plan_topic": "crsd/passage_plan",
    "gate_topic": "crsd/next_gate",
    "gate_reached_topic": "crsd/gate_reached",
    "quiet_warn_s": 15.0,
}


class RxlLinkNode(Node):

    def __init__(self):
        super().__init__("rxl_link_node")

        # DEFAULTS COME FROM crusader_params.yaml, not from this file. The
        # endpoint and the boat's port map are explained there, at the value.
        #
        # The ground station starts a node with a plain `ros2 run`, with no
        # --params-file (see process_manager.command_for), so a value that lives
        # only in the YAML never reaches a node started from the page -- it
        # silently runs on its code defaults instead. Every other node avoids
        # that by reading the same file for its declaration defaults; this one
        # did not, which is how it could be pointed at a UDP socket in the YAML
        # and still come up on 14555 when the page started it. A launch file
        # passing parameters= still wins over these, as it should.
        defaults = self._defaults()
        for name, fallback in FALLBACKS.items():
            self.declare_parameter(name, defaults.get(name, fallback))

        p = self.get_parameter
        endpoint = p("rxl_endpoint").value

        self.plan_pub = self.create_publisher(PassagePlan, p("plan_topic").value, 10)
        self.gate_pub = self.create_publisher(GatePair, p("gate_topic").value, 10)
        self.create_subscription(
            UInt8, p("gate_reached_topic").value, self._on_gate_reached, 10)
        self.radio_pub = self.create_publisher(
            RadioFrame, p("radio_traffic_topic").value, 50)
        self.create_service(Trigger, "crsd/radio/send_test", self._send_test_cb)
        self._test_n = 0

        # Ekko's side: what it has said (ekko_map), and where the boat is. The
        # pose is a StreamCache so a stale position cannot be read back as a
        # fresh one; the BOAT packet is simply not sent without one.
        self._ekko = ekko_map.EkkoMap()
        self._pose = StreamCache(self._pose_timeout())
        self.create_subscription(LatLonHead, POSE_TOPIC, self._on_pose, 10)
        self._seen = {}                  # what _changed has last been told, for the log

        # pymavlink connections are NOT thread-safe and this one is used from
        # two threads: the RX loop below, and the gate_reached callback on the
        # executor thread. One lock around every use of it.
        self._conn_lock = threading.Lock()
        self._stop = threading.Event()
        self._plan_version = 0
        self._last_sig = None
        self._last_rx = None
        self._warned_quiet = False

        self.conn = rxl_codec.connect(endpoint, p("source_system").value,
                                      baud=p("rxl_baud").value)
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx.start()
        self.create_timer(5.0, self._quiet_check)
        self.create_timer(BOAT_PERIOD_S, self._send_boat)

        self.get_logger().info(
            "rxl_link listening on %s as system %d -> %s, %s; replies "
            "RXL_USV_REACHED_GATE to whoever sent the last frame"
            % (endpoint, p("source_system").value,
               p("plan_topic").value, p("gate_topic").value))

    def _pose_timeout(self):
        """shared.pose_timeout_s: the age at which a /crsd/pose stops being usable
        -- the same number telemetry_bridge uses, as that key's comment asks."""
        try:
            return float(crsd_config.shared_params()["pose_timeout_s"])
        except Exception:                                # noqa: BLE001
            return POSE_TIMEOUT_FALLBACK_S

    def _uav_sysid(self):
        return int(self.get_parameter("uav_sysid").value)

    def _changed(self, key, value):
        """Remember `value` under `key`. -> (what it was, whether it differs).

        For lines that must be logged when something CHANGES -- the plan's
        source, Ekko's map number, the BOAT packet starting or stopping -- and
        never once a second while it stays the same.
        """
        was = self._seen.get(key)
        self._seen[key] = value
        return was, was != value

    def _defaults(self):
        """crusader_params.yaml's values for this node, or {} with a loud say-so.

        Not fatal: a bench workspace legitimately has no crusader_bringup, and a
        link node that refuses to start there would take the bench with it. What
        is NOT acceptable is running on unexplained defaults silently.
        """
        try:
            return dict(crsd_config.node_params("rxl_link_node"))
        except Exception as exc:                         # noqa: BLE001
            self.get_logger().warn(
                "crusader_params.yaml unreadable (%s) -- falling back to this "
                "file's defaults, which may not be what this boat is wired for"
                % exc)
            return {}

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
                self._record(RadioFrame.DIR_RX, msg)
                decoded = rxl_codec.decode(msg)
            except Exception as exc:                     # noqa: BLE001
                # A radio delivers corruption as confidently as data. One bad
                # frame must not take the link down with it.
                self.get_logger().warn("dropped a frame: %s" % exc)
                continue

            if decoded is None:
                continue                                 # someone else's traffic
            if decoded["msg"] == "TUNNEL" and decoded["src"] != self._uav_sysid():
                continue                                 # a TUNNEL, but not Ekko's
            self._last_rx = self.get_clock().now()
            self._warned_quiet = False

            try:
                if decoded["msg"] == "SAFE_PASSAGE":
                    self._publish_plan(decoded, SRC_NATIVE)
                elif decoded["msg"] == "NEXT_BUOY_SET":
                    self._publish_gate(decoded)
                elif decoded["msg"] == "TUNNEL":
                    self._on_ekko(decoded)
            except Exception as exc:                     # noqa: BLE001
                self.get_logger().warn("could not act on a frame: %s" % exc)

    def _on_ekko(self, d):
        """One TUNNEL from Ekko. Every LIGHTS frame (1 Hz) re-publishes the plan.

        Re-published on LIGHTS and not on POSITIONS because LIGHTS is the
        whole-map heartbeat Ekko sends every second, which is exactly what the
        tree's 15 s PassagePlanFresh guard wants; the plan VERSION still moves
        only when the content does (_publish_plan). A map with no usable buoy
        publishes nothing -- see ekko_map.
        """
        kind = self._ekko.hear(d["payload_type"], d["payload"], time.monotonic())
        if kind != "lights":
            return
        epoch = self._ekko.epoch()
        was, new = self._changed("ekko_epoch", epoch)
        if new:
            self.get_logger().info(
                "Ekko's map is now number %s (was %s): the old map is discarded"
                % (epoch, was))
        plan = self._ekko.plan()
        _, newly_empty = self._changed("ekko_empty", plan is None)
        if plan is not None:
            self._publish_plan(plan, SRC_EKKO)
        elif newly_empty:
            self.get_logger().warn(
                "Ekko's map has no buoy with a position and a light yet (%s): "
                "publishing no plan, so the last one ages out"
                % self._ekko.summary())

    def _publish_plan(self, d, source):
        m = PassagePlan()
        # RECEIPT time, not the aircraft's clock: the two vehicles share no
        # time base, so a UAV stamp cannot be aged against the boat's clock.
        m.header.stamp = self.get_clock().now().to_msg()
        m.entry_latitude, m.entry_longitude = d["entry"]
        m.exit_latitude, m.exit_longitude = d["exit"]

        # The version is counted HERE, because RXL_SAFE_PASSAGE carries no
        # version field of its own -- but it counts CHANGES, not arrivals.
        #
        # The aircraft retransmits the whole passage on a timer (0.2 Hz) so a
        # boat that missed one still gets it. Bumping on arrival made every one
        # of those look like a re-tasking: PlanChanged fired every 5 s, halted
        # the leg, and the boat re-requested a gate it already had. Seen in
        # SITL on 2026-09-13 as "gate 3 asked again" on a run where nothing
        # about the passage had changed.
        sig = self._signature(d)
        changed = sig != self._last_sig
        if changed:
            self._last_sig = sig
            self._plan_version += 1
        m.plan_version = self._plan_version

        # Whichever source spoke last is the plan; say when that changes, so a
        # second transmitter on the air is visible rather than a version that
        # bumps for no reason anyone can see.
        was, new = self._changed("plan_source", source)
        if new:
            self.get_logger().info(
                "passage plan now comes from %s (was %s)" % (source, was or "nothing"))

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
        # Only on CHANGE. The same line every 5 s reads as a stream of new
        # plans; the retransmissions are the link working, not news.
        if changed:
            self.get_logger().info(
                "plan v%d: %d buoys, entry %.7f %.7f, exit %.7f %.7f"
                % (m.plan_version, len(m.buoys), m.entry_latitude,
                   m.entry_longitude, m.exit_latitude, m.exit_longitude))

    @staticmethod
    def _signature(d):
        """What makes two plans the SAME passage.

        Positions are rounded to 1e-7 deg (~1 cm) before comparing: they arrive
        as degE7 integers so an unchanged plan is bit-identical, and rounding
        here only guards a future sender that recomputes them in floating point.
        """
        def ll(t):
            return (round(t[0], 7), round(t[1], 7))
        return (ll(d["entry"]), ll(d["exit"]),
                tuple((b["id"], round(b["lat"], 7), round(b["lon"], 7), b["beacon"])
                      for b in d["buoys"]))

    def _publish_gate(self, d, how="confirmed by the aircraft"):
        m = GatePair()
        m.header.stamp = self.get_clock().now().to_msg()
        m.gate_seq = d["gate_seq"]
        m.red_id = d["red_id"]
        m.green_id = d["green_id"]
        self.gate_pub.publish(m)

        # 255/255 is the NORMAL ack now. It used to mean "passage complete",
        # back when the aircraft assigned gates and the boat waited to be told
        # it was finished; the boat pairs its own buoys and counts its own gates
        # today, so these two fields carry nothing and are sent as NO_BUOY.
        # A pair that IS filled in means an aircraft on older software.
        stale = (d["red_id"] != rxl_codec.NO_BUOY or
                 d["green_id"] != rxl_codec.NO_BUOY)
        if stale:
            self.get_logger().warning(
                "checkpoint %d confirmed, but the aircraft also sent a pair "
                "(red %d, green %d). The boat ignores it and plans its own."
                % (m.gate_seq, m.red_id, m.green_id))
        else:
            self.get_logger().info("checkpoint %d: %s" % (m.gate_seq, how))

    # ------------------------------------------------------------------ TX

    def _have_peer(self):
        """Is there anywhere to send to yet? See rxl_codec.has_peer.

        On a udpin socket, before anything has arrived, there is genuinely
        nowhere to reply, and that is worth saying out loud rather than letting
        pymavlink swallow the socket error -- at that point the boat has not
        heard a plan either, so the tree should not have been transiting in the
        first place. On a serial radio there is always somewhere to send, which
        the old check here got wrong.
        """
        return rxl_codec.has_peer(self.conn)

    def _transmit(self, dst, send):
        """Put one frame on the air and record it. `send(conn)` does the sending
        and returns the sent message. -> None when sent, else why not: NO_PEER, or
        the exception's text. Never raises.

        The one place the boat transmits, so the has-anyone-been-heard rule, the
        connection lock and the Radio-tab record are never written three ways.
        """
        with self._conn_lock:
            if not self._have_peer():
                return NO_PEER
            try:
                sent = send(self.conn)
            except Exception as exc:                     # noqa: BLE001
                return str(exc) or type(exc).__name__
        # Outside the lock: recording a frame must never delay the next one.
        self._record(RadioFrame.DIR_TX, sent, dst=dst)
        return None

    def _send_tunnel(self, dst, payload_type, payload):
        return self._transmit(dst, lambda conn: rxl_codec.send_tunnel(
            conn, dst, payload_type, payload))

    def _on_pose(self, msg: LatLonHead):
        self._pose.set((msg.latitude, msg.longitude), time.monotonic())

    def _send_boat(self):
        """Once a second, tell Ekko where the boat is and which map it holds.

        That packet is the acknowledgement Ekko re-sends positions until it
        gets. Three things stop it, each a fact and not a guess:
          * Ekko has not been heard yet -- nobody to tell, and the boat does not
            talk first (the same rule as the test frame);
          * no fresh pose -- a position is never back-filled from the last one
            seen, the packet is just not sent;
          * the link has no peer address yet (_send_tunnel).
        Addressed to everyone (system 0), like Ekko's own packets: a frame to a
        system id the aircraft's autopilot has never heard is not forwarded.
        """
        pose = self._pose.get(time.monotonic())
        if not self._ekko.heard():
            return
        if pose is None:
            self._note_boat_sending(False, "no fresh %s" % POSE_TOPIC)
            return
        why = self._send_tunnel(0, tunnel_link.PAYLOAD_BOAT,
                                self._ekko.boat_payload(*pose))
        if why is None:
            self._note_boat_sending(True, "")
        elif why != NO_PEER:
            self.get_logger().warn("could not send the BOAT packet: %s" % why)

    def _note_boat_sending(self, sending, why):
        """Log only when the BOAT packet starts or stops, never once a second."""
        if not self._changed("boat_sending", sending)[1]:
            return
        if sending:
            self.get_logger().info(
                "sending the BOAT packet to Ekko at %g Hz (%s)"
                % (1.0 / BOAT_PERIOD_S, self._ekko.summary()))
        else:
            self.get_logger().warn(
                "NOT sending the BOAT packet: %s. Ekko keeps re-sending "
                "positions until it gets one." % why)

    def _on_gate_reached(self, msg: UInt8):
        """The boat cleared a gate. Tell the aircraft and ask for the next pair.

        Ekko never answers RXL_USV_REACHED_GATE -- it has no such message -- so
        while its LIGHTS are fresh the boat answers its own checkpoint, as if
        NEXT_BUOY_SET(seq, 255, 255) had arrived. The native request is still
        sent: an aircraft that does speak RXL answers it as before.
        """
        why = self._transmit(self._uav_sysid(), lambda conn:
                             rxl_codec.send_usv_reached_gate(conn, msg.data))
        if why is None:
            self.get_logger().info("sent gate_reached(%d)" % msg.data)
        elif why == NO_PEER:
            self.get_logger().warn(
                "gate_reached(%d) NOT sent: %s" % (msg.data, NO_PEER))
        else:
            self.get_logger().error(
                "could not send checkpoint %d: %s" % (msg.data, why))

        if self._ekko.lights_fresh(time.monotonic()):
            self._publish_gate(
                {"gate_seq": msg.data, "red_id": rxl_codec.NO_BUOY,
                 "green_id": rxl_codec.NO_BUOY},
                how="answered by the boat itself (Ekko's LIGHTS are under %gs "
                    "old and it never answers a checkpoint)"
                    % ekko_map.LIGHTS_FRESH_S)

    # ---------------------------------------------------- the radio, recorded

    def _record(self, direction, msg, dst=None):
        """Publish one RadioFrame for a frame that crossed the radio.

        NEVER RAISES ON THE RX PATH. This is called from the receive loop, where
        an exception would be reported as "dropped a frame" and blame the radio
        for a bug in the record of it.

        Our own frames are recorded as sent, not heard: on a mesh that echoes,
        a transmission arriving back would otherwise show up twice and inflate
        every count the tab draws.
        """
        try:
            src = msg.get_srcSystem()
            if direction == RadioFrame.DIR_RX and src == self.conn.mav.srcSystem:
                return
            name, ptype, summary = rxl_codec.describe(msg)
            m = RadioFrame()
            m.header.stamp = self.get_clock().now().to_msg()
            m.direction = direction
            m.src_system = int(src) & 0xFF
            m.src_component = int(msg.get_srcComponent()) & 0xFF
            if dst is None:
                dst = getattr(msg, "target_system", 0)
            m.dst_system = int(dst) & 0xFF
            m.msg_name = name
            m.payload_type = int(ptype) & 0xFFFF
            m.summary = summary
            m.frame_bytes = min(len(msg.get_msgbuf()), 0xFFFF)
            self.radio_pub.publish(m)
        except Exception as exc:                         # noqa: BLE001
            self.get_logger().warn("could not record a frame: %s" % exc)

    def _send_test_cb(self, request, response):
        """Put one text TUNNEL on the air, addressed to the aircraft.

        For the Radio tab's button. Neither vehicle acts on it: the point is a
        known frame an operator can watch arrive, on Ekko's Radio tab, in QGC's
        MAVLink Inspector, or with tools/rfd-tests/check_mesh.py on a laptop
        radio. It is refused rather than faked when there is nowhere to send --
        see _have_peer.
        """
        target = self._uav_sysid()
        text = "test %d from crusader" % (self._test_n + 1)
        why = self._send_tunnel(target, tunnel_link.PAYLOAD_TEST,
                                tunnel_link.pack_test(text))
        if why != NO_PEER:
            self._test_n += 1                        # nothing went out otherwise: no number used
        if why is None:
            response.success = True
            response.message = "sent %r to system %d" % (text, target)
            self.get_logger().info(response.message)
            return response
        response.success = False
        response.message = ("refused: %s" % why if why == NO_PEER
                            else "could not send the test frame: %s" % why)
        if why == NO_PEER:
            self.get_logger().warn(response.message)
        else:
            self.get_logger().error(response.message)
        return response

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
