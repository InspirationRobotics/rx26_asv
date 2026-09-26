"""ros_adapter — squirt_cal's inputs and its one output, over ROS 2.

In:   /crsd/wall_range (WallRange)      the range the shots are logged against
      /crsd/attitude   (Attitude)       rocking, for "steady"
      /crsd/rc_channels (RcChannels)    the sticks (gentleness) and, as a
                                        fallback, the pilot's pump switch
      /crsd/pump_state (PumpState)      the pump output: pilot-fired shots, and
                                        what the bridge did with ours
      /crsd/fcu_status, /crsd/pose      logged with each shot
      /crsd/current_task (latched)      no firing while a mission runs
Out:  /crsd/pump_cmd   (PumpCommand)    a burst REQUEST; telemetry_bridge decides

This process never opens MAVLink (README rule 1) and cannot hold the pump on:
a PumpCommand is a bounded burst the autopilot itself ends.
"""
import math
import os
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from crusader_msgs.msg import (Attitude, FcuStatus, LatLonHead, PumpCommand,
                               PumpState, RcChannels, WallRange)

PUMP_STATE_TIMEOUT_S = 1.0
RESULT_WORDS = {0: "none", 1: "sent", 2: "ACCEPTED", 3: "REJECTED by the autopilot",
                4: "REFUSED by the bridge"}

try:
    from crusader_groundstation.recorder import MjpegPuller
except ImportError:                     # no ground station on this path: no snapshots
    MjpegPuller = None


if MjpegPuller is not None:
    class LatestFramePuller(MjpegPuller):
        """Keeps the newest JPEG in memory instead of writing every frame.

        Stays attached for the whole session: the camera producers only encode
        while a viewer is attached, and the first frame a new viewer gets can
        be stale (crusader_common/mjpeg_view.py), so attaching per shot would
        photograph the moment BEFORE the water."""

        def __init__(self, url):
            super().__init__(url, tempfile.gettempdir(), rate_hz=15.0)
            self._latest = None
            self._lk = threading.Lock()

        def _save(self, frame):
            with self._lk:
                self._latest = (time.monotonic(), frame)

        def save_latest(self, path, max_age_s=1.0):
            with self._lk:
                latest = self._latest
            if latest is None or time.monotonic() - latest[0] > max_age_s:
                return False
            try:
                with open(path, "wb") as f:
                    f.write(latest[1])
                return True
            except OSError:
                return False


class RosAdapter:

    def __init__(self, cfg):
        self.cfg = cfg
        self.app = None
        rclpy.init()
        self.node = Node("squirt_cal")
        n = self.node
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pump_pub = n.create_publisher(PumpCommand, "/crsd/pump_cmd", 10)
        n.create_subscription(WallRange, cfg["wall_range_topic"], self._on_wall, 10)
        n.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)
        n.create_subscription(RcChannels, "/crsd/rc_channels", self._on_rc, 10)
        n.create_subscription(PumpState, "/crsd/pump_state", self._on_pump, 10)
        n.create_subscription(FcuStatus, "/crsd/fcu_status", self._on_fcu, 10)
        n.create_subscription(LatLonHead, "/crsd/pose", self._on_pose, 10)
        n.create_subscription(String, "/crsd/current_task", self._on_task, latched)
        self._pump = None                # (t, PumpState)
        self._pump_on = False            # last output state seen on /crsd/pump_state
        self._rc_on = False              # last pilot-switch state seen on RC
        self._wall_t = None
        self.puller = None
        if cfg["snapshot_url"] and MjpegPuller is not None:
            self.puller = LatestFramePuller(cfg["snapshot_url"])
            self.puller.start()
        n.get_logger().info(
            f"squirt_cal: wall range on {cfg['wall_range_topic']}, pump requests on "
            f"/crsd/pump_cmd, snapshots {'from ' + cfg['snapshot_url'] if self.puller else 'off'}")

    # ---- the App's side ----

    def attach(self, app):
        self.app = app

    def spin(self, stop):
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def stop(self):
        if self.puller is not None:
            self.puller.stop()
        try:
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

    def poll(self, now):
        pass                              # everything arrives by callback

    def _pump_fresh(self):
        if self._pump is None or time.monotonic() - self._pump[0] > PUMP_STATE_TIMEOUT_S:
            return None
        return self._pump[1]

    def info(self):
        ps = self._pump_fresh()
        if ps is None:
            line = "bridge: no /crsd/pump_state (log only)"
        else:
            line = (f"bridge: pump {'ON' if ps.on else 'off'} ({ps.output_pwm} us), "
                    f"last #{ps.last_seq} {RESULT_WORDS.get(ps.last_result, '?')}"
                    + (f": {ps.last_reason}" if ps.last_reason else ""))
        return dict(kind="ros", status_line=line)

    def can_fire(self):
        ps = self._pump_fresh()
        if ps is None:
            return False, ("no /crsd/pump_state: is telemetry_bridge running with a "
                           "pump path? Log only: flick the pump switch")
        if not ps.enabled:
            return False, f"the bridge's pump path is off: {ps.last_reason or 'pump_servo_channel 0'}"
        if not ps.output_fresh:
            return False, "the bridge cannot see the pump output (SERVO_OUTPUT_RAW)"
        return True, "ok"

    def fire(self, burst_s, seq):
        m = PumpCommand()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.duration_s = float(burst_s)
        m.seq = int(seq)
        m.source = "squirt_cal"
        self.pump_pub.publish(m)
        return True, f"burst {burst_s:.2f} s asked of the bridge (#{seq})"

    def snapshot(self, path):
        return bool(self.puller and self.puller.save_latest(path))

    def fake_state(self, target):
        return None

    def fake_action(self, path, payload, app):
        return dict(ok=False, message="not a fake boat")

    # ---- callbacks (the executor thread); App methods take its own lock ----

    def _on_wall(self, msg):
        t = time.monotonic()
        self._wall_t = t
        lat = msg.lat_m if math.isfinite(msg.lat_m) else None
        self.app.on_range(t, msg.valid, msg.range_m if msg.valid else None,
                          msg.angle_deg if msg.valid else None, lat)

    def _on_att(self, msg):
        d = math.degrees
        self.app.on_att(time.monotonic(), d(msg.roll), d(msg.pitch),
                        d(msg.rollspeed), d(msg.pitchspeed))

    def _on_rc(self, msg):
        t = time.monotonic()
        ch = list(msg.channels)
        self.app.on_sticks(t, ch[0:4])
        # The pilot's switch, used only when the bridge is not reporting the
        # pump output (log-only, before the pump path exists).
        i = self.cfg["pump_rc_channel"] - 1
        on = 0 <= i < len(ch) and ch[i] >= self.cfg["pump_on_threshold_us"]
        if self._pump_fresh() is None and on != self._rc_on:
            self.app.on_pump_edge(t, on)
        self._rc_on = on

    def _on_pump(self, msg):
        t = time.monotonic()
        self._pump = (t, msg)
        if bool(msg.on) != self._pump_on:
            self._pump_on = bool(msg.on)
            self.app.on_pump_edge(t, self._pump_on)

    def _on_fcu(self, msg):
        self.app.on_fcu(msg.mode, bool(msg.armed))

    def _on_pose(self, msg):
        self.app.on_pose(msg.latitude, msg.longitude,
                         None if math.isnan(msg.heading) else msg.heading)

    def _on_task(self, msg):
        self.app.on_mission(msg.data not in ("", "TASK_NONE"))
