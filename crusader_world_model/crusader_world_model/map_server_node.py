"""map_server_node — the boat and its tracked targets, live in a laptop browser.

NOTE: unverified on the boat. Bench-driven only, via
tools/bench/bench_world_model.py. It is deliberately NOT in core.launch.py.

    ros2 run crusader_world_model map_server
    # then, on the laptop:  http://<JETSON_IP>:8082

Subscribes (read-only; this node commands NOTHING and publishes NOTHING):
  /crsd/pose        crusader_msgs/LatLonHead        — position, heading, speed
  /crsd/attitude    crusader_msgs/Attitude          — roll/pitch, for the readout
  /crsd/fcu_status  crusader_msgs/FcuStatus         — mode + armed
  crsd/world_targets crusader_msgs/TrackedTargetArray — what to draw

Port 8082, NOT 8080 or 8081: buoy_detector's annotated view and tools/oak_view.py
already contend for 8080, and tools/lidar_view.py takes 8081. Two servers cannot
bind one port, and all three are useful at the same time — the camera view, the
LiDAR cloud, and the map are three different questions about one moment.

WHAT THIS NODE IS ALLOWED TO DO. It reads topics and renders. It has no
publishers, no services and no timers that touch anything but its own trail
buffer, so it can be started and killed at any point in a session — including
while the boat is under way — without the stack noticing. A display that can
affect the vehicle is a display nobody dares restart when it misbehaves.

WHY IT DOES NOT REUSE THE TRACKER'S WORLD ORIGIN. The tracker anchors its world
frame at its first fix; this node anchors at its own. They will usually differ,
because the two nodes start at different moments, and a display quietly drawing
its own metres against someone else's origin would put every target a few tens
of metres off with nothing to show for it. So targets are re-projected here
from the LAT/LON they carry, through this node's origin. lat/lon is the frame
both nodes genuinely share; metres are always relative to somebody's choice.

STALENESS IS THE FEATURE. Every stream is behind a StreamCache with its own
budget, and the page is told the age of each. A dead pose does not freeze the
boat marker at its last position and keep looking healthy — the marker greys
out, the readout goes red and a banner says so. This node exists to answer
"where is the boat", and the honest answer is sometimes "I do not know".
"""
import math
import time
from collections import deque

from rclpy.node import Node

from crusader_msgs.msg import (Attitude, FcuStatus, LatLonHead,
                               TrackedTargetArray)

from crusader_common import config as crsd_config
from crusader_common import geo
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config, make_set_callback
from crusader_common.stream_cache import StreamCache

from crusader_world_model.map_server_core import MapServer

# Structural [RO]: the port and the topic cannot change under a live browser,
# and the freshness budgets are what the display's honesty rests on. The trail
# is [DYN] because it is pure presentation — how much history to draw is a
# preference, and getting it wrong costs a scrollback, not a decision.
PARAM_SPEC = {
    "port": dict(read_only=True, lo=1024, hi=65535,
                 description="HTTP port; 8080/8081 are taken (see tools/)"),
    "bind_host": dict(read_only=True,
                      description="0.0.0.0 so the laptop can reach it"),
    "targets_topic": dict(read_only=True,
                          description="TrackedTargetArray to draw"),
    "pose_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                           description="= shared.pose_timeout_s"),
    "attitude_timeout_s": dict(read_only=True, lo=0.2, hi=10.0),
    "targets_timeout_s": dict(read_only=True, lo=0.2, hi=30.0,
                              description="stale -> 'world model silent'"),
    "status_timeout_s": dict(read_only=True, lo=0.2, hi=10.0),
    "poll_period_s": dict(read_only=True, lo=0.05, hi=5.0,
                          description="how often the browser asks for /state"),
    "trail_length": dict(read_only=False, lo=0, hi=20000,
                         description="track points kept for the wake line"),
    "trail_min_move_m": dict(read_only=False, lo=0.0, hi=50.0,
                             description="min movement before a trail point is "
                                         "added; 0 = every pose"),
}


class MapServerNode(Node):
    """Subscriptions in, one JSON snapshot out, served over HTTP."""

    def __init__(self):
        super().__init__("map_server")
        p = declare_from_config(self, crsd_config.node_params("map_server"),
                                PARAM_SPEC)
        self.p = p

        ranges = {n: (s["lo"], s["hi"]) for n, s in PARAM_SPEC.items()
                  if not s.get("read_only") and "lo" in s}
        self.add_on_set_parameters_callback(
            make_set_callback(self, ranges, self._apply))

        self._pose = StreamCache(p["pose_timeout_s"])
        self._att = StreamCache(p["attitude_timeout_s"])
        self._status = StreamCache(p["status_timeout_s"])
        self._targets = StreamCache(p["targets_timeout_s"])

        # This node's own display origin — see the module docstring on why it
        # is not the tracker's. First valid fix, held for the process lifetime.
        self._origin = None
        self._trail = deque(maxlen=int(p["trail_length"]) or 1)

        self.create_subscription(LatLonHead, "/crsd/pose", self._on_pose, 10)
        self.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)
        self.create_subscription(FcuStatus, "/crsd/fcu_status",
                                 self._on_status, 10)
        self.create_subscription(TrackedTargetArray, p["targets_topic"],
                                 self._on_targets, 10)

        self._server = MapServer(self._snapshot, self._clear_trail,
                                 poll_ms=p["poll_period_s"] * 1000.0)
        self._server.start(int(p["port"]), p["bind_host"])
        self.get_logger().info(
            f"map on http://<JETSON_IP>:{int(p['port'])} — vessel state and "
            f"'{p['targets_topic']}'. Open it from the laptop; nothing needs "
            "to be installed there.")

    def _apply(self, changes):
        """Apply validated dynamic params. Only the trail is dynamic."""
        if "trail_length" in changes:
            # A deque's maxlen is immutable, so resizing means rebuilding —
            # done here rather than pretending the set took effect.
            self._trail = deque(self._trail,
                                maxlen=int(changes["trail_length"]) or 1)
        self.p.update(changes)

    # ---------- inputs ----------

    def _on_pose(self, msg: LatLonHead):
        """Cache the fix and extend the trail.

        A NaN heading (GPS yaw unresolved) is NOT cached: this node's whole job
        is to show where the boat is and which way it is pointing, and half of
        that being unavailable is exactly the case the stale banner is for.
        Caching it with a substituted heading would draw an arrow pointing at
        a number nobody measured.
        """
        if math.isnan(msg.heading):
            return
        if self._origin is None:
            self._origin = (msg.latitude, msg.longitude)
        x, y = geo.latlon_to_xy(msg.latitude, msg.longitude, self._origin)
        self._pose.set((msg.latitude, msg.longitude, msg.heading,
                        msg.ground_speed, x, y), time.monotonic())

        # Distance-gated rather than time-gated: a boat holding station should
        # not spend the whole trail buffer on one spot, and a boat at 3 m/s
        # should not have its wake sampled coarsely just because the pose rate
        # happens to be 20 Hz.
        gate = self.p["trail_min_move_m"]
        if not self._trail or math.hypot(x - self._trail[-1][0],
                                         y - self._trail[-1][1]) >= gate:
            self._trail.append((x, y))

    def _on_att(self, msg: Attitude):
        self._att.set((msg.roll, msg.pitch), time.monotonic())

    def _on_status(self, msg: FcuStatus):
        self._status.set((msg.mode, msg.armed), time.monotonic())

    def _on_targets(self, msg: TrackedTargetArray):
        self._targets.set(msg, time.monotonic())

    def _clear_trail(self):
        self._trail.clear()

    # ---------- the snapshot ----------

    def _snapshot(self):
        """Everything the page draws, as one plain dict.

        Called from an HTTP thread, not the executor. It only READS the caches
        and the trail deque, both of which are written by executor callbacks —
        in CPython those individual reads are atomic, and the worst a race can
        produce here is one frame drawn from a pose and a target list a few
        milliseconds apart, which is a display artefact and not a correctness
        problem. A lock around it would put the ROS callbacks behind every
        browser poll, which is a real cost for no gain.

        Every stream reports `ok` (fresh) and `age` separately, so the page can
        distinguish "never arrived" from "stopped arriving" — the two look
        identical if you only publish the value.
        """
        now = time.monotonic()
        pose = self._pose.get(now)
        att = self._att.get(now)
        status = self._status.get(now)
        targets = self._targets.get(now)

        boat = {"ok": pose is not None,
                "stale": pose is None,
                "age": _round(self._pose.age(now)),
                "lat": None, "lon": None, "heading": None, "speed": None,
                "x": 0.0, "y": 0.0,
                "att_ok": att is not None,
                "att_age": _round(self._att.age(now)),
                "roll": None, "pitch": None}
        if pose is not None:
            lat, lon, hdg, spd, x, y = pose
            boat.update(lat=lat, lon=lon, heading=hdg, speed=spd, x=x, y=y)
        if att is not None:
            boat.update(roll=math.degrees(att[0]), pitch=math.degrees(att[1]))

        return {
            "boat": boat,
            "fcu": {"ok": status is not None,
                    "age": _round(self._status.age(now)),
                    "mode": status[0] if status else None,
                    "armed": bool(status[1]) if status else False},
            "targets": {"ok": targets is not None,
                        "age": _round(self._targets.age(now)),
                        "items": self._target_items(targets)},
            "trail": [[round(x, 2), round(y, 2)] for x, y in self._trail],
        }

    def _target_items(self, msg):
        """Targets re-projected into THIS node's display frame.

        Re-derived from each target's lat/lon rather than copied from its x/y,
        because those metres are relative to the tracker's origin and this map
        is drawn against its own. See the module docstring.
        """
        if msg is None or self._origin is None:
            return []
        items = []
        for t in msg.targets:
            x, y = geo.latlon_to_xy(t.latitude, t.longitude, self._origin)
            items.append({
                "id": int(t.id),
                "label": t.label,
                "conf": round(float(t.confidence), 2),
                "lat": t.latitude, "lon": t.longitude,
                "x": round(x, 2), "y": round(y, 2),
                "range": round(t.range, 2),
                "bearing": round(t.bearing, 1),
                "stddev": round(t.position_stddev, 2),
                "speed": round(math.hypot(t.velocity_east, t.velocity_north), 2),
                "hits": int(t.hits),
                "unseen": round(t.time_since_seen, 1),
                "confirmed": bool(t.confirmed),
                "sources": int(t.sources),
            })
        return items

    # ---------- teardown ----------

    def destroy_node(self):
        self._server.stop()
        super().destroy_node()


def _round(v, n=2):
    """None-tolerant round — StreamCache.age() is None before the first value,
    and JSON needs that distinction preserved rather than turned into 0.0."""
    return None if v is None else round(v, n)


def main(args=None):
    run_node(MapServerNode, args=args)


if __name__ == "__main__":
    main()
