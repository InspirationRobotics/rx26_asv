"""target_tracker_node — camera (+ LiDAR) + pose in, earth-anchored targets out.

NOTE: unverified on the boat. Exercised only against
tools/bench/bench_world_model.py's invented detections. It is deliberately NOT
in core.launch.py, and it commands nothing directly — but it is no longer
publishing into the void: crusader_bt's bt_runner_node subscribes
/crsd/world_targets and fills the behaviour tree's buoy field from it, taking
CONFIRMED tracks only. That is why a confirmed phantom here is a mission
problem and not just an untidy map.

Subscribes:
  oak/detections       crusader_msgs/Detection3DArray  — camera_link
  crsd/lidar_clusters  crusader_msgs/Cluster3DArray    — base_link, and
                       COUNTED BUT DISCARDED unless use_lidar (see below)
  /crsd/pose           crusader_msgs/LatLonHead        — position + heading
  /crsd/attitude       crusader_msgs/Attitude          — roll/pitch/yaw
Publishes:
  crsd/world_targets       crusader_msgs/TrackedTargetArray  (EVERY tick)
  crsd/world_model_health  std_msgs/String (JSON)            — where sightings went

All the geometry and all the state live in target_tracker_core, which has no
ROS imports and runs on a laptop. This file is I/O, parameters and freshness,
mirroring lidar_cluster_node.py.

CAMERA-ONLY BY DEFAULT. `use_lidar` is False, so a cluster reaches the health
counters and nothing else, and every track on the map came from a camera
detection. This is not a statement about which sensor is better — it is about
what an unlabelled cluster MEANS in the water you are in. On open water the
nearest hard surface is the object you wanted; in a pool or alongside a dock it
is the wall, which clusters perfectly, arrives with no label, survives
track_unlabeled, reaches confirm_hits in three sightings and then — with
track_timeout_s at 0 — never leaves the map. crusader_params.yaml records the
measurement: at r_max 40 m that was 100+ permanent tracks at 15-28 m.

The switch is [DYN], so it flips from the ground station's Tuning tab without a
restart. Turn it on for open water and the ranges go from stereo (a few per
cent of the distance) back to time-of-flight (centimetres).

WHY A TIMER AND NOT A CALLBACK PER DETECTION. The four inputs run at four
different rates — camera detections at ~30 Hz, LiDAR clusters at ~2 Hz windows,
pose at 20 Hz, attitude at 30 Hz — and fusing needs one of each. Driving the
cycle off the camera would run fusion 15 times against the same LiDAR window,
counting one cluster as fifteen sightings and making every LiDAR-only track
look confirmed within a second. So the node ticks at a fixed rate and takes the
newest of each input, and each sensor message is CONSUMED AT MOST ONCE: a
message whose header stamp has not changed since the last tick contributes
nothing. Dropping two camera frames in three is not a loss — a track needs
sightings from different vantage points, and three frames 33 ms apart are the
same vantage point.

WHAT HAPPENS WHEN AN INPUT GOES STALE, and why the answers differ:

  pose or attitude stale  -> NO OBSERVATIONS ARE INGESTED. There is no honest
        place to put a detection when the boat's own position or orientation is
        unknown, and guessing writes targets into the map at coordinates the
        boat has already left. The existing tracks still age and still expire.
        This is the frozen-pose failure StreamCache exists to prevent, and it
        is the one case where doing nothing is the correct action.
  detections stale        -> with use_lidar, the cycle runs on LiDAR alone:
        anonymous tracks, which is what the LiDAR can honestly support. WITHOUT
        it the camera is the only input, so the cycle ingests nothing and only
        ages the tracks — and the log says so rather than claiming another
        sensor is carrying it.
  clusters stale          -> the cycle runs on the camera alone. Coarse stereo
        ranges, which is what the camera can honestly support, and exactly the
        case the package's never-drop invariant is written for. Without
        use_lidar this is not an event at all and is not logged: an input that
        is ignored by configuration cannot go stale in any sense that matters.

The array is published on EVERY tick regardless, empty or not — silence means
this node is dead, not that the water is clear.
"""
import json
import math
import time

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from crusader_msgs.msg import (Attitude, Cluster3DArray, Detection3DArray,
                               LatLonHead, TrackedTarget, TrackedTargetArray)

from crusader_common import config as crsd_config
from crusader_common import geo
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config, make_set_callback
from crusader_common.stream_cache import StreamCache

from crusader_world_model import target_tracker_core as core

# [RO] = structural: topic names, the camera extrinsic and the freshness
# budgets. The extrinsic is [RO] for the same reason the LiDAR's is — a
# live-tunable mounting geometry silently invalidates every target already on
# the map, and there is no way to tell afterwards which positions came from
# which numbers. [DYN] = the fusion and tracking gates, which are exactly what
# a bench session needs to sweep.
PARAM_SPEC = {
    "detections_topic": dict(read_only=True, description="Detection3DArray in"),
    "clusters_topic": dict(read_only=True, description="Cluster3DArray in"),
    "targets_topic": dict(read_only=True, description="TrackedTargetArray out"),
    "health_topic": dict(read_only=True, description="JSON stats out"),
    "update_rate_hz": dict(read_only=True, lo=1.0, hi=30.0,
                           description="fuse/track cycle rate"),
    # --- freshness budgets [RO] ---
    "pose_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                           description="= shared.pose_timeout_s; stale -> no "
                                       "observations ingested"),
    "attitude_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                               description="stale -> no observations ingested"),
    "detection_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                                description="stale -> LiDAR-only cycles"),
    "cluster_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                              description="stale -> camera-only cycles"),
    # --- camera extrinsic [RO]: camera_link -> base_link ---
    "cam_x": dict(read_only=True, lo=-5.0, hi=5.0, description="BODY fwd+ [m]"),
    "cam_y": dict(read_only=True, lo=-5.0, hi=5.0, description="BODY left+ [m]"),
    "cam_z": dict(read_only=True, lo=-5.0, hi=5.0, description="BODY up+ [m]"),
    "cam_yaw_deg": dict(read_only=True, lo=-180.0, hi=180.0,
                        description="mount yaw, + = aimed to PORT"),
    "cam_pitch_deg": dict(read_only=True, lo=-90.0, hi=90.0,
                          description="mount tilt, + = aimed DOWN"),
    # --- fusion gates [DYN] ---
    # use_lidar leads the group because the five below it are inert while it is
    # False. [DYN] rather than [RO] on purpose: unlike the extrinsic, flipping
    # it does not invalidate the targets already on the map — the camera-derived
    # ones stay exactly as true as they were, and the LiDAR simply starts or
    # stops contributing range to new sightings.
    "use_lidar": dict(read_only=False,
                      description="fuse LiDAR clusters. False (default) = "
                                  "camera-only: clusters are counted and "
                                  "discarded, and nothing anonymous can reach "
                                  "the map. True for open water"),
    "fuse_bearing_deg": dict(read_only=False, lo=0.5, hi=45.0,
                             description="camera/LiDAR bearing agreement"),
    "fuse_range_m": dict(read_only=False, lo=0.5, hi=20.0,
                         description="absolute range-agreement floor [m]"),
    "fuse_range_frac": dict(read_only=False, lo=0.0, hi=1.0,
                            description="... or this fraction of camera range"),
    "min_confidence": dict(read_only=False, lo=0.0, hi=1.0,
                           description="re-threshold above buoy_detector's"),
    "track_unlabeled": dict(read_only=True,
                            description="keep LiDAR-only clusters as tracks"),
    # --- tracking gates [DYN] ---
    "assoc_radius_m": dict(read_only=False, lo=0.2, hi=20.0,
                           description="world-frame association radius [m]"),
    "pos_alpha": dict(read_only=False, lo=0.01, hi=1.0,
                      description="floor on the position EMA gain"),
    "vel_alpha": dict(read_only=False, lo=0.01, hi=1.0,
                      description="velocity EMA gain"),
    "confirm_hits": dict(read_only=False, lo=1, hi=100,
                         description="sightings before CONFIRMED"),
    "tentative_timeout_s": dict(read_only=False, lo=0.5, hi=60.0,
                                description="unconfirmed tracks expire fast"),
    "track_timeout_s": dict(read_only=False, lo=0.0, hi=3600.0,
                            description="confirmed tracks are held this long; "
                                        "0 = never expire (remember the whole "
                                        "course for the run)"),
    "max_tracks": dict(read_only=False, lo=1, hi=1000,
                       description="hard cap; runaway guard, not a normal path"),
    "health_period_s": dict(read_only=False, lo=1.0, hi=60.0,
                            description="JSON stats publish period"),
}

# Params that map straight onto a TrackerParams field of the same name. Listed
# rather than inferred so that adding a node-level parameter which is NOT a
# core parameter does not silently end up inside the core.
_CORE_PARAMS = (
    "cam_x", "cam_y", "cam_z", "cam_yaw_deg", "cam_pitch_deg", "use_lidar",
    "fuse_bearing_deg", "fuse_range_m", "fuse_range_frac", "min_confidence",
    "track_unlabeled", "assoc_radius_m", "pos_alpha", "vel_alpha",
    "confirm_hits", "tentative_timeout_s", "track_timeout_s", "max_tracks")


def stamp_key(header):
    """A hashable identity for a message's header stamp.

    Used to decide whether a cached sensor message has already been fed to the
    tracker. The pair of integers is compared rather than a float conversion
    because two stamps 20 ns apart are different messages and must not collapse
    into one value at float precision.
    """
    return (header.stamp.sec, header.stamp.nanosec)


class TargetTrackerNode(Node):
    """ROS wrapper: subscriptions, freshness, a fixed-rate cycle, publishing."""

    def __init__(self):
        super().__init__("target_tracker")
        p = declare_from_config(self, crsd_config.node_params("target_tracker"),
                                PARAM_SPEC)
        self.p = p

        self.params = core.TrackerParams(
            **{name: p[name] for name in _CORE_PARAMS})
        self.tracker = core.TargetTracker(self.params)

        ranges = {n: (s["lo"], s["hi"]) for n, s in PARAM_SPEC.items()
                  if not s.get("read_only") and "lo" in s}
        self.add_on_set_parameters_callback(
            make_set_callback(self, ranges, self._apply))

        # Every input gets a freshness budget; nothing here is ever read
        # without one. See the module docstring for what each staleness does.
        self._pose = StreamCache(p["pose_timeout_s"])
        self._att = StreamCache(p["attitude_timeout_s"])
        self._det = StreamCache(p["detection_timeout_s"])
        self._clu = StreamCache(p["cluster_timeout_s"])

        # The world frame's anchor: the first valid fix after startup, held for
        # the life of the process. Re-anchoring mid-run would move every track
        # already on the map, so it is set once and never updated.
        self._origin = None
        # Last boat state built from a FRESH pose+attitude pair. Kept here
        # rather than read back out of the StreamCaches because those
        # deliberately refuse to hand out a stale value — reaching past that to
        # `.value` is the exact habit the class was written to break. This is
        # an explicit, named "last place we knew we were", used only to keep
        # range/bearing from jumping to the origin during an outage.
        self._last_boat = None
        self._last_det_key = None
        self._last_clu_key = None
        self._last_stamp = None
        self._cycles = 0
        self._ingested = 0

        self.create_subscription(Detection3DArray, p["detections_topic"],
                                 self._on_detections, qos_profile_sensor_data)
        self.create_subscription(Cluster3DArray, p["clusters_topic"],
                                 self._on_clusters, 10)
        self.create_subscription(LatLonHead, "/crsd/pose", self._on_pose, 10)
        self.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)

        self.pub = self.create_publisher(TrackedTargetArray,
                                         p["targets_topic"], 10)
        self.health_pub = self.create_publisher(String, p["health_topic"], 10)

        self.create_timer(1.0 / p["update_rate_hz"], self._tick)
        self.create_timer(p["health_period_s"], self._health)
        # Which sensors are actually feeding it, in the first line of the log.
        # "subscribed and discarded" rather than silence: someone reading this
        # after wondering where the LiDAR went should not have to find the
        # parameter to learn that it is being received and thrown away.
        sources = (f"'{p['detections_topic']}' + '{p['clusters_topic']}'"
                   if p["use_lidar"] else
                   f"'{p['detections_topic']}' ONLY — use_lidar is FALSE, so "
                   f"'{p['clusters_topic']}' is subscribed and discarded")
        self.get_logger().info(
            f"tracking into '{p['targets_topic']}' from {sources}; camera "
            f"mount ({p['cam_x']:.2f}, {p['cam_y']:.2f}, {p['cam_z']:.2f}) m "
            f"yaw={p['cam_yaw_deg']:.1f} pitch={p['cam_pitch_deg']:.1f} deg")

    def _apply(self, changes):
        """Apply a validated `ros2 param set` to the live core.

        TrackerParams is a plain dataclass the core reads on every cycle, so
        assigning the attribute IS applying the change — there is no cached
        derivative to invalidate. health_period_s is the exception: it is a
        node-level timer period, and rclpy timers cannot be repriced, so it is
        stored and takes effect at the next restart rather than pretending.
        """
        for name, value in changes.items():
            if name in _CORE_PARAMS:
                setattr(self.params, name, value)
            self.p[name] = value

    # ---------- inputs ----------

    def _on_pose(self, msg: LatLonHead):
        """Cache position + heading, anchoring the world frame on the first fix.

        A NaN heading is GPS yaw unresolved (LatLonHead.msg) — a position with
        no orientation cannot place a bearing, so it is not a usable pose here
        and is dropped rather than cached with a substituted heading.
        """
        if math.isnan(msg.heading):
            return
        if self._origin is None:
            self._origin = (msg.latitude, msg.longitude)
            self.get_logger().info(
                f"world origin anchored at {msg.latitude:.7f}, "
                f"{msg.longitude:.7f}")
        e, n = geo.latlon_to_xy(msg.latitude, msg.longitude, self._origin)
        self._pose.set((e, n, math.radians(msg.heading)), time.monotonic(),
                       msg.header.stamp)

    def _on_att(self, msg: Attitude):
        self._att.set((msg.roll, msg.pitch, msg.yaw), time.monotonic(),
                      msg.header.stamp)

    def _on_detections(self, msg: Detection3DArray):
        self._det.set(msg, time.monotonic(), msg.header.stamp)

    def _on_clusters(self, msg: Cluster3DArray):
        self._clu.set(msg, time.monotonic(), msg.header.stamp)

    # ---------- the cycle ----------

    def _stale_consequence(self, name):
        """What a stale input costs, or None when it costs nothing.

        The wording is the whole value of the line this feeds. "continuing on
        the other sensor" is only true when there IS another sensor: with
        use_lidar False the camera is the only one, so a stale detections topic
        means the world model has stopped seeing entirely, and a stale clusters
        topic means nothing at all. Reporting that second case as an error
        teaches an operator to ignore the error, which is the expensive way to
        lose the one that mattered.
        """
        if name in ("pose", "attitude"):
            return "NOT ingesting observations until it returns"
        if self.params.use_lidar:
            return "continuing on the other sensor"
        if name == "clusters":
            return None                # ignored by configuration, not a fault
        return ("use_lidar is FALSE, so the camera is the ONLY input — no "
                "observations at all until it returns; tracks age but nothing "
                "new is placed")

    def _tick(self):
        """One fuse/track cycle, then publish — every tick, empty or not."""
        now = time.monotonic()
        for name, cache in (("pose", self._pose), ("attitude", self._att),
                            ("detections", self._det), ("clusters", self._clu)):
            if not cache.went_stale(now):
                continue
            why = self._stale_consequence(name)
            if why:
                self.get_logger().error(
                    f"world model input {name!r} went stale — {why}")

        pose = self._pose.get(now)
        att = self._att.get(now)
        if pose is None or att is None:
            # No boat state: nothing can be placed. Tracks still age out
            # through an empty update, so a dead pose does not freeze the map.
            self._cycle_without_pose(now)
            return

        east, north, _heading = pose
        roll, pitch, yaw = att
        boat = core.BoatState(east, north, roll, pitch, yaw)
        self._last_boat = boat

        cameras, stamp_cam = self._take_detections(now)
        lidars, stamp_lid = self._take_clusters(now)
        self._ingested += len(cameras) + len(lidars)

        tracks = self.tracker.update(cameras, lidars, boat, now)
        # The picture is current as of the NEWEST sensor stamp that fed it —
        # not publish time, which would claim freshness the data does not have.
        # With neither sensor contributing, the pose stamp is the honest answer:
        # all this cycle did was age the tracks.
        self._last_stamp = self._newest(stamp_cam, stamp_lid, self._pose.stamp)
        self._publish(tracks)

    def _cycle_without_pose(self, now: float):
        """Age the tracks and publish, with no boat state to place anything.

        Called when pose or attitude is stale. The tracker is stepped with an
        empty observation set against the last known boat position so tracks
        still expire on schedule; their world positions are untouched, because
        a track's position is a property of the earth and not of whether the
        boat currently knows where it is. range/bearing go stale with it, which
        is why they are documented as snapshots.
        """
        boat = self._last_boat or core.BoatState(0.0, 0.0, 0.0, 0.0, 0.0)
        self.tracker.update([], [], boat, now)
        self._publish(self.tracker.snapshot(boat))

    def _take_detections(self, now):
        """Newest unconsumed Detection3DArray as core objects, or empty.

        Returns ([], None) when the topic is stale OR when this exact message
        was already fed to the tracker last tick — see the module docstring on
        why one message must count as one sighting.
        """
        msg = self._det.get(now)
        if msg is None:
            return [], None
        key = stamp_key(msg.header)
        if key == self._last_det_key:
            return [], None
        self._last_det_key = key
        return ([core.CameraDetection(d.x, d.y, d.z, d.label, d.confidence)
                 for d in msg.detections], msg.header.stamp)

    def _take_clusters(self, now):
        """Newest unconsumed Cluster3DArray as core objects, or empty."""
        msg = self._clu.get(now)
        if msg is None:
            return [], None
        key = stamp_key(msg.header)
        if key == self._last_clu_key:
            return [], None
        self._last_clu_key = key
        return ([core.LidarCluster(c.x, c.y, c.z,
                                   (c.extent_x, c.extent_y, c.extent_z),
                                   c.n_points)
                 for c in msg.clusters], msg.header.stamp)

    @staticmethod
    def _newest(*stamps):
        """The latest of several builtin_interfaces/Time, ignoring None."""
        real = [s for s in stamps if s is not None]
        if not real:
            return None
        return max(real, key=lambda s: (s.sec, s.nanosec))

    # ---------- output ----------

    def _publish(self, tracks):
        out = TrackedTargetArray()
        # `_last_stamp` deliberately STOPS ADVANCING while no sensor data is
        # being ingested, so a consumer watching header.stamp sees the picture
        # stop being current — the freshness lie this repo keeps having to
        # design out. The clock fallback only ever applies before the first
        # sensor message has arrived, when there is no data to misdate.
        out.header.stamp = self._last_stamp or self.get_clock().now().to_msg()
        out.header.frame_id = "world"
        lat0, lon0 = self._origin if self._origin else (0.0, 0.0)
        out.origin_latitude, out.origin_longitude = lat0, lon0

        now = time.monotonic()
        for t in tracks:
            m = TrackedTarget()
            m.id = t.id
            m.label = t.label
            m.confidence = float(t.confidence)
            m.sources = t.sources
            if self._origin is not None:
                m.latitude, m.longitude = geo.xy_to_latlon(t.x, t.y,
                                                           self._origin)
            m.x, m.y, m.z = t.x, t.y, t.z
            m.velocity_east, m.velocity_north = t.vel_e, t.vel_n
            m.position_stddev = t.stddev
            m.range = t.range_h
            m.bearing = t.bearing_true
            m.hits = t.hits
            m.age = now - t.first_seen
            m.time_since_seen = now - t.last_seen
            m.confirmed = t.confirmed(self.params)
            m.extent_x, m.extent_y, m.extent_z = t.extent
            out.targets.append(m)

        self.pub.publish(out)          # EVERY tick, empty or not
        self._cycles += 1

    # ---------- health ----------

    def _health(self):
        """Publish where the sightings went, and say the quiet failures aloud.

        The two warnings below are the ones that look identical to a healthy
        world model from outside: a node happily publishing an empty array
        because it never receives anything, and a node fusing nothing because
        the extrinsic is wrong. Both need to be visible in the log, not
        inferred from a topic that is technically alive.
        """
        now = time.monotonic()
        payload = dict(self.tracker.stats)
        payload["cycles"] = self._cycles
        payload["origin"] = list(self._origin) if self._origin else None
        payload["ages"] = {name: (None if cache.age(now) is None
                                  else round(cache.age(now), 2))
                           for name, cache in (("pose", self._pose),
                                               ("attitude", self._att),
                                               ("detections", self._det),
                                               ("clusters", self._clu))}
        payload["fresh"] = {name: cache.fresh(now)
                            for name, cache in (("pose", self._pose),
                                                ("attitude", self._att),
                                                ("detections", self._det),
                                                ("clusters", self._clu))}
        self.health_pub.publish(String(data=json.dumps(payload)))

        if self._origin is None:
            self.get_logger().warn(
                "no valid pose yet — is telemetry_bridge up, and does "
                "/crsd/pose carry a non-NaN heading? (NaN = GPS yaw "
                "unresolved; nothing can be placed without it)",
                throttle_duration_sec=15.0)
        if self._ingested == 0 and self._cycles > 0:
            self.get_logger().warn(
                "no detections or clusters ingested yet — the world model is "
                "publishing empty arrays because nothing is feeding it. Check "
                f"'{self.p['detections_topic']}' and "
                f"'{self.p['clusters_topic']}' are actually publishing.",
                throttle_duration_sec=15.0)
        stats = self.tracker.stats
        if stats.get("lidar_ignored"):
            # INFO, not WARN: this is the configured state, and a warning that
            # fires forever on a correct configuration is one nobody reads. It
            # is said out loud at all because "the LiDAR is connected, healthy,
            # and contributing nothing" is otherwise invisible from the outside.
            self.get_logger().info(
                f"use_lidar is FALSE: {stats['lidar_ignored']} LiDAR clusters "
                "received and discarded in the last cycle. Every track on the "
                "map is "
                "a camera detection, so every range is stereo — a few per cent "
                "of the distance, not centimetres. Set use_lidar true (Tuning "
                "tab, or ros2 param set) on open water.",
                throttle_duration_sec=60.0)
        if (self.params.use_lidar and stats.get("cam_in")
                and stats.get("lidar_in") and not stats.get("fused")):
            self.get_logger().warn(
                f"both sensors are producing ({stats['cam_in']} detections, "
                f"{stats['lidar_in']} clusters) but NOTHING fused — every "
                "target is being tracked twice, once per sensor. Suspect the "
                "camera MOUNT ANGLES (cam_yaw_deg/cam_pitch_deg), which are "
                "assumed rather than bench-confirmed; the translations are "
                "measured and only worth 0.3 deg of parallax at 10 m.",
                throttle_duration_sec=15.0)


def main(args=None):
    run_node(TargetTrackerNode, args=args)


if __name__ == "__main__":
    main()
