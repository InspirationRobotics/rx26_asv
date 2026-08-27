#!/usr/bin/env python3
"""bench_world_model.py — an invented buoy field around the REAL boat, so the
world model can be driven before the camera and LiDAR are trustworthy.

    # on the boat, with core.launch.py already running:
    ros2 run crusader_world_model target_tracker      # terminal 1
    ros2 run crusader_world_model map_server          # terminal 2
    python3 tools/bench/bench_world_model.py          # terminal 3

DEFAULT MODE USES THE REAL VESSEL. It subscribes to `/crsd/pose` and
`/crsd/attitude` and publishes only the two SENSOR topics, computing what the
camera and LiDAR would have reported for a field of buoys that isn't there.
That is a far better test than a simulated boat: the pose carries real RTK
noise, the yaw is the real moving-baseline solution with its real latency, and
the roll and pitch are the real hull in the real water. A synthetic circle
tests the tracker against a boat that moves perfectly, which is the one kind of
boat it will never see.

It also means the bench does NOT contend with `telemetry_bridge` for
`/crsd/pose`. Two publishers on that topic interleave, and the tracker sees the
boat teleporting between two positions — which is why the simulated-vessel mode
now has to be asked for by name.

    --sim-pose      invent the vessel too: a boat circling the field, publishing
                    /crsd/pose, /crsd/attitude and /crsd/fcu_status itself. For
                    a DESK run with no boat and no telemetry_bridge. Do not use
                    it while the core stack is up.

WHERE THE BUOYS GO. The field is anchored at the boat's FIRST FIX and rotated
to its heading at that instant, so the buoys are laid out AHEAD OF THE BOW
wherever the boat happens to be. Anchoring to fixed lat/lon would put the field
in Florida while the boat sits in a car park, and nothing would ever come into
view. The resulting true lat/lon of every buoy is printed once, at anchor time.

THE GROUND TRUTH IS PRINTED. That is what makes this a check rather than a
demo: the tracker's output is a number you can compare against a number you
chose. A track that settles within a metre of its truth row, keeps its id, and
does not split in two as the boat swings past it, is a tracker that works.

Publishes:
  oak/detections      10 Hz   buoy_detector's topic, camera_link, EVERY frame
  crsd/lidar_clusters  2 Hz   lidar_cluster_node's accumulation windows
and, ONLY under --sim-pose:
  /crsd/pose          20 Hz   /crsd/attitude 30 Hz   /crsd/fcu_status 1 Hz

WHAT IT CANNOT PROVE. The body/world rotation here is
crusader_common.geo.world_to_body_ypr, the exact transpose of the one the
tracker runs — so a sign error shared by both cancels and this bench will show
a perfect map anyway. It proves the plumbing, the association, the decay, the
fusion arbitration and the display. It does NOT prove the frame convention, and
it never will; that is a bench exercise against real hardware, the way
docs/G2_lidar_orientation.md did it for the LiDAR.

REQUIREMENTS: rclpy + crusader_msgs + crusader_common + crusader_world_model —
i.e. the `asv` container, or any sourced workspace. No numpy, no cv2, no device
SDKs, and no camera or LiDAR.
"""
import argparse
import math
import random
import time

import rclpy
from rclpy.node import Node

from crusader_msgs.msg import (Attitude, Cluster3D, Cluster3DArray, Detection3D,
                               Detection3DArray, FcuStatus, LatLonHead)

from crusader_common import config as crsd_config
from crusader_common import geo
from crusader_common.stream_cache import StreamCache

from crusader_world_model import target_tracker_core as core

# Fallback origin for --sim-pose only: St Petersburg, FL — RoboNation's water.
# In the default (real vessel) mode the origin is the boat's own first fix and
# this is never used.
SIM_ORIGIN = (27.7745, -82.6320)

# The buoy field, in metres RELATIVE TO THE BOAT AT ANCHOR TIME:
#   right  = + to starboard of where the bow was pointing
#   ahead  = + along the bow
# A gate to run, a pair of channel markers beyond it, and one unlabelled object
# the camera will never name — that last one is there to prove an anonymous
# LiDAR cluster still reaches the map instead of being quietly filtered for
# having no label.
FIELD = [
    #  label          right  ahead
    ("red_buoy",       -3.0, 25.0),
    ("green_buoy",      3.0, 25.0),
    ("red_buoy",      -12.0, 48.0),
    ("green_buoy",     -4.0, 52.0),
    ("yellow_buoy",    18.0, 35.0),
    ("",               22.0, 60.0),    # LiDAR sees it; the camera never will
]

# Height of a buoy's centre above the hull-bottom datum that base_link sits on
# [m]: roughly the placeholder waterline (0.10) plus what floats above it. Not
# zero, because zero would put the buoys exactly in the base_link plane and let
# a wrong cam_z pass unnoticed.
BUOY_Z = 0.25

CAM_FOV_DEG = 40.0       # OAK-D half-angle, roughly
CAM_MAX_M = 25.0         # = buoy_detector.range_max_m
LIDAR_FOV_DEG = 90.0     # = lidar_cluster_node.fov_deg / 2
LIDAR_MAX_M = 40.0       # = lidar_cluster_node.r_max

# Sensor error models, deliberately DIFFERENT in character, because that
# asymmetry is the entire argument for fusing them. The camera's bearing is
# excellent and its range is a biased, range-dependent mess; the LiDAR's range
# is excellent and it knows nothing about what it hit.
CAM_BEARING_SD = 0.004   # rad — sub-quarter-degree
CAM_RANGE_BIAS = 0.06    # fraction of range, systematic (stereo baseline error)
CAM_RANGE_SD = 0.04      # fraction of range, random
LIDAR_RANGE_SD = 0.03    # m
LIDAR_MISS_P = 0.15      # windows where a small target returns nothing

POSE_TIMEOUT_S = 1.0     # = shared.pose_timeout_s


class WorldModelBench(Node):
    """Invents the two sensors. Invents the vessel too, only under --sim-pose."""

    def __init__(self, args):
        super().__init__("bench_world_model")
        self.args = args
        self.t0 = time.monotonic()
        random.seed(args.seed)

        # The camera extrinsic is READ FROM crusader_params.yaml, not restated
        # here — the same discipline tools/lidar_view.py follows for the LiDAR's.
        # A bench that models the mount differently from the node under test
        # passes and fails for reasons that have nothing to do with the code
        # being checked, which is worse than not checking at all.
        p = crsd_config.node_params("target_tracker")
        self.extrinsic = core.TrackerParams(
            cam_x=p["cam_x"], cam_y=p["cam_y"], cam_z=p["cam_z"],
            cam_yaw_deg=p["cam_yaw_deg"], cam_pitch_deg=p["cam_pitch_deg"])

        self.origin = None          # (lat, lon) of the field frame's zero
        self.field = []             # [(label, east, north)] once anchored
        self._warned_no_pose = False

        self.det_pub = self.create_publisher(Detection3DArray,
                                             "oak/detections", 10)
        self.clu_pub = self.create_publisher(Cluster3DArray,
                                             "crsd/lidar_clusters", 10)
        self.create_timer(1 / 10.0, self._detections)
        self.create_timer(1 / 2.0, self._clusters)

        if args.sim_pose:
            self._start_simulated_vessel()
        else:
            self._start_real_vessel()

    # ---------- vessel: the real one ----------

    def _start_real_vessel(self):
        """Subscribe to the boat. Publish nothing about it."""
        self._pose = StreamCache(POSE_TIMEOUT_S)
        self._att = StreamCache(POSE_TIMEOUT_S)
        self.create_subscription(LatLonHead, "/crsd/pose", self._on_pose, 10)
        self.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)
        self.create_timer(5.0, self._waiting_check)
        self.get_logger().info(
            "using the REAL vessel: /crsd/pose + /crsd/attitude. Publishing "
            "only oak/detections and crsd/lidar_clusters. The buoy field will "
            "be anchored ahead of the bow at the first fix.")

    def _on_pose(self, msg: LatLonHead):
        """Cache the fix, anchoring the field on the first usable one.

        A NaN heading is GPS yaw unresolved, and the field's orientation is
        defined by the heading at anchor time — so an unresolved first fix must
        not be allowed to anchor it. Dropped rather than defaulted to north:
        laying the field out on a guessed heading puts the buoys somewhere the
        boat may never look, and the ground-truth table would be wrong.
        """
        if math.isnan(msg.heading):
            return
        if self.origin is None:
            self._anchor(msg.latitude, msg.longitude,
                         math.radians(msg.heading))
        e, n = geo.latlon_to_xy(msg.latitude, msg.longitude, self.origin)
        self._pose.set((e, n, math.radians(msg.heading), msg.ground_speed),
                       time.monotonic())

    def _on_att(self, msg: Attitude):
        self._att.set((msg.roll, msg.pitch), time.monotonic())

    def _waiting_check(self):
        """Say loudly when nothing is arriving, and name the two likely causes."""
        if self.origin is not None:
            return
        self.get_logger().warn(
            "no usable /crsd/pose yet after "
            f"{time.monotonic() - self.t0:.0f}s — is telemetry_bridge running, "
            "and does the fix carry a non-NaN heading? (NaN = GPS yaw "
            "unresolved.) For a desk run with no boat, use --sim-pose.",
            throttle_duration_sec=5.0)

    # ---------- vessel: the simulated one ----------

    def _start_simulated_vessel(self):
        """Publish pose/attitude/status for a boat circling the field.

        Desk mode. A circle rather than a straight line because a straight run
        past a buoy never tests the two things most likely to be wrong —
        whether a track survives leaving the field of view, and whether it is
        still ONE track when it comes back into view from a different bearing.
        """
        self.pose_pub = self.create_publisher(LatLonHead, "/crsd/pose", 10)
        self.att_pub = self.create_publisher(Attitude, "/crsd/attitude", 10)
        self.status_pub = self.create_publisher(FcuStatus, "/crsd/fcu_status", 10)
        self.create_timer(1 / 20.0, self._sim_pose)
        self.create_timer(1 / 30.0, self._sim_attitude)
        self.create_timer(1.0, self._sim_status)
        # The simulated boat starts at the field frame's origin heading "ahead",
        # so one FIELD table serves both modes unchanged.
        self._anchor(SIM_ORIGIN[0], SIM_ORIGIN[1], 0.0)
        self.get_logger().warn(
            "--sim-pose: publishing /crsd/pose MYSELF. If telemetry_bridge is "
            "running, both are on that topic and the tracker will see the boat "
            "teleporting. Stop the core stack, or drop this flag.")

    def _sim_state(self):
        """Simulated (east, north, yaw, speed, roll, pitch) at this instant."""
        t = time.monotonic() - self.t0
        r, v = self.args.radius, self.args.speed
        w = v / r                                   # rad/s around the circle
        # Centre the circle on the field, and start the boat at the frame
        # origin heading along the bow — matching the anchor above.
        cx, cy = 0.0, r
        e = cx + r * math.sin(w * t)
        n = cy - r * math.cos(w * t)
        yaw = (w * t) % (2 * math.pi)
        roll = pitch = 0.0
        if self.args.chop:
            roll = math.radians(8.0) * math.sin(t * 1.7)
            pitch = math.radians(4.0) * math.sin(t * 2.3 + 1.0)
        return e, n, yaw, v, roll, pitch

    def _sim_pose(self):
        if (self.args.drop_pose_after
                and time.monotonic() - self.t0 > self.args.drop_pose_after):
            return                        # silence, exactly as a dead bridge
        e, n, yaw, v, _r, _p = self._sim_state()
        lat, lon = geo.xy_to_latlon(e, n, self.origin)
        m = LatLonHead()
        m.header.stamp = self.get_clock().now().to_msg()
        m.latitude, m.longitude = lat, lon
        m.heading = math.degrees(yaw) % 360.0
        m.ground_speed = v
        self.pose_pub.publish(m)

    def _sim_attitude(self):
        _e, _n, yaw, _v, roll, pitch = self._sim_state()
        m = Attitude()
        m.header.stamp = self.get_clock().now().to_msg()
        m.roll, m.pitch, m.yaw = roll, pitch, yaw
        self.att_pub.publish(m)

    def _sim_status(self):
        m = FcuStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.mode, m.armed, m.system_status = "MANUAL", False, 3
        self.status_pub.publish(m)

    # ---------- the field ----------

    def _anchor(self, lat0: float, lon0: float, heading0: float):
        """Place the buoy field ahead of the bow and print the ground truth.

        FIELD is written as (right, ahead) so the layout reads the way a person
        describes it from the helm. Rotating by the heading at anchor time is
        what puts the buoys where the boat can actually see them: the same table
        works whether the dock faces north or south-west, and the operator does
        not have to recompute a field every time the boat is put in the water at
        a different angle.
        """
        self.origin = (lat0, lon0)
        ch, sh = math.cos(heading0), math.sin(heading0)
        self.field = []
        for label, right, ahead in FIELD:
            east = right * ch + ahead * sh
            north = -right * sh + ahead * ch
            self.field.append((label, east, north))

        log = self.get_logger()
        log.info(f"field anchored at {lat0:.7f}, {lon0:.7f} "
                 f"heading {math.degrees(heading0):.1f} deg")
        log.info("GROUND TRUTH — compare the tracker's output against this:")
        for label, e, n in self.field:
            lat, lon = geo.xy_to_latlon(e, n, self.origin)
            log.info(f"  {label or '(unlabelled)':>14}  "
                     f"E{e:+7.1f} N{n:+7.1f}   {lat:.7f}, {lon:.7f}")
        if self.args.no_camera:
            log.warn("--no-camera: every track will be ANONYMOUS (empty label)")
        if self.args.no_lidar:
            log.warn(f"--no-lidar: ranges carry the stereo bias "
                     f"(+{CAM_RANGE_BIAS:.0%}); targets will sit BEYOND truth")

    def _vessel(self):
        """(east, north, yaw, roll, pitch) now, or None if the boat is unknown.

        None is a real answer and callers must handle it. Substituting a last
        known pose would put the invented buoys at bearings computed from a
        position the boat has left, which is the frozen-pose failure this repo
        keeps having to design out — and here it would be self-inflicted.
        """
        if self.args.sim_pose:
            e, n, yaw, _v, roll, pitch = self._sim_state()
            return e, n, yaw, roll, pitch
        now = time.monotonic()
        pose = self._pose.get(now)
        if pose is None or self.origin is None:
            return None
        att = self._att.get(now)
        # A stale attitude falls back to level rather than blocking. It costs
        # nothing: the tracker refuses to ingest anything without a fresh
        # attitude of its own, so these detections are computed and then
        # correctly ignored. Blocking here would instead make a dead ATTITUDE
        # stream look like a dead CAMERA, which is the wrong diagnosis to hand
        # someone at the dock.
        roll, pitch = att if att is not None else (0.0, 0.0)
        return pose[0], pose[1], pose[2], roll, pitch

    def _visible(self, half_fov_deg, max_m):
        """Field members inside a sensor's cone, as (label, x, y, z, range).

        One function for both sensors because the difference between them is
        the cone and the error model, not the geometry — and two copies of a
        visibility test drift until one sensor "sees" something the other
        cannot, which looks exactly like a fusion bug.

        Returns None (not an empty list) when the boat's position is unknown:
        "no buoys in view" and "no idea where the boat is" are different
        answers and the callers publish differently for each.
        """
        v = self._vessel()
        if v is None:
            return None
        e, n, yaw, roll, pitch = v
        out = []
        for label, be, bn in self.field:
            x, y, z = geo.world_to_body_ypr(be, bn, BUOY_Z, roll, pitch, yaw,
                                            e, n)
            r = math.hypot(x, y)
            if r > max_m or r < 0.5:
                continue
            if abs(math.degrees(math.atan2(y, x))) > half_fov_deg:
                continue
            out.append((label, x, y, z, r))
        return out

    def _no_pose(self):
        """Warn once per dropout, then stay quiet. Returns True if unknown."""
        if not self._warned_no_pose:
            self._warned_no_pose = True
            self.get_logger().warn(
                "vessel position unknown — NOT publishing detections. Cannot "
                "compute what a sensor would have seen without knowing where "
                "the boat is, and publishing empty arrays would tell the "
                "tracker the water is clear.")
        return True

    # ---------- the sensors ----------

    def _detections(self):
        """What buoy_detector would publish: camera_link, EVERY frame.

        The unlabelled field member is skipped here and only here — a camera
        that cannot name a thing does not report it, which is precisely why the
        LiDAR-only path has to work.

        Positions are converted out of base_link into TRUE camera_link before
        publishing. Without that step the mount offset would be missing from
        the bench and present in the tracker, and every camera-derived position
        would land 37 cm forward and 65 cm high of truth — a clean, constant
        error that looks exactly like a fusion bug and is not one.
        """
        if self.args.no_camera:
            return
        seen = self._visible(CAM_FOV_DEG, CAM_MAX_M)
        if seen is None:
            return self._no_pose()
        self._warned_no_pose = False

        array = Detection3DArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = "camera_link"
        for label, x, y, z, r in seen:
            if not label:
                continue
            # Perturb the BEARING slightly and the RANGE badly, then rebuild
            # the position from the pair — applying independent noise to x and
            # y instead would give the camera a range accuracy it does not have.
            b = math.atan2(y, x) + random.gauss(0, CAM_BEARING_SD)
            rr = r * (1 + CAM_RANGE_BIAS + random.gauss(0, CAM_RANGE_SD))
            cx, cy, cz = core.body_to_camera(rr * math.cos(b), rr * math.sin(b),
                                             z, self.extrinsic)
            d = Detection3D()
            d.label, d.confidence = label, round(random.uniform(0.62, 0.95), 2)
            d.x, d.y, d.z = cx, cy, cz
            d.bbox = [0, 0, 0, 0]
            array.detections.append(d)
        self.det_pub.publish(array)       # every frame, empty or not

    def _clusters(self):
        """What lidar_cluster_node would publish: base_link, every window."""
        if self.args.no_lidar:
            return
        seen = self._visible(LIDAR_FOV_DEG, LIDAR_MAX_M)
        if seen is None:
            return self._no_pose()
        self._warned_no_pose = False

        array = Cluster3DArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = "base_link"
        found = []
        for label, x, y, z, r in seen:
            if random.random() < LIDAR_MISS_P:
                continue                  # a small target missed this window
            k = 1 + random.gauss(0, LIDAR_RANGE_SD) / max(r, 1e-3)
            c = Cluster3D()
            c.x, c.y, c.z = x * k, y * k, z
            c.extent_x = c.extent_y = 0.32
            c.extent_z = 0.45
            # Roughly the return count the real sensor gives a 0.3 m buoy:
            # ~37 at 5 m, falling off as 1/r^2 (Cluster3D.msg).
            c.n_points = max(2, int(37 * (5.0 / max(r, 1.0)) ** 2))
            c.range = math.hypot(c.x, c.y)
            found.append(c)
        found.sort(key=lambda c: c.range)   # nearest first, per the contract
        array.clusters = found
        self.clu_pub.publish(array)        # every window, empty or not


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim-pose", action="store_true",
                    help="invent the VESSEL too and publish /crsd/pose myself "
                         "(desk mode; do NOT use while core.launch.py is up)")
    ap.add_argument("--no-camera", action="store_true",
                    help="LiDAR only — every track goes anonymous")
    ap.add_argument("--no-lidar", action="store_true",
                    help="camera only — ranges carry the stereo bias")
    ap.add_argument("--seed", type=int, default=1, help="RNG seed")
    sim = ap.add_argument_group("--sim-pose only")
    sim.add_argument("--radius", type=float, default=30.0,
                     help="circle radius around the buoy field [m]")
    sim.add_argument("--speed", type=float, default=2.0,
                     help="ground speed [m/s]")
    sim.add_argument("--chop", action="store_true",
                     help="add roll/pitch; targets must NOT move on the map")
    sim.add_argument("--drop-pose-after", type=float, default=0.0,
                     help="stop publishing /crsd/pose after N s")
    args = ap.parse_args()

    # Fail loudly rather than silently ignoring a flag. A --chop run that
    # quietly did nothing because the real attitude was in use would be read as
    # "roll compensation works", which is the opposite of what it proved.
    if not args.sim_pose:
        used = [name for name, on in (("--chop", args.chop),
                                      ("--drop-pose-after", args.drop_pose_after))
                if on]
        if used:
            ap.error(f"{', '.join(used)} only applies with --sim-pose; the real "
                     "vessel supplies its own attitude and its own dropouts "
                     "(stop telemetry_bridge to test a dropout for real)")
    if args.no_camera and args.no_lidar:
        ap.error("--no-camera and --no-lidar together publish nothing at all")

    rclpy.init()
    node = WorldModelBench(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
