#!/usr/bin/env python3
"""bench_world_model.py — a boat and a buoy field, invented, so the world model
can be driven with no boat, no camera, no LiDAR and no GPS.

    ros2 run crusader_world_model target_tracker      # terminal 1
    ros2 run crusader_world_model map_server          # terminal 2
    python3 tools/bench/bench_world_model.py          # terminal 3
    # then open http://localhost:8082

This is the reason crusader_world_model is a package separate from
crusader_perception: everything downstream of a detection is geometry and time
decay, and geometry can be checked at a desk. Publishes exactly the topics the
real stack does, at roughly the real rates, in the real frames:

  /crsd/pose          20 Hz   telemetry_bridge's rate
  /crsd/attitude      30 Hz   SR0_EXTRA1, matched to the camera
  /crsd/fcu_status     1 Hz   so the map's mode/armed readout has something
  oak/detections      10 Hz   buoy_detector, camera_link, EVERY frame
  crsd/lidar_clusters  2 Hz   lidar_cluster_node's accumulation windows

THE GROUND TRUTH IS PRINTED AT STARTUP. That is what makes this a check rather
than a demo: the tracker's output is a number you can compare against a number
you chose. A track that settles within a metre of its truth row, keeps its id,
and does not split into two when the boat circles it, is a tracker that works.

WHAT IT CANNOT PROVE. The body/world rotation here is
crusader_common.geo.world_to_body_ypr, the exact transpose of the one the
tracker runs — so a sign error shared by both cancels and this bench will show
a perfect map anyway. It proves the plumbing, the association, the decay, the
fusion arbitration and the display. It does NOT prove the frame convention, and
it never will; that is a bench exercise against real hardware, the way
docs/G2_lidar_orientation.md did it for the LiDAR.

FAILURE MODES IT IS BUILT TO REPRODUCE — the flags exist because each of these
is a thing that will happen on the water and must not surprise anyone:

  --no-camera        LiDAR only. Every track goes anonymous (empty label).
  --no-lidar         Camera only. Ranges carry the stereo bias below; targets
                     sit further out than truth and wander as the boat moves.
                     Nothing is dropped — that is the package's invariant.
  --drop-pose-after  Kill /crsd/pose N seconds in. The tracker must stop
                     ingesting, the map must go red and grey the boat out, and
                     the existing tracks must age and expire on schedule.
  --chop             Roll and pitch. With attitude compensation working the
                     targets hold still; the same run against a tracker that
                     ignored roll would show every target breathing in and out
                     by a metre or two at 20 m.

REQUIREMENTS: rclpy + crusader_msgs + crusader_common — i.e. the `asv`
container, or any sourced workspace. No numpy, no cv2, no device SDKs.
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

from crusader_world_model import target_tracker_core as core

# St Petersburg, FL — RoboNation's water. Any origin works; a real one makes the
# lat/lon on the map readable as a place rather than as noise.
ORIGIN = (27.7745, -82.6320)

# The buoy field, in WORLD metres east/north from ORIGIN. A gate to run, a pair
# of channel markers beyond it, and one unlabelled object the camera will never
# name — that last one is there to prove an anonymous LiDAR cluster still
# reaches the map instead of being quietly filtered for having no label.
FIELD = [
    ("red_buoy",     -3.0, 25.0),
    ("green_buoy",    3.0, 25.0),
    ("red_buoy",    -12.0, 48.0),
    ("green_buoy",   -4.0, 52.0),
    ("yellow_buoy",  18.0, 35.0),
    ("",             22.0, 60.0),      # LiDAR sees it; the camera never will
]

# Height of a buoy's centre above the hull-bottom datum that base_link sits on
# [m]: roughly the placeholder waterline (0.10) plus what floats above it. Not
# zero, because zero would put the buoys exactly in the base_link plane and let
# a wrong cam_z pass unnoticed — the whole reason Detection3D keeps z is that it
# is the cheapest disagreement detector available.
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
LIDAR_MISS_P = 0.15      # sweeps where a small target returns nothing


class WorldModelBench(Node):
    """Simulates the boat and its two sensors; publishes what they would."""

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

        self.pose_pub = self.create_publisher(LatLonHead, "/crsd/pose", 10)
        self.att_pub = self.create_publisher(Attitude, "/crsd/attitude", 10)
        self.status_pub = self.create_publisher(FcuStatus, "/crsd/fcu_status", 10)
        self.det_pub = self.create_publisher(Detection3DArray,
                                             "oak/detections", 10)
        self.clu_pub = self.create_publisher(Cluster3DArray,
                                             "crsd/lidar_clusters", 10)

        self.create_timer(1 / 20.0, self._pose)
        self.create_timer(1 / 30.0, self._attitude)
        self.create_timer(1.0, self._status)
        if not args.no_camera:
            self.create_timer(1 / 10.0, self._detections)
        if not args.no_lidar:
            self.create_timer(1 / 2.0, self._clusters)

        self._announce()

    def _announce(self):
        log = self.get_logger()
        log.info(f"origin {ORIGIN[0]:.6f}, {ORIGIN[1]:.6f} — "
                 f"boat circles r={self.args.radius:g} m at "
                 f"{self.args.speed:g} m/s")
        log.info("GROUND TRUTH — compare the tracker's output against this:")
        for label, e, n in FIELD:
            lat, lon = geo.xy_to_latlon(e, n, ORIGIN)
            log.info(f"  {label or '(unlabelled)':>14}  "
                     f"E{e:+7.1f} N{n:+7.1f}   {lat:.7f}, {lon:.7f}")
        if self.args.no_camera:
            log.warn("--no-camera: every track will be ANONYMOUS (empty label)")
        if self.args.no_lidar:
            log.warn(f"--no-lidar: ranges carry the stereo bias "
                     f"(+{CAM_RANGE_BIAS:.0%}); targets will sit BEYOND truth")

    # ---------- the simulated boat ----------

    def _state(self):
        """Boat state now: (east, north, yaw_rad, speed, roll, pitch).

        A circle rather than a straight line on purpose. A straight run past a
        buoy never tests the two things most likely to be wrong — whether a
        track survives leaving the field of view, and whether it stays ONE
        track when it comes back into view from a different bearing. Circling
        the field does both every lap.
        """
        t = time.monotonic() - self.t0
        r, v = self.args.radius, self.args.speed
        w = v / r                                   # rad/s around the circle
        cx, cy = 0.0, 40.0                          # centre of the buoy field
        e = cx + r * math.sin(w * t)
        n = cy - r * math.cos(w * t)
        # Tangential heading, in compass convention (0 = north, clockwise +).
        yaw = (w * t) % (2 * math.pi)
        roll = pitch = 0.0
        if self.args.chop:
            roll = math.radians(8.0) * math.sin(t * 1.7)
            pitch = math.radians(4.0) * math.sin(t * 2.3 + 1.0)
        return e, n, yaw, v, roll, pitch

    def _visible(self, half_fov_deg, max_m):
        """Field members inside a sensor's cone, as (label, body xyz, range).

        One function for both sensors because the difference between them is
        the cone and the error model, not the geometry — and two copies of a
        visibility test drift until one sensor "sees" something the other
        cannot, which looks exactly like a fusion bug.
        """
        e, n, yaw, _v, roll, pitch = self._state()
        out = []
        for label, be, bn in FIELD:
            x, y, z = geo.world_to_body_ypr(be, bn, BUOY_Z, roll, pitch, yaw,
                                            e, n)
            r = math.hypot(x, y)
            if r > max_m or r < 0.5:
                continue
            if abs(math.degrees(math.atan2(y, x))) > half_fov_deg:
                continue
            out.append((label, x, y, z, r))
        return out

    # ---------- publishers ----------

    def _pose(self):
        if (self.args.drop_pose_after
                and time.monotonic() - self.t0 > self.args.drop_pose_after):
            return                        # silence, exactly as a dead bridge
        e, n, yaw, v, _r, _p = self._state()
        lat, lon = geo.xy_to_latlon(e, n, ORIGIN)
        m = LatLonHead()
        m.header.stamp = self.get_clock().now().to_msg()
        m.latitude, m.longitude = lat, lon
        m.heading = math.degrees(yaw) % 360.0
        m.ground_speed = v
        self.pose_pub.publish(m)

    def _attitude(self):
        _e, _n, yaw, _v, roll, pitch = self._state()
        m = Attitude()
        m.header.stamp = self.get_clock().now().to_msg()
        m.roll, m.pitch, m.yaw = roll, pitch, yaw
        self.att_pub.publish(m)

    def _status(self):
        m = FcuStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.mode, m.armed, m.system_status = "MANUAL", False, 3
        self.status_pub.publish(m)

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
        array = Detection3DArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = "camera_link"
        for label, x, y, z, r in self._visible(CAM_FOV_DEG, CAM_MAX_M):
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
        array = Cluster3DArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = "base_link"
        found = []
        for label, x, y, z, r in self._visible(LIDAR_FOV_DEG, LIDAR_MAX_M):
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
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--radius", type=float, default=30.0,
                    help="circle radius around the buoy field [m]")
    ap.add_argument("--speed", type=float, default=2.0, help="ground speed [m/s]")
    ap.add_argument("--chop", action="store_true",
                    help="add roll/pitch; targets must NOT move on the map")
    ap.add_argument("--no-camera", action="store_true",
                    help="LiDAR only — every track goes anonymous")
    ap.add_argument("--no-lidar", action="store_true",
                    help="camera only — ranges carry the stereo bias")
    ap.add_argument("--drop-pose-after", type=float, default=0.0,
                    help="stop publishing /crsd/pose after N s")
    ap.add_argument("--seed", type=int, default=1, help="RNG seed")
    args = ap.parse_args()

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
