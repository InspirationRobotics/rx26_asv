"""lidar_fusion_node — Livox MID360 <-> camera detection fusion.

Subscribes:
  /crsd/detections_body  (interfaces/DetectionArray, "body") — camera detections,
                          i.e. depth_association.associate()'s output from
                          perception_node (bearing reliable, stereo range coarse).
  /livox/lidar           (sensor_msgs/PointCloud2)           — MID360 returns, in
                          the Livox sensor frame.
Publishes:
  /crsd/detections_fused (interfaces/DetectionArray, "body") — same detections
                          with LiDAR-refined range where supported.
  /crsd/fusion_health    (std_msgs/String, JSON)             — cloud age, counts.

Design: the geometry lives in the ROS-free lidar_fusion core (unit-tested); this
node only does I/O + parameter plumbing, mirroring perception_node.py. On a stale
or missing LiDAR cloud it PASSES CAMERA DETECTIONS THROUGH (never drops them) and
WARNs — a silently-empty fused stream would be an objective-1 safety hole, not a
nuisance (CLAUDE.md: fail loudly).

This node consumes the Livox driver's ROS topic; it opens no device itself — the
MID360 is owned by livox_ros_driver2 (launch/lidar_fusion.launch.py), the same way
telemetry_bridge (not this node) owns the MAVProxy link.
"""
import json
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from interfaces.msg import Detection, DetectionArray

from robotx_2026.api.common import config as crsd_config
from robotx_2026.api.common.node_main import run_node
from robotx_2026.api.common.param_utils import declare_from_config, make_set_callback
from robotx_2026.api.perception.depth_association import BodyDetection
from robotx_2026.api.perception.lidar_fusion import (FusedDetection, FusionParams, LidarExtrinsics,
                           config_extrinsic_nonidentity, fuse, to_body)

PARAM_SPEC = {
    # LiDAR pose in BODY [RO] — calibrate against the OAK-D before trusting fusion
    "lidar_roll_deg": dict(read_only=True, lo=-180.0, hi=180.0),
    "lidar_pitch_deg": dict(read_only=True, lo=-180.0, hi=180.0),
    "lidar_yaw_deg": dict(read_only=True, lo=-180.0, hi=180.0),
    "lidar_x": dict(read_only=True, lo=-5.0, hi=5.0, description="BODY starboard+ [m]"),
    "lidar_y": dict(read_only=True, lo=-5.0, hi=5.0, description="BODY forward+ [m]"),
    "lidar_z": dict(read_only=True, lo=-5.0, hi=5.0, description="BODY up+ [m]"),
    # fusion gates [DYN]
    "bearing_gate_deg": dict(read_only=False, lo=0.5, hi=30.0),
    "z_min": dict(read_only=False, lo=-5.0, hi=5.0),
    "z_max": dict(read_only=False, lo=-5.0, hi=20.0),
    "r_min": dict(read_only=False, lo=0.0, hi=10.0),
    "r_max": dict(read_only=False, lo=1.0, hi=100.0),
    "min_points": dict(read_only=False, lo=1, hi=100),
    "cloud_timeout_s": dict(read_only=False, lo=0.05, hi=5.0),
}
DYNAMIC_RANGES = {k: (v["lo"], v["hi"]) for k, v in PARAM_SPEC.items()
                  if not v["read_only"]}


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract (N,3) float32 xyz from a PointCloud2, tolerant of extra fields
    (Livox adds intensity/tag/line). Assumes little-endian float32 x/y/z."""
    offs = {f.name: f.offset for f in msg.fields}
    if not all(k in offs for k in ("x", "y", "z")):
        return np.empty((0, 3))
    n_pts = msg.width * msg.height
    if n_pts == 0:
        return np.empty((0, 3))
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n_pts, msg.point_step)

    def col(o):
        return raw[:, o:o + 4].copy().view(np.float32).reshape(-1)

    xyz = np.stack([col(offs["x"]), col(offs["y"]), col(offs["z"])], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


class LidarFusionNode(Node):
    def __init__(self):
        super().__init__("lidar_fusion_node")
        p = declare_from_config(self, crsd_config.node_params("lidar_fusion_node"),
                                PARAM_SPEC)

        self.extr = LidarExtrinsics.from_degrees(
            p["lidar_roll_deg"], p["lidar_pitch_deg"], p["lidar_yaw_deg"],
            p["lidar_x"], p["lidar_y"], p["lidar_z"])
        self._check_driver_extrinsic()   # fail loud on a double-transform config
        self.fparams = FusionParams(
            bearing_gate_rad=math.radians(p["bearing_gate_deg"]),
            z_min=p["z_min"], z_max=p["z_max"],
            r_min=p["r_min"], r_max=p["r_max"], min_points=int(p["min_points"]))
        self.cloud_timeout_s = p["cloud_timeout_s"]

        self._pts_body = None
        self._cloud_t = None
        self._last_n_points = 0
        self._last_fused = 0
        self._last_total = 0
        self._fused_total = 0
        self._pass_total = 0

        self.pub = self.create_publisher(DetectionArray, "/crsd/detections_fused", 10)
        self.health_pub = self.create_publisher(String, "/crsd/fusion_health", 10)
        self.create_subscription(PointCloud2, "/livox/lidar", self._on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(DetectionArray, "/crsd/detections_body",
                                 self._on_dets, 10)

        self.add_on_set_parameters_callback(
            make_set_callback(self, DYNAMIC_RANGES, self._apply_params))
        self.create_timer(1.0, self._publish_health)
        self.get_logger().info(
            "lidar_fusion_node up: /crsd/detections_body + /livox/lidar "
            "-> /crsd/detections_fused")

    def _check_driver_extrinsic(self):
        """Enforce the single-source-of-truth extrinsic rule (CLAUDE.md): this
        node owns the camera<->LiDAR extrinsic via lidar_* params, so the Livox
        driver's MID360_config.json must stay identity. Fail LOUD when both are
        non-identity (points would be transformed twice); WARN if only the driver
        is (safe, but off-convention)."""
        import json
        import os

        from ament_index_python.packages import get_package_share_directory

        try:
            cfg_path = os.path.join(get_package_share_directory("robotx_2026"),
                                    "config", "MID360_config.json")
            with open(cfg_path) as f:
                cfg = json.load(f)
        except (OSError, ValueError) as e:
            self.get_logger().warn(
                f"could not read MID360_config.json for extrinsic check ({e}); "
                "skipping — verify the driver extrinsic is identity manually")
            return

        nz = config_extrinsic_nonidentity(cfg)
        if not nz:
            self.get_logger().info("driver extrinsic identity check ok")
            return
        if self.extr.is_identity():
            self.get_logger().warn(
                f"MID360_config.json sets a non-identity extrinsic {nz}; node "
                "extrinsic is identity so no double-transform, but repo convention "
                "is to keep the driver identity and set lidar_* in crusader_params.yaml")
            return
        raise RuntimeError(
            f"double extrinsic: MID360_config.json {nz} AND this node's lidar_* "
            "params are both non-identity — LiDAR points would be transformed "
            "twice. Zero one of them (convention: zero the driver, keep lidar_*).")

    def _apply_params(self, changes):
        if "bearing_gate_deg" in changes:
            self.fparams.bearing_gate_rad = math.radians(changes["bearing_gate_deg"])
        for k in ("z_min", "z_max", "r_min", "r_max"):
            if k in changes:
                setattr(self.fparams, k, changes[k])
        if "min_points" in changes:
            self.fparams.min_points = int(changes["min_points"])
        if "cloud_timeout_s" in changes:
            self.cloud_timeout_s = changes["cloud_timeout_s"]

    def _on_cloud(self, msg):
        xyz = pointcloud2_to_xyz(msg)
        self._pts_body = to_body(xyz, self.extr) if xyz.size else np.empty((0, 3))
        self._cloud_t = time.monotonic()
        self._last_n_points = int(self._pts_body.shape[0])

    def _on_dets(self, msg):
        cam = [BodyDetection(d.label, d.confidence, d.x, d.y, d.radius)
               for d in msg.detections]
        fresh = (self._cloud_t is not None
                 and (time.monotonic() - self._cloud_t) <= self.cloud_timeout_s)

        if fresh and self._pts_body is not None:
            fused, n_fused = fuse(cam, self._pts_body, self.fparams)
        else:
            fused = [FusedDetection(d.label, d.confidence, d.x, d.y, d.radius,
                                    False, 0) for d in cam]
            n_fused = 0
            if cam:
                why = "no LiDAR cloud yet" if self._cloud_t is None else "LiDAR cloud stale"
                self.get_logger().warn(f"{why} -> camera passthrough",
                                       throttle_duration_sec=5.0)

        out = DetectionArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.frame = "body"
        for f in fused:
            m = Detection()
            m.label = f.label
            m.x, m.y = float(f.x), float(f.y)
            m.radius = float(f.radius)
            m.confidence = float(f.confidence)
            m.source = Detection.SOURCE_PERCEPTION
            out.detections.append(m)
        self.pub.publish(out)

        self._last_fused, self._last_total = n_fused, len(fused)
        self._fused_total += n_fused
        self._pass_total += len(fused) - n_fused

    def _publish_health(self):
        age = None if self._cloud_t is None else round(time.monotonic() - self._cloud_t, 3)
        healthy = age is not None and age <= self.cloud_timeout_s
        snap = {
            "cloud_age_s": age, "n_points": self._last_n_points,
            "fused_last": self._last_fused, "total_last": self._last_total,
            "fused_total": self._fused_total, "passthrough_total": self._pass_total,
            "healthy": healthy,
        }
        self.health_pub.publish(String(data=json.dumps(snap)))
        if not healthy:
            self.get_logger().warn(f"fusion degraded (no fresh LiDAR): {snap}",
                                   throttle_duration_sec=5.0)


def main(args=None):
    run_node(LidarFusionNode, args=args)


if __name__ == "__main__":
    main()
