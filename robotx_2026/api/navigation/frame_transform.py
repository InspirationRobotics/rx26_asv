"""frame_transform — the diagram's 'Coordinate Transform Node (rotation matrix)'.

Subscribes BODY-frame detections (from the perception pipeline) and the fused
pose (from telemetry_bridge — i.e. ArduRover's EK3, the only estimator), and
republishes detections in the WORLD frame for the occupancy grid (Phase 3).

The world origin is the first valid RTK fix seen, published once on
/crsd/world_origin (LatLonHead, latched) so every consumer anchors identically.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from interfaces.msg import LatLonHead, DetectionArray

from ..common import geo
from ..common.node_main import run_node


class FrameTransform(Node):
    def __init__(self):
        super().__init__("frame_transform")
        self.origin = None           # (lat, lon) — first valid fix
        self.pose_xy = None          # (x, y) world meters
        self.heading = None          # radians

        latched_qos = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.origin_pub = self.create_publisher(LatLonHead, "/crsd/world_origin",
                                                latched_qos)
        self.world_pub = self.create_publisher(DetectionArray,
                                               "/crsd/detections_world", 10)
        self.create_subscription(LatLonHead, "/crsd/pose", self._pose_cb, 10)
        self.create_subscription(DetectionArray, "/crsd/detections_body",
                                 self._detections_cb, 10)

    def _pose_cb(self, msg: LatLonHead):
        if math.isnan(msg.heading):
            return                   # GPS yaw not resolved yet (needs open sky, 2-3 min)
        if self.origin is None:
            self.origin = (msg.latitude, msg.longitude)
            om = LatLonHead()
            om.header.stamp = self.get_clock().now().to_msg()
            om.latitude, om.longitude, om.heading = msg.latitude, msg.longitude, 0.0
            self.origin_pub.publish(om)
            self.get_logger().info(f"world origin anchored at {self.origin}")
        self.pose_xy = geo.latlon_to_xy(msg.latitude, msg.longitude, self.origin)
        self.heading = math.radians(msg.heading)

    def _detections_cb(self, msg: DetectionArray):
        if msg.frame != "body":
            self.get_logger().warn(f"ignoring DetectionArray with frame={msg.frame!r}")
            return
        if self.pose_xy is None or self.heading is None:
            # no pose yet: dropping silently would be the 'silent starvation'
            # failure mode — log at throttled warn so it is visible
            self.get_logger().warn("detections dropped: no pose/heading yet",
                                   throttle_duration_sec=5.0)
            return
        out = DetectionArray()
        out.header = msg.header
        out.frame = "world"
        bx_by = [(d.x, d.y) for d in msg.detections]
        out.detections = msg.detections
        for d, (bx, by) in zip(out.detections, bx_by):
            d.x, d.y = geo.body_to_world(bx, by, self.pose_xy[0], self.pose_xy[1],
                                         self.heading)
        self.world_pub.publish(out)


def main(args=None):
    run_node(FrameTransform, args=args)


if __name__ == "__main__":
    main()
