"""livox_shim — Gazebo's MID360 cloud, re-shaped into what livox_ros_driver2 sends.

    ros2 run crusader_sim livox_shim

Runs in: the crsd-sim container (sim only — never on the boat).

    in   /sim/mid360/points   PointCloud2 from ros_gz_bridge (x y z intensity ring,
                              one point per ray, INCLUDING rays that hit nothing)
    out  /livox/lidar         PointCloud2 exactly as the boat's livox container
                              publishes it: frame livox_frame, RELIABLE/VOLATILE,
                              fields x y z intensity tag line timestamp (26-byte
                              packed points), and ONLY real returns

Why not bridge straight onto /livox/lidar: a gz scan carries a point for every
ray, with +inf/NaN where nothing was hit, and the real driver never sends those.
Feeding them to lidar_cluster_node would test it against data the boat cannot
produce. The frame convention needs no work here: gen_crusader mounts the sim
sensor rolled 180 deg from the same lidar_sign_y/z params, so the raw points are
already x fwd, y STARBOARD, z DOWN, like the real upside-down MID360.
"""
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from crusader_common.node_main import run_node

# livox_ros_driver2's LivoxPointXyzrtlt, packed (no padding): 26 bytes
LIVOX_DTYPE = np.dtype({
    "names": ["x", "y", "z", "intensity", "tag", "line", "timestamp"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"],
    "offsets": [0, 4, 8, 12, 16, 17, 18],
    "itemsize": 26,
})
LIVOX_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
    PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
    PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
]


def _field_view(msg, name, np_type):
    """One float field of a PointCloud2 as a strided numpy view."""
    off = {f.name: f.offset for f in msg.fields}[name]
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    n = msg.width * msg.height
    return np.ndarray((n,), dtype=np_type, buffer=buf, offset=off,
                      strides=(msg.point_step,))


class LivoxShim(Node):
    def __init__(self):
        super().__init__("livox_shim")
        self.declare_parameter("in_topic", "/sim/mid360/points")
        self.declare_parameter("out_topic", "/livox/lidar")
        self.declare_parameter("frame_id", "livox_frame")
        self.declare_parameter("range_max_m", 40.0)
        self.frame = self.get_parameter("frame_id").value
        self.rmax = float(self.get_parameter("range_max_m").value)
        # the real driver's QoS (Boat/CLAUDE.md sensor contract): RELIABLE/VOLATILE
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("out_topic").value, qos)
        self.create_subscription(PointCloud2, self.get_parameter("in_topic").value,
                                 self._on_cloud, qos_profile_sensor_data)
        self.n_in = self.n_out = 0
        self.create_timer(10.0, self._health)

    def _on_cloud(self, msg):
        x = _field_view(msg, "x", "<f4")
        y = _field_view(msg, "y", "<f4")
        z = _field_view(msg, "z", "<f4")
        r2 = x * x + y * y + z * z
        keep = np.isfinite(r2) & (r2 > 1e-4) & (r2 < self.rmax * self.rmax)
        n = int(keep.sum())
        out = np.zeros(n, dtype=LIVOX_DTYPE)
        out["x"], out["y"], out["z"] = x[keep], y[keep], z[keep]
        names = {f.name for f in msg.fields}
        out["intensity"] = _field_view(msg, "intensity", "<f4")[keep] if "intensity" in names else 100.0
        out["tag"] = 0
        out["line"] = 0
        now = self.get_clock().now()
        out["timestamp"] = now.nanoseconds * 1e-9
        pc = PointCloud2()
        pc.header.stamp = now.to_msg()       # receipt time, like a live driver
        pc.header.frame_id = self.frame
        pc.height, pc.width = 1, n
        pc.fields = LIVOX_FIELDS
        pc.is_bigendian = False
        pc.point_step = LIVOX_DTYPE.itemsize
        pc.row_step = pc.point_step * n
        pc.is_dense = True
        pc.data = out.tobytes()
        self.pub.publish(pc)
        self.n_in += 1
        self.n_out += n

    def _health(self):
        if self.n_in == 0:
            self.get_logger().warn("no cloud from the gz bridge in 10 s "
                                   "(is gz running, and GZ_PARTITION the same on both sides?)")
        else:
            self.get_logger().info(f"{self.n_in / 10.0:.1f} Hz, "
                                   f"{self.n_out / max(self.n_in, 1):.0f} pts/scan")
        self.n_in = self.n_out = 0


def main(args=None):
    run_node(LivoxShim, args)


if __name__ == "__main__":
    main()
