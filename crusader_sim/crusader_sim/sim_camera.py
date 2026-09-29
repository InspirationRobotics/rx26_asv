"""sim_camera — the OAK-D LR, for the sim: frames on the boat's topics, and
ground-truth buoy detections in the detector's own message.

    ros2 run crusader_sim sim_camera --ros-args -p course:=task1_core

Runs in: the crsd-sim container (sim only — never on the boat).

    in   /sim/oak/rgb/image     Image rgb8  (gz camera, via ros_gz_bridge)
         /sim/oak/depth/image   Image 32FC1 metres (gz depth camera)
         /sim/crusader/odometry Odometry — the boat's TRUE world pose
                                (gz OdometryPublisher; sim-only ground truth)
    out  oak/rgb                Image bgr8, frame oak_rgb_camera_optical_frame
         oak/depth              Image 16UC1 millimetres, same frame
                                (= oakd_publisher's contract; only converted
                                while someone subscribes — frames are expensive)
         crsd/oak/detections    Detection3DArray, frame camera_link (REP-103),
                                labels as oak_detector/buoy_detector emit them

WHY AN ORACLE, NOT THE REAL DETECTOR. oak_detector and buoy_detector read the
OAK-D through depthai and run the network on the device or TensorRT; neither
exists in a sim. So this publishes what a detector that works WOULD report: the
buoys inside the camera's field of view, big enough in the image to classify
(min_bbox_px, from oak_detector_core.buoy_patch's 24 px floor), labelled by
their SIDE beacon — an unlit side beacon reads "black_buoy", which is what the
boat really sees in the Advanced tier, where only the UAV can see colours.
Positions come from the boat's TRUE pose, so the tracker still has to cope with
the EKF's pose error, exactly as on the water. Not modelled: occlusion, sun,
misclassification (bench_world_model --miscolour does that for the bench).

The frames ARE real renders — use them to exercise a detector offline, or to
record a dataset (tools/oak_record.py) — the oracle does not look at them.
"""
import math

import numpy as np
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from crusader_common.node_main import run_node
from crusader_msgs.msg import Detection3D, Detection3DArray
from crusader_sim import course as C
from crusader_sim.paths import default_hull_yaml, default_params_yaml

BUOY_W = 0.432            # RoboBuoy panel width (gen_world.robobuoy)
BUOY_H = 0.44             # body + beacon + hat above the water
BUOY_ZC = 0.18            # the point a depth-based detector would report


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class SimCamera(Node):
    def __init__(self):
        super().__init__("sim_camera")
        self.declare_parameter("course", "task1_core")
        self.declare_parameter("max_range_m", 40.0)
        self.declare_parameter("min_bbox_px", 24.0)
        self.declare_parameter("range_noise_frac", 0.015)
        self.declare_parameter("bearing_noise_deg", 0.3)
        self.declare_parameter("rate_hz", 15.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.course = C.load(g("course"))
        self.buoys = C.buoys(self.course)
        with open(default_params_yaml(), encoding="utf-8") as f:
            tt = yaml.safe_load(f)["target_tracker"]["ros__parameters"]
        self.cam_pos = np.array([tt["cam_x"], tt["cam_y"], tt["cam_z"]], dtype=float)
        # + yaw = aimed to PORT, + pitch = aimed DOWN (REP-103 +pitch)
        self.R_cam = rot_z(math.radians(tt.get("cam_yaw_deg", 0.0))) @ \
            rot_y(math.radians(tt.get("cam_pitch_deg", 0.0)))
        with open(default_hull_yaml(), encoding="utf-8") as f:
            oak = yaml.safe_load(f)["sensors"]["oak_d_lr"]
        self.W, self.H = int(oak["rgb_width"]), int(oak["rgb_height"])
        self.fx = (self.W / 2.0) / math.tan(math.radians(oak["hfov_deg"]) / 2.0)
        self.rng = np.random.default_rng(1)

        self.pose = None                 # geometry_msgs/Pose, world frame
        self.create_subscription(Odometry, "/sim/crusader/odometry", self._on_odom, 10)
        self.det_pub = self.create_publisher(Detection3DArray, "crsd/oak/detections", 10)
        self.rgb_pub = self.create_publisher(Image, "oak/rgb", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, "oak/depth", qos_profile_sensor_data)
        self.create_subscription(Image, "/sim/oak/rgb/image", self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, "/sim/oak/depth/image", self._on_depth,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / float(g("rate_hz")), self._tick)
        self.get_logger().info(f"course {g('course')}: {len(self.buoys)} buoys; "
                               f"camera {self.W}x{self.H}, fx {self.fx:.0f} px")

    # ---------------------------------------------------------------- frames
    def _on_rgb(self, msg):
        if self.rgb_pub.get_subscription_count() == 0:
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        out = Image(header=msg.header, height=msg.height, width=msg.width,
                    encoding="bgr8", is_bigendian=0, step=msg.width * 3)
        out.header.frame_id = "oak_rgb_camera_optical_frame"
        out.data = np.ascontiguousarray(img[:, :, ::-1]).tobytes()
        self.rgb_pub.publish(out)

    def _on_depth(self, msg):
        if self.depth_pub.get_subscription_count() == 0:
            return
        d = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        mm = np.where(np.isfinite(d), np.clip(d * 1000.0, 0, 65535), 0).astype(np.uint16)
        out = Image(header=msg.header, height=msg.height, width=msg.width,
                    encoding="16UC1", is_bigendian=0, step=msg.width * 2)
        out.header.frame_id = "oak_rgb_camera_optical_frame"
        out.data = mm.tobytes()
        self.depth_pub.publish(out)

    # ---------------------------------------------------------------- oracle
    def _on_odom(self, msg):
        self.pose = msg.pose.pose

    def _tick(self):
        arr = Detection3DArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = "camera_link"
        if self.pose is not None:
            arr.detections = self._detect()
        self.det_pub.publish(arr)

    def _detect(self):
        p = self.pose
        R = quat_to_R(p.orientation)
        t = np.array([p.position.x, p.position.y, p.position.z])
        rmax = float(self.get_parameter("max_range_m").value)
        min_px = float(self.get_parameter("min_bbox_px").value)
        rn = float(self.get_parameter("range_noise_frac").value)
        bn = math.radians(float(self.get_parameter("bearing_noise_deg").value))
        dets = []
        for name, bx, by, state, side_on, _up in self.buoys:
            pw = np.array([bx, by, BUOY_ZC])
            pc = self.R_cam.T @ (R.T @ (pw - t) - self.cam_pos)
            x, y, z = pc
            rng = float(np.linalg.norm(pc))
            if x < 0.5 or rng > rmax:
                continue
            u = self.W / 2.0 - self.fx * y / x
            v = self.H / 2.0 - self.fx * z / x
            if not (0 <= u < self.W and 0 <= v < self.H):
                continue
            hpx = self.fx * BUOY_H / x
            if hpx < min_px:
                continue
            # noise: range and bearing, the way a stereo detector errs
            k = 1.0 + self.rng.normal(0, rn)
            b = math.atan2(y, x) + self.rng.normal(0, bn)
            rh = math.hypot(x, y) * k
            wpx = self.fx * BUOY_W / x
            d = Detection3D()
            d.label = C.LABEL_OF[state] if side_on else C.LABEL_OF["off"]
            d.confidence = 0.9
            d.x, d.y, d.z = rh * math.cos(b), rh * math.sin(b), float(z) * k
            d.bbox = [int(max(0, u - wpx / 2)), int(max(0, v - hpx / 2)),
                      int(min(self.W - 1, u + wpx / 2)), int(min(self.H - 1, v + hpx / 2))]
            dets.append(d)
        return dets


def main(args=None):
    run_node(SimCamera, args)


if __name__ == "__main__":
    main()
