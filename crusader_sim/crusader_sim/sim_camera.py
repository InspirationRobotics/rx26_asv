"""sim_camera — the OAK-D LR, for the sim: frames on the boat's topics, and
buoy detections in the detector's own message.

    ros2 run crusader_sim sim_camera --ros-args -p course:=task1_core
    ros2 run crusader_sim sim_camera --ros-args -p position_source:=truth

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

WHY AN ORACLE FOR THE BOX, NOT THE REAL DETECTOR. oak_detector and
buoy_detector read the OAK-D through depthai and run the network on the device
or TensorRT; neither exists in a sim. So existence, bbox and LABEL are what a
detector that works WOULD report: the buoys inside the camera's field of view,
big enough in the image to classify (min_bbox_px, from
oak_detector_core.buoy_patch's 24 px floor), labelled by their SIDE beacon — an
unlit side beacon reads "black_buoy", which is what the boat really sees in the
Advanced tier, where only the UAV can see colours. Not modelled: occlusion of
the BOX, sun, misclassification (bench_world_model --miscolour does that).

POSITION IS NOT AN ORACLE (position_source:=depth, the default). The boat has no
other way to see a buoy's position than the stereo depth inside its box, and
that is what oak_detector._position does (oak_detector.py:812-866), so this does
the same on the SIMULATED depth image:

    1. the sampling patch — the real one: centre u = box centre, v = 55 % down
       the box, half-size box_w/8 x box_h/6 px clamped to (4..12) x (5..20) RGB
       px (oak_detector.py:112-114, 835-842) — moved into depth pixels by the
       ratio of the two focal lengths (depth is aligned to the RGB camera, so
       one scale covers both axes);
    2. the MEDIAN of the valid depths in it (range_min_m..range_max_m, at least
       min_depth_samples of them — the real oak_detector params, read from
       crusader_params.yaml). None valid (too far, sky, empty water) => the
       detection is DROPPED, as oak_detector drops it (oak_detector.py:753-758).
       So is one whose median is not the buoy: the box is an oracle, but a real
       network does not box a buoy another buoy hides, and without this the
       depth would report the OCCLUDER's range at the hidden buoy's bearing — a
       ghost 5-10 m short of it (seen in the first full run, from buoys lined
       up behind each other during the ENTRY orbit). occlusion_tol_m is how far
       the median may sit from the buoy's true range (surface bias ~0.15 m);
    3. stereo noise on that median: sigma_z = z^2 * sigma_disp / (f_px * B), the
       disparity-domain model (z = f*B/d, so dz = z^2/(f*B) * dd). Applied once,
       AFTER the median: a stereo matcher's errors are correlated across a
       patch (the same disparity error moves every pixel of it), so the median
       does not average them away the way it would independent noise;
    4. back-projection of the patch CENTRE, z forward: x = z, y = -(u-cx)z/fx,
       z_up = -(v-cy)z/fy — the conversion oak_detector makes from the optical
       frame to camera_link (oak_detector.py:862-866).

So what the tracker now sees is what the real pipeline gives it: range from the
buoy's NEAR SURFACE (a bias of roughly half its depth, ~0.1-0.2 m toward the
boat, which the old oracle did not have), range noise that grows as z^2, and
lateral position from the box centre. position_source:=truth is the old
oracle (truth + 1.5 % range + 0.3 deg bearing noise) and needs no depth.

Detections are computed per DEPTH FRAME, with the odometry closest to that
frame's stamp (both are sim time). The depth stream is therefore subscribed
whenever detections are produced; the RGB stream stays lazy, because 1920x1200
is what cost RTF (0.2 with the GUI up) and nothing here reads it.

error_log:=<path> (settable at runtime: `ros2 param set /sim_camera error_log
/tmp/e.csv`) appends one row per published detection — the measured position
against the buoy's true one in camera_link — so a run can report the depth
error by range instead of inferring it from tracks.

The frames ARE real renders — use them to exercise a detector offline, or to
record a dataset (tools/oak_record.py).
"""
import math
from collections import deque, namedtuple

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

# RoboBuoy geometry, from gen_world (RB_*): a frustum 0.432 m square at the
# waterline, 0.2785 m high, with the beacon and sun hat on its 0.184 m top.
BUOY_W = 0.432            # base width — the widest the bbox gets
BUOY_H = 0.41             # frustum + beacon + hat above the water
BUOY_ZC = 0.15            # the point a depth-based detector would report: the
                          # side silhouette's area centroid (the frustum puts
                          # most of the area low; it was 0.18 for the old box)

# oak_detector._position's sampling patch, copied (not imported: the module
# pulls depthai-side imports the sim container does not have). Keep equal to
# oak_detector.py:112-114 — they are RGB pixels at the detector's own RGB size.
ROI_V = 0.55              # patch centre, as a fraction of the box height
ROI_HALF_W = (4, 12)      # half-width = box_w // 8, clamped
ROI_HALF_H = (5, 20)      # half-height = box_h // 6, clamped

# one oracle detection before it gets a position: pc is the buoy's TRUE place in
# camera_link (x fwd, y left, z up), bbox is [x1, y1, x2, y2] in RGB pixels
Cand = namedtuple("Cand", "name label pc bbox")
ERR_HEAD = ("sim_t,buoy,label,true_z,true_y,true_up,meas_z,meas_y,meas_up,"
            "median_z,samples,sigma_z,bbox_h_px\n")


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


def focal_px(width_px, hfov_deg):
    """Pinhole focal length [px] of an image `width_px` wide."""
    return (width_px / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class SimCamera(Node):
    def __init__(self):
        super().__init__("sim_camera")
        self.declare_parameter("course", "task1_core")
        self.declare_parameter("max_range_m", 40.0)
        self.declare_parameter("min_bbox_px", 24.0)
        self.declare_parameter("range_noise_frac", 0.015)     # truth mode only
        self.declare_parameter("bearing_noise_deg", 0.3)      # truth mode only
        self.declare_parameter("rate_hz", 15.0)               # truth mode only
        # "depth": position from the simulated depth image (see the docstring);
        # "truth": the old oracle, truth + noise
        self.declare_parameter("position_source", "depth")
        # OAK-D LR stereo baseline, 15 cm (docs.luxonis.com OAK-D LR: "large
        # (15cm) baseline"); this is the pair the depth comes from
        self.declare_parameter("stereo_baseline_m", 0.15)
        # 1-sigma disparity error. AN ASSUMPTION, anchored on Luxonis's own
        # subpixel error bounds (docs.luxonis.com, Stereo Depth Accuracy: <2 %
        # at disparity 20, <4 % at 8, <8 % at 4 = 0.3-0.4 px, i.e. about 2 sigma
        # of a 0.15-0.2 px error). Measure it on the water before trusting it.
        self.declare_parameter("disparity_sigma_px", 0.2)
        # a depth frame with no odometry this close (sim s) is skipped, not
        # paired with a stale pose: a blank is a fact, a wrong pose is a guess
        self.declare_parameter("max_pose_skew_s", 0.25)
        # a box whose depth median is further than this from the buoy's true
        # range is looking at something else (an occluder): dropped, see the
        # docstring. 0 turns the check off and lets the ghosts through.
        self.declare_parameter("occlusion_tol_m", 1.0)
        self.declare_parameter("error_log", "")
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.source = str(g("position_source")).lower()
        if self.source not in ("depth", "truth"):
            raise ValueError(f"position_source {self.source!r}: use depth or truth")
        self.course = C.load(g("course"))
        self.buoys = C.buoys(self.course)
        with open(default_params_yaml(), encoding="utf-8") as f:
            params = yaml.safe_load(f)
        tt = params["target_tracker"]["ros__parameters"]
        od = params["oak_detector"]["ros__parameters"]
        self.cam_pos = np.array([tt["cam_x"], tt["cam_y"], tt["cam_z"]], dtype=float)
        # + yaw = aimed to PORT, + pitch = aimed DOWN (REP-103 +pitch)
        self.R_cam = rot_z(math.radians(tt.get("cam_yaw_deg", 0.0))) @ \
            rot_y(math.radians(tt.get("cam_pitch_deg", 0.0)))
        # the depth thresholds the REAL ranging applies (oak_detector._position)
        self.depth_lo, self.depth_hi = float(od["range_min_m"]), float(od["range_max_m"])
        self.min_samples = int(od["min_depth_samples"])
        with open(default_hull_yaml(), encoding="utf-8") as f:
            oak = yaml.safe_load(f)["sensors"]["oak_d_lr"]
        self.W, self.H = int(oak["rgb_width"]), int(oak["rgb_height"])
        # one HFOV today; a per-sensor key wins if the hull file grows one. The
        # depth image's own width is taken from each frame, so only the FOV is
        # read here — the two cameras are co-located, so any FOV difference is
        # a pure focal-length ratio between their pixels (see _roi_in_depth)
        self.rgb_hfov = float(oak.get("rgb_hfov_deg", oak["hfov_deg"]))
        self.depth_hfov = float(oak.get("depth_hfov_deg", oak["hfov_deg"]))
        self.fx = focal_px(self.W, self.rgb_hfov)
        self.rng = np.random.default_rng(1)

        self.pose = None                 # geometry_msgs/Pose, world frame
        self.poses = deque(maxlen=600)   # (sim s, Pose), for the depth stamp
        self.stats = dict(frames=0, dets=0, no_depth=0, occluded=0, no_pose=0, skew_max=0.0)
        self._last_frame_t = None
        self._err_path, self._err_f = "", None
        self.create_subscription(Odometry, "/sim/crusader/odometry", self._on_odom, 10)
        self.det_pub = self.create_publisher(Detection3DArray, "crsd/oak/detections", 10)
        self.rgb_pub = self.create_publisher(Image, "oak/rgb", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, "oak/depth", qos_profile_sensor_data)
        # Frames are subscribed upstream ONLY while someone downstream wants
        # them: the bridge entries are lazy, and gz renders a camera only while
        # it has a subscriber, so an idle oak/rgb costs no rendering at all.
        # (Always-on frames at 1920x1200 took the sim to RTF 0.2 with the GUI up.)
        # Depth is the exception in depth mode: it IS the detections' input.
        self._frame_subs = {"rgb": None, "depth": None}
        self.create_timer(1.0, self._manage_frame_subs)
        if self.source == "truth":
            self.create_timer(1.0 / float(g("rate_hz")), self._tick)
        else:
            self.create_timer(10.0, self._report)
        self.get_logger().info(
            f"course {g('course')}: {len(self.buoys)} buoys; RGB {self.W}x{self.H} fx "
            f"{self.fx:.0f} px, depth HFOV {self.depth_hfov:.1f} deg; positions from "
            + ("TRUTH + noise" if self.source == "truth" else
               f"the depth image (B {g('stereo_baseline_m'):.2f} m, sigma_disp "
               f"{g('disparity_sigma_px'):.2f} px, valid {self.depth_lo:.2f}-"
               f"{self.depth_hi:.0f} m, >= {self.min_samples} px)"))

    # ---------------------------------------------------------------- frames
    def _manage_frame_subs(self):
        for key, pub, topic, cb, always in (
                ("rgb", self.rgb_pub, "/sim/oak/rgb/image", self._on_rgb, False),
                ("depth", self.depth_pub, "/sim/oak/depth/image", self._on_depth,
                 self.source == "depth")):
            want = always or pub.get_subscription_count() > 0
            have = self._frame_subs[key]
            if want and have is None:
                self._frame_subs[key] = self.create_subscription(
                    Image, topic, cb, qos_profile_sensor_data)
                self.get_logger().info(f"{key} frames: a consumer appeared, rendering on")
            elif not want and have is not None:
                self.destroy_subscription(have)
                self._frame_subs[key] = None
                self.get_logger().info(f"{key} frames: no consumer, rendering off")

    def _on_rgb(self, msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        out = Image(header=msg.header, height=msg.height, width=msg.width,
                    encoding="bgr8", is_bigendian=0, step=msg.width * 3)
        out.header.frame_id = "oak_rgb_camera_optical_frame"
        out.data = np.ascontiguousarray(img[:, :, ::-1]).tobytes()
        self.rgb_pub.publish(out)

    def _on_depth(self, msg):
        d = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        # converting is only worth it for a consumer of oak/depth; in depth mode
        # this callback runs with none, for the detections' sake
        if self.depth_pub.get_subscription_count() > 0:
            mm = np.where(np.isfinite(d), np.clip(d * 1000.0, 0, 65535), 0).astype(np.uint16)
            out = Image(header=msg.header, height=msg.height, width=msg.width,
                        encoding="16UC1", is_bigendian=0, step=msg.width * 2)
            out.header.frame_id = "oak_rgb_camera_optical_frame"
            out.data = mm.tobytes()
            self.depth_pub.publish(out)
        if self.source == "depth":
            self._on_depth_detect(d, stamp_s(msg.header))

    # ---------------------------------------------------------------- oracle
    def _on_odom(self, msg):
        self.pose = msg.pose.pose
        self.poses.append((stamp_s(msg.header), msg.pose.pose))

    def _pose_at(self, t):
        """The buoy-truth pose closest to sim time t, or None if none is within
        max_pose_skew_s. Scans back from the newest: a depth frame is nearly
        always paired with one of the last few samples."""
        best, skew = None, float("inf")
        for ts, pose in reversed(self.poses):
            s = abs(ts - t)
            if s < skew:
                best, skew = pose, s
            elif ts < t:
                break                    # older than t and getting further away
        if best is None or skew > float(self.get_parameter("max_pose_skew_s").value):
            return None
        self.stats["skew_max"] = max(self.stats["skew_max"], skew)
        return best

    def _publish(self, dets):
        arr = Detection3DArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = "camera_link"
        arr.detections = dets
        self.det_pub.publish(arr)
        self.stats["dets"] += len(dets)

    def _tick(self):
        """position_source:=truth — a timer, as before depth existed."""
        dets = []
        if self.pose is not None:
            dets = [self._detection(c, self._truth_position(c))
                    for c in self._visible(self.pose)]
        self._publish(dets)

    def _on_depth_detect(self, depth, t):
        self.stats["frames"] += 1
        self._last_frame_t = self.get_clock().now()
        pose = self._pose_at(t)
        if pose is None:
            self.stats["no_pose"] += 1
            return                       # nothing published: the tracker sees it stale
        dets = []
        for c in self._visible(pose):
            sample = self._sample_depth(depth, c.bbox)
            if sample is None:
                # sky behind the box, featureless water, or past range_max_m:
                # oak_detector counts these as no_depth and does not publish them
                self.stats["no_depth"] += 1
                continue
            tol = float(self.get_parameter("occlusion_tol_m").value)
            if tol > 0 and abs(sample[0] - c.pc[0]) > tol:
                self.stats["occluded"] += 1   # the depth is another object's
                continue
            pos = self._stereo_position(*sample)
            dets.append(self._detection(c, pos))
            self._log_error(t, c, pos, sample)
        self._publish(dets)
        if self._err_f is not None:
            self._err_f.flush()

    def _visible(self, pose):
        """The oracle half: [Cand] for every buoy a detector would box — in
        front of the camera, in the frame, inside max_range_m, and at least
        min_bbox_px tall. The box is the only thing the depth half is given."""
        R = quat_to_R(pose.orientation)
        t = np.array([pose.position.x, pose.position.y, pose.position.z])
        rmax = float(self.get_parameter("max_range_m").value)
        min_px = float(self.get_parameter("min_bbox_px").value)
        out = []
        for name, bx, by, state, side_on, _up in self.buoys:
            pw = np.array([bx, by, BUOY_ZC])
            pc = self.R_cam.T @ (R.T @ (pw - t) - self.cam_pos)
            x, y, z = pc
            if x < 0.5 or float(np.linalg.norm(pc)) > rmax:
                continue
            u = self.W / 2.0 - self.fx * y / x
            v = self.H / 2.0 - self.fx * z / x
            if not (0 <= u < self.W and 0 <= v < self.H):
                continue
            hpx = self.fx * BUOY_H / x
            if hpx < min_px:
                continue
            wpx = self.fx * BUOY_W / x
            label = C.LABEL_OF[state] if side_on else C.LABEL_OF["off"]
            out.append(Cand(name, label, pc,
                            [int(max(0, u - wpx / 2)), int(max(0, v - hpx / 2)),
                             int(min(self.W - 1, u + wpx / 2)),
                             int(min(self.H - 1, v + hpx / 2))]))
        return out

    @staticmethod
    def _detection(c, pos):
        d = Detection3D()
        d.label = c.label
        d.confidence = 0.9
        d.x, d.y, d.z = (float(p) for p in pos)
        d.bbox = list(c.bbox)
        return d

    # ---------------------------------------------------------- truth source
    def _truth_position(self, c):
        """The old oracle: range and bearing noise on the true position, the way
        a stereo detector errs, with none of its structure."""
        rn = float(self.get_parameter("range_noise_frac").value)
        bn = math.radians(float(self.get_parameter("bearing_noise_deg").value))
        x, y, z = c.pc
        k = 1.0 + self.rng.normal(0, rn)
        b = math.atan2(y, x) + self.rng.normal(0, bn)
        rh = math.hypot(x, y) * k
        return rh * math.cos(b), rh * math.sin(b), float(z) * k

    # ---------------------------------------------------------- depth source
    @staticmethod
    def _patch(bbox):
        """(u, v, half_w, half_h) of oak_detector._position's sampling patch, in
        RGB pixels (oak_detector.py:827-842), or None for a degenerate box."""
        x1, y1, x2, y2 = bbox
        if x2 <= x1 or y2 <= y1:
            return None
        return ((x1 + x2) // 2, int(y1 + ROI_V * (y2 - y1)),
                max(ROI_HALF_W[0], min(ROI_HALF_W[1], (x2 - x1) // 8)),
                max(ROI_HALF_H[0], min(ROI_HALF_H[1], (y2 - y1) // 6)))

    def _sample_depth(self, depth, bbox):
        """(median z [m], patch centre u, v in RGB px, valid samples) from the
        depth image, or None when the patch holds too few valid pixels.

        The depth camera shares the RGB camera's pose, so a pixel maps between
        them by the ratio of the focal lengths about the image centre — exact
        for any pair of FOVs, and 1/3 (640 / 1920) today. The patch is at
        least one depth pixel, as oak_detector's is: a half-width that rounds
        to zero is an empty slice, which would read as "no depth here"."""
        patch = self._patch(bbox)
        if patch is None:
            return None
        u, v, half_w, half_h = patch
        hd, wd = depth.shape
        s = focal_px(wd, self.depth_hfov) / self.fx
        du = int(wd / 2.0 + (u - self.W / 2.0) * s)
        dv = int(hd / 2.0 + (v - self.H / 2.0) * s)
        dhw, dhh = max(1, int(half_w * s)), max(1, int(half_h * s))
        roi = depth[max(0, dv - dhh):min(hd, dv + dhh + 1),
                    max(0, du - dhw):min(wd, du + dhw + 1)]
        valid = roi[np.isfinite(roi) & (roi >= self.depth_lo) & (roi <= self.depth_hi)]
        if valid.size < self.min_samples:
            return None
        return float(np.median(valid)), u, v, int(valid.size), wd

    def _sigma_z(self, z, width_px):
        """Stereo range error at z. z = f*B/d, so dz = z^2/(f*B) * dd, with f
        the depth image's focal length at ITS width (depth is computed and
        aligned at 640x400, not at the colour camera's 1920x1200)."""
        return (z * z * float(self.get_parameter("disparity_sigma_px").value)
                / (focal_px(width_px, self.depth_hfov)
                   * float(self.get_parameter("stereo_baseline_m").value)))

    def _stereo_position(self, z_med, u, v, _n, width_px):
        """camera_link (x fwd, y left, z up) of the patch centre at the noisy
        median depth — oak_detector._position's last two lines, with noise."""
        z = z_med + self.rng.normal(0, self._sigma_z(z_med, width_px))
        z = max(z, 1e-3)
        return z, -(u - self.W / 2.0) * z / self.fx, -(v - self.H / 2.0) * z / self.fx

    # ------------------------------------------------------------ bookkeeping
    def _log_error(self, t, c, pos, sample):
        """One CSV row per published detection, when error_log is set."""
        path = str(self.get_parameter("error_log").value)
        if path != self._err_path:
            if self._err_f is not None:
                self._err_f.close()
            self._err_path, self._err_f = path, None
            if path:
                fresh = True
                try:
                    with open(path, encoding="utf-8") as f:
                        fresh = not f.read(1)
                except OSError:
                    pass
                self._err_f = open(path, "a", encoding="utf-8", newline="\n")
                if fresh:
                    self._err_f.write(ERR_HEAD)
        if self._err_f is None:
            return
        z_med, _u, _v, n, wd = sample
        self._err_f.write("%.3f,%s,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%d\n" % (
            t, c.name, c.label, *c.pc, *pos, z_med, n, self._sigma_z(z_med, wd),
            c.bbox[3] - c.bbox[1]))

    def _report(self):
        s = self.stats
        now = self.get_clock().now()
        if self._last_frame_t is None or (now - self._last_frame_t).nanoseconds > 5e9:
            self.get_logger().warn(
                "no depth frame for 5 s: the tracker gets NO detections (the "
                "sim is paused, or /sim/oak/depth/image is not bridged)")
            return
        self.get_logger().info(
            f"depth mode: {s['frames']} frames, {s['dets']} detections published, "
            f"{s['no_depth']} boxes dropped for no valid depth, {s['occluded']} for an "
            f"occluded view, {s['no_pose']} frames "
            f"skipped for no odometry within {self.get_parameter('max_pose_skew_s').value} "
            f"s, worst pose skew {s['skew_max'] * 1000:.0f} ms")


def main(args=None):
    run_node(SimCamera, args)


if __name__ == "__main__":
    main()
