#!/usr/bin/env python3
"""lidar_view.py — MID360 point cloud as plan / elevation views in a browser.

THE ORIENTATION CHECK, and the general-purpose "what is the LiDAR seeing" tool.
Procedure and results: docs/G2_lidar_orientation.md.

    python3 tools/lidar_view.py                   # body frame, forward 180 deg
    python3 tools/lidar_view.py --frame raw       # what the driver actually sends

Then open http://<JETSON_IP>:8081. Port 8081, NOT 8080 — buoy_detector's
annotated view and oak_view.py already contend for that one.

TABS, not a split canvas. Plan and Elevation each get the full window; "Both"
puts them side by side when you want to compare. Only the visible tab streams —
switching clears the <img> src, which drops the old connection — and a panel
nobody is watching is never rendered or encoded at all.

  Plan       x-y, looking down. x forward is UP the screen, y left is LEFT —
             the nautical convention, so a target on the port bow belongs in the
             upper-left and nowhere else. THIS is the orientation check.
  Elevation  x-z, from starboard. x forward is RIGHT, z up is UP. The
             ground/water plane must sit BELOW the sensor marker; if it is
             above, the z sign is wrong.

WHAT IS FILTERED, and why both filters run in the RAW sensor frame (so the raw
and body views show the same population and stay comparable):

  --r-min 0.5   a clear sphere around the sensor. The mount, the cabling and the
                deck immediately under the LiDAR return constantly; they are not
                the world, and left in they cluster as a permanent obstacle at
                arm's length.
  --fov 180     forward sector only. The LiDAR is at the bow and the hull blocks
                most of the view aft of the beam, so returns behind it are the
                boat's own structure.

Both are drawn on the plan panel — the clear-zone circle and the FOV wedge edges
— so what was excluded is visible rather than implied. `--fov 360 --r-min 0`
shows the unfiltered cloud.

With --fov <= 180 the plan panel puts the boat at the BOTTOM and spends its full
height on the range ahead, twice the resolution of a centred 360 view whose
lower half is hull. Both axes keep the same scale, so range rings still match a
tape measure; the trade is lateral coverage of +/-range/2, printed on the panel.

MESSAGE TYPE. The Livox driver publishes `sensor_msgs/PointCloud2` when its
`xfer_format` is 0 and `livox_ros_driver2/CustomMsg` when it is 1. ROS 2 matches
NOTHING across types, so a subscriber guessing wrong sees perfect-looking output
from `ros2 topic list`/`ros2 topic info` and receives not one message. This tool
looks up the publisher's actual type before subscribing, reads either, and says
plainly which it found. `ros2 topic info -v /livox/lidar` shows the same per
endpoint, plus QoS.

Prefer `xfer_format: 0`. PointCloud2 is the standard type, rviz and every ROS
tool speak it, `asv` needs no livox package to deserialise it, and it decodes as
a numpy stride view instead of a Python loop over 20k objects.

REQUIREMENTS: rclpy + sensor_msgs + numpy + cv2 — i.e. the `asv` container.
Run it there, with --network host, or DDS will not see the livox container's
publisher.
"""
import argparse
import math
import sys
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

import mjpeg_server                       # sibling module

DEFAULT_TOPIC = "/livox/lidar"
DEFAULT_PORT = 8081                       # 8080 belongs to the camera views
DEFAULT_PANEL = 700                       # px per panel
JPEG_QUALITY = 75

# Extrinsic fallback. Overridden by crusader_params.yaml once
# `lidar_cluster_node` exists there, and by CLI flags always.
#
# sign_y = -1 is BENCH-CONFIRMED (G2, 2026-08-14): the raw frame has +y to
# STARBOARD, so it negates into REP-103's y-left. Resources.md originally
# described the mount as "y left, z down", which is left-handed and could not
# come from rotating a rigid sensor; the truth is the plain 180-degree roll
# about the forward axis, giving x forward / y right / z down. Right-handed,
# and consistent with what "mounted upside down" physically means.
FALLBACK_EXTRINSIC = dict(sign_y=-1.0, sign_z=-1.0, x=0.32, y=0.05, z=0.52)

BG = 18                                   # panel background grey
GRID = (58, 58, 58)
AXIS = (92, 92, 92)
TEXT = (200, 200, 200)
ACCENT = (80, 200, 255)
CLIP = (70, 70, 110)                      # filter boundaries (FOV wedge, clear zone)


def load_extrinsic():
    """crusader_params.yaml's copy if it is reachable, else FALLBACK_EXTRINSIC.

    Returns (dict, source_str). Never raises: this is a bench tool that has to
    start before the node it is checking exists, and a hard failure here would
    just send someone to rviz2.
    """
    try:
        from crusader_common import config as crsd_config
        p = crsd_config.node_params("lidar_cluster_node")
        if p:
            return (dict(sign_y=float(p["lidar_sign_y"]),
                         sign_z=float(p["lidar_sign_z"]),
                         x=float(p["lidar_x"]), y=float(p["lidar_y"]),
                         z=float(p["lidar_z"])), "crusader_params.yaml")
    except Exception:
        pass
    return dict(FALLBACK_EXTRINSIC), "built-in fallback"


def custommsg_to_xyz(msg) -> np.ndarray:
    """(N,3) xyz from a livox_ros_driver2/CustomMsg.

    CustomPoint carries x/y/z as plain float32 members, so this is a straight
    read — but it is a Python loop over ~20k objects per sweep (~10-30 ms),
    where the PointCloud2 path is a numpy stride view. Fine for a viewer at
    10 Hz; the clustering node should be fed PointCloud2 (xfer_format: 0).
    """
    pts = msg.points
    if not pts:
        return np.empty((0, 3))
    out = np.empty((len(pts), 3), dtype=np.float64)
    for i, p in enumerate(pts):
        out[i, 0], out[i, 1], out[i, 2] = p.x, p.y, p.z
    return out[np.isfinite(out).all(axis=1)]


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract (N,3) float32 xyz from a PointCloud2, tolerant of extra fields
    (Livox adds intensity/tag/line). Assumes little-endian float32 x/y/z.

    Hand-rolled rather than sensor_msgs_py.point_cloud2.read_points: that helper
    builds a structured array per call and is far slower than a stride view at
    20k points a sweep.
    """
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


def to_body(pts: np.ndarray, ext: dict) -> np.ndarray:
    """Sensor frame -> REP-103 body (x fwd, y left, z up).

    Sign flips first, then translation — the offsets in Resources.md are
    measured in BODY directions, so applying them before the flip would move the
    sensor the wrong way along any flipped axis.
    """
    if pts.shape[0] == 0:
        return pts
    out = np.empty_like(pts)
    out[:, 0] = pts[:, 0] + ext["x"]
    out[:, 1] = pts[:, 1] * ext["sign_y"] + ext["y"]
    out[:, 2] = pts[:, 2] * ext["sign_z"] + ext["z"]
    return out


def near_mask(pts: np.ndarray, r_min: float) -> np.ndarray:
    """Drop returns inside a sphere of `r_min` around the SENSOR.

    Run in the RAW sensor frame, where the origin is the sensor itself — the
    mount, the cabling and the deck directly beneath it return on every sweep.
    They are not the world, and left in they form a permanent cluster at arm's
    length that no amount of downstream gating removes.

    3D range, not horizontal: the strongest self-returns are straight down from
    an upside-down sensor, and a horizontal-only test would keep every one.
    """
    if pts.shape[0] == 0 or r_min <= 0.0:
        return np.ones(pts.shape[0], dtype=bool)
    return (pts ** 2).sum(axis=1) >= r_min * r_min


def fov_mask(pts: np.ndarray, fov_deg: float) -> np.ndarray:
    """Keep points within +/- fov/2 of dead ahead.

    The MID360 sees 360 degrees, but this LiDAR is mounted at the bow and the
    hull blocks most of the aft view — so returns behind the beam are the boat's
    own structure, not the world. Left in, they cluster as a large obstacle at
    half a metre that never moves and never goes away.

    Symmetric about the x axis, so it does not care which way y points: the
    filter is valid in the raw frame and the body frame alike.
    """
    if pts.shape[0] == 0 or fov_deg >= 360.0:
        return np.ones(pts.shape[0], dtype=bool)
    half = math.radians(fov_deg) / 2.0
    return np.abs(np.arctan2(pts[:, 1], pts[:, 0])) <= half


def _scatter(img, rows, cols, colors):
    """Plot points, dropping anything off-panel."""
    h, w = img.shape[:2]
    ok = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
    img[rows[ok], cols[ok]] = colors[ok]


def _height_colors(z, z_lo, z_hi):
    import cv2
    span = max(z_hi - z_lo, 1e-6)
    norm = np.clip((z - z_lo) / span * 255.0, 0, 255).astype(np.uint8)
    return cv2.applyColorMap(norm.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)


def render_panels(pts, ext, args, mode, want=("plan", "elev")):
    """(N,3) points -> {"plan": img, "elev": img} for the panels in `want`.

    `mode` is 'raw' or 'body'. Rendering only what a browser is watching is the
    point of the `want` filter — see mjpeg_server.FrameBuffer.has_clients.
    """
    import cv2

    body = mode == "body"
    P = args.panel
    rng, z_lo, z_hi = args.range, args.z_lo, args.z_hi
    fwd_only = args.fov <= 180.0
    out = {}

    # Forward-sector layout puts the boat at the BOTTOM of the plan panel and
    # spends the whole height on the range ahead — double the resolution of a
    # centred 360 view, whose lower half would be nothing but hull returns.
    # Scale stays EQUAL on both axes: a distorted plan would break the "does the
    # range ring match the tape measure" check this tool exists for.
    if fwd_only:
        s = P / rng                      # full height = rng ahead
        ox, oy = P / 2, P                # origin bottom-centre
        ex_scale, ex_off = P / rng, 0.0              # elevation: x in 0..rng
    else:
        s = (P / 2) / rng
        ox, oy = P / 2, P / 2
        ex_scale, ex_off = P / (2 * rng), rng        # elevation: x in -rng..rng
    sz = P / max(z_hi - z_lo, 1e-6)
    colors = _height_colors(pts[:, 2], z_lo, z_hi) if pts.shape[0] else None
    k = np.ones((args.dot, args.dot), np.uint8)

    def z_row(z):
        return int(P - (z - z_lo) * sz)

    # ---------------- plan ----------------
    if "plan" in want:
        plan = np.full((P, P, 3), BG, np.uint8)
        if colors is not None:
            # x forward -> UP, y left -> LEFT (nautical)
            _scatter(plan,
                     (oy - pts[:, 0] * s).astype(np.int32),
                     (ox - pts[:, 1] * s).astype(np.int32), colors)
            # one dilate makes single returns visible; a lone point on a 700px
            # panel is otherwise a pixel nobody sees, which is fatal for a tool
            # whose job is showing you a buoy that returned four of them
            plan = cv2.dilate(plan, k)

        oxi, oyi = int(ox), int(oy)
        cv2.line(plan, (oxi, 0), (oxi, oyi), AXIS, 1)
        cv2.line(plan, (0, oyi), (P, oyi), AXIS, 1)
        for ring in range(5, int(rng) + 1, 5):
            rpx = int(ring * s)
            if rpx < 8:
                continue
            cv2.circle(plan, (oxi, oyi), rpx, GRID, 1)
            cv2.putText(plan, f"{ring}m", (oxi + 4, oyi - rpx + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, GRID, 1, cv2.LINE_AA)
        # FOV wedge edges — show what the filter removed rather than imply it
        if args.fov < 360.0:
            half = math.radians(args.fov) / 2.0
            reach = P * 1.5
            for sign in (-1, 1):
                cv2.line(plan, (oxi, oyi),
                         (int(oxi - sign * math.sin(half) * reach),
                          int(oyi - math.cos(half) * reach)), CLIP, 1)
        # clear zone around the SENSOR (offset from the body origin in body mode)
        if args.r_min > 0:
            cxp = int(ox - (ext["y"] if body else 0.0) * s)
            cyp = int(oy - (ext["x"] if body else 0.0) * s)
            rpx = max(2, int(args.r_min * s))
            cv2.circle(plan, (cxp, cyp), rpx, CLIP, 1)
        # boat marker: a triangle pointing up (+x). Only meaningful in body frame.
        if body:
            tip = oyi - 26 if fwd_only else oyi - 13
            base = oyi - 3 if fwd_only else oyi + 10
            cv2.drawContours(plan, [np.array([[oxi, tip], [oxi - 8, base],
                                              [oxi + 8, base]])], 0, ACCENT, -1)
        fwd, left, right = (("BOW +x", "PORT +y", "STBD -y") if body
                            else ("+x", "+y", "-y"))
        labels = [(fwd, (oxi + 8, 22)), (left, (8, oyi - 8)),
                  (right, (P - 78, oyi - 8))]
        if not fwd_only:
            labels.append(("AFT -x" if body else "-x", (oxi + 8, P - 10)))
        for txt, org in labels:
            cv2.putText(plan, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT, 1,
                        cv2.LINE_AA)
        sub = (f"PLAN  fwd 0..{rng:.0f}m  lateral +/-{rng / 2:.0f}m" if fwd_only
               else "PLAN (looking down)")
        cv2.putText(plan, sub, (8, P - 34), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    TEXT, 1, cv2.LINE_AA)
        out["plan"] = plan

    # ---------------- elevation ----------------
    if "elev" in want:
        elev = np.full((P, P, 3), BG, np.uint8)
        if colors is not None:
            # x forward -> RIGHT, z up -> UP
            _scatter(elev,
                     (P - (pts[:, 2] - z_lo) * sz).astype(np.int32),
                     ((pts[:, 0] + ex_off) * ex_scale).astype(np.int32), colors)
            elev = cv2.dilate(elev, k)

        x0_col = int(ex_off * ex_scale)              # where x=0 sits
        cv2.line(elev, (x0_col, 0), (x0_col, P), AXIS, 1)
        for zt in range(int(math.floor(z_lo)), int(math.ceil(z_hi)) + 1):
            r = z_row(zt)
            if 0 <= r < P:
                cv2.line(elev, (0, r), (P, r), GRID, 1)
                cv2.putText(elev, f"{zt:+d}m", (4, r - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, GRID, 1, cv2.LINE_AA)
        if body:
            r0 = z_row(0.0)                          # hull-bottom datum
            if 0 <= r0 < P:
                cv2.line(elev, (0, r0), (P, r0), (90, 140, 90), 1)
                cv2.putText(elev, "z=0 hull datum", (P - 150, r0 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 190, 120), 1,
                            cv2.LINE_AA)
            if args.water_z is not None:
                rw = z_row(args.water_z)
                if 0 <= rw < P:
                    cv2.line(elev, (0, rw), (P, rw), (200, 130, 60), 1)
                    cv2.putText(elev, f"water z={args.water_z:.2f}", (8, rw - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 160, 90),
                                1, cv2.LINE_AA)
            # where the sensor itself sits — returns must fall BELOW this
            sr, sc = z_row(ext["z"]), int((ext["x"] + ex_off) * ex_scale)
            if 0 <= sr < P and 0 <= sc < P:
                cv2.drawMarker(elev, (sc, sr), ACCENT, cv2.MARKER_TILTED_CROSS,
                               14, 2)
                cv2.putText(elev, "LiDAR", (sc + 10, sr - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, ACCENT, 1,
                            cv2.LINE_AA)
        cv2.putText(elev, "UP +z" if body else "+z", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT, 1, cv2.LINE_AA)
        cv2.putText(elev, ("BOW +x ->" if body else "+x ->"), (P - 110, P - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT, 1, cv2.LINE_AA)
        cv2.putText(elev, "ELEVATION (from starboard)", (8, P - 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT, 1, cv2.LINE_AA)
        out["elev"] = elev

    return out


def annotate(img, mode, stats):
    """Stats along the top, frame banner along the bottom."""
    import cv2
    body = mode == "body"
    P = img.shape[0]
    cv2.putText(img, stats, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT, 1,
                cv2.LINE_AA)
    banner = ("BODY FRAME (extrinsic applied)" if body
              else "RAW SENSOR FRAME (no transform)")
    cv2.putText(img, banner, (8, P - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (90, 220, 90) if body else (90, 180, 255), 2, cv2.LINE_AA)
    return img


def stats_line(pts):
    if pts.shape[0] == 0:
        return "0 points"
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    return (f"{pts.shape[0]:6d} pts   "
            f"x[{lo[0]:+6.1f},{hi[0]:+6.1f}] "
            f"y[{lo[1]:+6.1f},{hi[1]:+6.1f}] "
            f"z[{lo[2]:+6.1f},{hi[2]:+6.1f}]")


class LidarView(Node):

    def __init__(self, args, ext, bufs):
        super().__init__("lidar_view")
        self.args, self.ext, self.bufs = args, ext, bufs
        self.frames = 0
        self.sweeps = deque(maxlen=max(1, args.accumulate))
        self.last_stats = ""
        self.sub = None

        # Subscription is DEFERRED until the publisher's type is known. The
        # Livox driver publishes livox_ros_driver2/CustomMsg when xfer_format=1
        # and sensor_msgs/PointCloud2 when xfer_format=0, and ROS 2 matches
        # nothing across types: `ros2 topic info` shows the publisher, `ros2
        # topic list` looks perfect, and not one message arrives. Guessing the
        # type and then reporting "no clouds yet" describes the symptom and
        # hides the cause.
        self._resolve_timer = self.create_timer(1.0, self._resolve)
        self.create_timer(5.0, self._health)

    # ---------- type resolution ----------

    def _resolve(self):
        """Find the publisher, match its type, subscribe. Retries until it does."""
        try:
            infos = self.get_publishers_info_by_topic(self.args.topic)
        except Exception as e:                      # topic name not yet valid
            self.get_logger().warn(f"cannot query {self.args.topic}: {e}")
            return
        if not infos:
            self.get_logger().warn(
                f"no publisher on {self.args.topic} yet — is the livox container "
                "up, and does this container have --network host?",
                throttle_duration_sec=10.0)
            return

        types = {i.topic_type for i in infos}
        for i in infos:
            q = i.qos_profile
            self.get_logger().info(
                f"publisher {i.node_name}: {i.topic_type} "
                f"reliability={q.reliability.name} durability={q.durability.name}")

        if "sensor_msgs/msg/PointCloud2" in types:
            self.sub = self.create_subscription(
                PointCloud2, self.args.topic,
                lambda m: self._on_points(pointcloud2_to_xyz(m)),
                qos_profile_sensor_data)
            self.get_logger().info("subscribed as sensor_msgs/PointCloud2")
        elif any(t.endswith("CustomMsg") for t in types):
            try:
                from livox_ros_driver2.msg import CustomMsg
            except ImportError:
                self.get_logger().error(
                    f"{self.args.topic} carries livox_ros_driver2/CustomMsg, but "
                    "that message package is not on this container's ROS path, so "
                    "it cannot be deserialised here. Two ways out, and the first "
                    "is the right one:\n"
                    "  1. set the driver's `xfer_format: 0` in the livox "
                    "container so it publishes sensor_msgs/PointCloud2 — the "
                    "standard type, which rviz and every ROS tool also speak.\n"
                    "  2. build livox_ros_driver2's interface package into this "
                    "workspace, then rerun.\n"
                    "NOT a QoS problem and not a network problem — the types "
                    "simply do not match, so ROS 2 connects nothing.")
                return                              # keep retrying; they may fix it
            self.sub = self.create_subscription(
                CustomMsg, self.args.topic,
                lambda m: self._on_points(custommsg_to_xyz(m)),
                qos_profile_sensor_data)
            self.get_logger().warn(
                "subscribed as livox_ros_driver2/CustomMsg. This works for the "
                "bench view, but prefer xfer_format: 0 (PointCloud2) — the "
                "clustering node wants the numpy-friendly layout.")
        else:
            self.get_logger().error(
                f"{self.args.topic} publishes {sorted(types)}, which this tool "
                "cannot read. Expected sensor_msgs/PointCloud2.")
            return

        self._resolve_timer.cancel()
        self.get_logger().info(f"frame={self.args.frame}")

    def _health(self):
        if self.sub is None:
            return                                  # _resolve is already loud
        if self.frames == 0:
            self.get_logger().warn(
                f"subscribed to {self.args.topic} but no messages yet — check "
                "the publisher QoS above; a RELIABLE subscriber matches a "
                "BEST_EFFORT publisher not at all",
                throttle_duration_sec=10.0)
        else:
            self.get_logger().info(self.last_stats)

    # ---------- render ----------

    def _on_points(self, pts):
        import cv2
        self.frames += 1

        # BOTH filters run in the RAW sensor frame: each describes what the
        # sensor can physically see, the origin there IS the sensor, and doing
        # it before the transform makes --frame raw and --frame body show the
        # same population of points — which is what makes comparing them a
        # valid diagnostic rather than two different pictures.
        pts = pts[near_mask(pts, self.args.r_min)]
        pts = pts[fov_mask(pts, self.args.fov)]
        if self.args.frame == "body":
            pts = to_body(pts, self.ext)

        # Accumulation is NOT motion-compensated — valid only with the boat
        # stationary, which the bench is. The clustering node does this properly.
        self.sweeps.append(pts)
        allpts = np.vstack(self.sweeps) if len(self.sweeps) > 1 else pts
        self.last_stats = stats_line(allpts)

        # Render only panels somebody has open. On a Jetson also running
        # inference, encoding three JPEGs for an empty room is CPU that belongs
        # elsewhere — the same reason buoy_detector only annotates while a
        # browser is connected.
        both = self.bufs["both"].has_clients
        want = set()
        if self.bufs["plan"].has_clients or both:
            want.add("plan")
        if self.bufs["elev"].has_clients or both:
            want.add("elev")
        if not want:
            return

        panels = render_panels(allpts, self.ext, self.args, self.args.frame, want)

        def encode(img):
            ok, jpg = cv2.imencode(".jpg", img,
                                   [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            return jpg.tobytes() if ok else None

        for name in ("plan", "elev"):
            if self.bufs[name].has_clients and name in panels:
                data = encode(annotate(panels[name].copy(), self.args.frame,
                                       self.last_stats))
                if data:
                    self.bufs[name].put(data)
        if both and "plan" in panels and "elev" in panels:
            canvas = np.hstack([panels["plan"], panels["elev"]])
            cv2.line(canvas, (self.args.panel, 0),
                     (self.args.panel, self.args.panel), (70, 70, 70), 1)
            data = encode(annotate(canvas, self.args.frame, self.last_stats))
            if data:
                self.bufs["both"].put(data)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--topic", default=DEFAULT_TOPIC)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--frame", choices=("raw", "body"), default="body",
                    help="raw = cloud as the driver sends it; body = extrinsic "
                         "applied (default). Compare the two.")
    ap.add_argument("--fov", type=float, default=180.0,
                    help="keep points within +/- FOV/2 of dead ahead (default "
                         "180 = forward half). The hull blocks the aft view, so "
                         "returns behind the beam are the boat itself. Pass 360 "
                         "to see everything, including those self-returns.")
    ap.add_argument("--r-min", type=float, default=0.5, dest="r_min",
                    help="clear sphere around the SENSOR [m] (default 0.5). "
                         "Drops the mount, cabling and deck under the LiDAR. "
                         "0 disables.")
    ap.add_argument("--range", type=float, default=25.0,
                    help="forward range [m] (default 25). With --fov <= 180 the "
                         "plan panel puts the boat at the bottom and shows 0..R "
                         "ahead by +/-R/2 abeam, at equal scale.")
    ap.add_argument("--panel", type=int, default=DEFAULT_PANEL,
                    help=f"px per panel (default {DEFAULT_PANEL})")
    ap.add_argument("--z-lo", type=float, default=-1.0, dest="z_lo")
    ap.add_argument("--z-hi", type=float, default=3.0, dest="z_hi")
    ap.add_argument("--water-z", type=float, default=None, dest="water_z",
                    help="draw the waterline at this body z [m]")
    ap.add_argument("--accumulate", type=int, default=1,
                    help="hold N sweeps (STATIONARY BOAT ONLY — not motion "
                         "compensated). Helps see sparse distant targets.")
    ap.add_argument("--dot", type=int, default=3,
                    help="point size in px (default 3)")
    # extrinsic overrides — for trying a sign flip without editing config
    ap.add_argument("--sign-y", type=float, default=None, dest="sign_y")
    ap.add_argument("--sign-z", type=float, default=None, dest="sign_z")
    args, ros_args = ap.parse_known_args()

    try:
        import cv2                                    # noqa: F401
    except ImportError:
        print("ERROR: lidar_view needs cv2 (present in the asv image).",
              file=sys.stderr)
        raise SystemExit(2)

    ext, source = load_extrinsic()
    for k, v in (("sign_y", args.sign_y), ("sign_z", args.sign_z)):
        if v is not None:
            ext[k], source = v, f"{source} + CLI override"
    print(f"extrinsic from {source}: sign_y={ext['sign_y']:+.0f} "
          f"sign_z={ext['sign_z']:+.0f} t=({ext['x']}, {ext['y']}, {ext['z']})",
          file=sys.stderr)

    rclpy.init(args=ros_args)
    bufs = {n: mjpeg_server.FrameBuffer() for n in ("plan", "elev", "both")}
    node = LidarView(args, ext, bufs)

    views = {"plan": (bufs["plan"], "Plan"),
             "elev": (bufs["elev"], "Elevation"),
             "both": (bufs["both"], "Both")}
    caption = (f"{args.topic} | {args.frame} | fov {args.fov:.0f}&deg; | "
               f"r_min {args.r_min:.2f}m | "
               f"sign_y={ext['sign_y']:+.0f} sign_z={ext['sign_z']:+.0f}")
    server = mjpeg_server.start(views, args.port, caption, "lidar_view")
    print(f"Open http://<JETSON_IP>:{args.port} in a browser (Ctrl+C to stop).",
          file=sys.stderr)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
