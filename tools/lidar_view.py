#!/usr/bin/env python3
"""lidar_view.py — MID360 point cloud as a plan + elevation view in a browser.

THE ORIENTATION CHECK. The mounting geometry in Resources.md describes the LiDAR
frame as "x forward, y left, z down", which is LEFT-handed and cannot come from
rotating a rigid sensor — a 180 deg roll about the forward axis gives y RIGHT.
One of those two axes is not what the driver actually emits, and a y-sign error
mirrors the world port-for-starboard: every red/green gate decision in Mission
Task 1 inverts, silently and confidently. This tool settles it by eye in about
thirty seconds. Procedure: docs/G2_lidar_orientation.md.

    python3 tools/lidar_view.py --frame body      # what the stack will believe
    python3 tools/lidar_view.py --frame raw       # what the driver actually sends

Then open http://<JETSON_IP>:8081. Port 8081, NOT 8080 — buoy_detector's
annotated view and oak_view.py already contend for that one.

TWO PANELS, and the pair is the whole diagnostic:
  LEFT  plan view (x-y, looking down). x forward is UP the screen, y left is
        LEFT of screen — the nautical convention, so a target on the port bow
        belongs in the UPPER-LEFT quadrant and nowhere else.
  RIGHT elevation (x-z, looking from starboard). x forward is RIGHT, z up is UP.
        The ground/water plane must sit BELOW the sensor marker. If it is above,
        the z sign is wrong.

Points are coloured by height, so the ground plane reads as a single flat band
of colour rather than something you have to infer.

RAW vs BODY. `--frame raw` plots the cloud exactly as it arrives and labels the
axes with the sensor's own +x/+y/+z. `--frame body` applies the configured
extrinsic (sign flips + translation) and labels them BOW/PORT/STBD, because in
body frame those words mean something. Flip between the two and the transform
either does what you expected or obviously does not.

The extrinsic comes from crusader_params.yaml (`lidar_cluster_node` section) when
that is built, so this view and the clustering node cannot disagree — a bench
check against different numbers than the node uses would be worse than no check.
CLI flags override, and there are built-in defaults so this tool works before
Phase 1 lands.

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
PANEL = 620                               # px per panel; canvas is 2*PANEL wide
JPEG_QUALITY = 75

# Fallback extrinsic — Resources.md, "Mounting Information". Overridden by
# crusader_params.yaml once `lidar_cluster_node` exists there, and by CLI flags
# always. sign_y = +1 encodes the frame AS WRITTEN; if the bench check shows a
# target on the port bow landing to starboard, this is the number to flip.
FALLBACK_EXTRINSIC = dict(sign_y=1.0, sign_z=-1.0, x=0.32, y=0.05, z=0.52)

BG = 18                                   # panel background grey
GRID = (58, 58, 58)
AXIS = (92, 92, 92)
TEXT = (200, 200, 200)
ACCENT = (80, 200, 255)


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


def render(pts, ext, args, mode):
    """(N,3) points -> BGR canvas. `mode` is 'raw' or 'body'."""
    import cv2

    body = mode == "body"
    rng, z_lo, z_hi = args.range, args.z_lo, args.z_hi
    plan = np.full((PANEL, PANEL, 3), BG, np.uint8)
    elev = np.full((PANEL, PANEL, 3), BG, np.uint8)

    if pts.shape[0]:
        colors = _height_colors(pts[:, 2], z_lo, z_hi)
        # plan: x forward -> UP, y left -> LEFT (nautical)
        s = (PANEL / 2) / rng
        c = PANEL / 2
        _scatter(plan,
                 (c - pts[:, 0] * s).astype(np.int32),
                 (c - pts[:, 1] * s).astype(np.int32), colors)
        # elevation: x forward -> RIGHT, z up -> UP
        sx, sz = PANEL / (2 * rng), PANEL / max(z_hi - z_lo, 1e-6)
        _scatter(elev,
                 (PANEL - (pts[:, 2] - z_lo) * sz).astype(np.int32),
                 ((pts[:, 0] + rng) * sx).astype(np.int32), colors)
        # one dilate makes single returns visible; a lone point on a 620px panel
        # is otherwise a pixel nobody sees, which is fatal for a tool whose job
        # is showing you a buoy that returned four of them
        k = np.ones((args.dot, args.dot), np.uint8)
        plan, elev = cv2.dilate(plan, k), cv2.dilate(elev, k)

    # ---- plan overlay (after dilation, so it stays crisp) ----
    ctr = PANEL // 2
    cv2.line(plan, (ctr, 0), (ctr, PANEL), AXIS, 1)
    cv2.line(plan, (0, ctr), (PANEL, ctr), AXIS, 1)
    for ring in range(5, int(rng) + 1, 5):
        rpx = int(ring * (PANEL / 2) / rng)
        cv2.circle(plan, (ctr, ctr), rpx, GRID, 1)
        cv2.putText(plan, f"{ring}m", (ctr + 4, ctr - rpx + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, GRID, 1, cv2.LINE_AA)
    # boat marker: a triangle pointing up (+x). Only meaningful in body frame.
    if body:
        cv2.drawContours(plan, [np.array([[ctr, ctr - 13], [ctr - 8, ctr + 10],
                                          [ctr + 8, ctr + 10]])], 0, ACCENT, -1)
    fwd, left, right, aft = (("BOW +x", "PORT +y", "STBD -y", "AFT -x") if body
                             else ("+x", "+y", "-y", "-x"))
    for txt, org in ((fwd, (ctr + 8, 22)), (aft, (ctr + 8, PANEL - 10)),
                     (left, (8, ctr - 8)), (right, (PANEL - 78, ctr - 8))):
        cv2.putText(plan, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT, 1,
                    cv2.LINE_AA)
    cv2.putText(plan, "PLAN (looking down)", (8, PANEL - 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT, 1, cv2.LINE_AA)

    # ---- elevation overlay ----
    def z_row(z):
        return int(PANEL - (z - z_lo) * PANEL / max(z_hi - z_lo, 1e-6))

    cv2.line(elev, (PANEL // 2, 0), (PANEL // 2, PANEL), AXIS, 1)
    for zt in range(int(math.floor(z_lo)), int(math.ceil(z_hi)) + 1):
        r = z_row(zt)
        if 0 <= r < PANEL:
            cv2.line(elev, (0, r), (PANEL, r), GRID, 1)
            cv2.putText(elev, f"{zt:+d}m", (4, r - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, GRID, 1, cv2.LINE_AA)
    if body:
        r0 = z_row(0.0)                                   # hull-bottom datum
        if 0 <= r0 < PANEL:
            cv2.line(elev, (0, r0), (PANEL, r0), (90, 140, 90), 1)
            cv2.putText(elev, "z=0 hull datum", (PANEL - 150, r0 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 190, 120), 1,
                        cv2.LINE_AA)
        if args.water_z is not None:
            rw = z_row(args.water_z)
            if 0 <= rw < PANEL:
                cv2.line(elev, (0, rw), (PANEL, rw), (200, 130, 60), 1)
                cv2.putText(elev, f"water z={args.water_z:.2f}", (8, rw - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 160, 90), 1,
                            cv2.LINE_AA)
        # where the sensor itself sits — returns must fall BELOW this
        sr, sc = z_row(ext["z"]), int((ext["x"] + rng) * PANEL / (2 * rng))
        if 0 <= sr < PANEL and 0 <= sc < PANEL:
            cv2.drawMarker(elev, (sc, sr), ACCENT, cv2.MARKER_TILTED_CROSS, 14, 2)
            cv2.putText(elev, "LiDAR", (sc + 10, sr - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, ACCENT, 1, cv2.LINE_AA)
    cv2.putText(elev, "UP +z" if body else "+z", (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT, 1, cv2.LINE_AA)
    cv2.putText(elev, ("BOW +x ->" if body else "+x ->"), (PANEL - 110, PANEL - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT, 1, cv2.LINE_AA)
    cv2.putText(elev, "ELEVATION (from starboard)", (8, PANEL - 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT, 1, cv2.LINE_AA)

    canvas = np.hstack([plan, elev])
    cv2.line(canvas, (PANEL, 0), (PANEL, PANEL), (70, 70, 70), 1)
    banner = ("BODY FRAME (extrinsic applied)" if body
              else "RAW SENSOR FRAME (no transform)")
    cv2.putText(canvas, banner, (8, PANEL - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (90, 220, 90) if body else (90, 180, 255), 2, cv2.LINE_AA)
    return canvas


def stats_line(pts):
    if pts.shape[0] == 0:
        return "0 points"
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    return (f"{pts.shape[0]:6d} pts   "
            f"x[{lo[0]:+6.1f},{hi[0]:+6.1f}] "
            f"y[{lo[1]:+6.1f},{hi[1]:+6.1f}] "
            f"z[{lo[2]:+6.1f},{hi[2]:+6.1f}]")


class LidarView(Node):

    def __init__(self, args, ext, buf):
        super().__init__("lidar_view")
        self.args, self.ext, self.buf = args, ext, buf
        self.frames = 0
        self.sweeps = deque(maxlen=max(1, args.accumulate))
        self.last_stats = ""

        # BEST_EFFORT. A RELIABLE subscription to a BEST_EFFORT publisher matches
        # NOTHING: `ros2 topic list` looks perfect, `ros2 topic hz` shows the
        # publisher, and this node receives zero clouds. That mismatch is the
        # single most likely reason for an empty page here.
        self.create_subscription(PointCloud2, args.topic, self._on_cloud,
                                 qos_profile_sensor_data)
        self.get_logger().info(f"subscribed to {args.topic}  frame={args.frame}")
        self.create_timer(5.0, self._health)

    def _health(self):
        if self.frames == 0:
            self.get_logger().warn(
                f"no clouds yet on {self.args.topic} — is the livox container "
                "running? check `ros2 topic list`, that this container has "
                "--network host, and that the publisher QoS is BEST_EFFORT",
                throttle_duration_sec=10.0)
        else:
            self.get_logger().info(self.last_stats)

    def _on_cloud(self, msg: PointCloud2):
        import cv2
        pts = pointcloud2_to_xyz(msg)
        if self.args.frame == "body":
            pts = to_body(pts, self.ext)
        # Accumulation is NOT motion-compensated — valid only with the boat
        # stationary, which the bench is. The clustering node does this properly.
        self.sweeps.append(pts)
        allpts = np.vstack(self.sweeps) if len(self.sweeps) > 1 else pts

        self.frames += 1
        self.last_stats = stats_line(allpts)
        canvas = render(allpts, self.ext, self.args, self.args.frame)
        cv2.putText(canvas, self.last_stats, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, TEXT, 1, cv2.LINE_AA)
        ok, jpg = cv2.imencode(".jpg", canvas,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            self.buf.put(jpg.tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--topic", default=DEFAULT_TOPIC)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--frame", choices=("raw", "body"), default="body",
                    help="raw = cloud as the driver sends it; body = extrinsic "
                         "applied (default). Compare the two.")
    ap.add_argument("--range", type=float, default=25.0,
                    help="plan/elevation half-width [m] (default 25)")
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
    buf = mjpeg_server.FrameBuffer()
    node = LidarView(args, ext, buf)
    caption = (f"{args.topic} | frame={args.frame} | "
               f"sign_y={ext['sign_y']:+.0f} sign_z={ext['sign_z']:+.0f}")
    server = mjpeg_server.start(buf, args.port, caption, "lidar_view")
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
