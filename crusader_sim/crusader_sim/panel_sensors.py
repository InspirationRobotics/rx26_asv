"""panel_sensors — the Task 1 panel's three sensor views: what Crusader's OAK-D
and MID360 see, as JPEGs a browser can poll.

    view   gz topic                  message                          served as
    rgb    /crusader/preview/rgb     Image R8G8B8 480x300, 5 Hz       the frame, JPEG
    depth  /crusader/preview/depth   Image R_FLOAT32 m, 320x200, 5 Hz colourised + legend
    lidar  /crusader/mid360/points   PointCloudPacked ~10k pts, 10 Hz 400x400 top-down, ~3 Hz

Runs in: task1_panel's process on the WSL2 HOST — plain python3, no ROS. Needs
numpy and cv2 (apt python3-numpy / python3-opencv); task1_panel runs without
this module, with the views reporting why.

LAZY, BECAUSE A CAMERA NOBODY SUBSCRIBES TO IS NOT RENDERED. The preview
cameras (gen_crusader.py:256-263, crusader_hull.yaml `preview`) are gz sensors,
and gz-sensors renders a camera only while its topic has a subscriber; every
render is GPU time the sim spends out of real time. So a view subscribes on its
first request and unsubscribes IDLE_S after its last one: a closed tab, a
toggled-off view or a hidden page costs the sim nothing within seconds.

NO GZ CALL ON AN HTTP THREAD. get() only stamps the request and hands back the
latest JPEG. Subscribing and unsubscribing happen on this module's worker
thread; conversion happens in gz-transport's callback thread, rate-limited to
the view's rate — a message that arrives sooner is dropped unconverted, and
only the newest JPEG is kept.

BLANKS OVER GUESSES (RobotX_2026/CLAUDE.md). An unsubscribed view drops its
frame: when it comes back on, the page says "no frames" until a fresh one
arrives instead of showing a picture that looks live and is minutes old. Every
JPEG is served with its age.

THE LIDAR FRAME. The MID360 is mounted upside down at the bow, and the sim
mounts it the same way from the same params (gen_crusader.py:195-220: roll pi
from lidar_sign_y = lidar_sign_z = -1), so its raw frame is x FORWARD, y
STARBOARD, z DOWN. The bird's-eye view is drawn in the BOAT frame (REP-103:
x forward, y port, z up) — bow up, port left, starboard right — using the
transform lidar_cluster_core.to_body applies on the boat
(lidar_cluster_core.py:163-166: signs first, then the offsets), with the numbers
read from crusader_params.yaml through gen_crusader's own reader, so the view
un-mounts the sensor with exactly the numbers the sim mounted it with.

THE UNSCANNED HALF. The sim LiDAR scans only the 180 deg in front of the sensor
(crusader_hull.yaml mid360.h_fov_deg; the real one's rear half sees only the
boat's own hull, Resources.md:116-118), so the view hatches everything aft of the
sensor as "no coverage". Empty there means NOT SCANNED, never "open water".
"""
import threading
import time

import cv2
import numpy as np
from gz.msgs10 import image_pb2
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked

IDLE_S = 5.0                    # unsubscribe this long after the last request
TICK_S = 0.5                    # worker cadence when no request wakes it
JPEG_QUALITY = 80
# Every age here is a difference of two readings, so the MONOTONIC clock: WSL
# steps its wall clock (-0.78 s seen mid-run, 2026-09-30), and a frame then came
# out with a negative age.
_now = time.monotonic

DEPTH_NEAR_M, DEPTH_FAR_M = 0.58, 30.0     # = crusader_hull.yaml oak_d_lr depth_min/max_m
BEV_PX, BEV_RANGE_M, BEV_RING_M = 400, 25.0, 5.0
BEV_Z_LO, BEV_Z_HI = 0.0, 2.0              # body z (up), colour-ramp ends [m]
# Turbo when this cv2 has it (>= 4.1); Jet otherwise. 0 = blue, 255 = red in both.
CMAP = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)

INK = (220, 232, 239)                      # BGR of the page's --ink
DIM = (160, 143, 124)                      # --dim
GRID = (74, 58, 37)                        # --line
BG = (38, 26, 10)                          # the map canvas's #0a1a26
SHADE_FILL = (54, 50, 48)                  # the LiDAR's unscanned half: a grey a step
SHADE_HATCH = (104, 96, 92)                # lighter than BG, with diagonal hatching

_F = PointCloudPacked.Field
_NP_TYPE = {_F.INT8: "i1", _F.UINT8: "u1", _F.INT16: "i2", _F.UINT16: "u2",
            _F.INT32: "i4", _F.UINT32: "u4", _F.FLOAT32: "f4", _F.FLOAT64: "f8"}


# ------------------------------------------------------------------ drawing

def _text(img, s, org, scale=0.38, color=INK):
    """Text with a dark outline, legible over points and colour ramps alike."""
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def _text_w(s, scale=0.38):
    return cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0]


def _ramp(values01):
    """[0, 1] floats -> BGR uint8 rows of CMAP, one per value."""
    v = (np.clip(values01, 0.0, 1.0) * 255).astype(np.uint8).reshape(-1, 1)
    return cv2.applyColorMap(v, CMAP).reshape(-1, 3)


def _legend(img, x, y, w, lo, hi, reverse=False):
    """A colour bar `w` px wide at (x, y) labelled `lo` on its left, `hi` on its
    right. reverse=True runs the bar red -> blue (near -> far for depth)."""
    t = np.linspace(1.0, 0.0, w) if reverse else np.linspace(0.0, 1.0, w)
    img[y:y + 6, x:x + w] = _ramp(t)[None, :, :]
    _text(img, lo, (x - _text_w(lo) - 4, y + 7))
    _text(img, hi, (x + w + 4, y + 7))


def _jpeg(img):
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        raise ValueError("JPEG encoding failed")
    return buf.tobytes()


def _rows(data, h, row, step, what):
    """`h` rows of `row` bytes, `step` bytes apart in `data` (0 = packed), as
    one contiguous (h, row) uint8 array with the padding between rows cut out.

    Both messages carry a stride (Image.step, PointCloudPacked.row_step) and
    either may pad a row: reading padding as data shears a picture sideways
    and puts phantom points in a cloud. ValueError if `data` is too short."""
    step = step or row
    if step < row or len(data) < step * (h - 1) + row:
        raise ValueError("%s: %d rows of %d bytes every %d need more than the %d bytes sent"
                         % (what, h, row, step, len(data)))
    raw = np.frombuffer(data, np.uint8, count=step * (h - 1) + row)
    return np.ascontiguousarray(np.lib.stride_tricks.as_strided(raw, (h, row), (step, 1)))


# ------------------------------------------------------------------ cameras

def _pixels(msg, want_fmt, dtype, channels):
    """A gz Image's pixels as an (h, w[, c]) array, or ValueError."""
    if msg.pixel_format_type != want_fmt:
        raise ValueError("pixel format %s, expected %s" % (
            image_pb2.PixelFormatType.Name(msg.pixel_format_type),
            image_pb2.PixelFormatType.Name(want_fmt)))
    w, h = msg.width, msg.height
    if w == 0 or h == 0:
        raise ValueError("empty %dx%d image" % (w, h))
    px = _rows(msg.data, h, w * channels * np.dtype(dtype).itemsize, msg.step,
               "%dx%d image" % (w, h)).view(dtype)
    return px.reshape(h, w, channels) if channels > 1 else px.reshape(h, w)


def rgb_jpeg(msg):
    """/crusader/preview/rgb (R8G8B8) -> JPEG, as rendered."""
    rgb = _pixels(msg, image_pb2.RGB_INT8, np.uint8, 3)
    return _jpeg(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))


def depth_jpeg(msg):
    """/crusader/preview/depth (R_FLOAT32 metres) -> colourised JPEG.

    Near is red, far is blue, clipped to the OAK's own 0.58-30 m range. NaN, inf
    and <= 0 are "no return" and drawn BLACK — never as the far colour, which
    would claim open water that the camera did not actually see. A 20 px
    legend strip goes BELOW the picture so it hides no pixels."""
    d = _pixels(msg, image_pb2.R_FLOAT32, np.dtype("<f4"), 1)
    valid = np.isfinite(d) & (d > 0)
    near01 = (DEPTH_FAR_M - np.clip(np.where(valid, d, DEPTH_FAR_M), DEPTH_NEAR_M, DEPTH_FAR_M)) \
        / (DEPTH_FAR_M - DEPTH_NEAR_M)
    img = _ramp(near01.ravel()).reshape(d.shape + (3,))
    img[~valid] = 0
    h, w = d.shape
    out = np.zeros((h + 20, w, 3), np.uint8)
    out[:h] = img
    _legend(out, 44, h + 6, 110, "%.1f m" % DEPTH_NEAR_M, "%.0f m" % DEPTH_FAR_M, reverse=True)
    _text(out, "black: no return", (w - _text_w("black: no return") - 4, h + 14), color=DIM)
    return _jpeg(out)


# ------------------------------------------------------------------ lidar

def cloud_xyz(msg):
    """PointCloudPacked -> (N, 3) float64 x, y, z in the SENSOR frame, finite
    points only (a gz scan carries a point for every ray, +inf where nothing
    was hit — livox_shim.py:7-9).

    Offsets, types and strides come from the message's own `field` list,
    point_step and row_step, never assumed: the gz layout (x y z intensity ring,
    padded to 32 bytes) is not the Livox driver's (26 bytes packed), and a
    hard-coded offset would read one as the other without complaint.
    """
    f = {fl.name: fl for fl in msg.field}
    missing = [k for k in "xyz" if k not in f]
    if missing:
        raise ValueError("cloud has no %s field (has %s)" % (",".join(missing), sorted(f)))
    end = ">" if msg.is_bigendian else "<"
    try:
        dt = np.dtype({"names": list("xyz"),
                       "formats": [end + _NP_TYPE[f[k].datatype] for k in "xyz"],
                       "offsets": [f[k].offset for k in "xyz"],
                       "itemsize": msg.point_step})
    except (KeyError, ValueError) as e:
        raise ValueError("cloud fields do not fit point_step %d: %s" % (msg.point_step, e))
    w, h = msg.width, msg.height
    if h == 0 or w == 0:
        return np.zeros((0, 3))
    pts = _rows(msg.data, h, w * msg.point_step, msg.row_step,
                "%dx%d cloud" % (w, h)).reshape(-1).view(dt)
    xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def lidar_mount():
    """{"t": (x, y, z), "sign": (y, z), "fov_deg": h} — the mount from
    crusader_params.yaml, through the reader gen_crusader mounted the sim's sensor
    with (gen_crusader.py:33-45), and the horizontal scan window from
    crusader_hull.yaml's mid360.h_fov_deg, the key gen_crusader scans with (360
    when absent, as there)."""
    import yaml
    from crusader_sim.gen_crusader import _sensor_mounts
    from crusader_sim.paths import default_hull_yaml, default_params_yaml
    m = _sensor_mounts(default_params_yaml())
    with open(default_hull_yaml(), encoding="utf-8") as f:
        fov = float(yaml.safe_load(f)["sensors"]["mid360"].get("h_fov_deg", 360.0))
    return {"t": m["lidar"], "sign": m["lidar_signs"], "fov_deg": fov}


def to_body(xyz, mount):
    """Sensor -> body, as lidar_cluster_core.to_body (lidar_cluster_core.py:163-166):
    sign flips FIRST, then the offsets, which are measured in body axes."""
    (tx, ty, tz), (sy, sz) = mount["t"], mount["sign"]
    return np.column_stack([xyz[:, 0] + tx, xyz[:, 1] * sy + ty, xyz[:, 2] * sz + tz])


def _shade_uncovered(img, mount):
    """Hatch the part of the picture the LiDAR does not scan. Returns where to
    write the caption (drawn by the caller, over the grid) and its lines, or None
    when the whole circle is scanned.

    The sim scans only the window mount["fov_deg"] wide about the sensor's forward
    axis (crusader_hull.yaml mid360.h_fov_deg; 180 = the bow half). Everywhere
    else is "no coverage", NOT "no return": without this the empty half of the
    picture reads as open water behind the boat, and a real gap in the data is
    drawn exactly like a clear sea. The sensor sits at body (x, y) = mount["t"],
    and to_body leaves sensor +x as body +x, so the window's axis is body forward
    through that point — at 180 deg, the line x = lidar_x."""
    fov = mount.get("fov_deg", 360.0) if mount else 360.0
    if fov >= 360.0:
        return None
    (tx, ty, _), c, k = mount["t"], BEV_PX // 2, BEV_PX / (2 * BEV_RANGE_M)
    su, sv = c - ty * k, c - tx * k                     # the sensor, in pixels
    # bearing from the bow, + toward port: fov/2 round through dead astern to -fov/2
    th = np.radians(np.linspace(fov / 2.0, 360.0 - fov / 2.0, 73))
    far = 2.0 * BEV_PX                                  # past every corner of the image
    poly = np.vstack([[su, sv], np.column_stack([su - far * np.sin(th), sv - far * np.cos(th)])])
    mask = np.zeros((BEV_PX, BEV_PX), np.uint8)
    cv2.fillPoly(mask, [np.round(poly).astype(np.int32)], 1)
    mask = mask.astype(bool)
    yy, xx = np.mgrid[0:BEV_PX, 0:BEV_PX]
    img[mask] = SHADE_FILL
    img[mask & ((xx + yy) % 9 == 0)] = SHADE_HATCH      # diagonal hatch: "no data"
    # caption 12 m astern of the sensor, which is inside the unscanned sector
    # whenever there is one
    return (int(su), int(sv + 12.0 * k)), ("NO LIDAR COVERAGE", "scan is %g deg, bow side" % fov)


def _bev_grid(mount=None):
    """The static half of the bird's-eye view: no-coverage shading, range rings,
    axes, labels."""
    img = np.full((BEV_PX, BEV_PX, 3), BG, np.uint8)
    c, k = BEV_PX // 2, BEV_PX / (2 * BEV_RANGE_M)
    # under the rings and axes, which stay legible over it
    caption = _shade_uncovered(img, mount) if mount is not None else None
    cv2.line(img, (c, 0), (c, BEV_PX), GRID, 1)
    cv2.line(img, (0, c), (BEV_PX, c), GRID, 1)
    r = BEV_RING_M
    while r <= BEV_RANGE_M + 1e-6:
        cv2.circle(img, (c, c), int(round(r * k)), GRID, 1, cv2.LINE_AA)
        d = r * k * 0.7071
        _text(img, "%g" % r, (int(c + d) + 2, int(c - d) - 2), 0.35, DIM)
        r += BEV_RING_M
    _text(img, "BOW", (c - _text_w("BOW") // 2, 12), 0.38, DIM)
    _text(img, "PORT", (4, c - 4), 0.38, DIM)
    _text(img, "STBD", (BEV_PX - _text_w("STBD") - 4, c - 4), 0.38, DIM)
    rings = "rings %g m" % BEV_RING_M
    _text(img, rings, (BEV_PX - _text_w(rings) - 4, 12), 0.35, DIM)
    _legend(img, 40, BEV_PX - 14, 100, "%g m" % BEV_Z_LO, "%g m height" % BEV_Z_HI)
    if caption:
        (x, y), lines = caption
        for i, s in enumerate(lines):
            _text(img, s, (x - _text_w(s) // 2, y + 14 * i), 0.38, DIM)
    return img


_GRID = None                    # (mount key, image): rebuilt if the mount changes


def lidar_jpeg(msg, mount):
    """/crusader/mid360/points -> 400x400 top-down JPEG in the BOAT frame:
    bow up, port left, starboard right, +/-25 m, points coloured by height. The
    half the LiDAR does not scan is hatched "no coverage" (_shade_uncovered)."""
    global _GRID
    key = (tuple(mount["t"]), float(mount.get("fov_deg", 360.0)))
    if _GRID is None or _GRID[0] != key:
        _GRID = (key, _bev_grid(mount))
    img = _GRID[1].copy()
    b = to_body(cloud_xyz(msg), mount)
    c, k = BEV_PX // 2, BEV_PX / (2 * BEV_RANGE_M)
    u = np.round(c - b[:, 1] * k).astype(np.int64)      # +y (port) -> left
    v = np.round(c - b[:, 0] * k).astype(np.int64)      # +x (forward) -> up
    keep = (u >= 0) & (u < BEV_PX - 1) & (v >= 0) & (v < BEV_PX - 1)
    u, v, z = u[keep], v[keep], b[keep, 2]
    order = np.argsort(z, kind="stable")                # higher points drawn last, on top
    u, v = u[order], v[order]
    col = _ramp((z[order] - BEV_Z_LO) / (BEV_Z_HI - BEV_Z_LO))
    for du, dv in ((0, 0), (1, 0), (0, 1), (1, 1)):     # 2x2 px dots
        img[v + dv, u + du] = col
    # the boat, bow up, drawn over its own hull returns
    cv2.fillPoly(img, [np.array([(c, c - 9), (c - 5, c + 6), (c + 5, c + 6)], np.int32)], INK)
    _text(img, "%d pts" % len(z), (4, 12), 0.35, DIM)
    return _jpeg(img)


# ------------------------------------------------------------------ hub

class View:
    """One sensor view's state. Every field is guarded by SensorHub.lock."""

    def __init__(self, name, topic, msg_type, convert, period_s):
        self.name, self.topic, self.msg_type = name, topic, msg_type
        self.convert, self.period_s = convert, period_s
        self.subscribed = False
        self.wanted_t = 0.0             # time of the last HTTP request
        self.last_conv = 0.0            # time of the last conversion attempt
        self.jpeg, self.jpeg_t = None, None
        self.frames = 0                 # converted since this subscription began
        self.error = None


class SensorHub:
    """The three views over one gz Node (task1_panel's), subscribed lazily.

    LOCK: self.lock is a leaf. Never held across a gz call or a conversion, so
    neither the HTTP threads nor gz's callback thread can wait on the other.
    """

    def __init__(self, node, idle_s=IDLE_S):
        self.node, self.idle_s = node, idle_s
        self.lock = threading.Lock()
        self._wake = threading.Event()
        self._quit = threading.Event()
        try:
            mount, mount_err = lidar_mount(), None
        except Exception as e:                 # noqa: BLE001 -- the view reports it
            mount, mount_err = None, "no LiDAR mount from crusader_params.yaml: %s" % e

        def lidar(msg):
            # no mount -> no picture: a guessed frame could draw the course mirrored
            if mount is None:
                raise ValueError(mount_err)
            return lidar_jpeg(msg, mount)

        self.views = {v.name: v for v in (
            View("rgb", "/crusader/preview/rgb", Image, rgb_jpeg, 0.18),
            View("depth", "/crusader/preview/depth", Image, depth_jpeg, 0.18),
            View("lidar", "/crusader/mid360/points", PointCloudPacked, lidar, 0.28),
        )}

    # ---------------------------------------------------------- HTTP threads
    def get(self, name):
        """(jpeg or None, age_s or None, why-no-frame or None). KeyError if
        there is no such view. Stamps the request; never touches gz."""
        v = self.views[name]
        now = _now()
        with self.lock:
            fresh = now - v.wanted_t >= self.idle_s
            v.wanted_t = now
            out = (v.jpeg, None if v.jpeg_t is None else now - v.jpeg_t,
                   self._why_blank(v))
        if fresh:
            self._wake.set()            # subscribe now, not on the next tick
        return out

    def status(self):
        now = _now()
        with self.lock:
            return {n: {"on": v.subscribed, "frames": v.frames,
                        "age": None if v.jpeg_t is None else now - v.jpeg_t,
                        "error": self._why_blank(v) if v.jpeg is None else v.error}
                    for n, v in self.views.items()}

    def _why_blank(self, v):
        if self.node is None:
            return "gz-transport unavailable"
        if v.error:
            return v.error
        return "no frames" if v.subscribed else "subscribing"

    # ---------------------------------------------------------- worker
    def start(self):
        threading.Thread(target=self._run, name="panel_sensors", daemon=True).start()
        return self

    def stop(self):
        self._quit.set()
        self._wake.set()

    def _run(self):
        while not self._quit.is_set():
            self.tick()
            self._wake.wait(TICK_S)
            self._wake.clear()
        for v in self.views.values():
            self._set_subscribed(v, False)

    def tick(self, now=None):
        """Subscribe views that are asked for, unsubscribe those that are not."""
        if self.node is None:
            return
        now = _now() if now is None else now
        for v in self.views.values():
            with self.lock:
                want = now - v.wanted_t < self.idle_s
                have = v.subscribed
            if want != have:
                self._set_subscribed(v, want)

    def _set_subscribed(self, v, on):
        """The only place gz is called. Worker thread only."""
        if on:
            with self.lock:                     # before subscribe: a frame may land at once
                v.subscribed, v.frames, v.error = True, 0, None
            ok = self.node.subscribe(v.msg_type, v.topic, lambda msg: self._on_msg(v, msg))
            if not ok:
                with self.lock:
                    v.subscribed, v.error = False, "gz subscribe to %s failed" % v.topic
        else:
            with self.lock:
                was = v.subscribed
                v.subscribed, v.jpeg, v.jpeg_t = False, None, None
            if was:
                self.node.unsubscribe(v.topic)

    # ---------------------------------------------------------- gz thread
    def _on_msg(self, v, msg):
        now = _now()
        with self.lock:
            if not v.subscribed or now - v.last_conv < v.period_s:
                return
            v.last_conv = now
        try:
            jpeg, err = v.convert(msg), None
        except Exception as e:                  # noqa: BLE001 -- shown on the page
            jpeg, err = None, "%s: %s" % (type(e).__name__, e)
        with self.lock:
            if not v.subscribed:
                return                          # unsubscribed mid-conversion: stay blank
            v.jpeg, v.jpeg_t, v.error = jpeg, now if jpeg else None, err
            if jpeg:
                v.frames += 1
