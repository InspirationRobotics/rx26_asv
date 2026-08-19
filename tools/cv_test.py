#!/usr/bin/env python3
"""
OAK-D LR + Jetson: two-stage buoy detection with TensorRT engines.

    stage 1   detector.engine    -> where the buoys are, and diamond vs circle
    stage 2   classifier.engine  -> what colour each LED is
    stage 3   track + vote       -> a stable answer per buoy
    output    red_diamond, blue_circle, off_diamond, ...  + range from stereo

Streams MJPEG to http://<JETSON_IP>:8080

Put this next to `buoylib.py` -- the crop geometry MUST match what
`4_export.py` produced, or the classifier sees a different picture at runtime
than it was trained on. Importing the same module is how that stays true.

Two things this does that the single-stage script did not:

1. **Crops come from the ISP stream, not the NN preview.** The detector runs on
   640x352, but a 640x352 LED strip is a handful of pixels. The ISP gives
   1280x800 of the same scene, so cropping from it hands the classifier ~2x the
   linear resolution for free. Boxes are mapped across with the same centre-crop
   maths the depth lookup uses.

2. **Batching adapts to the engine.** A TensorRT engine is built for a fixed
   batch size (default 1). Passing a list of 5 crops to a batch-1 engine either
   errors or silently processes one. This probes once at startup and falls back
   to a per-crop loop.
"""

import threading
import time
from collections import defaultdict
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import cv2
import depthai as dai
import numpy as np
from ultralytics import YOLO

import buoylib as B

# --------------------------------------------------------------------------
# config
# --------------------------------------------------------------------------
HERE = Path(__file__).parent
DET_ENGINE = str((HERE / "../../../models/detector_good.engine").resolve())
CLS_ENGINE = str((HERE / "../../../models/classifier_good.engine").resolve())

SHAPES = ["circle", "diamond"]          # detector class order (check data.yaml!)
COLOURS = ["blue", "green", "off", "red"]   # classifier folder order (alphabetical)

DET_CONF = 0.60
NN_W, NN_H = 640, 640                   # detector input
CROP_SIZE = 96                          # must match 4_export.py --crop-size
CROP_FRAC = 0.40                        # must match 4_export.py --crop-frac

USE_HIRES_CROPS = False                  # crop from ISP instead of the preview
VOTE_DECAY = 0.85                       # per-frame forgetting factor
MIN_VOTE = 0.55                         # publish only above this
MIN_LED_PX = 10                         # below this the crop is too small to trust

BOX_COLOUR = {"red": (0, 0, 255), "green": (0, 255, 0),
              "blue": (255, 80, 0), "off": (160, 160, 160), None: (200, 200, 200)}


# --------------------------------------------------------------------------
# camera
# --------------------------------------------------------------------------
pipeline = dai.Pipeline()

cam = pipeline.create(dai.node.ColorCamera)
cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
cam.setIspScale(2, 3)                   # 1920x1200 -> 1280x800
cam.setPreviewSize(NN_W, NN_H)
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam.initialControl.setAutoExposureCompensation(-9)   # range -9 .. +9
cam.setFps(30)

left = pipeline.create(dai.node.ColorCamera)
left.setCamera("left")
left.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
left.setIspScale(1, 3)
left.setFps(30)
right = pipeline.create(dai.node.ColorCamera)
right.setCamera("right")
right.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
right.setIspScale(1, 3)
right.setFps(30)

stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(False)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
left.isp.link(stereo.left)
right.isp.link(stereo.right)

xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam.preview.link(xout_rgb.input)

xout_depth = pipeline.create(dai.node.XLinkOut)
xout_depth.setStreamName("depth")
stereo.depth.link(xout_depth.input)

if USE_HIRES_CROPS:
    xout_isp = pipeline.create(dai.node.XLinkOut)
    xout_isp.setStreamName("isp")
    cam.isp.link(xout_isp.input)

device = dai.Device(pipeline)
device.setLogLevel(dai.LogLevel.WARN)
device.setLogOutputLevel(dai.LogLevel.WARN)
print("USB:", device.getUsbSpeed())
q_rgb = device.getOutputQueue("rgb", maxSize=4, blocking=False)
q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)
q_isp = device.getOutputQueue("isp", maxSize=2, blocking=False) if USE_HIRES_CROPS else None


# --------------------------------------------------------------------------
# models
# --------------------------------------------------------------------------
det_model = YOLO(DET_ENGINE, task="detect")
cls_model = YOLO(CLS_ENGINE, task="classify")

# Probe the classifier engine once: can it take a batch, or is it batch-1?
_CLS_BATCHED = True


def classify_crops(crops):
    """Run the colour classifier over a list of 96x96 crops -> list of prob vectors."""
    global _CLS_BATCHED
    if not crops:
        return []
    if _CLS_BATCHED:
        try:
            outs = cls_model.predict(crops, imgsz=CROP_SIZE, verbose=False)
            return [o.probs.data.cpu().numpy() for o in outs]
        except Exception as e:
            print(f"[classifier] batched inference failed ({type(e).__name__}), "
                  f"falling back to one crop at a time.")
            print("            Re-export the engine with a batch dimension for "
                  "more speed:")
            print("            yolo export model=classifier_best.pt format=engine "
                  "batch=8 imgsz=96 half=True")
            _CLS_BATCHED = False
    return [cls_model.predict(c, imgsz=CROP_SIZE, verbose=False)[0]
            .probs.data.cpu().numpy() for c in crops]


# --------------------------------------------------------------------------
# geometry: preview pixels -> ISP / depth pixels
# --------------------------------------------------------------------------
def preview_to(frame_shape, u, v):
    """Map a preview (NN_W x NN_H) pixel into a larger frame's pixels.

    The preview is a horizontal resize plus a VERTICAL CENTRE CROP of the ISP,
    which is why a plain scale factor is not enough.
    """
    fh, fw = frame_shape[:2]
    s = fw / float(NN_W)
    crop_off = (fh / s - NN_H) / 2.0
    return int(u * s), int((v + crop_off) * s)


def depth_at(depth_img, u, v, half=8):
    du, dv = preview_to(depth_img.shape, u, v)
    dh, dw = depth_img.shape[:2]
    u0, u1 = max(0, du - half), min(dw, du + half + 1)
    v0, v1 = max(0, dv - half), min(dh, dv + half + 1)
    roi = depth_img[v0:v1, u0:u1].astype(np.float32)
    valid = roi[roi > 0]
    if valid.size == 0:
        return None, du, dv
    return float(np.median(valid)) / 1000.0, du, dv


# --------------------------------------------------------------------------
# tracking: greedy IoU association, no external tracker dependency
# --------------------------------------------------------------------------
def iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / max(ua, 1e-6)


class Tracker:
    """Minimal IoU tracker.

    Deliberately not ByteTrack: ultralytics' trackers are fine, but wiring one
    to a TensorRT engine adds a moving part on the boat for no benefit here.
    Buoys are slow, few, and rarely occlude each other -- greedy IoU is enough,
    and it is 30 lines you can read.
    """

    def __init__(self, iou_thresh=0.25, max_missed=10):
        self.iou_thresh = iou_thresh
        self.max_missed = max_missed
        self.tracks = {}          # id -> [box, missed]
        self.next_id = 1

    def update(self, boxes):
        ids = [None] * len(boxes)
        used = set()
        pairs = []
        for tid, (tbox, _) in self.tracks.items():
            for i, b in enumerate(boxes):
                v = iou(tbox, b)
                if v >= self.iou_thresh:
                    pairs.append((v, tid, i))
        pairs.sort(reverse=True)
        claimed_t = set()
        for v, tid, i in pairs:
            if tid in claimed_t or i in used:
                continue
            claimed_t.add(tid)
            used.add(i)
            ids[i] = tid
            self.tracks[tid] = [boxes[i], 0]

        for i, b in enumerate(boxes):
            if ids[i] is None:
                ids[i] = self.next_id
                self.tracks[self.next_id] = [b, 0]
                self.next_id += 1

        for tid in list(self.tracks):
            if tid not in claimed_t and tid not in ids:
                self.tracks[tid][1] += 1
                if self.tracks[tid][1] > self.max_missed:
                    del self.tracks[tid]
        return ids


class Voter:
    """Exponentially decayed accumulation of class probabilities, per track.

    Better than a majority vote: a confident observation outweighs several
    ambiguous ones, and it can still change its mind about an early misread.
    """

    def __init__(self, classes):
        self.classes = classes
        self.acc = defaultdict(lambda: np.zeros(len(classes), np.float32))

    def update(self, tid, probs):
        a = self.acc[tid] * VOTE_DECAY + np.asarray(probs, np.float32)
        self.acc[tid] = a
        s = float(a.sum())
        if s <= 0:
            return None, 0.0
        p = a / s
        i = int(p.argmax())
        return (self.classes[i], float(p[i])) if p[i] >= MIN_VOTE else (None, float(p[i]))

    def prune(self, alive):
        for tid in list(self.acc):
            if tid not in alive:
                del self.acc[tid]


# --------------------------------------------------------------------------
# main loop
# --------------------------------------------------------------------------
latest = {"frame": None}
lock = threading.Lock()
tracker = Tracker()
voter = Voter(COLOURS)
fx = cx = None


def main_loop():
    global fx, cx
    depth_img = None
    isp_img = None
    t_prev, fps = time.time(), 0.0
    print(f"detector : {DET_ENGINE}")
    print(f"classifier: {CLS_ENGINE}")
    print("open http://<JETSON_IP>:8080")

    while True:
        d = q_depth.tryGet()
        if d is not None:
            depth_img = d.getFrame()
            if fx is None:
                calib = device.readCalibration()
                M = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A,
                                              depth_img.shape[1], depth_img.shape[0])
                fx, cx = M[0][0], M[0][2]
        if q_isp is not None:
            i = q_isp.tryGet()
            if i is not None:
                isp_img = i.getCvFrame()

        rgb_msg = q_rgb.tryGet()

        if rgb_msg is None:
            print("WARNING: No RGB frame from OAK-D")
            time.sleep(0.01)
            continue

        frame = rgb_msg.getCvFrame()

        # ---------------- stage 1: detect ----------------
        t0 = time.time()
        res = det_model.predict(frame, imgsz=(NN_H, NN_W), conf=DET_CONF,
                                verbose=False)[0]
        t_det = (time.time() - t0) * 1000

        boxes = res.boxes
        xyxy = boxes.xyxy.cpu().numpy() if boxes is not None and len(boxes) else np.zeros((0, 4))
        clsi = boxes.cls.int().cpu().tolist() if boxes is not None and len(boxes) else []

        ids = tracker.update([tuple(b) for b in xyxy])
        voter.prune(set(ids))

        # ---------------- stage 2: crop + classify ----------------
        # Crop from the ISP frame when we have one: same scene, ~2x the linear
        # resolution, which is 2x the LED pixels the classifier gets to see.
        src = isp_img if (isp_img is not None and USE_HIRES_CROPS) else frame
        crops, keep = [], []
        for i, (x1, y1, x2, y2) in enumerate(xyxy):
            if src is frame:
                px1, py1, px2, py2 = x1, y1, x2, y2
            else:
                px1, py1 = preview_to(src.shape, x1, y1)
                px2, py2 = preview_to(src.shape, x2, y2)
            poly = np.asarray([[px1, py1], [px2, py1], [px2, py2], [px1, py2]], np.float32)
            # 4-point box -> buoy_crop returns just the crop (a mask of a box
            # is the box, so it does not build one). Same call 4_export.py makes.
            crop = B.buoy_crop(src, poly)
            if isinstance(crop, tuple):
                crop = crop[0]
            if crop is None or crop.size == 0 or crop.shape[0] < MIN_LED_PX:
                continue
            n = max(8, int(round(crop.shape[0] * CROP_FRAC)))
            crops.append(cv2.resize(crop[:n], (CROP_SIZE, CROP_SIZE)))
            keep.append(i)

        t1 = time.time()
        probs = classify_crops(crops)
        t_cls = (time.time() - t1) * 1000

        prob_by_i = {i: p for i, p in zip(keep, probs)}

        # ---------------- stage 3: vote + draw ----------------
        for i, (x1, y1, x2, y2) in enumerate(xyxy):
            tid = ids[i]
            shape = SHAPES[clsi[i]] if i < len(clsi) and clsi[i] < len(SHAPES) else "buoy"
            colour, cconf = (None, 0.0)
            if i in prob_by_i:
                colour, cconf = voter.update(tid, prob_by_i[i])

            name = f"{colour}_{shape}" if colour else f"?_{shape}"
            col = BOX_COLOUR.get(colour, BOX_COLOUR[None])
            x1i, y1i, x2i, y2i = int(x1), int(y1), int(x2), int(y2)
            cv2.rectangle(frame, (x1i, y1i), (x2i, y2i), col, 2)

            label = f"#{tid} {name} {int(cconf*100)}%"
            if depth_img is not None and fx is not None:
                z, du, dv = depth_at(depth_img, (x1i + x2i) // 2, (y1i + y2i) // 2)
                if z is not None and z > 0.3:
                    x_m = (du - cx) * z / fx
                    label += f" | {z:.1f}m {x_m:+.1f}m"
            cv2.putText(frame, label, (x1i, max(y1i - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)

        now = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
        t_prev = now
        hud = (f"{fps:.0f} fps | det {t_det:.0f}ms | cls {t_cls:.0f}ms "
               f"({len(crops)}) | {'ISP' if src is not frame else 'preview'} crops")
        cv2.putText(frame, hud, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)

        with lock:
            latest["frame"] = frame


# --------------------------------------------------------------------------
# MJPEG server
# --------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with lock:
                    fr = None if latest["frame"] is None else latest["frame"].copy()
                if fr is None:
                    time.sleep(0.05)
                    continue
                ok, jpg = cv2.imencode(".jpg", fr, [cv2.IMWRITE_JPEG_QUALITY, 60])
                if ok:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b"\r\n")
                time.sleep(0.03)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    threading.Thread(target=main_loop, daemon=True).start()
    HTTPServer(("0.0.0.0", 8000), Handler).serve_forever()