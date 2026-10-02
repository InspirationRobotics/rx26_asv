"""yolo_detect — the team's REAL YOLO detector + LED classifier on sim frames.

Sim only, ROS-free (sim_camera.py wires it to the topics). Runs in: the crsd-sim
container, with the venv that crusader_sim/scripts/setup_yolo_venv.sh builds
(torch, ultralytics). Offline, on any BGR image:

    /root/robotx_ws/venvs/yolo/bin/python -m crusader_sim.yolo_detect frame.png

WHAT IS THE REAL PIPELINE HERE, AND WHAT IS NOT. oak_detector (the boat's node)
is: detector -> association -> LED crop -> colour classifier -> per-track light
state -> label. Everything that DECIDES something is a pure function in
crusader_perception.oak_detector_core and is imported, not copied:

    led_patch      the classifier's crop (the contract with the training exporter)
    TrackTable     one identity per buoy across frames
    FlashTracker   flashing / solid / off from the lit score over time, and the
                   label words (flash_red_diamond, red_diamond, off_diamond, ...)
    LabelVoter     the colour vote used when flash_enable is false

and every tunable comes from the same oak_detector block of crusader_params.yaml
the boat reads. What differs is only the inference backend: the boat loads
TensorRT .engine files, the sim loads the .pt files they were exported from
(ultralytics on CPU). `YoloPipeline.process` is oak_detector._run_pipeline's
stages 1, 3, 4 and 5 in the same order; stage 2 (marking_shape) is skipped on
purpose: the sim's RoboBuoy carries no circle/diamond marking and Task 1 never
branches on shape, so the geometry stage would only abstain, which already means
"the detector's own class stands".

TIME. FlashTracker is fed the SIM clock (the frame's stamp), not wall time. The
boat's node uses time.monotonic() because there the world runs on wall time; in
Gazebo the beacons blink on sim time, and at RTF 0.6 a 1 s ON / 1 s OFF light
would look like a 1.7 s one to a wall-clock tracker and fail its periodicity test.

`Scoreboard` grades the detections against the sim's own truth (boxes projected
from buoy poses by sim_camera): recall, precision, colour and ENTRY/EXIT calls.
"""
import os
import queue
import threading
import time
from collections import namedtuple

import numpy as np

from crusader_sim import course as C

MODEL_DIR = "/root/robotx_ws/models/sim_yolo"
DET_PT = "crusader_det_yolo26n.pt"
CLS_PT = "crusader_led_cls.pt"
VENV_PYTHON = "/root/robotx_ws/venvs/yolo/bin/python"

# oak_detector.py's own constants (module scope there, parameters nowhere)
MIN_CROP_PX = 10
OFF_CLASS = "off"

# box (x1, y1, x2, y2) RGB px, clamped to the frame; label as oak_detector would
# publish it; colour = the lit colour or None; state = flashing|solid|off|unknown
Det = namedtuple("Det", "box label conf shape colour state track")


class YoloUnavailable(RuntimeError):
    """The venv, the models or their label order is not what the pipeline needs.
    sim_camera catches it and runs the truth detector instead, loudly."""


def _oak_core():
    """crusader_perception's pure half, imported late so this module can be read
    (and the Scoreboard tested) without the perception package on the path."""
    from crusader_perception import oak_detector_core as core
    return core


# ======================================================================
# the pipeline
# ======================================================================
class YoloPipeline:
    """oak_detector's stages around two ultralytics models.

    `od` is the oak_detector ros__parameters dict from crusader_params.yaml.
    imgsz / conf_min override the boat's det_imgsz_* (640) and det_conf_min (0.60)
    when above 0 — experiments only: the boat runs fixed-size engines at those."""

    def __init__(self, od, model_dir=MODEL_DIR, device="cpu", threads=4, imgsz=0, conf_min=0.0):
        try:
            import torch
            from ultralytics import YOLO
        except Exception as e:           # ImportError, or a numpy ABI mismatch
            raise YoloUnavailable(
                f"torch/ultralytics do not import ({type(e).__name__}: {e}). Run "
                "crusader_sim/scripts/setup_yolo_venv.sh and start the node with "
                f"{VENV_PYTHON}") from e
        paths = [os.path.join(model_dir, f) for f in (DET_PT, CLS_PT)]
        for p in paths:
            if not os.path.isfile(p):
                raise YoloUnavailable(f"model file {p} is missing "
                                      "(setup_yolo_venv.sh copies it)")
        torch.set_num_threads(max(1, int(threads)))
        self.device = device
        self.core = _oak_core()
        self.detector = YOLO(paths[0], task="detect")
        self.classifier = YOLO(paths[1], task="classify")

        self.shapes = list(od["det_labels"])
        self.colours = list(od["cls_labels"])
        # the boat trusts these lists by INDEX; if the model disagrees, every
        # label is wrong and nothing errors, so the sim refuses instead
        for what, want, model in (("detector", self.shapes, self.detector),
                                  ("classifier", self.colours, self.classifier)):
            have = [model.names[i] for i in sorted(model.names)]
            if have != want:
                raise YoloUnavailable(
                    f"{what} class names {have} != crusader_params.yaml {want} "
                    "(det_labels / cls_labels): the boat would mislabel too")
        self.det_imgsz = ((int(imgsz), int(imgsz)) if imgsz > 0 else
                          (int(od["det_imgsz_height"]), int(od["det_imgsz_width"])))
        self.cls_imgsz = int(od["cls_imgsz"])
        self.conf_min = float(conf_min) if conf_min > 0 else float(od["det_conf_min"])
        self.crop_pad = float(od["crop_pad"])
        self.crop_top = float(od["crop_top_frac"])
        self.off_index = self.colours.index(OFF_CLASS) if OFF_CLASS in self.colours else None
        self.flash_enable = bool(od["flash_enable"]) and self.off_index is not None
        self.od = od
        self.reset()
        # first predict() pays model fusion and allocator warm-up (seconds): pay it here
        self.process(np.zeros((self.det_imgsz[0], self.det_imgsz[1], 3), np.uint8), 0.0)
        self.reset()

    def reset(self):
        """Fresh tracks and light-state windows (the boat's node starts with these)."""
        od, core = self.od, self.core
        self.tracks = core.TrackTable(iou_match=od["iou_match"], max_missed=int(od["max_missed"]))
        self.voter = core.LabelVoter(self.colours, decay=od["vote_decay"], min_vote=od["vote_min"])
        self.flash = core.FlashTracker(
            window_s=od["flash_window_s"], min_span_s=od["flash_min_span_s"],
            min_samples=int(od["flash_min_samples"]), solid_min=od["solid_min"],
            off_max=od["off_max"], colour_min_share=od["colour_min_share"],
            flash_period_s=od["flash_period_s"], flash_corr_min=od["flash_corr_min"])

    # -- stage 1
    def _detect(self, bgr):
        r = self.detector.predict(bgr, verbose=False, imgsz=self.det_imgsz,
                                  conf=self.conf_min, device=self.device)[0]
        xyxy = r.boxes.xyxy.cpu().numpy()
        cls = r.boxes.cls.cpu().numpy().astype(int)
        conf = r.boxes.conf.cpu().numpy()
        keep = [i for i in range(len(cls)) if 0 <= cls[i] < len(self.shapes)]
        return ([tuple(int(v) for v in xyxy[i]) for i in keep],
                [self.shapes[cls[i]] for i in keep], [float(conf[i]) for i in keep])

    # -- stage 4
    def _classify(self, bgr, boxes):
        """{box index: probability vector}; a box too small to crop has no entry."""
        crops, index = [], []
        for i, box in enumerate(boxes):
            patch = self.core.led_patch(bgr, box, pad=self.crop_pad, top_frac=self.crop_top,
                                        size=self.cls_imgsz, min_height_px=MIN_CROP_PX)
            if patch is not None:
                crops.append(patch)
                index.append(i)
        if not crops:
            return {}
        results = self.classifier.predict(crops, imgsz=self.cls_imgsz, verbose=False,
                                          device=self.device)
        return {i: r.probs.data.cpu().numpy() for i, r in zip(index, results)}

    # -- stages 3 and 5
    def _label(self, track, probs, shape, now):
        """(label, colour, state word) — oak_detector._run_pipeline's labelling."""
        if probs is None:
            return shape, None, None
        frame_colour = self.colours[int(np.argmax(probs))]
        if frame_colour == OFF_CLASS:
            frame_colour = None
        if self.flash_enable:
            # the soft lit score, as on the boat (argmax would turn a shrug into a vote)
            state = self.flash.update(track, now, 1.0 - float(probs[self.off_index]),
                                      frame_colour)
            return state.label(shape), state.colour, state.state
        colour, _ = self.voter.update(track, probs)
        return (f"{colour}_{shape}" if colour else shape), colour, None

    def process(self, bgr, now):
        """[Det] for one BGR frame stamped `now` (seconds; sim time)."""
        h, w = bgr.shape[:2]
        boxes, shapes, confs = self._detect(bgr)
        # association BEFORE classification, as on the boat: a crop that fails to
        # cut must not cost a track its light-state window
        ids = self.tracks.update(boxes)
        live = self.tracks.live_ids
        self.voter.prune(live)
        self.flash.prune(live)
        probs = self._classify(bgr, boxes)
        out = []
        for i, box in enumerate(boxes):
            label, colour, state = self._label(ids[i], probs.get(i), shapes[i], now)
            x1, y1, x2, y2 = box
            out.append(Det((max(0, x1), max(0, y1), min(w - 1, x2), min(h - 1, y2)),
                           label, confs[i], shapes[i], colour, state, ids[i]))
        return out


class AsyncRunner:
    """Runs YoloPipeline on the newest frame in a worker thread, one frame at a time.

    The ROS node never waits for inference: it offers each frame to `wants()` /
    `submit()` and collects finished work with `take()`. A frame offered while
    the worker is busy is simply not taken (the node does not even convert it),
    so a slow CPU lowers the detection rate instead of building a backlog."""

    def __init__(self, pipeline, rate_hz, log):
        self.pipe, self.log = pipeline, log
        self.period = 1.0 / float(rate_hz)
        self.last_t = -1e9
        self.busy = False
        self.errors = 0
        self.infer_ms = 0.0
        self._in = queue.Queue(maxsize=1)
        self._out = queue.SimpleQueue()
        threading.Thread(target=self._loop, name="yolo", daemon=True).start()

    def wants(self, t):
        """True when a frame stamped t (sim s) should be handed over now."""
        # 1e-6: stamps are floats, and 10.2 - 10.0 is 0.19999999999999929 s
        return not self.busy and (t - self.last_t >= self.period - 1e-6 or t < self.last_t)

    def submit(self, bgr, t):
        self.busy, self.last_t = True, t
        self._in.put((bgr, t))

    def take(self):
        """[(stamp, [Det])] finished since the last call, oldest first."""
        out = []
        while True:
            try:
                out.append(self._out.get_nowait())
            except queue.Empty:
                return out

    def _loop(self):
        while True:
            bgr, t = self._in.get()
            t0 = time.monotonic()
            try:
                dets = self.pipe.process(bgr, t)
                self.infer_ms = 0.8 * self.infer_ms + 0.2 * (time.monotonic() - t0) * 1000.0
                self._out.put((t, dets))
            except Exception as e:           # a bad frame must not end the thread
                self.errors += 1
                self.log(f"yolo inference failed ({type(e).__name__}: {e})")
            finally:
                self.busy = False


# ======================================================================
# the score
# ======================================================================
# One truth buoy as the Scoreboard needs it. label = what a perfect detector
# would call it (course.LABEL_OF); scored = big enough that the oracle detector
# would box it (min_bbox_px), so a buoy the model finds but the oracle would not
# is still a true positive for PRECISION without being a recall target;
# occluded = another surface is in front of it in the depth image, so no
# detector could see it and it leaves the recall denominator.
Truth = namedtuple("Truth", "name label rng hpx box scored occluded")

IOU_MATCH = 0.30                 # truth boxes are the idealised 0.43 x 0.41 m
                                 # silhouette, not the mesh outline: loose on purpose
RANGE_BANDS = ((0.0, 10.0), (10.0, 20.0), (20.0, 1e9))
_STATE_OF_LABEL = {v: k for k, v in C.LABEL_OF.items()}       # red_buoy -> flash_red
CSV_HEAD = ("sim_t,buoy,truth_label,range_m,bbox_h_px,scored,occluded,matched,iou,"
            "det_conf,det_label,det_state\n")


def truth_beacon(label):
    """(colour, flashing) a truth label stands for: red_buoy -> ("red", True)."""
    state = _STATE_OF_LABEL[label]
    return ("off", False) if state == "off" else (state.split("_")[-1], state.startswith("flash"))


def match_boxes(truth_boxes, det_boxes, iou_min=IOU_MATCH):
    """{truth index: (det index, IoU)}, greedy by IoU (each side used once)."""
    iou = _oak_core().iou
    pairs = sorted(((iou(t, d), ti, di) for ti, t in enumerate(truth_boxes)
                    for di, d in enumerate(det_boxes)), reverse=True)
    out, used = {}, set()
    for v, ti, di in pairs:
        if v < iou_min:
            break
        if ti not in out and di not in used:
            out[ti] = (di, v)
            used.add(di)
    return out


class Tally:
    """Counts over a stretch of frames."""
    FIELDS = ("frames", "truth", "occluded", "dets", "tp", "tp_scored", "colour_ok",
              "colour_bad", "colour_none", "blue_ok", "blue_n", "blue_undecided", "iou_sum")

    def __init__(self):
        self.n = dict.fromkeys(self.FIELDS, 0)
        self.band = [[0, 0] for _ in RANGE_BANDS]            # [found, truth] per range band

    def line(self, with_bands):
        n = self.n

        def frac(a, b):                         # blank over a guess when b is 0
            return f"{a / b:.2f} ({a}/{b})" if b else "n/a (0)"
        s = (f"recall {frac(n['tp_scored'], n['truth'])} precision {frac(n['tp'], n['dets'])}"
             f" | colour {n['colour_ok']} right / {n['colour_bad']} wrong / {n['colour_none']} unresolved"
             f" | ENTRY/EXIT {n['blue_ok']}/{n['blue_n']} ({n['blue_undecided']} undecided)"
             f" | mean IoU {n['iou_sum'] / n['tp_scored']:.2f}" if n["tp_scored"] else
             f"recall {frac(n['tp_scored'], n['truth'])} precision {frac(n['tp'], n['dets'])}"
             " | colour n/a | ENTRY/EXIT n/a | mean IoU n/a")
        if with_bands:
            s += " | recall by range " + ", ".join(
                (f"{lo:.0f}-{hi:.0f} m" if hi < 1e8 else f">{lo:.0f} m") + f" {f}/{t}"
                for (lo, hi), (f, t) in zip(RANGE_BANDS, self.band))
        return s


class Scoreboard:
    """Detections against truth, frame by frame: `update`, then `report` every ~10 s."""

    def __init__(self, log_path="", iou_min=IOU_MATCH):
        self.iou_min = iou_min
        self.run, self.window = Tally(), Tally()
        self._f = None
        if log_path:
            fresh = not os.path.isfile(log_path) or os.path.getsize(log_path) == 0
            self._f = open(log_path, "a", encoding="utf-8", newline="\n")
            if fresh:
                self._f.write(CSV_HEAD)

    def update(self, t, truths, dets):
        """Grade one frame's [Det] against its [Truth]."""
        hit = match_boxes([x.box for x in truths], [d.box for d in dets], self.iou_min)
        for tally in (self.run, self.window):
            self._count(tally, truths, dets, hit)
        if self._f is not None:
            self._write(t, truths, dets, hit)

    @staticmethod
    def _count(tally, truths, dets, hit):
        n = tally.n
        n["frames"] += 1
        n["dets"] += len(dets)
        n["tp"] += len(hit)
        for ti, x in enumerate(truths):
            if x.occluded:
                n["occluded"] += 1
            if not x.scored or x.occluded:
                continue
            n["truth"] += 1
            band = next(b for b, (lo, hi) in zip(tally.band, RANGE_BANDS) if lo <= x.rng < hi)
            band[1] += 1
            if ti not in hit:
                continue
            di, v = hit[ti]
            n["tp_scored"] += 1
            band[0] += 1
            n["iou_sum"] += v
            colour, flashing = truth_beacon(x.label)
            seen = dets[di].colour or ("off" if dets[di].state == "off" else None)
            # a label with no colour in it (a track too young for the flash tracker, or a
            # classifier hedging) is UNRESOLVED, which is not the same as wrong
            n["colour_none" if seen is None else "colour_ok" if seen == colour else "colour_bad"] += 1
            if colour == "blue":
                n["blue_n"] += 1
                if dets[di].state in ("flashing", "solid"):
                    n["blue_ok"] += int((dets[di].state == "flashing") == flashing)
                else:
                    n["blue_undecided"] += 1

    def _write(self, t, truths, dets, hit):
        for ti, x in enumerate(truths):
            d, v = (dets[hit[ti][0]], hit[ti][1]) if ti in hit else (None, 0.0)
            self._f.write("%.3f,%s,%s,%.2f,%.0f,%d,%d,%d,%.3f,%s,%s,%s\n" % (
                t, x.name, x.label, x.rng, x.hpx, x.scored, x.occluded, d is not None, v,
                "%.2f" % d.conf if d else "", d.label if d else "", d.state if d else ""))
        matched = {di for di, _ in hit.values()}
        for di, d in enumerate(dets):
            if di not in matched:                       # a box that is no buoy in view
                self._f.write("%.3f,,,,,,,0,0,%.2f,%s,%s\n" % (t, d.conf, d.label, d.state))
        self._f.flush()

    def close(self):
        if self._f is not None:
            self._f.close()
            self._f = None

    def report(self):
        """The ~10 s log line; resets the window."""
        w, r = self.window, self.run
        s = (f"yolo vs truth | last window {w.n['frames']} frames: {w.line(False)}"
             f" || run {r.n['frames']} frames: {r.line(True)}"
             + (f" ({r.n['occluded']} hidden-buoy sightings left out)" if r.n["occluded"] else ""))
        self.window = Tally()
        return s


# ======================================================================
# offline: python -m crusader_sim.yolo_detect frame.png [frame2.png ...]
# ======================================================================
def main(argv=None):
    import argparse
    import cv2
    import yaml
    from crusader_sim.paths import default_params_yaml

    ap = argparse.ArgumentParser(description="the real YOLO + LED classifier on image files")
    ap.add_argument("images", nargs="+")
    ap.add_argument("--model-dir", default=MODEL_DIR)
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--imgsz", type=int, default=0, help="detector input size (boat: 640)")
    ap.add_argument("--conf", type=float, default=0.0, help="detector confidence floor (boat: 0.60)")
    args = ap.parse_args(argv)
    with open(default_params_yaml(), encoding="utf-8") as f:
        od = yaml.safe_load(f)["oak_detector"]["ros__parameters"]
    pipe = YoloPipeline(od, args.model_dir, threads=args.threads, imgsz=args.imgsz,
                        conf_min=args.conf)
    for i, path in enumerate(args.images):
        img = cv2.imread(path)
        if img is None:
            print(f"{path}: cannot read")
            continue
        t0 = time.monotonic()
        dets = pipe.process(img, float(i))       # frames 1 s apart: the light state needs a clip
        print(f"{path} {img.shape[1]}x{img.shape[0]}: {len(dets)} detections "
              f"({(time.monotonic() - t0) * 1000:.0f} ms)")
        for d in dets:
            print(f"  {d.label:<22} conf {d.conf:.2f} box {d.box} track {d.track} state {d.state}")


if __name__ == "__main__":
    main()
