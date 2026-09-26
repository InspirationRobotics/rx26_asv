#!/usr/bin/env python3
"""dock_view — the CV team's Task 3 dock model, live on the OAK-D, in the ground station.

    python3 tools/dock_view.py                 # or: Start "dock_view" on the ground station

A VIEWER TO TEST THE MODEL, not the dock_detector node (that one is specified in
firefighting-cv specs/dock_detector_node.md and publishes DockObservation). This
publishes no topics (only its camera parameters). It runs, per frame, the chain the CV team runs offline
(firefighting-cv ffcv/demo/run_demo.py @ e1173c5):

  1. area-downscale the 1920x1200 frame to the detector's 640x400 (exact 3x3 box,
     the same as their training export; NOT bilinear)
  2. their YOLO11n (best.pt: bay_face / window / dock_indicator), on the GPU
  3. geometry_core: drop water reflections, group windows + indicator per face,
     window index from the template's slots (never from colour)
  4. colour_core on the FULL-RES frame: window states, lit window, indicator colour
  5. dock_sequence_core: steady / flash / code timing per window

and draws it on :8080 (/stream/annotated, /stream/raw) where oak_detector serves,
so the Camera tab shows it and the Record tab can record the raw view.

THE MODEL IS INTERIM: its own model card says it was trained on the incorrectly
built mock-up bay. Expect misses on the rebuilt one; that is what this is for.

OWNS THE OAK-D: stop oak_detector / buoy_detector / oakd_publisher first (the
ground station's exclusive tag does that for you). Camera settings START from
oak_detector's section of crusader_params.yaml (same camera, same profile) and
are live parameters of a ROS node named /dock_view, declared from the same
oak_controls table, so the ground station's Camera tab tunes it and applies
saved profiles to it exactly as it does oak_detector.

Files (downloaded from firefighting-cv @ e1173c5 into --ffcv):
  best.pt, colour_core.py, geometry_core.py, dock_sequence_core.py,
  window_template.json, ffcv.yaml, colour_eval_full.json (fitted thresholds)
"""
import argparse
import json
import os
import signal
import sys
import threading
import time

import numpy as np

COL = {"red": (40, 40, 255), "green": (40, 230, 40), "blue": (255, 120, 60),
       "off": (200, 200, 200), "unknown": (0, 160, 255)}          # BGR
FACE, IND_NONE = (0, 255, 255), (255, 0, 255)
DET_W, DET_H = 640, 400


class Log:
    def info(self, m, **k): print(f"[dock_view] {m}", flush=True)
    def warn(self, m, **k): print(f"[dock_view] WARN {m}", flush=True)
    warning = warn
    def error(self, m, **k): print(f"[dock_view] ERROR {m}", flush=True)


def load_ffcv(d):
    """The CV team's cores, config, thresholds and template from directory d."""
    import yaml
    sys.path.insert(0, d)
    import colour_core as cc
    import geometry_core as gc
    from dock_sequence_core import DockSequence
    with open(os.path.join(d, "ffcv.yaml")) as f:
        cfg = yaml.safe_load(f)
    with open(os.path.join(d, "window_template.json")) as f:
        template = json.load(f)
    ccfg = json.loads(json.dumps(cfg["colour"]))
    thr = "config defaults"
    ev_path = os.path.join(d, "colour_eval_full.json")
    if os.path.exists(ev_path):              # the all-data fit, as run_demo uses
        with open(ev_path) as f:
            ev = json.load(f)
        w, i = ev["all_data_thresholds_window"], ev["all_data_thresholds_indicator"]
        ccfg["window"]["lit_excess_min"], ccfg["window"]["margin_min"] = w["threshold"], w["margin"]
        ccfg["indicator"]["min_excess"], ccfg["indicator"]["margin_min"] = i["threshold"], i["margin"]
        thr = "all-data fit (colour_eval_full.json)"
    return cc, gc, DockSequence, cfg, ccfg, template, thr


def observe(rgb, dets, cfg, ccfg, template, cc, gc):
    """run_demo.observe, on a numpy RGB frame: dets {class: [(box, conf)]}
    in full-res pixels -> bay dicts, left to right."""
    H, W = rgb.shape[:2]
    cam = cfg["camera"]
    s = W / float(cam["full_width"])
    fy, fx, cx = cam["fy_full"] * s, cam["fx_full"] * s, cam["cx_full"] * s
    faces = sorted(dets.get("bay_face", []), key=lambda x: -x[1])
    kept, rej = gc.reject_reflections([f[0] for f in faces], cfg["geometry"]["reflection_overlap"])
    faces = [faces[i] for i in kept]
    wins = list(dets.get("window", []))
    inds = list(dets.get("dock_indicator", []))
    bays = []
    for fbox, fconf in sorted(faces, key=lambda f: f[0][0]):
        full, trunc = gc.complete_face(fbox, W, H, template.get("face_aspect", 1.0),
                                       cam["occluded_top_rows_full"] * s)
        mine = [(b, c) for b, c in wins if gc.inside_frac(b, full) >= 0.5]
        wins = [w for w in wins if w not in mine]
        below = (full[0], full[1], full[2], full[3] + 0.25 * (full[3] - full[1]))
        ind = [(b, c) for b, c in inds if gc.inside_frac(b, below) >= 0.5]
        ind = max(ind, key=lambda x: x[1]) if ind else None
        if ind:
            inds.remove(ind)
        ident = gc.assign_windows(fbox, [b for b, _ in mine], template, img_size=(W, H),
                                  top_limit=cam["occluded_top_rows_full"] * s) if mine else []
        wout = []
        for (b, c), idn in zip(mine, ident):
            others = [x for x, _ in mine if x is not b] + ([ind[0]] if ind else [])
            st = cc.window_state(rgb, b, fbox, others, ccfg)
            wout.append(dict(index=idn["index"], slot=idn["slot"], box=b, conf=c,
                             state=st["state"], state_conf=st["confidence"]))
        wout.sort(key=lambda w: (w["index"] is None, w["index"] or 0))
        lit = cc.lit_window([dict(state=w["state"], confidence=w["state_conf"]) for w in wout])
        ic = None
        if ind:
            d = cc.indicator_colour(rgb, ind[0], fbox, [w["box"] for w in wout], ccfg)
            ic = dict(box=ind[0], conf=ind[1], colour=d["colour"], colour_conf=d["confidence"])
        hs = [w["box"][3] - w["box"][1] for w in wout if w["box"][1] > 1 and w["box"][3] < H - 1]
        rng = gc.range_from_size(float(np.median(hs)), cfg["geometry"]["window_size_m"], fy) if hs else None
        bays.append(dict(face=fbox, face_conf=fconf, truncated=trunc, indicator=ic,
                         windows=wout, lit=wout[lit]["index"] if lit is not None else None,
                         range_m=rng,
                         bearing_deg=gc.bearing_deg((fbox[0] + fbox[2]) / 2, fx, cx)))
    return bays, len(rej)


def draw(bgr, bays, status, cv2):
    """Boxes and words on the full-res frame; returns a half-size copy to stream."""
    img = bgr.copy()
    fs, th = 1.1, 2
    for i, bay in enumerate(bays):
        f = [int(v) for v in bay["face"]]
        cv2.rectangle(img, (f[0], f[1]), (f[2], f[3]), FACE, 4)
        ind = bay["indicator"]
        txt = f"bay {i}  ind={ind['colour'] if ind else '-'}"
        if bay["range_m"]:
            txt += f"  ~{bay['range_m']:.1f} m (size)"
        txt += f"  {bay['bearing_deg']:+.0f} deg"
        cv2.putText(img, txt, (f[0], max(30, f[1] - 12)), cv2.FONT_HERSHEY_SIMPLEX, fs, FACE, th + 1)
        if ind:
            b = [int(v) for v in ind["box"]]
            cv2.rectangle(img, (b[0], b[1]), (b[2], b[3]), COL.get(ind["colour"], IND_NONE), 4)
        for w in bay["windows"]:
            c = COL.get(w["state"], (255, 255, 255))
            b = [int(v) for v in w["box"]]
            cv2.rectangle(img, (b[0], b[1]), (b[2], b[3]), c,
                          6 if w["state"] in ("red", "green", "blue") else 2)
            cv2.putText(img, f"#{w['index']} {w['slot']} {w['state']} {w['state_conf']:.2f}",
                        (b[0], b[3] + 34), cv2.FONT_HERSHEY_SIMPLEX, fs, c, th)
        seq = bay.get("patterns")
        if seq:
            cv2.putText(img, "pattern " + seq, (f[0], min(img.shape[0] - 10, f[3] + 80)),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 255), th)
    y = 40
    for line in status:
        cv2.putText(img, line, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 6)
        cv2.putText(img, line, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        y += 40
    return cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2), interpolation=cv2.INTER_AREA)


def start_param_node(profile, control_q, dai, oak_controls, log):
    """A ROS node named `dock_view` whose only job is the camera settings.

    It declares the SAME camera controls as oak_detector (oak_controls'
    table, the same ranges and choices), starting from oak_detector's values,
    so the ground station's Camera tab can tune it and apply saved profiles
    exactly as it does oak_detector. A set is pushed to the running camera the
    way oak_detector._apply does it: validated first, the WHOLE profile
    re-sent, committed only once the device has taken it.

    Spun on its own thread; the frame loop never waits on it."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from crusader_common.param_utils import declare_from_config, make_set_callback

    rclpy.init(args=None)
    node = Node("dock_view")
    spec = oak_controls.param_spec()
    current = dict(declare_from_config(node, dict(profile), spec))
    ranges = {k: (v["lo"], v["hi"]) for k, v in spec.items() if "lo" in v}

    def apply(changes):
        err = oak_controls.validate(changes)
        if err:
            raise ValueError(err)
        merged = dict(current, **changes)
        ctl = dai.CameraControl()
        refused = oak_controls.apply(dai, ctl, merged)
        if refused:
            raise RuntimeError("; ".join(refused))
        control_q.send(ctl)
        current.update(changes)
        log.info("camera: " + oak_controls.summary(merged))

    node.add_on_set_parameters_callback(make_set_callback(node, ranges, apply))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    log.info("camera settings tunable from the ground station Camera tab (node /dock_view)")
    return node, executor


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--ffcv", default="/root/robotx_ws/models/ffcv",
                    help="directory with best.pt + the CV team's cores and config")
    ap.add_argument("--weights", default=None, help="default: <ffcv>/best.pt")
    ap.add_argument("--conf", type=float, default=None, help="default: ffcv.yaml eval.conf")
    ap.add_argument("--port", type=int, default=None, help="default: oak_detector.stream_port")
    ap.add_argument("--log", default="", help="append one JSON line per frame here")
    a = ap.parse_args(argv)
    log = Log()

    import cv2
    from ultralytics import YOLO
    import depthai as dai
    from crusader_common import config as crsd_config
    from crusader_common.mjpeg_view import FrameBuffer, serve_mjpeg, stop_mjpeg
    from crusader_perception import oak_controls, oak_pipeline

    cc, gc, DockSequence, cfg, ccfg, template, thr = load_ffcv(a.ffcv)
    classes = cfg["classes"]
    conf = a.conf if a.conf is not None else float(cfg["eval"]["conf"])
    weights = a.weights or os.path.join(a.ffcv, "best.pt")
    log.info(f"loading {weights} (classes {classes}, conf {conf}, colour thresholds: {thr})")
    model = YOLO(weights, task="detect")

    op = crsd_config.node_params("oak_detector")
    profile = {c.name: op[c.name] for c in oak_controls.ALL}
    pipeline, width, height = oak_pipeline.build_rgbd(
        isp_denominator=op["isp_denominator"], fps=op["fps"], controls=profile,
        rgb_isp_denominator=op["rgb_isp_denominator"])
    device = dai.Device(pipeline)
    queue = device.getOutputQueue("rgbd", int(op["queue_size"]), blocking=False)
    log.info(f"OAK-D open: {width}x{height} @ {op['fps']:g} fps, usb={device.getUsbSpeed().name}")
    if width != 1920:
        log.warn(f"colour frame is {width} wide, not 1920: the detector input is "
                 "still resized to 640x400, but it is no longer the exact 3x box")

    control_q = device.getInputQueue(oak_pipeline.CONTROL_STREAM, maxSize=1, blocking=False)
    node, executor = start_param_node(profile, control_q, dai, oak_controls, log)

    buf, raw = FrameBuffer(), FrameBuffer()
    views = {"annotated": (buf, "Annotated"), "raw": (raw, "Raw")}
    port = a.port or int(op["stream_port"])
    server = serve_mjpeg(port, int(op["stream_quality"]), views, log, title="dock_view")

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    exposure = ""
    logf = open(a.log, "a", buffering=1) if a.log else None
    seqs, t0, n, fps, det_ms, last = {}, None, 0, 0.0, 0.0, time.monotonic()
    try:
        while not stop.is_set():
            try:
                group = queue.tryGet()
            except RuntimeError as e:
                log.error(f"camera link dropped: {e}")
                break
            if group is None:
                time.sleep(0.005)
                continue
            msgs = {name: m for name, m in group}
            if "rgb" not in msgs:
                continue
            try:                     # what the camera actually did, for tuning
                exposure = (f"exp {msgs['rgb'].getExposureTime().total_seconds() * 1e6:.0f} us"
                            f"  ISO {msgs['rgb'].getSensitivity()}"
                            f"  {msgs['rgb'].getColorTemperature()} K")
            except Exception:
                pass
            bgr = msgs["rgb"].getCvFrame()
            t = msgs["rgb"].getTimestamp().total_seconds()   # device clock: timing layer
            t0 = t if t0 is None else t0
            small = cv2.resize(bgr, (DET_W, DET_H), interpolation=cv2.INTER_AREA)
            f = bgr.shape[1] / float(DET_W)
            t1 = time.monotonic()
            res = model.predict(small, imgsz=DET_W, conf=conf, verbose=False)[0]
            det_ms = 0.8 * det_ms + 0.2 * (time.monotonic() - t1) * 1000.0
            dets = {}
            for b, c, p in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.cls.cpu().numpy(),
                               res.boxes.conf.cpu().numpy()):
                dets.setdefault(classes[int(c)], []).append(([float(v) * f for v in b], float(p)))
            rgb = np.ascontiguousarray(bgr[:, :, ::-1])
            bays, nrej = observe(rgb, dets, cfg, ccfg, template, cc, gc)
            for i, bay in enumerate(bays):
                sq = seqs.setdefault(i, DockSequence(cfg["sequence"]))
                ev = sq.update(t - t0, {w["index"]: w["state"] for w in bay["windows"]
                                        if w["index"] is not None})
                bay["patterns"] = " ".join(f"#{w}:{p[0]}" + (":" + "/".join(p[1]) if p[1] else "")
                                           for w, p in sq.patterns.items())
                bay["events"] = ev
            n += 1
            now = time.monotonic()
            if now - last >= 1.0:
                fps, last, n = n / (now - last), now, 0
            counts = {k: len(v) for k, v in dets.items()}
            status = [f"dock_view  INTERIM model (mock-up bay)  {fps:.1f} fps  det {det_ms:.0f} ms",
                      "boxes: " + (", ".join(f"{k} {v}" for k, v in counts.items()) or "none")
                      + (f"   reflections dropped {nrej}" if nrej else ""),
                      exposure]
            if buf.viewers > 0:
                buf.put(draw(bgr, bays, status, cv2))
            if raw.viewers > 0:
                raw.put(bgr)
            if logf:
                logf.write(json.dumps(dict(
                    t=round(t - t0, 3), dets={k: [[round(x, 1) for x in b] + [round(p, 3)] for b, p in v]
                                              for k, v in dets.items()},
                    bays=[dict(indicator=b["indicator"] and b["indicator"]["colour"],
                               windows=[(w["index"], w["state"]) for w in b["windows"]],
                               lit=b["lit"], range_m=b["range_m"], patterns=b["patterns"],
                               events=b["events"]) for b in bays])) + "\n")
    finally:
        stop_mjpeg(server, buf, log)
        raw.close()
        try:
            executor.shutdown(timeout_sec=1.0)
            node.destroy_node()
            import rclpy
            rclpy.try_shutdown()
        except Exception:
            pass
        try:
            device.close()
        except Exception:
            pass
        if logf:
            logf.close()
        log.info("stopped")


if __name__ == "__main__":
    main()
