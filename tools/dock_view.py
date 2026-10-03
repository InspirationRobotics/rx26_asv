#!/usr/bin/env python3
"""dock_view — the CV team's Task 3 dock model, live on the OAK-D: the dock detector.

    python3 tools/dock_view.py                 # or: Start "dock_view" on the ground station

THE DOCK DETECTOR (firefighting-cv specs/dock_detector_node.md): it publishes
crusader_msgs/DockObservation on dock/observations - the bays, their indicator,
each window's state AND its aim point x, y, z in camera_link - which is what
bt_runner_node's Task 3 trees read. Numbers only, a few hundred bytes a frame:
the image and the depth stay here (crusader_perception/dock_obs_core.py fits the
face plane to the stereo depth and puts each window on it). --no-publish makes
it a viewer again. It runs, per frame, the chain the CV team runs offline
(firefighting-cv ffcv/demo/run_demo.py @ e1173c5):

  1. area-downscale the 1920x1200 frame to the detector's 640x400 (exact 3x3 box,
     the same as their training export; NOT bilinear)
  2. their YOLO11n (best.pt: bay_face / window / dock_indicator), on the GPU
  3. geometry_core: drop water reflections, group windows + indicator per face,
     window index from the template's slots (never from colour)
  4. colour_core on the FULL-RES frame: window states, lit window, indicator colour
  5. dock_sequence_core: steady / flash / code timing per window
  6. dock_obs_core: the face plane from depth, the windows' x, y, z, and the
     DockObservation (its timing verdict from the bay the spec says to track:
     the GREEN indicator's, or the only one in view)

and draws it on :8080 (/stream/annotated, /stream/raw) where oak_detector serves,
so the Camera tab shows it and the Record tab can record the raw view.

THE MODEL is whatever bundle sits in --ffcv (default ~/robotx_ws/models/ffcv).
Since 2026-10-02 that is the REBUILT-bay model (firefighting-cv
deploy/ffcv_rebuilt_2026-10-02_bayfix); the interim mock-up model is kept beside
it as models/ffcv_interim_2026-09-25. The overlay and crsd/dock_view_health name
the bundle in use. Its weak spots, measured on held-out sessions: the bay filling
the frame at firing distance, and low light.

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

# One OpenBLAS thread, set before numpy loads. The face-plane RANSAC
# (dock_obs_core.fit_plane) makes 120 small matrix products a frame; up close
# (~25k depth points) OpenBLAS splits each over all 6 cores, which are already
# busy with the LiDAR chain - measured on the boat 2026-10-02: 2.1-2.7 s a frame
# threaded, 30-70 ms single. setdefault, so a caller can still override it.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import numpy as np  # noqa: E402  (after the thread setting above, on purpose)

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
                             state=st["state"], state_conf=st["confidence"],
                             ident_conf=idn.get("confidence", idn.get("score", 1.0)),
                             lit_score=st.get("lit_score", st.get("excess", 0.0))))
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
            if w.get("xyz") is not None:     # the aim point the tree steers by
                x, y, z = w["xyz"]
                cv2.putText(img, f"{x:.2f} m  {abs(y) * 100:.0f} cm {'L' if y >= 0 else 'R'}",
                            (b[0], b[3] + 72), cv2.FONT_HERSHEY_SIMPLEX, fs, c, th)
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


def to_msg(obs, stamp, frame_id):
    """dock_obs_core's dict -> crusader_msgs/DockObservation."""
    from crusader_msgs.msg import DockBay, DockObservation, DockWindow
    m = DockObservation()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    for b in obs["bays"]:
        mb = DockBay()
        for k in ("bay_index", "detector_confidence", "truncated", "indicator_present",
                  "indicator_colour", "indicator_confidence", "lit_window_index", "lit_state",
                  "has_plane", "plane_offset", "plane_rms_m", "range_from_size_m", "bearing_deg"):
            setattr(mb, k, b[k])
        mb.bbox, mb.indicator_bbox, mb.plane_normal = b["bbox"], b["indicator_bbox"], b["plane_normal"]
        for w in b["windows"]:
            mw = DockWindow()
            for k in ("index", "slot", "identity_confidence", "state", "state_confidence",
                      "lit_score", "detector_confidence", "has_position", "x", "y", "z"):
                setattr(mw, k, w[k])
            mw.bbox = w["bbox"]
            mb.windows.append(mw)
        m.bays.append(mb)
    m.target_pattern = obs["target_pattern"]
    m.target_colours = obs["target_colours"]
    m.target_window_index = obs["target_window_index"]
    m.last_event = obs["last_event"]
    m.observed_fps = obs["observed_fps"]
    return m


def start_param_node(profile, control_q, dai, oak_controls, log):
    """A ROS node named `dock_view`: the camera settings, and (see main) the
    DockObservation publisher.

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
    ap.add_argument("--topic", default="dock/observations",
                    help="DockObservation out (= bt_runner_node.dock_topic)")
    ap.add_argument("--no-publish", action="store_true", help="a viewer only: publish nothing")
    a = ap.parse_args(argv)
    log = Log()

    import cv2
    from ultralytics import YOLO
    import depthai as dai
    from crusader_common import config as crsd_config
    from crusader_common.mjpeg_view import FrameBuffer, serve_mjpeg, stop_mjpeg
    from crusader_perception import dock_obs_core, oak_controls, oak_pipeline

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
    # Intrinsics AT THE RGB SIZE IN USE (oak_pipeline.rgb_intrinsics says why);
    # the depth is aligned to this camera, so it maps by a ratio.
    intr = oak_pipeline.rgb_intrinsics(device, width, height)
    pub = None
    if not a.no_publish:
        from crusader_msgs.msg import DockObservation
        pub = node.create_publisher(DockObservation, a.topic, 10)
        log.info(f"publishing DockObservation on {node.resolve_topic_name(a.topic)} "
                 f"(window x,y,z from the face plane; intrinsics fx {intr[0]:.0f})")
    # crsd/dock_view_health: how long each stage takes, once a second, as JSON -
    # the same String-of-JSON shape as crsd/wall_range_health. The ground
    # station's Telemetry tab shows it. Published even with --no-publish: a
    # viewer's latency is still worth seeing.
    from std_msgs.msg import String
    health_pub = node.create_publisher(String, "crsd/dock_view_health", 10)
    # Name the model by a short checksum of its weights, not its folder: bundles
    # are COPIED into models/ffcv, so the folder name is the same for every model
    # and would hide which one is loaded. 2026-10-02 rebuilt bay = 331e82f5;
    # the interim mock-up model = bdc5138c.
    import hashlib
    with open(weights, "rb") as fh:
        model_name = "model " + hashlib.sha256(fh.read()).hexdigest()[:8]
    track_seq = DockSequence(cfg["sequence"])      # the tracked bay's timing, for the message

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
    # Per-stage latency, smoothed like det_ms (EMA 0.2). pre: the 3x downscale.
    # chain: geometry + colour (full-res) + timing + the face plane - everything
    # after the detector up to the message. total: frame in hand -> published.
    pre_ms = chain_ms = total_ms = 0.0
    det_max = 0.0                         # worst detector time in the last second
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
            tf = time.monotonic()                          # frame in hand
            small = cv2.resize(bgr, (DET_W, DET_H), interpolation=cv2.INTER_AREA)
            f = bgr.shape[1] / float(DET_W)
            t1 = time.monotonic()
            pre_ms = 0.8 * pre_ms + 0.2 * (t1 - tf) * 1000.0
            res = model.predict(small, imgsz=DET_W, conf=conf, verbose=False)[0]
            t2 = time.monotonic()
            det_ms = 0.8 * det_ms + 0.2 * (t2 - t1) * 1000.0
            det_max = max(det_max, (t2 - t1) * 1000.0)
            dets = {}
            for b, c, p in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.cls.cpu().numpy(),
                               res.boxes.conf.cpu().numpy()):
                dets.setdefault(classes[int(c)], []).append(([float(v) * f for v in b], float(p)))
            # A view, not a copy: colour_core reads only crops (it converts each
            # one itself), and a full 1920x1200 copy cost 35-50 ms on the boat.
            rgb = bgr[:, :, ::-1]
            bays, nrej = observe(rgb, dets, cfg, ccfg, template, cc, gc)
            for i, bay in enumerate(bays):
                sq = seqs.setdefault(i, DockSequence(cfg["sequence"]))
                ev = sq.update(t - t0, {w["index"]: w["state"] for w in bay["windows"]
                                        if w["index"] is not None})
                bay["patterns"] = " ".join(f"#{w}:{p[0]}" + (":" + "/".join(p[1]) if p[1] else "")
                                           for w, p in sq.patterns.items())
                bay["events"] = ev
            # the message: the plane, the aim points, the tracked bay's timing
            ti = dock_obs_core.tracked_bay(bays)
            tev = track_seq.update(t - t0, {} if ti is None else
                                   {w["index"]: w["state"] for w in bays[ti]["windows"]
                                    if w["index"] is not None})
            depth = msgs["depth"].getFrame() if "depth" in msgs else None
            obs = dock_obs_core.observation(bays, depth, (bgr.shape[1], bgr.shape[0]), intr,
                                            seq=track_seq, seq_events=tev, fps=fps)
            for bay, ob in zip(bays, obs["bays"]):
                pos = {w["index"]: (w["x"], w["y"], w["z"]) for w in ob["windows"] if w["has_position"]}
                for w in bay["windows"]:
                    w["xyz"] = pos.get(w["index"])
            if pub is not None:
                try:
                    pub.publish(to_msg(obs, node.get_clock().now().to_msg(), "camera_link"))
                except Exception as e:                  # never let a message stop the camera
                    log.error(f"DockObservation not published: {e}")
            t3 = time.monotonic()
            chain_ms = 0.8 * chain_ms + 0.2 * (t3 - t2) * 1000.0
            total_ms = 0.8 * total_ms + 0.2 * (t3 - tf) * 1000.0
            n += 1
            now = time.monotonic()
            if now - last >= 1.0:
                fps, last, n = n / (now - last), now, 0
                try:                                    # never let health stop the camera
                    health_pub.publish(String(data=json.dumps(dict(
                        fps=round(fps, 2), pre_ms=round(pre_ms, 1), det_ms=round(det_ms, 1),
                        det_max_ms=round(det_max, 1), chain_ms=round(chain_ms, 1),
                        total_ms=round(total_ms, 1), model=model_name,
                        publishing=pub is not None))))
                except Exception as e:
                    log.warn(f"dock_view_health not published: {e}")
                det_max = 0.0
            counts = {k: len(v) for k, v in dets.items()}
            status = [f"dock_view  {model_name}  {fps:.1f} fps  det {det_ms:.0f} ms"
                      f"  chain {chain_ms:.0f} ms  total {total_ms:.0f} ms"
                      + ("  -> " + a.topic if pub is not None else "  (not publishing)"),
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
                               windows=[(w["index"], w["state"],
                                         w.get("xyz") and [round(v, 3) for v in w["xyz"]])
                                        for w in b["windows"]],
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
