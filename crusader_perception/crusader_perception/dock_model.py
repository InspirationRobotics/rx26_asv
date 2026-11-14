"""dock_model: camera_node's `dock` model, the CV team's Task 3 dock detector
(firefighting-cv specs/dock_detector_node.md). Each frame -> a DockObservation: the bays,
their indicator, each window's state and its aim point x, y, z in camera_link, which is
what bt_runner_node's Task 3 trees read.

Per frame, the chain the CV team runs offline (ffcv/demo/run_demo.py @ e1173c5):
  1. area-downscale to the detector's 640x400 (the exact box filter of their training
     export, not bilinear)
  2. their YOLO11n (best.pt: bay_face / window / dock_indicator)
  3. geometry_core: drop water reflections, group windows and indicator per face, window
     index from the template's slots (never from colour)
  4. colour_core on the full-resolution frame: window states, lit window, indicator colour
  5. dock_sequence_core: steady / flash / code timing per window
  6. dock_obs_core: the face plane from depth, the windows' x, y, z, the message

THE INDICATOR, two ways (`dock_indicator`, live: the tree's SetCameraModel sets it):
  yolo  the detector's dock_indicator box, its colour by colour_core (step 4). The default.
  cv    colour thresholds at the base of each face (indicator_cv_core): no indicator box
        needed, for when the detector finds the faces but not the indicators. The windows
        are read as before. The picture shows the search box and the red / green shares.

THE MODEL IS INTERIM: its card says it was trained on the incorrectly built mock-up bay.

dock_ffcv_dir holds best.pt, colour_core.py, geometry_core.py, dock_sequence_core.py,
window_template.json, ffcv.yaml and colour_eval_full.json (the fitted thresholds).

THE WEIGHTS, `dock_weights` (camera_node, dynamic, a change reloads the dock model): "" is
dock_ffcv_dir's best.pt, the current (new) detector; a path is another detector's .pt, run
with dock_ffcv_dir's cores, config and thresholds all the same. Task 3 trees set it with
SetCameraModel (task3_scan_fire_old_dock names the Oct 3 detector), so the GCS mission picks
the detector and no YAML is edited.
"""
import json
import os
import sys
import time

import numpy as np

from crusader_msgs.msg import DockBay, DockObservation, DockWindow
from crusader_perception import dock_obs_core, indicator_cv_core, indicator_cv_settings

INDICATOR_MODES = ("yolo", "cv")
PARAMS = {
    "dock_ffcv_dir": "/root/robotx_ws/models/ffcv",
    "dock_conf": 0.0,                   # detector confidence; 0 = ffcv.yaml's eval.conf
    "dock_indicator": "yolo",           # yolo | cv: how the indicator's colour is read (above)
    "dock_weights": "",                 # "" = dock_ffcv_dir/best.pt; else a .pt path (above)
    # cv's settings, dock_cv_<name> (indicator_cv_settings.DEFAULTS). A call into a pure-Python
    # module: check_config imports it to read these without ROS or numpy
    **indicator_cv_settings.as_params(),
}
# Changed on the running model, between two frames (camera_node's DYNAMIC has them)
LIVE = ("dock_indicator", *indicator_cv_settings.as_params())


def live_settings(p):
    """(mode, cv settings) from camera_node's parameters `p`. ValueError, saying what is
    wrong, for a value that cannot work, so a bad `ros2 param set` is refused whole."""
    mode = str(p.get("dock_indicator", "yolo")).strip().lower()
    if mode not in INDICATOR_MODES:
        raise ValueError(f"dock_indicator must be one of {', '.join(INDICATOR_MODES)}, "
                         f"not {mode!r}")
    return mode, indicator_cv_settings.from_params(p)


# Changing one reloads the dock model (camera_node), unlike LIVE
RELOAD = ("dock_weights",)


def weights_path(p, check=True):
    """The detector's .pt from camera_node's parameters `p`: dock_weights, or dock_ffcv_dir's
    best.pt when it is "". With `check`, ValueError, saying what is wrong, for a named file
    that is not an existing .pt, so a bad `ros2 param set` is refused before anything reloads.
    The default is not checked here: a missing best.pt fails the load itself, as it always has."""
    named = str(p.get("dock_weights", "") or "").strip()
    if not named:
        return os.path.join(p["dock_ffcv_dir"], "best.pt")
    if check:
        if not named.endswith(".pt"):
            raise ValueError(f"dock_weights must be a .pt file, not {named!r}")
        if not os.path.isfile(named):
            raise ValueError(f"dock_weights {named!r} does not exist")
    return named


DET_W, DET_H = 640, 400
FRAME_ID = "camera_link"
COL = {"red": (40, 40, 255), "green": (40, 230, 40), "blue": (255, 120, 60),
       "off": (200, 200, 200), "unknown": (0, 160, 255)}          # BGR
FACE, IND_NONE = (0, 255, 255), (255, 0, 255)


def load_ffcv(d):
    """The CV team's cores, config, thresholds and template from directory d."""
    import yaml
    if d not in sys.path:
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


def observe(rgb, dets, cfg, ccfg, template, cc, gc, indicator="yolo", cv=None):
    """run_demo.observe on a numpy RGB frame: dets {class: [(box, conf)]} in full-res
    pixels -> bay dicts, left to right, and the number of reflections dropped.
    indicator "cv": each bay's indicator read by indicator_cv_core with the settings `cv`
    instead (its read also kept, as the bay's "cv", for the picture)."""
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
        ic, cvr = None, None
        if indicator == "cv":
            # the completed face: its base is where the indicator is, even when the image
            # edge cut the detector's box
            cvr = indicator_cv_core.read_indicator(rgb, full, [w["box"] for w in wout], cv)
            if cvr["present"]:
                ic = dict(box=cvr["box"], conf=cvr["confidence"], colour=cvr["colour"],
                          colour_conf=cvr["confidence"])
        elif ind:
            d = cc.indicator_colour(rgb, ind[0], fbox, [w["box"] for w in wout], ccfg)
            ic = dict(box=ind[0], conf=ind[1], colour=d["colour"], colour_conf=d["confidence"])
        hs = [w["box"][3] - w["box"][1] for w in wout if w["box"][1] > 1 and w["box"][3] < H - 1]
        rng = gc.range_from_size(float(np.median(hs)), cfg["geometry"]["window_size_m"], fy) if hs else None
        bays.append(dict(face=fbox, face_conf=fconf, truncated=trunc, indicator=ic,
                         windows=wout, lit=wout[lit]["index"] if lit is not None else None,
                         range_m=rng, cv=cvr,
                         bearing_deg=gc.bearing_deg((fbox[0] + fbox[2]) / 2, fx, cx)))
    return bays, len(rej)


def to_msg(obs, stamp):
    """dock_obs_core's dict -> crusader_msgs/DockObservation."""
    m = DockObservation()
    m.header.stamp = stamp
    m.header.frame_id = FRAME_ID
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


class DockModel:
    name = "dock"
    topic = "/dock/observations"
    msg_type = DockObservation

    def __init__(self, p, log):
        from ultralytics import YOLO            # CUDA container only
        self.log = log
        d = p["dock_ffcv_dir"]
        (self.cc, self.gc, self.DockSequence, self.cfg, self.ccfg, self.template,
         thr) = load_ffcv(d)
        self.classes = self.cfg["classes"]
        self.conf = p["dock_conf"] or float(self.cfg["eval"]["conf"])
        weights = self.weights = weights_path(p)
        log.info(f"loading {weights} (classes {self.classes}, conf {self.conf}, colour "
                 f"thresholds: {thr}, cores from {d})")
        self.model = YOLO(weights, task="detect")
        self.seqs = {}                          # per bay index, for the picture
        self.track_seq = self.DockSequence(self.cfg["sequence"])   # the tracked bay
        self.t0 = None
        self.det_ms = 0.0
        self.bays, self.counts, self.reflections = [], {}, 0
        self.frames, self.fps, self._rate_t, self._rate_n = 0, 0.0, time.monotonic(), 0
        self.indicator, self.cv = None, None
        self.set_params(p)
        log.info("dock model ready (INTERIM: trained on the mock-up bay)")

    def set_params(self, p):
        """The live settings (LIVE) from camera_node's parameters `p`. camera_node calls it
        between two frames (under its run lock) and has checked the values already."""
        mode, cv = live_settings(p)
        if mode != self.indicator:
            self.log.info(f"dock indicator read by {mode}" + (
                " (colour thresholds at the base of each face)" if mode == "cv" else
                " (the detector's dock_indicator box, colour_core)"))
        self.indicator, self.cv = mode, cv

    def close(self):
        pass

    def run(self, frame):
        import cv2
        bgr = frame.bgr
        if bgr.shape[1] != 1920 and self.frames == 0:
            self.log.warn(f"colour frame is {bgr.shape[1]} wide, not 1920: the detector input "
                          "is still 640x400 but no longer the exact 3x box filter")
        self.t0 = frame.t if self.t0 is None else self.t0
        t = frame.t - self.t0
        small = cv2.resize(bgr, (DET_W, DET_H), interpolation=cv2.INTER_AREA)
        scale = bgr.shape[1] / float(DET_W)
        start = time.monotonic()
        res = self.model.predict(small, imgsz=DET_W, conf=self.conf, verbose=False)[0]
        self.det_ms = 0.8 * self.det_ms + 0.2 * (time.monotonic() - start) * 1000.0
        dets = {}
        for b, c, p in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.cls.cpu().numpy(),
                           res.boxes.conf.cpu().numpy()):
            dets.setdefault(self.classes[int(c)], []).append(
                ([float(v) * scale for v in b], float(p)))
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        bays, self.reflections = observe(rgb, dets, self.cfg, self.ccfg, self.template,
                                         self.cc, self.gc, self.indicator, self.cv)
        for i, bay in enumerate(bays):
            sq = self.seqs.setdefault(i, self.DockSequence(self.cfg["sequence"]))
            bay["events"] = sq.update(t, {w["index"]: w["state"] for w in bay["windows"]
                                          if w["index"] is not None})
            bay["patterns"] = " ".join(f"#{w}:{p[0]}" + (":" + "/".join(p[1]) if p[1] else "")
                                       for w, p in sq.patterns.items())
        ti = dock_obs_core.tracked_bay(bays)
        tev = self.track_seq.update(t, {} if ti is None else
                                    {w["index"]: w["state"] for w in bays[ti]["windows"]
                                     if w["index"] is not None})
        obs = dock_obs_core.observation(bays, frame.depth, (bgr.shape[1], bgr.shape[0]),
                                        frame.intrinsics, seq=self.track_seq, seq_events=tev,
                                        fps=self.fps)
        for bay, ob in zip(bays, obs["bays"]):
            pos = {w["index"]: (w["x"], w["y"], w["z"]) for w in ob["windows"] if w["has_position"]}
            for w in bay["windows"]:
                w["xyz"] = pos.get(w["index"])
        self.bays = bays
        self.counts = {k: len(v) for k, v in dets.items()}
        self.frames += 1
        self._rate_n += 1
        now = time.monotonic()
        if now - self._rate_t >= 1.0:
            self.fps, self._rate_t, self._rate_n = self._rate_n / (now - self._rate_t), now, 0
        return to_msg(obs, frame.stamp)

    def annotate(self, img):
        """Boxes and words drawn onto `img` (full resolution, a copy the caller owns)."""
        import cv2
        fs, th = 1.1, 2
        for i, bay in enumerate(self.bays):
            f = [int(v) for v in bay["face"]]
            cv2.rectangle(img, (f[0], f[1]), (f[2], f[3]), FACE, 4)
            ind = bay["indicator"]
            txt = f"bay {i}  ind{'(cv)' if bay.get('cv') else ''}={ind['colour'] if ind else '-'}"
            if bay["range_m"]:
                txt += f"  ~{bay['range_m']:.1f} m (size)"
            txt += f"  {bay['bearing_deg']:+.0f} deg"
            cv2.putText(img, txt, (f[0], max(30, f[1] - 12)), cv2.FONT_HERSHEY_SIMPLEX, fs, FACE,
                        th + 1)
            if ind:
                b = [int(v) for v in ind["box"]]
                cv2.rectangle(img, (b[0], b[1]), (b[2], b[3]), COL.get(ind["colour"], IND_NONE), 4)
            cvr = bay.get("cv")
            if cvr and cvr["roi"]:              # where cv looked, and what it counted there
                x0, y0, x1, y1 = cvr["roi"]
                cv2.rectangle(img, (x0, y0), (x1, y1), (255, 255, 255), 2)
                cv2.putText(img, f"cv R {100 * cvr['red']:.1f}% G {100 * cvr['green']:.1f}%",
                            (x0, min(img.shape[0] - 10, y1 + 34)), cv2.FONT_HERSHEY_SIMPLEX, fs,
                            (255, 255, 255), th)
            for w in bay["windows"]:
                c = COL.get(w["state"], (255, 255, 255))
                b = [int(v) for v in w["box"]]
                cv2.rectangle(img, (b[0], b[1]), (b[2], b[3]), c,
                              6 if w["state"] in ("red", "green", "blue") else 2)
                cv2.putText(img, f"#{w['index']} {w['slot']} {w['state']} {w['state_conf']:.2f}",
                            (b[0], b[3] + 34), cv2.FONT_HERSHEY_SIMPLEX, fs, c, th)
                if w.get("xyz") is not None:     # the aim point the tree steers by
                    x, y, _z = w["xyz"]
                    cv2.putText(img, f"{x:.2f} m  {abs(y) * 100:.0f} cm {'L' if y >= 0 else 'R'}",
                                (b[0], b[3] + 72), cv2.FONT_HERSHEY_SIMPLEX, fs, c, th)
            if bay.get("patterns"):
                cv2.putText(img, "pattern " + bay["patterns"],
                            (f[0], min(img.shape[0] - 10, f[3] + 80)),
                            cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 255), th)
        return (f"dock INTERIM model  indicator by {self.indicator}  det {self.det_ms:.0f} ms  "
                "boxes: " + (", ".join(f"{k} {v}" for k, v in self.counts.items()) or "none")
                + (f"  reflections dropped {self.reflections}" if self.reflections else ""))

    def health(self):
        return dict(weights=self.weights, det_ms=round(self.det_ms, 1), boxes=self.counts,
                    reflections=self.reflections, indicator_by=self.indicator,
                    bays=[dict(indicator=b["indicator"] and b["indicator"]["colour"],
                               cv=b["cv"] and dict(red=round(b["cv"]["red"], 4),
                                                   green=round(b["cv"]["green"], 4)),
                               windows=[(w["index"], w["state"]) for w in b["windows"]],
                               lit=b["lit"], patterns=b.get("patterns", ""))
                          for b in self.bays])

    def warnings(self, fps):
        return []
