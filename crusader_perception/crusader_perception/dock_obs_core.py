"""dock_obs_core — one camera frame's dock detections -> a DockObservation, with no ROS.

tools/dock_view.py runs the CV team's chain (their YOLO, geometry_core,
colour_core, dock_sequence_core) and hands the result here; this module adds
what the chain does not compute and the tree needs, and shapes it into the
message crusader_msgs/DockObservation carries (firefighting-cv
specs/dock_detector_node.md):

  * THE FACE PLANE, from the OAK-D's stereo depth inside the face box - the
    window and indicator openings cut out (a window's own depth is a hole: you
    see through it) - by RANSAC and a least-squares refine. Its normal points
    AT the camera, and n.p + d = 0 with d > 0.
  * EACH WINDOW'S AIM POINT: the ray through the window box's centre pixel,
    pushed onto that plane. x, y, z in camera_link (REP-103: x along the
    optical axis, y left, z up) - the same frame as the plane.

Nothing image-sized leaves the Jetson: the depth is read here, and what is
published is a few hundred bytes per frame.

Pixels: every box is in RGB pixels at the size the intrinsics were read at.
Depth is ALIGNED to the RGB camera (oak_pipeline.build_rgbd), so an RGB pixel
maps to depth by a plain ratio of the two frame sizes.

numpy only; unit-tested in crusader_perception/test/test_dock_obs_core.py.
"""
import math

import numpy as np

# DockWindow.STATE_* / DockBay.COLOUR_* (the same numbering)
STATE = {"unknown": 0, "off": 1, "red": 2, "green": 3, "blue": 4}


def _state(name):
    return STATE.get(str(name).lower() if name is not None else "unknown", 0)


def pixel_ray(u, v, intr):
    """REP-103 direction (x fwd, y left, z up) of the ray through RGB pixel (u, v).
    intr = (fx, fy, cx, cy)."""
    fx, fy, cx, cy = intr
    return np.array([1.0, -(u - cx) / fx, -(v - cy) / fy])


def face_points(depth_mm, box, holes, rgb_size, intr, rmin=0.3, rmax=12.0,
                shrink=0.1, stride=2):
    """REP-103 points [m] from the depth inside `box` (RGB px), minus `holes`
    (RGB px boxes: the windows and the indicator), with `shrink` of the box
    trimmed off each side so its edges (background, the next bay) stay out."""
    W, H = rgb_size
    dh, dw = depth_mm.shape[:2]
    sx, sy = dw / float(W), dh / float(H)
    x1, y1, x2, y2 = box
    mx, my = shrink * (x2 - x1), shrink * (y2 - y1)
    u0, u1 = int(max(0, (x1 + mx) * sx)), int(min(dw, (x2 - mx) * sx))
    v0, v1 = int(max(0, (y1 + my) * sy)), int(min(dh, (y2 - my) * sy))
    if u1 <= u0 or v1 <= v0:
        return np.zeros((0, 3))
    vv, uu = np.mgrid[v0:v1:stride, u0:u1:stride]
    z = depth_mm[vv, uu].astype(float) / 1000.0
    keep = (z >= rmin) & (z <= rmax)
    ur, vr = (uu + 0.5) / sx - 0.5, (vv + 0.5) / sy - 0.5          # back to RGB px
    for hx1, hy1, hx2, hy2 in holes:
        keep &= ~((ur >= hx1) & (ur <= hx2) & (vr >= hy1) & (vr <= hy2))
    ur, vr, z = ur[keep], vr[keep], z[keep]
    fx, fy, cx, cy = intr
    return np.column_stack([z, -(ur - cx) * z / fx, -(vr - cy) * z / fy])


def fit_plane(pts, tol=0.04, iters=120, min_points=150, seed=0):
    """(normal, d, rms, n_inliers) of the plane n.p + d = 0 through pts, with
    the normal pointing AT the camera (d > 0); None if it cannot be trusted."""
    n_pts = pts.shape[0]
    if n_pts < min_points:
        return None
    rng = np.random.default_rng(seed)
    best, best_n = None, 0
    idx = rng.integers(0, n_pts, size=(iters, 3))
    for i, j, k in idx:
        a, b, c = pts[i], pts[j], pts[k]
        nrm = np.cross(b - a, c - a)
        ln = np.linalg.norm(nrm)
        if ln < 1e-9:
            continue
        nrm /= ln
        d = -float(nrm @ a)
        cnt = int((np.abs(pts @ nrm + d) <= tol).sum())
        if cnt > best_n:
            best, best_n = (nrm, d), cnt
    if best is None or best_n < min_points:
        return None
    nrm, d = best
    for _ in range(2):                       # least squares on the inliers
        inl = pts[np.abs(pts @ nrm + d) <= tol]
        if inl.shape[0] < 3:
            break
        c = inl.mean(axis=0)
        _, _, vt = np.linalg.svd(inl - c, full_matrices=False)
        nrm = vt[2] / np.linalg.norm(vt[2])
        d = -float(nrm @ c)
    if d < 0:                                 # face the camera
        nrm, d = -nrm, -d
    res = pts @ nrm + d
    mask = np.abs(res) <= tol
    rms = float(np.sqrt(np.mean(res[mask] ** 2))) if mask.any() else float("nan")
    return nrm, d, rms, int(mask.sum())


def ray_plane(ray, nrm, d):
    """Where the ray from the camera meets the plane, or None (edge-on or behind)."""
    den = float(nrm @ ray)
    if abs(den) < 1e-6:
        return None
    t = -d / den
    if t <= 0:
        return None
    return ray * t


def tracked_bay(bays):
    """The bay the timing layer should follow (dock_detector_node.md 4.6): the
    one whose indicator reads GREEN, else the only one in view, else None."""
    green = [i for i, b in enumerate(bays)
             if b.get("indicator") and str(b["indicator"].get("colour")).lower() == "green"]
    if len(green) == 1:
        return green[0]
    return 0 if len(bays) == 1 else None


def _box(b):
    return [int(max(0, min(65535, round(v)))) for v in b]


def bay_obs(i, bay, depth_mm, rgb_size, intr, plane_kw=None):
    """One DockBay (as a dict of its fields) from dock_view's bay dict. The
    windows' aim points come from the face plane when depth gives one."""
    wins = [w for w in bay["windows"] if w.get("index") is not None]
    holes = [w["box"] for w in wins] + ([bay["indicator"]["box"]] if bay.get("indicator") else [])
    plane = None
    if depth_mm is not None:
        pts = face_points(depth_mm, bay["face"], holes, rgb_size, intr)
        plane = fit_plane(pts, **(plane_kw or {}))
    windows = []
    for w in wins:
        pos = None
        if plane is not None:
            b = w["box"]
            pos = ray_plane(pixel_ray((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0, intr),
                            plane[0], plane[1])
        windows.append(dict(
            index=int(w["index"]), slot=str(w.get("slot") or ""),
            identity_confidence=float(w.get("ident_conf", 1.0)),
            state=_state(w.get("state")), state_confidence=float(w.get("state_conf") or 0.0),
            lit_score=float(w.get("lit_score") or 0.0),
            detector_confidence=float(w.get("conf") or 0.0), bbox=_box(w["box"]),
            has_position=pos is not None,
            x=float(pos[0]) if pos is not None else 0.0,
            y=float(pos[1]) if pos is not None else 0.0,
            z=float(pos[2]) if pos is not None else 0.0))
    windows.sort(key=lambda w: w["index"])
    ind = bay.get("indicator")
    lit = bay.get("lit")
    lit_state = next((w["state"] for w in windows if w["index"] == lit), 0) if lit is not None else 0
    rng = bay.get("range_m")
    return dict(
        bay_index=i, detector_confidence=float(bay.get("face_conf") or 0.0),
        bbox=_box(bay["face"]), truncated=bool(bay.get("truncated")),
        indicator_present=ind is not None,
        indicator_colour=_state(ind["colour"]) if ind else 0,
        indicator_confidence=float(ind.get("colour_conf") or 0.0) if ind else 0.0,
        indicator_bbox=_box(ind["box"]) if ind else [0, 0, 0, 0],
        windows=windows,
        lit_window_index=int(lit) if lit is not None else -1, lit_state=lit_state,
        has_plane=plane is not None,
        plane_normal=[float(v) for v in plane[0]] if plane is not None else [0.0, 0.0, 0.0],
        plane_offset=float(plane[1]) if plane is not None else 0.0,
        plane_rms_m=float(plane[2]) if plane is not None else float("nan"),
        range_from_size_m=float(rng) if rng else float("nan"),
        bearing_deg=float(bay.get("bearing_deg", float("nan"))))


def observation(bays, depth_mm, rgb_size, intr, seq=None, seq_events=(), fps=0.0):
    """A DockObservation as a dict. `seq` is the DockSequence following the
    tracked bay (or None) and `seq_events` what its update returned this frame."""
    out = dict(bays=[bay_obs(i, b, depth_mm, rgb_size, intr) for i, b in enumerate(bays)],
               target_pattern="", target_colours=[], target_window_index=-1,
               last_event="", observed_fps=float(fps))
    if seq is not None and seq.target is not None:
        out["target_window_index"] = int(seq.target)
        pat = seq.patterns.get(seq.target)
        if pat:
            out["target_pattern"] = str(pat[0])
            out["target_colours"] = [str(c) for c in pat[1]]
    for e in seq_events:
        if e.get("type") in ("hit", "hit_done", "lost"):
            out["last_event"] = e["type"]
    return out
