"""Detection↔depth association: bbox + aligned depth frame -> BODY-frame position.

This is the diagram's 'pixel -> point cloud mapping' stage, simplified for Phase 2:
instead of building a full point cloud per frame, each detection samples a robust
median depth over the central region of its bbox and projects through the pinhole
model. (A full voxel/ICP path can slot in behind the same associate() signature
later if buoy-scale objects ever need it — flag for a Level-2 Explore round, don't
build it speculatively.)

Frame convention (matches interfaces/msg/Detection.msg):
  BODY: x = starboard+ [m], y = forward+ [m].
  Camera looks forward: image u right -> starboard, depth Z -> forward.

No ROS imports — unit-tested on any machine with numpy.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float = 0.001      # OAK-D depth is uint16 mm -> meters


@dataclass
class BodyDetection:
    label: str
    confidence: float
    x: float                        # starboard+ [m]
    y: float                        # forward+ [m]
    radius: float                   # estimated object radius [m]


def bbox_median_depth(depth: np.ndarray, bbox, shrink: float = 0.25,
                      min_valid_px: int = 20):
    """Median valid depth (meters, pre-scale applied by caller? -> raw units)
    over the central region of bbox=(x1, y1, x2, y2). Returns None if the
    region has too few valid (>0) pixels — callers must treat None as
    'no position', never as zero range."""
    x1, y1, x2, y2 = (int(v) for v in bbox)
    w, h = x2 - x1, y2 - y1
    if w <= 0 or h <= 0:
        return None
    sx, sy = int(w * shrink), int(h * shrink)
    region = depth[max(0, y1 + sy):max(0, y2 - sy),
                   max(0, x1 + sx):max(0, x2 - sx)]
    valid = region[region > 0]
    if valid.size < min_valid_px:
        return None
    return float(np.median(valid))


def project_to_body(u: float, v: float, z_m: float, cam: CameraModel,
                    mount_offset=(0.0, 0.0)):
    """Pixel (u, v) at range z_m [m] -> BODY (x starboard, y forward) [m].
    mount_offset = camera position in BODY frame (x starboard, y forward)."""
    bx = (u - cam.cx) * z_m / cam.fx + mount_offset[0]
    by = z_m + mount_offset[1]
    return bx, by


def associate(boxes, depth: np.ndarray, cam: CameraModel,
              mount_offset=(0.0, 0.0)):
    """boxes: iterable of (label, confidence, (x1, y1, x2, y2)).
    Returns list[BodyDetection]; detections without valid depth are DROPPED
    (position unknown != position zero) — callers see the count via len()."""
    out = []
    for label, conf, bbox in boxes:
        raw = bbox_median_depth(depth, bbox)
        if raw is None:
            continue
        z = raw * cam.depth_scale
        x1, y1, x2, y2 = bbox
        u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        bx, by = project_to_body(u, v, z, cam, mount_offset)
        radius = max(0.05, (x2 - x1) * z / cam.fx / 2.0)
        out.append(BodyDetection(label, float(conf), bx, by, radius))
    return out
