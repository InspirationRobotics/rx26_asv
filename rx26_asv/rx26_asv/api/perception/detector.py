"""Detector — thin wrapper over the TensorRT buoy engine (or .pt for dev).

ultralytics loads .engine files natively on the Jetson (this is the ~30 fps GPU
path from CLAUDE.md). The wrapper adds:
  * class-name canonicalization via config/class_map.json — the model's 13
    MHSeals-V16 class names are mapped to the canonical labels the rest of the
    stack uses (5 LED buoy states + dock features); classes mapped to null are
    dropped at the source.
  * a hard load-time assertion that the model actually loaded and reports
    classes — a silently-empty detector is the 'silent fallback' failure mode.

Engines are per-Jetson/per-JetPack: regenerate with tools/training/train_buoy.py
--export-only if the model file fails to load after a JetPack change.
"""
import json
from dataclasses import dataclass
from pathlib import Path
from ultralytics import YOLO   # available in the asv container

CANONICAL_LABELS = {
    "buoy_off", "buoy_flash_red", "buoy_flash_green", "buoy_flash_blue",
    "buoy_solid_blue", "dock_green", "dock_red", "misc",
}
DEFAULT_CLASS_MAP = Path(__file__).parent / "config" / "class_map.json"


def load_class_map(path=None):
    with open(path or DEFAULT_CLASS_MAP) as f:
        raw = json.load(f)
    # JSON has no comments: keys starting with "_" are documentation, not model
    # classes. Dropping them here keeps the fail-loud check below about REAL
    # entries — a "_comment" string was being read as a canonical label and
    # crashed the detector at startup.
    cmap = {k: v for k, v in raw.items() if not k.startswith("_")}
    bad = {v for v in cmap.values() if v is not None} - CANONICAL_LABELS
    if bad:
        raise ValueError(f"class_map maps to unknown canonical labels: {bad}")
    return cmap


@dataclass
class Box:
    label: str          # canonical label
    confidence: float
    bbox: tuple         # (x1, y1, x2, y2) pixels


class Detector:
    def __init__(self, model_path: str, class_map_path=None, conf: float = 0.4):
        self.model = YOLO(model_path)
        self.conf = conf
        self.class_map = load_class_map(class_map_path)
        names = getattr(self.model, "names", None) or {}
        if not names:
            raise RuntimeError(
                f"model {model_path} loaded but reports no class names — refusing "
                "to run a detector whose output cannot be interpreted")
        unmapped = set(names.values()) - set(self.class_map)
        if unmapped:
            raise RuntimeError(
                f"model classes missing from class_map.json: {sorted(unmapped)} — "
                "map them (or map to null to drop) before running")

    def infer(self, frame_bgr):
        """Returns list[Box] with canonical labels; null-mapped classes dropped."""
        result = self.model(frame_bgr, conf=self.conf, verbose=False)[0]
        out = []
        names = result.names
        for b in result.boxes:
            raw = names[int(b.cls[0])]
            label = self.class_map.get(raw)
            if label is None:
                continue
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
            out.append(Box(label, float(b.conf[0]), (x1, y1, x2, y2)))
        return out
