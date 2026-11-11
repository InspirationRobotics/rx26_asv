"""indicator_cv_settings: the settings indicator_cv_core reads a bay's indicator with - their
defaults, the same as camera_node parameters (dock_cv_<name>), and the check every new value
passes. indicator_cv_core's docstring says what each one does.

Pure Python, no numpy: tools/scripts/check_config.py imports it to read camera_node's
parameters where there is neither ROS nor numpy (CI's config job), as it does oak_controls.
"""

PREFIX = "dock_cv_"                     # camera_node's parameter for each: dock_cv_<name>
DEFAULTS = {
    "roi": [0.15, 0.85, 0.65, 1.25],    # x0, x1, y0, y1 in face units; the LR window ends ~0.65
    "sat_min": 0.40,                    # HSV saturation, 0..1
    "val_min": 0.20,                    # HSV value, 0..1
    "red_hue": [340.0, 20.0],           # degrees; lo > hi wraps through 0
    "green_hue": [75.0, 160.0],         # degrees; past ~160 is teal water
    "min_share": 0.002,                 # the larger colour's pixels, as a share of the face
    "ratio": 2.0,                       # the larger at least this many times the other
}


def as_params():
    """DEFAULTS as camera_node parameters: {dock_cv_<name>: default}."""
    return {PREFIX + k: v for k, v in DEFAULTS.items()}


def from_params(p):
    """The settings in camera_node's parameters `p` (dock_cv_<name>), through check()."""
    return check({k: p[PREFIX + k] for k in DEFAULTS if PREFIX + k in p})


def check(cfg):
    """A complete, normalised copy of `cfg` (DEFAULTS filled in); ValueError, saying what is
    wrong, for a value that cannot work."""
    k = dict(DEFAULTS)
    k.update(cfg or {})
    try:
        roi = [float(v) for v in k["roi"]]
        red = [float(v) for v in k["red_hue"]]
        green = [float(v) for v in k["green_hue"]]
        out = dict(roi=roi, red_hue=red, green_hue=green, sat_min=float(k["sat_min"]),
                   val_min=float(k["val_min"]), min_share=float(k["min_share"]),
                   ratio=float(k["ratio"]))
    except (TypeError, ValueError) as e:
        raise ValueError(f"indicator cv settings: {e}") from e
    if len(roi) != 4 or not (roi[0] < roi[1] and roi[2] < roi[3]):
        raise ValueError(f"roi must be x0, x1, y0, y1 with x0 < x1 and y0 < y1, not {roi}")
    for name in ("red_hue", "green_hue"):
        h = out[name]
        if len(h) != 2 or not all(0.0 <= v <= 360.0 for v in h):
            raise ValueError(f"{name} must be two hues in degrees, 0..360, not {h}")
    for name in ("sat_min", "val_min"):
        if not 0.0 <= out[name] <= 1.0:
            raise ValueError(f"{name} must be 0..1, not {out[name]}")
    if not 0.0 < out["min_share"] <= 1.0:
        raise ValueError(f"min_share must be above 0 and at most 1, not {out['min_share']}")
    if out["ratio"] < 1.0:
        raise ValueError(f"ratio must be at least 1, not {out['ratio']}")
    return out
