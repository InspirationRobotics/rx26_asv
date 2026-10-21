"""oak_controls — every tunable OAK-D camera setting, in one table.

WHY A TABLE. These settings reach the device through fifteen different depthai setters; an
if-branch per setting puts each control's name in four places (parameter, YAML, apply path,
log line), and forgetting one gives a control that reads back a value it never applied. Here
a control is one row, and camera_node's parameters, defaults and ranges come from it.

RANGES ARE DEPTHAI'S OWN, read off CameraControl's docstrings on the boat (depthai 2.32.0.0):

    brightness/contrast/saturation  -10..10, default 0
    sharpness/luma/chroma denoise     0..4,  default 1
    ae_compensation                  -9..9,  default 0
    iso                            100..1600
    white balance                 1000..12000 K

WHAT IS NOT HERE. Focus: the OAK-D LR's M12 lenses are fixed, so every
setManualFocus/setAutoFocus* call is a no-op that would read as a knob that does nothing.
Effect and scene modes: they change pixels in ways a detector was not trained on.

THE SETTINGS THAT ARE NOT ONE CALL. Exposure mode (auto vs manual are mutually exclusive and
depthai silently ignores the loser), the AE metering region (two fractions, one rectangle)
and white balance (a mode OR a temperature) are composites, handled explicitly below the
table: pretending they are simple gives a manual exposure that silently does nothing because
AE is still enabled.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Control:
    """One camera setting: what it is called, where it goes, what it accepts."""
    name: str
    group: str
    doc: str
    setter: str = ""          # CameraControl method, for the simple ones
    kind: str = "int"         # int | float | bool | enum
    lo: float = None
    hi: float = None
    choices: tuple = ()       # enum: accepted values, lower case
    enum_type: str = ""       # enum: the dai.CameraControl nested enum name
    default: object = None    # the boat's profile; camera_profiles.yaml holds the others


# ---- the simple ones: one parameter, one setter, one argument ----
SIMPLE = (
    Control("ae_compensation", "exposure",
            "AE bias in EV steps; negative is darker. Also a training "
            "contract — tools/oak_record.py takes the same flag, and the "
            "classifier learned colours at whatever it was shot at",
            setter="setAutoExposureCompensation", kind="int", lo=-9, hi=9, default=-3),
    Control("ae_max_exposure_us", "exposure",
            "shutter cap; AE spends gain past it. Without a cap depthai "
            "prioritises exposure over ISO up to frame time, which on a moving "
            "boat buys brightness with smear. 0 = uncapped",
            setter="setAutoExposureLimit", kind="int", lo=0, hi=100000, default=8000),
    Control("ae_lock", "exposure",
            "freeze AE where it is. Useful for a survey leg where the meter "
            "would otherwise breathe as the boat turns into the sun",
            setter="setAutoExposureLock", kind="bool", default=False),
    Control("anti_banding", "flicker",
            "cancels mains-frequency flicker in artificial light by "
            "constraining exposure time. Irrelevant outdoors; matters in a "
            "shed or under dock lights",
            setter="setAntiBandingMode", kind="enum",
            choices=("off", "mains_50_hz", "mains_60_hz", "auto"),
            enum_type="AntiBandingMode", default="auto"),
    Control("brightness", "image",
            "post-ISP brightness offset. Moves the picture without changing "
            "what the sensor captured, so it cannot recover a crushed shadow "
            "— fix exposure first, then trim here",
            setter="setBrightness", kind="int", lo=-10, hi=10, default=0),
    Control("contrast", "image", "post-ISP contrast",
            setter="setContrast", kind="int", lo=-10, hi=10, default=0),
    Control("saturation", "image",
            "post-ISP saturation. Reach for this carefully: the LED colour "
            "classifier separates a lit buoy from its hull ON saturation, so "
            "this moves the exact quantity the model is reading",
            setter="setSaturation", kind="int", lo=-10, hi=10, default=0),
    Control("sharpness", "image", "edge enhancement, 1 is the depthai default",
            setter="setSharpness", kind="int", lo=0, hi=4, default=1),
    Control("luma_denoise", "image",
            "luminance noise reduction, 1 is the depthai default. Worth "
            "raising once high ISO is in play, at the cost of fine detail",
            setter="setLumaDenoise", kind="int", lo=0, hi=4, default=1),
    Control("chroma_denoise", "image",
            "colour noise reduction, 1 is the depthai default",
            setter="setChromaDenoise", kind="int", lo=0, hi=4, default=1),
)

# ---- the composites: more than one parameter, or mutually exclusive ----
COMPOSITE = (
    Control("ae_mode", "exposure",
            "'auto' lets the camera meter; 'manual' pins exposure_us and iso "
            "and ignores every ae_* setting. Explicit because depthai silently "
            "ignores whichever of the two you did not mean",
            kind="enum", choices=("auto", "manual"), default="auto"),
    Control("exposure_us", "exposure",
            "shutter time when ae_mode is 'manual'. 8000 is the cap "
            "oak_record.py uses to keep a dataset free of motion blur",
            kind="int", lo=1, hi=100000, default=8000),
    Control("iso", "exposure",
            "sensitivity when ae_mode is 'manual'. 100 is this sensor's floor "
            "— measured on the boat, AE was sitting at exactly 100 with the "
            "whole gain range unused while the frame was 4 stops under",
            kind="int", lo=100, hi=1600, default=400),
    Control("ae_region_top", "exposure",
            "top of the metering band, as a fraction of frame height",
            kind="float", lo=0.0, hi=1.0, default=0.35),
    Control("ae_region_height", "exposure",
            "height of the metering band, as a fraction of frame height. "
            "0 meters the WHOLE FRAME, which is what makes a backlit scene "
            "dark: the sky averages out to a number that looks correct and "
            "puts the waterline on the floor",
            kind="float", lo=0.0, hi=1.0, default=0.45),
    Control("awb_mode", "white balance",
            "'off' uses wb_temperature_k; everything else is a preset the "
            "camera applies for you",
            kind="enum",
            choices=("auto", "off", "daylight", "cloudy_daylight", "shade",
                     "twilight", "incandescent", "fluorescent",
                     "warm_fluorescent"),
            enum_type="AutoWhiteBalanceMode", default="auto"),
    Control("wb_temperature_k", "white balance",
            "colour temperature, used only when awb_mode is 'off'. Water and "
            "sky are both strongly blue, which is the scene auto white balance "
            "is worst at — it corrects the blue away and takes the buoys' "
            "colour with it",
            kind="int", lo=1000, hi=12000, default=5000),
)

ALL = SIMPLE + COMPOSITE
BY_NAME = {c.name: c for c in ALL}

# The sensor is 1920x1200 whatever isp_denominator is, and the AE region is
# specified against the SENSOR, before ISP scaling — depthai 2.32's own words.
# Fractions therefore mean the same part of the scene at every output size,
# where a literal rectangle would silently start metering somewhere else.
SENSOR_WIDTH = 1920
SENSOR_HEIGHT = 1200


def defaults():
    """{name: default} for every control. All of them are dynamic parameters: a camera
    setting you cannot change while looking at the picture is one nobody tunes."""
    return {c.name: c.default for c in ALL}


def ranges():
    """{name: (lo, hi)} for the numeric controls; ROS enforces them on every set."""
    return {c.name: (c.lo, c.hi) for c in ALL if c.lo is not None}


def validate(values):
    """Error string for the first unacceptable value, or None.

    Enums only: ROS checks the numeric ranges itself, while a misspelled mode would otherwise
    reach depthai as a lookup failure after some other settings had already been applied.
    """
    for name, value in values.items():
        c = BY_NAME.get(name)
        if c is None or not c.choices:
            continue
        if str(value).strip().lower() not in c.choices:
            return (f"{name}={value!r} is not one of: "
                    + ", ".join(c.choices))
    return None


def region_rect(top_fraction, height_fraction):
    """The AE metering rectangle as (x, y, w, h) in SENSOR pixels, or None.

    The ISP output size would put the band at rows 140-320 of 1200 (the sky), so the meter
    would brighten for the part of the frame being excluded and the picture would get DARKER.

    Full width always: a buoy can be anywhere across the frame, and what is worth excluding is
    above the horizon, not beside it.
    """
    if not height_fraction:
        return None
    y = int(round(max(0.0, min(1.0, top_fraction)) * SENSOR_HEIGHT))
    h = int(round(max(0.0, min(1.0, height_fraction)) * SENSOR_HEIGHT))
    h = max(1, min(h, SENSOR_HEIGHT - y))
    return 0, y, SENSOR_WIDTH, h


def _enum(dai, c, value):
    return getattr(getattr(dai.CameraControl, c.enum_type),
                   str(value).strip().upper())


def apply(dai, control, values):
    """Put `values` onto a depthai CameraControl. Returns what was refused.

    Every setting is applied on every call: a CameraControl carries only what was set on it and
    the device keeps the rest, so this makes the camera's state the whole of what the
    parameters say, not a history of which sliders were touched.

    Each call is guarded separately and failures are COLLECTED, not raised: a setter missing
    from one depthai build should cost that one control, not the other fourteen or the camera
    (this runs against a device that is already open and producing frames).
    """
    refused = []

    def attempt(label, fn, *args):
        try:
            fn(*args)
        except Exception as e:
            refused.append(f"{label}: {e}")

    manual_exposure = str(values.get("ae_mode", "auto")).lower() == "manual"

    for c in SIMPLE:
        if c.name not in values:
            continue
        # Every ae_* setting is inert under manual exposure. Skipped rather
        # than sent, so the refusal list stays a list of real problems.
        if manual_exposure and c.name.startswith("ae_"):
            continue
        value = values[c.name]
        if c.kind == "enum":
            attempt(c.name, lambda v: getattr(control, c.setter)(_enum(dai, c, v)),
                    value)
        elif c.kind == "bool":
            attempt(c.name, getattr(control, c.setter), bool(value))
        else:
            attempt(c.name, getattr(control, c.setter), int(value))

    if manual_exposure:
        attempt("manual exposure", control.setManualExposure,
                int(values.get("exposure_us", 8000)),
                int(values.get("iso", 400)))
    else:
        attempt("auto exposure", control.setAutoExposureEnable)
        rect = region_rect(float(values.get("ae_region_top", 0.0)),
                           float(values.get("ae_region_height", 0.0)))
        if rect:
            attempt("ae_region", control.setAutoExposureRegion, *rect)

    awb = str(values.get("awb_mode", "auto")).strip().lower()
    if awb == "off":
        # OFF first, then the temperature: setting a temperature while the
        # camera is still auto-balancing is a value the next AWB frame
        # overwrites, which reads as the control not working.
        attempt("awb_mode", control.setAutoWhiteBalanceMode,
                _enum(dai, BY_NAME["awb_mode"], "off"))
        attempt("wb_temperature_k", control.setManualWhiteBalance,
                int(values.get("wb_temperature_k", 5000)))
    else:
        attempt("awb_mode", control.setAutoWhiteBalanceMode,
                _enum(dai, BY_NAME["awb_mode"], awb))

    return refused


def summary(values):
    """One line naming the profile, for the log and the annotated view."""
    if str(values.get("ae_mode", "auto")).lower() == "manual":
        exposure = (f"manual {int(values.get('exposure_us', 0))}us "
                    f"ISO{int(values.get('iso', 0))}")
    else:
        rect = region_rect(float(values.get("ae_region_top", 0.0)),
                           float(values.get("ae_region_height", 0.0)))
        exposure = (f"auto {int(values.get('ae_compensation', 0)):+d}EV "
                    f"cap {int(values.get('ae_max_exposure_us', 0))}us "
                    + (f"band y{rect[1]}+{rect[3]}" if rect else "full-frame"))
    awb = str(values.get("awb_mode", "auto")).lower()
    wb = (f"{int(values.get('wb_temperature_k', 0))}K" if awb == "off" else awb)
    return f"{exposure} | wb {wb}"
