"""RoboCommand event/status types — the planner-side mirror of
proto/robocommand.proto (and of mock_robocommand's JSON fallback shapes).

Positions arrive as lat/lon from the real link; episode scenarios may supply
world-frame x/y directly. Each event carries whichever it was given; the
planner resolves to world XY at the ingestion boundary via its converter.

No ROS imports; shared by the planner core, the robocomms client, and tests.
"""
from dataclasses import dataclass
from enum import Enum


class StatusKind(Enum):
    """Outbound VehicleStatus kinds (mirrors proto VehicleStatus.Kind)."""
    ACK_RECEIPT = "ACK_RECEIPT"
    ACK_INTENT = "ACK_INTENT"
    READINESS = "READINESS"
    RESUMPTION = "RESUMPTION"
    KEEPOUT_ACK = "KEEPOUT_ACK"
    ALLCLEAR_ACK = "ALLCLEAR_ACK"


@dataclass
class AssistanceRequest:
    request_id: str
    latitude: float = None
    longitude: float = None
    x: float = None               # world-frame alternative (scenario-driven)
    y: float = None
    domain: str = "surface"


@dataclass
class KeepOutZone:
    zone_id: str
    latitude: float = None
    longitude: float = None
    x: float = None
    y: float = None
    radius: float = 0.0


@dataclass
class MovingObjectReport:
    object_id: str
    latitude: float = None
    longitude: float = None
    x: float = None
    y: float = None
    heading_deg: float = 0.0
    speed_mps: float = 0.0
    system_type: str = "usv"


@dataclass
class AllClear:
    ref_id: str


@dataclass
class Clearance:
    request_id: str


_JSON_TYPES = {
    "assistance_request": (AssistanceRequest,
                           ("request_id", "latitude", "longitude", "x", "y",
                            "domain")),
    "keep_out_zone": (KeepOutZone,
                      ("zone_id", "latitude", "longitude", "x", "y", "radius")),
    "moving_object": (MovingObjectReport,
                      ("object_id", "latitude", "longitude", "x", "y",
                       "heading_deg", "speed_mps", "system_type")),
    "all_clear": (AllClear, ("ref_id",)),
    "clearance": (Clearance, ("request_id",)),
}


def from_json_dict(d: dict):
    """Decode one mock/JSON-fallback event dict. Unknown types raise — a
    silently-dropped tasking message is a scoring failure, not a nuisance."""
    t = d.get("type")
    if t not in _JSON_TYPES:
        raise ValueError(f"unknown RoboCommand event type: {t!r}")
    cls, fields = _JSON_TYPES[t]
    kwargs = {f: d[f] for f in fields if f in d}
    # mock scripts use x0/y0 for moving objects (scenario shape) — accept both
    if t == "moving_object":
        if "x0" in d:
            kwargs["x"] = d["x0"]
        if "y0" in d:
            kwargs["y"] = d["y0"]
    return cls(**kwargs)
