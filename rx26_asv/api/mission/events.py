"""RoboCommand event/status types — the planner-side mirror of
proto/robocommand.proto (and of mock_robocommand's JSON fallback shapes).

Positions arrive as lat/lon from the real link; episode scenarios may supply
world-frame x/y directly. Each event carries whichever it was given; the
planner resolves to world XY at the ingestion boundary via its converter.

No ROS imports; shared by the planner core, the robocomms client, and tests.
"""
import math
from dataclasses import dataclass
from enum import Enum

from rx26_asv.api.common import geo


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


def keepout_circle(points, zone_id: str) -> dict:
    """Circumscribe a keep-out polygon: centroid + max vertex distance.

    proto/robocommand.proto's KeepOutZone carries only a `polygon`, and the
    mock's documented script shape uses one too, so this is the only place a
    radius comes from on either framing. It must return a STRICTLY POSITIVE
    radius: radius <= 0 is the All Clear sentinel on /crsd/keepouts, so a zero
    does not merely under-report a zone — it cancels it in both consumers
    (telemetry_bridge's fence uploader and occupancy_grid_node) while the
    KEEPOUT_ACK still goes out. That is an unenforced zone that scores as
    compliant.

    Circumscribing (max, not mean) vertex distance is deliberate: the circle
    must CONTAIN the polygon, so approximation error spends clearance margin
    instead of eating into it (objective 1).

    Raises ValueError on an empty or zero-extent polygon — a keep-out we cannot
    size must fail loudly onto the malformed-frame counter, never degrade into
    a zone that is silently not enforced.
    """
    pts = [(float(a), float(b)) for a, b in points]
    if not pts:
        raise ValueError(f"keep_out_zone {zone_id!r} has an empty polygon — "
                         "cannot derive a radius")
    lat = sum(p[0] for p in pts) / len(pts)
    lon = sum(p[1] for p in pts) / len(pts)
    radius = max(math.hypot(*geo.latlon_to_xy(plat, plon, (lat, lon)))
                 for plat, plon in pts)
    if not radius > 0.0:
        raise ValueError(
            f"keep_out_zone {zone_id!r} polygon has zero extent "
            f"({len(pts)} vertices at one point) — radius <= 0 is the All "
            "Clear sentinel and would cancel the zone")
    return {"latitude": lat, "longitude": lon, "radius": radius}


def from_json_dict(d: dict):
    """Decode one mock/JSON-fallback event dict. Unknown types raise — a
    silently-dropped tasking message is a scoring failure, not a nuisance."""
    t = d.get("type")
    if t not in _JSON_TYPES:
        raise ValueError(f"unknown RoboCommand event type: {t!r}")
    cls, fields = _JSON_TYPES[t]
    kwargs = {f: d[f] for f in fields if f in d}
    # A keep-out may arrive as a polygon (proto schema + the mock's documented
    # script shape) or pre-circled as lat/lon/radius (scenario shape). Handle
    # both here so the protobuf and JSON framings cannot disagree about a
    # zone's size — the polygon form silently decoding to radius=0.0 is exactly
    # how a zone became an All Clear.
    if t == "keep_out_zone" and "polygon" in d and "radius" not in d:
        kwargs.update(keepout_circle(d["polygon"], d.get("zone_id", "?")))
    # mock scripts use x0/y0 for moving objects (scenario shape) — accept both
    if t == "moving_object":
        if "x0" in d:
            kwargs["x"] = d["x0"]
        if "y0" in d:
            kwargs["y"] = d["y0"]
    return cls(**kwargs)
