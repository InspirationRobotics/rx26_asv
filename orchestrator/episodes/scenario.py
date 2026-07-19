"""Scenario definitions for the fixed evaluation suite (plan: 'Scenario suite').

Scenarios are versioned JSON files in orchestrator/scenarios/. All geometry is in a
local world frame: meters east (x) / north (y) of `origin` (lat, lon). Keeping them
declarative means the same file drives the kinematic backend, SITL, and eventually
field runs (fidelity increases, scenario stays comparable).

Stdlib only — the orchestrator must run anywhere (dev laptop, CI, Jetson host).
"""
from dataclasses import dataclass, field
import json
import math


@dataclass
class Obstacle:
    """Static physical obstacle (buoy, dock pile) as a circle."""
    x: float
    y: float
    radius: float = 0.3
    label: str = "obstacle"


@dataclass
class KeepOut:
    """RoboCommand static keep-out zone, circle approximation.

    Persists until All Clear — never decays (Mission 4 Advanced).
    """
    x: float
    y: float
    radius: float
    zone_id: str = "K?"
    active_from: float = 0.0     # sim seconds; injected mid-mission via events
    active_until: float = math.inf


@dataclass
class MovingObject:
    """RoboCommand moving virtual obstacle (Mission 4 Disruptive).

    Hard rule: stay outside CLEARANCE_M of current (and projected) position.
    """
    CLEARANCE_M = 10.0
    x0: float = 0.0
    y0: float = 0.0
    heading_deg: float = 0.0     # course, 0 = +y (north), clockwise
    speed_mps: float = 0.0
    object_id: str = "M?"
    system_type: str = "usv"
    active_from: float = 0.0

    def position(self, t: float):
        dt = max(0.0, t - self.active_from)
        h = math.radians(self.heading_deg)
        return (self.x0 + math.sin(h) * self.speed_mps * dt,
                self.y0 + math.cos(h) * self.speed_mps * dt)


@dataclass
class Event:
    """Timed scenario event. Phase 0 supports keep-out / moving-object injection;
    Phase 4 adds assistance_request (interrupt/resume)."""
    t: float
    type: str                    # inject_keepout | inject_moving | all_clear
    data: dict = field(default_factory=dict)


@dataclass
class Scenario:
    name: str
    version: str
    origin: tuple                       # (lat, lon) world anchor
    waypoints: list                     # [(x, y), ...] the scripted route
    obstacles: list = field(default_factory=list)      # [Obstacle]
    keepouts: list = field(default_factory=list)       # [KeepOut] active at t=0
    moving_objects: list = field(default_factory=list) # [MovingObject]
    events: list = field(default_factory=list)         # [Event]
    timeout_s: float = 600.0
    wp_radius: float = 2.0              # acceptance radius (mirrors WP_RADIUS)
    avoid_margin: float = 2.0           # clearance floor (mirrors AVOID_MARGIN)

    @staticmethod
    def load(path: str) -> "Scenario":
        with open(path) as f:
            d = json.load(f)
        return Scenario(
            name=d["name"],
            version=d.get("version", "0"),
            origin=tuple(d.get("origin", (0.0, 0.0))),
            waypoints=[tuple(w) for w in d["waypoints"]],
            obstacles=[Obstacle(**o) for o in d.get("obstacles", [])],
            keepouts=[KeepOut(**k) for k in d.get("keepouts", [])],
            moving_objects=[MovingObject(**m) for m in d.get("moving_objects", [])],
            events=[Event(**e) for e in d.get("events", [])],
            timeout_s=d.get("timeout_s", 600.0),
            wp_radius=d.get("wp_radius", 2.0),
            avoid_margin=d.get("avoid_margin", 2.0),
        )
