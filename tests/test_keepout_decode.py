"""Keep-out zones must survive decoding as ENFORCEABLE zones, on both framings.

Regression cover for the review finding that proto/mock keep-out zones decoded
to radius=0.0. Because `radius <= 0` is the All Clear sentinel on
/crsd/keepouts, that zero did not merely under-size a zone -- it CANCELLED it
in both consumers (telemetry_bridge's fence uploader and occupancy_grid_node)
while the KEEPOUT_ACK still went out, so an unenforced zone scored as
compliant.

The old tests missed it because they only ever used the pre-circled
{x, y, radius} scenario shape, while both the protobuf schema and the mock's
own documented script shape carry a `polygon`.
"""
import math
import queue

import pytest

from rx26_asv.api.common import geo
from rx26_asv.api.mission.events import KeepOutZone, from_json_dict, keepout_circle
from rx26_asv.api.mission.planner import MissionPlanner
from rx26_asv.api.mission.tasks.waypoint_mission import WaypointMission

# a ~110 m square keep-out off Point Loma
POLYGON = [[32.7010, -117.2510], [32.7020, -117.2510],
           [32.7020, -117.2500], [32.7010, -117.2500]]

# The identical branch both consumers use (telemetry_bridge._keepouts_cb and
# occupancy_grid_node._keepouts_cb): radius <= 0 means "All Clear, drop it".
def reads_as_all_clear(radius):
    return radius <= 0


# ---------------------------------------------------------------- decoding

def test_polygon_keepout_decodes_to_a_positive_radius():
    ev = from_json_dict({"type": "keep_out_zone", "zone_id": "K1",
                         "polygon": POLYGON})
    assert isinstance(ev, KeepOutZone) and ev.zone_id == "K1"
    assert ev.radius > 0.0
    assert not reads_as_all_clear(ev.radius)


def test_circle_contains_every_vertex():
    """Circumscribed, not averaged — the circle must CONTAIN the polygon so
    approximation error spends clearance margin instead of eating into it."""
    c = keepout_circle(POLYGON, "K1")
    for lat, lon in POLYGON:
        d = math.hypot(*geo.latlon_to_xy(lat, lon, (c["latitude"],
                                                    c["longitude"])))
        assert d <= c["radius"] + 1e-9


def test_centroid_lands_inside_the_polygon():
    c = keepout_circle(POLYGON, "K1")
    assert 32.7010 <= c["latitude"] <= 32.7020
    assert -117.2510 <= c["longitude"] <= -117.2500


def test_prewrapped_radius_shape_still_works():
    """The scenario shape {x, y, radius} must keep decoding unchanged."""
    ev = from_json_dict({"type": "keep_out_zone", "zone_id": "K2",
                         "x": 6.0, "y": 30.0, "radius": 3.0})
    assert ev.x == 6.0 and ev.y == 30.0 and ev.radius == 3.0


def test_explicit_radius_wins_over_polygon():
    ev = from_json_dict({"type": "keep_out_zone", "zone_id": "K3",
                         "polygon": POLYGON, "radius": 7.5})
    assert ev.radius == 7.5


# ------------------------------------------------------- fail-loud contract

def test_empty_polygon_raises_rather_than_cancelling_the_zone():
    with pytest.raises(ValueError, match="empty polygon"):
        from_json_dict({"type": "keep_out_zone", "zone_id": "K4",
                        "polygon": []})


def test_zero_extent_polygon_raises():
    with pytest.raises(ValueError, match="zero extent"):
        keepout_circle([[32.70, -117.25]] * 3, "K5")


# ------------------------------------------------- end-to-end through planner

class FakeComms:
    def __init__(self):
        self.sent = []

    def send_status(self, kind, ref_id, t, position=None):
        self.sent.append((kind.name, ref_id))


class RecordingSink:
    """Mirrors mission_planner_node.NodeSink."""

    def __init__(self):
        self.published = []

    def keepout(self, zone_id, x, y, radius, t):
        self.published.append((zone_id, radius))

    def clear(self, ref, t):
        self.published.append((ref, -1.0))

    def moving(self, ev, t):
        pass


def test_polygon_zone_reaches_the_avoidance_sink_enforceable():
    q = queue.Queue()
    sink, comms = RecordingSink(), FakeComms()
    planner = MissionPlanner(WaypointMission([(0.0, 50.0)], 2.0), comms, q,
                             avoidance_sink=sink,
                             to_xy=lambda la, lo: (10.0, 20.0), strict=True)
    q.put(from_json_dict({"type": "keep_out_zone", "zone_id": "K1",
                          "polygon": POLYGON}))
    planner.tick(1.0, 0.0, 0.0)

    assert ("KEEPOUT_ACK", "K1") in comms.sent
    zone_id, radius = sink.published[0]
    assert zone_id == "K1"
    assert not reads_as_all_clear(radius), \
        "zone acknowledged but published as an All Clear — not enforced"
