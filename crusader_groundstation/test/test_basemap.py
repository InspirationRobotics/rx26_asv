"""fetch_basemap: ring assembly, classification, compaction. Stdlib only, no network."""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools", "scripts"))
import fetch_basemap as fb  # noqa: E402

BBOX = (1.2740, 103.8480, 1.2880, 103.8640)


def sq(lat, lon, d=0.001):
    """A closed square ring [(lat, lon)], first point repeated."""
    return [(lat, lon), (lat, lon + d), (lat + d, lon + d), (lat + d, lon), (lat, lon)]


def geom(pts):
    return [{"lat": a, "lon": b} for a, b in pts]


def way(tags, pts):
    return {"type": "way", "tags": tags, "geometry": geom(pts)}


def test_classify():
    assert fb.classify({"natural": "water"}) == "water"
    assert fb.classify({"waterway": "riverbank"}) == "water"
    assert fb.classify({"natural": "coastline"}) == "water"
    assert fb.classify({"man_made": "pier"}) == "pier"
    assert fb.classify({"man_made": "bridge", "building": "yes"}) == "pier"
    assert fb.classify({"leisure": "marina"}) == "park"
    assert fb.classify({"landuse": "grass"}) == "park"
    assert fb.classify({"building": "yes"}) == "building"
    assert fb.classify({"highway": "primary_link"}) == "road"
    assert fb.classify({"highway": "residential"}) == "road"
    assert fb.classify({"highway": "footway"}) == "path"
    assert fb.classify({"highway": "steps"}) == "path"
    assert fb.classify({"highway": "bus_stop"}) is None
    assert fb.classify({"landuse": "farmland"}) is None
    assert fb.classify(None) is None


def test_assemble_split_and_reversed_ways():
    ring = sq(1.0, 103.0)
    a, b, c, d = ring[0], ring[1], ring[2], ring[3]
    closed, opened = fb.assemble_rings([[b, c], [a, b], [d, c], [d, a]])   # [d, c] runs backwards
    assert not opened and len(closed) == 1
    r = closed[0]
    assert r[0] == r[-1] and len(r) == 5 and set(r) == set(ring)


def test_assemble_reports_open_chain():
    closed, opened = fb.assemble_rings([[(0, 0), (0, 1)], [(0, 1), (1, 1)]])
    assert not closed and len(opened) == 1 and len(opened[0]) == 3


def test_two_separate_rings():
    r1, r2 = sq(1.0, 103.0), sq(2.0, 104.0)
    closed, _ = fb.assemble_rings([r1[:3], r2[:2], r1[2:], r2[1:]])
    assert len(closed) == 2


def test_holes_go_to_containing_outer():
    big, other, hole = sq(1.0, 103.0, 0.01), sq(5.0, 103.0, 0.01), sq(1.002, 103.002, 0.001)
    polys = fb.build_polys([other, big], [hole])
    by_outer = {tuple(p[0][0]): p for p in polys}
    assert len(by_outer[big[0]]) == 2 and len(by_outer[other[0]]) == 1


def test_relation_with_hole_and_split_members():
    outer, hole = sq(1.2750, 103.8490, 0.004), sq(1.2760, 103.8500, 0.001)
    rel = {"type": "relation", "tags": {"type": "multipolygon", "natural": "water", "name": "Lake"},
           "members": [{"type": "way", "role": "outer", "geometry": geom(outer[:3])},
                       {"type": "way", "role": "outer", "geometry": geom(outer[2:])},
                       {"type": "way", "role": "inner", "geometry": geom(hole)}]}
    layers, cands = fb.extract([rel], BBOX)
    assert len(layers["water"]) == 1
    poly = layers["water"][0]
    assert len(poly) == 2 and len(poly[0]) == 8 and len(poly[1]) == 8
    assert cands and cands[0][1] == "Lake"


def test_drops_outside_bbox_and_non_multipolygon_relation():
    far = way({"building": "yes"}, sq(2.0, 100.0))
    route = {"type": "relation", "tags": {"type": "route", "natural": "water"}, "members": []}
    layers, _ = fb.extract([far, route], BBOX)
    assert all(not v for v in layers.values())


def test_lines_and_piers():
    road = way({"highway": "service"}, [(1.28, 103.85), (1.28, 103.851)])
    pier = way({"man_made": "pier"}, [(1.28, 103.85), (1.2805, 103.8505)])
    pier_area = way({"man_made": "pier"}, sq(1.28, 103.85, 0.0003))
    coast = way({"natural": "coastline"}, sq(1.28, 103.852, 0.0003)[:3])
    layers, _ = fb.extract([road, pier, pier_area, coast], BBOX)
    assert layers["road"] == [[1.28, 103.85, 1.28, 103.851]]
    kinds = sorted(type(p[0]).__name__ for p in layers["pier"])
    assert kinds == ["float", "float", "list"]       # two lines (pier, coast), one area
    assert not layers["water"]


def test_rounding_and_simplify():
    pts = [(1.2800000004, 103.85), (1.2800000001, 103.8500005), (1.28, 103.851)]
    line = fb.compact_line(pts)
    assert line == [1.28, 103.85, 1.28, 103.851]            # middle point is within 0.3 m
    assert all(len(repr(v).split(".")[-1]) <= 6 for v in line)
    assert len(fb.simplify([(0, 0), (0.0001, 0.0001), (0, 0.0002)], 0.3)) == 3


def test_ring_is_open_and_degenerate_dropped():
    assert fb.compact_ring(sq(1.0, 103.0))[:2] == [1.0, 103.0] and len(fb.compact_ring(sq(1.0, 103.0))) == 8
    assert fb.compact_ring([(1, 1), (1, 1), (1, 1), (1, 1)]) is None


def test_build_respects_size_budget():
    els = [way({"building": "yes"}, sq(1.275 + i * 0.00002, 103.849 + (i % 50) * 0.0002, 0.00003))
           for i in range(400)] + [way({"building": "yes"}, sq(1.2800, 103.855, 0.0006))]
    big = fb.dumps(fb.build(els, BBOX, "t", 10**9)[0])
    doc, text = fb.build(els, BBOX, "t", max_bytes=len(big) // 3)
    assert len(text.encode()) < len(big)
    assert any(len(p[0]) for p in doc["layers"]["building"])      # the big one survives
    assert json.loads(text)["attribution"].startswith("© OpenStreetMap")


def test_labels_capped_and_deduped():
    cands = [(i, f"n{i % 80}", 1.28, 103.85, "park") for i in range(200)]
    labels = fb.pick_labels(cands)
    assert len(labels) == fb.MAX_LABELS and len({l["name"] for l in labels}) == len(labels)
