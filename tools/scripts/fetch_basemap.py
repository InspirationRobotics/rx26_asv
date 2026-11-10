#!/usr/bin/env python3
"""Fetch an offline street basemap for the ground station from OpenStreetMap DATA.

    python tools/scripts/fetch_basemap.py [--bbox S,W,N,E] [--out PATH]

One Overpass query (ways and relations), compacted into a small JSON file the GUI
draws itself (map.js, CrsdMap). Vector data, not tiles: the OSM tile policy forbids
bulk download and offline use of tiles; ODbL data may be used offline with attribution.

Output schema (all coordinates rounded to 6 decimals):

    {"attribution": "(c) OpenStreetMap contributors (ODbL)", "source": "overpass-api.de",
     "fetched": "<UTC ISO>", "bbox": [S, W, N, E],
     "layers": {"water": [poly...], "park": [poly...], "building": [poly...],
                "pier": [poly-or-line...], "road": [line...], "path": [line...]},
     "labels": [{"name", "lat", "lon", "kind"}]}

  ring / line : flat list [lat, lon, lat, lon, ...]. A ring is NOT closed (no repeated
                first point); canvas closePath() closes it. A line keeps its ends.
  poly        : list of rings, first = outer, rest = holes.
  pier layer  : a poly (list of rings) when the OSM feature is an area, a line (flat
                list of numbers) when it is an open way; tell them apart by the type of
                the first element. Coastline ways land here as lines too.
  road        : motorway..tertiary, residential, service, unclassified (+ _link).
  path        : footway, cycleway, path, steps, pedestrian, track.
  labels kind : water | pier | park. Capped at MAX_LABELS, biggest features first.

Pure functions (classify, assemble_rings, build_polys, simplify, extract, build) are
imported by crusader_groundstation/test/test_basemap.py; only fetch()/main() touch the network.
"""
import argparse
import json
import math
import os
import sys
import urllib.parse
import urllib.request
from datetime import datetime, timezone

DEFAULT_BBOX = (1.2720, 103.8460, 1.2920, 103.8660)      # Marina Bay, ~2.2 km box
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.normpath(os.path.join(
    HERE, "..", "..", "crusader_groundstation", "crusader_groundstation", "laptop", "basemap.json"))
OVERPASS = "https://overpass-api.de/api/interpreter"
USER_AGENT = "crusader-gcs/1.0 (RobotX 2026 Team Inspiration boat ground station)"
ATTRIBUTION = "© OpenStreetMap contributors (ODbL)"
MAX_BYTES = 700_000
MAX_LABELS = 60
M_PER_DEG = 111320.0
SIMPLIFY_M = 0.3
ROAD = {"motorway", "trunk", "primary", "secondary", "tertiary", "residential", "service", "unclassified"}
PATH = {"footway", "cycleway", "path", "steps", "pedestrian", "track"}
LAYERS = ("water", "park", "building", "pier", "road", "path")
# Tightening steps if the file is too big: (smallest building kept in m2, simplify tolerance in m)
SIZE_STEPS = ((0.0, SIMPLIFY_M), (6.0, SIMPLIFY_M), (15.0, 0.5), (30.0, 0.8), (60.0, 1.2))

# Tag filters for the query: (key, value regex or None for any value)
WANTED = (("natural", "^(water|coastline)$"), ("waterway", "^riverbank$"),
          ("man_made", "^(pier|breakwater|groyne|bridge)$"), ("building", None),
          ("highway", None), ("leisure", "^(park|marina|stadium)$"),
          ("landuse", "^(grass|recreation_ground)$"))


def build_query(bbox):
    box = ",".join(str(v) for v in bbox)
    parts = []
    for kind in ("way", "relation"):
        for key, rx in WANTED:
            cond = f'["{key}"~"{rx}"]' if rx else f'["{key}"]'
            parts.append(f"  {kind}{cond}({box});")
    return "[out:json][timeout:90];\n(\n" + "\n".join(parts) + "\n);\nout geom;"


# ---- classification ------------------------------------------------------------------
def classify(tags):
    """The layer an OSM feature belongs to, or None. First match wins, in drawing priority."""
    t = tags or {}
    if t.get("natural") in ("water", "coastline") or t.get("waterway") == "riverbank":
        return "water"
    if t.get("man_made") in ("pier", "breakwater", "groyne", "bridge"):
        return "pier"
    if t.get("leisure") in ("park", "marina", "stadium") or t.get("landuse") in ("grass", "recreation_ground"):
        return "park"
    if t.get("building"):
        return "building"
    h = (t.get("highway") or "").replace("_link", "")
    if h in ROAD:
        return "road"
    if h in PATH:
        return "path"
    return None


# ---- geometry ------------------------------------------------------------------------
def local_xy(pts, lat0):
    k = math.cos(math.radians(lat0)) * M_PER_DEG
    return [(lon * k, lat * M_PER_DEG) for lat, lon in pts]


def simplify(pts, tol_m=SIMPLIFY_M):
    """Douglas-Peucker on [(lat, lon)], tolerance in metres. Keeps both ends."""
    if len(pts) < 3:
        return list(pts)
    xy = local_xy(pts, pts[0][0])
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        a, b = stack.pop()
        (ax, ay), (bx, by) = xy[a], xy[b]
        dx, dy = bx - ax, by - ay
        norm = math.hypot(dx, dy)
        far, far_i = 0.0, -1
        for i in range(a + 1, b):
            px, py = xy[i]
            d = math.hypot(px - ax, py - ay) if norm == 0 else abs(dx * (ay - py) - dy * (ax - px)) / norm
            if d > far:
                far, far_i = d, i
        if far > tol_m:
            keep[far_i] = True
            stack += [(a, far_i), (far_i, b)]
    return [p for p, k in zip(pts, keep) if k]


def ring_area_m2(pts):
    if len(pts) < 3:
        return 0.0
    xy = local_xy(pts, pts[0][0])
    s = sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(xy, xy[1:] + xy[:1]))
    return abs(s) / 2


def point_in_ring(pt, ring):
    y, x = pt
    inside = False
    for (y1, x1), (y2, x2) in zip(ring, ring[1:] + ring[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def assemble_rings(ways):
    """Join member ways ([(lat, lon), ...] each, any order or direction) end to end.
    Returns (closed, open): closed rings (first point repeated at the end) and the chains
    that never closed."""
    pool = [list(w) for w in ways if len(w) >= 2]
    closed, opened = [], []
    while pool:
        chain = pool.pop()
        while chain[0] != chain[-1]:
            for i, w in enumerate(pool):
                if w[0] == chain[-1]:
                    chain += w[1:]
                elif w[-1] == chain[-1]:
                    chain += w[-2::-1]
                elif w[-1] == chain[0]:
                    chain = w[:-1] + chain
                elif w[0] == chain[0]:
                    chain = w[:0:-1] + chain
                else:
                    continue
                pool.pop(i)
                break
            else:
                break
        (closed if chain[0] == chain[-1] and len(chain) >= 4 else opened).append(chain)
    return closed, opened


def build_polys(outers, inners):
    """Rings -> polys. Each inner ring goes to the smallest outer that contains it."""
    polys = [[o] for o in outers]
    order = sorted(range(len(outers)), key=lambda i: ring_area_m2(outers[i]))
    for inner in inners:
        for i in order:
            if point_in_ring(inner[0], outers[i][:-1]):
                polys[i].append(inner)
                break
    return polys


def geom_points(geom):
    return [(g["lat"], g["lon"]) for g in (geom or []) if g]


def way_ring(el):
    pts = geom_points(el.get("geometry"))
    return pts if len(pts) >= 4 and pts[0] == pts[-1] else None


def element_polys(el, layer):
    """The polys of one OSM element, rings closed (first == last). [] when it has none."""
    if el["type"] == "way":
        ring = way_ring(el)
        return [[ring]] if ring else []
    if (el.get("tags") or {}).get("type") not in ("multipolygon", "boundary"):
        return []
    outer, inner = [], []
    for m in el.get("members", []):
        if m.get("type") == "way":
            (inner if m.get("role") == "inner" else outer).append(geom_points(m.get("geometry")))
    o_closed, o_open = assemble_rings(outer)
    i_closed, _ = assemble_rings(inner)
    if layer in ("water", "park"):                  # a chain cut at the query edge: fill it as drawn
        o_closed += [c + [c[0]] for c in o_open if len(c) >= 3]
    return build_polys(o_closed, i_closed)


def bounds(rings):
    lats = [p[0] for r in rings for p in r]
    lons = [p[1] for r in rings for p in r]
    return min(lats), min(lons), max(lats), max(lons)


def in_bbox(b, bbox):
    s, w, n, e = bbox
    return not (b[2] < s or b[0] > n or b[3] < w or b[1] > e)


# ---- compaction ----------------------------------------------------------------------
def flat(pts):
    return [round(v, 6) for p in pts for v in p]


def compact_ring(ring, tol_m=SIMPLIFY_M):
    """Closed ring [(lat, lon)] -> flat, open, simplified; None if it collapses."""
    pts = simplify(ring[:-1] if ring[0] == ring[-1] else ring, tol_m)
    return flat(pts) if len(pts) >= 3 else None


def compact_poly(poly, tol_m=SIMPLIFY_M):
    rings = [compact_ring(r, tol_m) for r in poly]
    return [r for r in rings if r] if rings and rings[0] else None


def compact_line(pts, tol_m=SIMPLIFY_M):
    pts = simplify(pts, tol_m)
    return flat(pts) if len(pts) >= 2 else None


def centroid(ring):
    s, w, n, e = bounds([ring])
    return (s + n) / 2, (w + e) / 2


def is_line_feature(el, layer, tags):
    """Coastlines and open piers are lines; so is every road and path."""
    if layer in ("road", "path") or tags.get("natural") == "coastline":
        return True
    return layer == "pier" and el["type"] == "way" and not way_ring(el)


def extract(elements, bbox, min_building_m2=0.0, tol_m=SIMPLIFY_M):
    """OSM elements -> (layers, label candidates [(area, name, lat, lon, kind)])."""
    layers = {k: [] for k in LAYERS}
    cands = []
    for el in elements:
        tags = el.get("tags") or {}
        layer = classify(tags)
        if not layer:
            continue
        if is_line_feature(el, layer, tags):
            pts = geom_points(el.get("geometry")) if el["type"] == "way" else []
            if len(pts) >= 2 and in_bbox(bounds([pts]), bbox):
                line = compact_line(pts, tol_m)
                if line:
                    layers["pier" if layer == "water" else layer].append(line)
            continue
        for poly in element_polys(el, layer):
            if not in_bbox(bounds(poly), bbox):
                continue
            area = ring_area_m2(poly[0])
            if layer == "building" and area < min_building_m2:
                continue
            cp = compact_poly(poly, tol_m)
            if not cp:
                continue
            layers[layer].append(cp)
            name = tags.get("name")
            if name and layer != "building":
                lat, lon = centroid(poly[0])
                if bbox[0] <= lat <= bbox[2] and bbox[1] <= lon <= bbox[3]:
                    cands.append((area, name, lat, lon, layer))
    return layers, cands


def pick_labels(cands, cap=MAX_LABELS):
    seen, out = set(), []
    for area, name, lat, lon, kind in sorted(cands, key=lambda c: -c[0]):
        if (name, kind) in seen:
            continue
        seen.add((name, kind))
        out.append({"name": name, "lat": round(lat, 6), "lon": round(lon, 6), "kind": kind})
        if len(out) >= cap:
            break
    return out


def dumps(obj):
    return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))


def build(elements, bbox, fetched=None, max_bytes=MAX_BYTES):
    """Compact an Overpass response into (document, JSON text), tightening until it fits."""
    stamp = fetched or datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    for min_b, tol in SIZE_STEPS:
        layers, cands = extract(elements, bbox, min_b, tol)
        doc = {"attribution": ATTRIBUTION, "source": "overpass-api.de", "fetched": stamp,
               "bbox": list(bbox), "layers": layers, "labels": pick_labels(cands)}
        text = dumps(doc)
        if len(text.encode("utf-8")) <= max_bytes:
            break
    return doc, text


# ---- network + CLI ---------------------------------------------------------------------
def fetch(bbox):
    body = urllib.parse.urlencode({"data": build_query(bbox)}).encode()
    req = urllib.request.Request(OVERPASS, data=body, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=120) as r:       # one request, no retries
        return json.loads(r.read().decode("utf-8"))


def parse_bbox(text):
    s, w, n, e = (float(v) for v in text.split(","))
    if not (-90 <= s < n <= 90 and -180 <= w < e <= 180):
        raise ValueError("bbox must be S,W,N,E with S<N and W<E")
    return s, w, n, e


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--bbox", type=parse_bbox, default=DEFAULT_BBOX, metavar="S,W,N,E")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--from-file", help="compact a saved Overpass response instead of fetching")
    a = ap.parse_args(argv)
    if a.from_file:
        with open(a.from_file, encoding="utf-8") as f:
            raw = json.load(f)
    else:
        raw = fetch(a.bbox)
    doc, text = build(raw.get("elements", []), a.bbox)
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    counts = ", ".join(f"{k} {len(v)}" for k, v in doc["layers"].items())
    print(f"{a.out}: {len(text.encode('utf-8')) / 1024:.0f} KB; {counts}; labels {len(doc['labels'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
