"""gen_dense_course — a random DENSE Task 1 field (neighbours 2-4 m apart) as a course YAML.

    python3 -m crusader_sim.gen_dense_course --seed 3 [--rows 6] [--blacks 2] [--out courses]

Runs anywhere (standard library only). Writes courses/task1_dense_s<seed>.yaml.

WHY. The 2026 Singapore field (task1_sg.yaml) is 11 buoys in about 8 x 13 m, neighbours 2.3-4 m
apart, the channel winding rather than in tidy gates. From inside a field that dense the near
buoys hide the far ones in the camera; task1_core.yaml (10 m between gates) never shows that.

LAYOUT (ENU metres; the channel runs NORTH, so starboard is EAST):
  * ROWS, each 2-4 m beyond the last along the channel; the channel's middle moves sideways up to
    +-1.2 m per row; a row's gate is 2-4 m wide, RED on the east side (kept to starboard going
    north), GREEN on the west. The first and last rows are pairs; the rest are a pair or a single
    red or green, at random (as on the real field).
  * ENTRY (flash_blue) before the first row, EXIT (steady_blue) after the last, both near the
    channel's middle and >= --anchor-gap (3.4 m) from every other buoy: the boat has to CIRCLE
    them, and the real field gave them 3.4-4 m (task1_sg.yaml; the planner's orbit_min_m 1.5
    assumes it). Seeds 1-3 were made before this rule (2-3 m): there no orbit fits the ENTRY.
  * BLACK buoys (off) beside the channel, 2-4 m from their nearest neighbour.
  * Every buoy >= --min-gap (2.0 m) from every other: a draw that breaks it is drawn again.
  * field_centre: the centroid of every buoy, what the operator would put on the map
    (task1_goal sets it as the Run panel's t1_field_lat/lon).
  * boat_start: 10 m south of the ENTRY, heading north.

--shape circle (task1_circle_s<seed>.yaml): the field as the team saw it on the water
(2026-11-13): TEN buoys in all, spread EVENLY over a circle, the boat scanning from its centre.
  * ENTRY on the south rim and EXIT on the north rim of a disc of --radius (5.0 m: 10 m across);
    no buoy within --anchor-gap (3.0 m) of either, so both can be circled.
  * the CHANNEL: a gentle curve from the ENTRY to the EXIT through the middle (its middle up to
    0.75 m off the straight line); no buoy within --corridor (1.0 m) of it, so the centre itself
    is open water: the 360 scan's spot.
  * --buoys (8) more, by Mitchell's best candidate (each the farthest of 60 random spots from
    those placed): EVEN over the rest of the disc, both banks within one of each other, none
    nearer than --min-gap (2.2 m): about 2.3-3 m to the nearest neighbour (the measured field,
    task1_sg.yaml: median 2.35 m).
  * colours by side: RED right of the channel going north (starboard), GREEN left; then --blacks
    (1) of them, at random, has its light off.
"""
import argparse
import math
import os
import random

ORIGIN = (1.28060, 103.85570)    # = tools/sitl/start_sitl.sh SITL_HOME, as task1_core.yaml


def _far_enough(p, placed, gap):
    return all(math.hypot(p[0] - q[0], p[1] - q[1]) >= gap for q in placed)


def layout(seed, rows=6, blacks=2, min_gap=2.0, tries=2000, anchor_gap=3.4):
    """[(name, x, y, beacon)] for one field, drawn from `seed` (the same seed, the same field)."""
    rng = random.Random(seed)
    for _ in range(tries):
        out, pts = [], []
        ok = True
        y, cx = 0.0, 0.0
        row_info = []
        nred = ngrn = 0
        for i in range(rows):
            y += rng.uniform(2.0, 4.0)
            cx += rng.uniform(-1.2, 1.2)
            w = rng.uniform(2.0, 4.0)
            kind = "pair" if i in (0, rows - 1) else rng.choice(["pair", "pair", "red", "green"])
            row_info.append((y, cx, w))
            for colour in ("red", "green"):
                if kind not in ("pair", colour):
                    continue
                x = cx + w / 2.0 if colour == "red" else cx - w / 2.0
                p = (x, y + rng.uniform(-0.4, 0.4))
                if not _far_enough(p, pts, min_gap):
                    ok = False
                    break
                if colour == "red":
                    nred += 1
                    name, beacon = f"r{nred}", "flash_red"
                else:
                    ngrn += 1
                    name, beacon = f"g{ngrn}", "flash_green"
                pts.append(p)
                out.append((name, p[0], p[1], beacon))
            if not ok:
                break
        if not ok:
            continue
        y0, cx0, _ = row_info[0]
        yl, cxl, _ = row_info[-1]
        entry = (cx0 + rng.uniform(-0.8, 0.8), y0 - rng.uniform(anchor_gap, anchor_gap + 0.8))
        exitp = (cxl + rng.uniform(-0.8, 0.8), yl + rng.uniform(anchor_gap, anchor_gap + 0.8))
        if not (_far_enough(entry, pts, anchor_gap) and _far_enough(exitp, pts + [entry], anchor_gap)):
            continue
        pts += [entry, exitp]
        out.insert(0, ("entry", entry[0], entry[1], "flash_blue"))
        out.append(("exit", exitp[0], exitp[1], "steady_blue"))
        nb = 0
        for _b in range(blacks * 50):
            if nb == blacks:
                break
            ry, rcx, rw = row_info[rng.randrange(1, rows - 1)]
            side = rng.choice((-1.0, 1.0))
            p = (rcx + side * (rw / 2.0 + rng.uniform(2.0, 3.0)), ry + rng.uniform(-1.0, 1.0))
            near = min(math.hypot(p[0] - q[0], p[1] - q[1]) for q in pts)
            if min_gap <= near <= 4.0 and _far_enough(p, [entry, exitp], anchor_gap):
                nb += 1
                pts.append(p)
                out.append((f"black{nb}", p[0], p[1], "off"))
        if nb < blacks:
            continue
        return out
    raise RuntimeError(f"seed {seed}: no field with every buoy >= {min_gap} m apart in {tries} draws")


def _seg_dist(p, a, b):
    """Distance from point p to segment a-b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0.0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def _line_dist(p, line):
    return min(_seg_dist(p, line[i], line[i + 1]) for i in range(len(line) - 1))


def _bezier(p0, p1, p2, n=41):
    """n points along the quadratic Bezier from p0 to p2, pulled toward p1."""
    out = []
    for i in range(n):
        t = i / (n - 1)
        a, b, c = (1 - t) * (1 - t), 2 * (1 - t) * t, t * t
        out.append((a * p0[0] + b * p1[0] + c * p2[0], a * p0[1] + b * p1[1] + c * p2[1]))
    return out


def _nearest_on(p, line):
    """(the point of the polyline nearest p, distance, unit direction of that segment)."""
    best = (None, math.inf, (0.0, 1.0))
    for i in range(len(line) - 1):
        a, b = line[i], line[i + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 == 0.0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
        q = (a[0] + t * dx, a[1] + t * dy)
        d = math.hypot(p[0] - q[0], p[1] - q[1])
        if d < best[1]:
            n = math.sqrt(l2) or 1.0
            best = (q, d, (dx / n, dy / n))
    return best


def _left_of(p, line):
    q, _d, (ux, uy) = _nearest_on(p, line)
    return (ux * (p[1] - q[1]) - uy * (p[0] - q[0])) > 0


def layout_circle(seed, buoys=8, radius=5.0, min_gap=2.2, corridor=1.0, blacks=1,
                  anchor_gap=3.0, tries=2000, candidates=60):
    """[(name, x, y, beacon)] for one CIRCLE field (module docstring, --shape circle)."""
    rng = random.Random(seed)
    for _ in range(tries):
        # ENTRY on the disc's south rim, EXIT on its north rim; the channel a gentle curve between
        entry = (rng.uniform(-0.5, 0.5), -radius)
        exitp = (rng.uniform(-0.5, 0.5), radius)
        mid = _bezier(entry, (rng.uniform(-1.5, 1.5), rng.uniform(-1.0, 1.0)), exitp)
        # EVEN over the rest of the disc: Mitchell's best candidate (each new buoy the farthest of
        # `candidates` random spots from those placed), none in the channel or the clearings
        pts = []
        while len(pts) < buoys:
            best, best_d = None, -1.0
            for _k in range(candidates):
                r, th = radius * math.sqrt(rng.random()), rng.uniform(0.0, 2.0 * math.pi)
                q = (r * math.cos(th), r * math.sin(th))
                if _nearest_on(q, mid)[1] < corridor or not _far_enough(q, [entry, exitp], anchor_gap):
                    continue
                d = min((math.hypot(q[0] - p[0], q[1] - p[1]) for p in pts), default=math.inf)
                if d > best_d:
                    best, best_d = q, d
            if best is None or best_d < min_gap:
                break
            pts.append(best)
        if len(pts) < buoys:
            continue
        left = [_left_of(p, mid) for p in pts]
        if abs(left.count(True) - left.count(False)) > 1:     # both banks about as full
            continue
        dark = set(rng.sample(range(len(pts)), min(blacks, len(pts))))
        reds = sorted((p for i, p in enumerate(pts) if not left[i] and i not in dark), key=lambda p: p[1])
        grns = sorted((p for i, p in enumerate(pts) if left[i] and i not in dark), key=lambda p: p[1])
        blks = sorted((pts[i] for i in dark), key=lambda p: p[1])
        out = [("entry", entry[0], entry[1], "flash_blue")]
        out += [(f"r{k + 1}", p[0], p[1], "flash_red") for k, p in enumerate(reds)]
        out += [(f"g{k + 1}", p[0], p[1], "flash_green") for k, p in enumerate(grns)]
        out.append(("exit", exitp[0], exitp[1], "steady_blue"))
        out += [(f"black{k + 1}", p[0], p[1], "off") for k, p in enumerate(blks)]
        return out
    raise RuntimeError(f"seed {seed}: no circle field of {buoys} buoys fits in {tries} draws "
                       "(fewer --buoys, a smaller --min-gap or a bigger --radius)")


def nearest_gaps(field):
    gaps = []
    for i, a in enumerate(field):
        gaps.append(min(math.hypot(a[1] - b[1], a[2] - b[2]) for j, b in enumerate(field) if j != i))
    return gaps


def course_yaml(name, seed, field, shape="channel"):
    cx = sum(b[1] for b in field) / len(field)
    cy = sum(b[2] for b in field) / len(field)
    entry = next(b for b in field if b[3] == "flash_blue")
    gaps = nearest_gaps(field)
    xs, ys = [b[1] for b in field], [b[2] for b in field]
    if shape == "circle":
        what = ["# CIRCLE: buoys spread evenly over a disc, the channel curving from the ENTRY (south rim)",
                "# to the EXIT (north rim): RED to starboard, GREEN to port, some lights off (BLACK).",
                "# Regenerate rather than hand-edit."]
    else:
        what = ["# The channel runs north: RED east of it (starboard), GREEN west (port).",
                "# Regenerate rather than hand-edit."]
    lines = [
        f"# {name} — a RANDOM DENSE Task 1 field: gen_dense_course.py --shape {shape} --seed {seed}.",
        "# Neighbours 2-4 m apart (as on the 2026 Singapore field, task1_sg.yaml).",
        *what,
        f"# Nearest neighbour per buoy: min {min(gaps):.2f} m, mean {sum(gaps) / len(gaps):.2f} m, "
        f"max {max(gaps):.2f} m. {len(field)} buoys in {max(xs) - min(xs):.1f} x "
        f"{max(ys) - min(ys):.1f} m (east x north).",
        "",
        f"name: {name}",
        f"origin: {{lat: {ORIGIN[0]:.5f}, lon: {ORIGIN[1]:.5f}}}",
        "draft_m: 0.24",
        "depth_m: 5.0",
        f"boat_start: {{x: {entry[1]:.2f}, y: {entry[2] - 10.0:.2f}, yaw_deg: 90.0}}",
        f"field_centre: {{x: {cx:.2f}, y: {cy:.2f}}}   # the operator's click: the centroid",
        f"staging: {{x: {entry[1]:.2f}, y: {entry[2] - 6.0:.2f}}}   # 6 m off the ENTRY, outside: task1_goal --staging",
        "tier: core",
        "",
        "elements:",
    ]
    for n, x, y, beacon in field:
        lit = "" if beacon == "off" else ", up_beacon: false"
        lines.append(f"  - {{type: robobuoy, name: {n}, x: {x:.2f}, y: {y:.2f}, "
                     f"beacon: {beacon if beacon != 'off' else chr(34) + 'off' + chr(34)}{lit}}}")
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--shape", choices=("channel", "circle"), default="channel",
                    help="channel: a strip running north (task1_dense_s*); circle: tight and even in a "
                         "disc, as on the water (task1_circle_s*)")
    ap.add_argument("--buoys", type=int, default=8,
                    help="circle: buoys besides the ENTRY and EXIT")
    ap.add_argument("--radius", type=float, default=5.0, help="circle: the disc's radius, m")
    ap.add_argument("--corridor", type=float, default=1.0,
                    help="circle: no buoy this near the channel's middle line, m")
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--blacks", type=int, default=None, help="default 2 (channel) or 1 (circle)")
    ap.add_argument("--min-gap", type=float, default=None, help="default 2.0 (channel) or 2.2 (circle)")
    ap.add_argument("--anchor-gap", type=float, default=None, help="default 3.4 (channel) or 3.0 (circle)")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "courses"))
    a = ap.parse_args()
    if a.shape == "circle":
        name = f"task1_circle_s{a.seed}"
        field = layout_circle(a.seed, a.buoys, a.radius, 2.2 if a.min_gap is None else a.min_gap,
                              a.corridor, 1 if a.blacks is None else a.blacks,
                              anchor_gap=3.0 if a.anchor_gap is None else a.anchor_gap)
    else:
        name = f"task1_dense_s{a.seed}"
        field = layout(a.seed, a.rows, 2 if a.blacks is None else a.blacks,
                       2.0 if a.min_gap is None else a.min_gap,
                       anchor_gap=3.4 if a.anchor_gap is None else a.anchor_gap)
    path = os.path.join(a.out, name + ".yaml")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(course_yaml(name, a.seed, field, a.shape))
    gaps = nearest_gaps(field)
    print(f"{path}: {len(field)} buoys, nearest neighbour {min(gaps):.2f}-{max(gaps):.2f} m")


if __name__ == "__main__":
    main()
