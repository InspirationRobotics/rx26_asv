"""nav_checks — the probes and scorers behind gz_nav_extra.sh (spec docs/nav2_avoidance_spec.md 10.1 N2/N3, 10.2 S6/S9).

    python3 -m crusader_sim.nav_checks s6-precond --course open_water_platform --element plat
    python3 -m crusader_sim.nav_checks n2 [--seconds 10]
    python3 -m crusader_sim.nav_checks twin-params --ns n3 --out /tmp/n3_twin.yaml
    python3 -m crusader_sim.nav_checks n3-watch --course task1_core --in entry --out g1_grn [--ns n3]
    python3 -m crusader_sim.nav_checks s9-watch --seconds 300
    python3 -m crusader_sim.nav_checks score-n3 --log F --removed entry=<epoch> ...
    python3 -m crusader_sim.nav_checks score-s9 --watch F --t-off <epoch> --t-on <epoch> --bt F --task1 F

The first four run in the crsd-sim container (rclpy, the boat's topics, the costmap) and only
LOOK: they publish nothing and set no parameter. The two score-* subcommands are pure python
over the logs the watchers wrote, so they run on the WSL host (no ROS) or anywhere. rclpy is
imported inside the run_* functions, which is what lets the unit test import this module bare.

Wall-clock epoch seconds (time.time()) stamp every watcher line, because the removal in N3 and
the SITL parameter change in S9 happen on the WSL host and the scorer must put both on one clock
(the container shares the host's).
"""
import argparse
import contextlib
import json
import math
import re
import struct
import sys
import time

from crusader_sim import course as C
from crusader_sim.task1_judge import Task1Judge, _point_rect_dist

LETHAL = 100                    # nav_msgs/OccupancyGrid value for cost 254 (crusader_nav.costmap_probe)
FLOAT32 = 7                     # sensor_msgs/PointField.FLOAT32


# ------------------------------------------------------------------ pure helpers

def cloud_xyz(msg):
    """[(x, y, z)] of the finite points of a PointCloud2 (float32 x, y, z fields), any offsets."""
    off = {f.name: f.offset for f in msg.fields if f.name in ("x", "y", "z") and f.datatype == FLOAT32}
    if len(off) < 3:
        raise ValueError("PointCloud2 has no float32 x, y and z fields")
    data, step = bytes(msg.data), msg.point_step
    fmt = ">f" if msg.is_bigendian else "<f"
    pts = []
    for i in range(msg.width * msg.height):
        p = tuple(struct.unpack_from(fmt, data, i * step + off[k])[0] for k in "xyz")
        if all(math.isfinite(v) for v in p):
            pts.append(p)
    return pts


def yaw_of_quat(x, y, z, w):
    """ENU yaw [rad] of a quaternion (the formula task1_goal feeds the judge with)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def body_to_world(pt, pose):
    """(x, y) in the world of a point in the boat's levelled body frame (x forward, y left);
    pose = (x, y, yaw) of the boat from ground truth."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0] + c * pt[0] - s * pt[1], pose[1] + s * pt[0] + c * pt[1]


def count_near_rects(points_xy, rects, margin_m):
    """How many of the (x, y) points lie within margin_m of any rectangle (task1_judge's
    (cx, cy, yaw, half_x, half_y)); 0 distance = inside."""
    return sum(1 for x, y in points_xy
               if min(_point_rect_dist(x, y, r) for r in rects) <= margin_m)


def lethal_count(cells, width, height, res, ox, oy, x, y, radius_m, lethal=LETHAL):
    """How many cells of value >= lethal have their centre within radius_m of (x, y), in a
    row-major grid whose cell (0, 0) has its lower-left corner at (ox, oy)."""
    i0, i1 = max(0, int((x - radius_m - ox) / res)), min(width - 1, int((x + radius_m - ox) / res))
    j0, j1 = max(0, int((y - radius_m - oy) / res)), min(height - 1, int((y + radius_m - oy) / res))
    n = 0
    for j in range(j0, j1 + 1):
        for i in range(i0, i1 + 1):
            cx, cy = ox + (i + 0.5) * res, oy + (j + 0.5) * res
            if (cx - x) ** 2 + (cy - y) ** 2 <= radius_m ** 2 and cells[j * width + i] >= lethal:
                n += 1
    return n


def hazards_near(hazards, x, y, radius_m):
    """How many known hazards [(cx, cy, radius_m)] reach within radius_m of (x, y)."""
    return sum(1 for cx, cy, r in hazards if math.hypot(cx - x, cy - y) <= radius_m + r)


def tf_error_m(tf_xy, lat, lon, datum):
    """Distance between a TF translation and frames_core.to_local of the same pose, metres."""
    from crusader_nav import frames_core as fc
    x, y = fc.to_local(lat, lon, *datum)
    return math.hypot(tf_xy[0] - x, tf_xy[1] - y)


def score_n2(errors, misses, limit_m, min_samples):
    """('PASS'|'FAIL', detail) from the per-pose errors [m] and the poses with no TF."""
    worst = max(errors, default=None)
    detail = (f"max error {'n/a' if worst is None else f'{worst:.3f}'} m over {len(errors)} samples "
              f"(limit {limit_m:g}), {misses} pose(s) with no TF")
    if len(errors) < max(min_samples, 1):
        return "FAIL", detail + f"; fewer than {min_samples} samples: no TF, pose or datum?"
    return ("PASS" if worst <= limit_m else "FAIL"), detail


# ---- N3: the costmap series the watcher writes, and what the spec asks of it

N3_LINE = re.compile(r"^\[n3\] t=(\d+(?:\.\d+)?) (.*)$")


def parse_n3(text):
    """[(t, {name: lethal cells, 'hz': hazards near a target})] from a n3-watch log."""
    out = []
    for line in text.splitlines():
        m = N3_LINE.match(line.strip())
        if m:
            out.append((float(m.group(1)),
                        {k: int(v) for k, v in (tok.split("=", 1) for tok in m.group(2).split())}))
    return out


def decay_times(series, name, t_removed):
    """(persist_s, gone_s) of `name` after t_removed: how long after it the last sample that
    still showed it, and when the first sample without it came. (None, None) with no sample
    after t_removed; gone_s None while it was still there at the last sample."""
    after = [(t, c[name]) for t, c in series if t >= t_removed and name in c]
    if not after:
        return None, None
    last_seen = max((t for t, n in after if n > 0), default=None)
    if last_seen is None:                       # already gone at the first sample after the removal
        return 0.0, after[0][0] - t_removed
    later = [t for t, _ in after if t > last_seen]
    return last_seen - t_removed, (later[0] - t_removed if later else None)


def score_n3(series, removed, in_name, out_name, in_max_s=5.0, out_min_s=20.0, out_max_s=35.0):
    """(verdict, detail). `removed` is {name: epoch the entity was removed}; a name missing from it
    was never marked, so that half cannot be judged. Spec 10.1 N3: removed in view -> gone within
    in_max_s; out of view -> still there at out_min_s and gone by out_max_s. INVALID when a known
    hazard (the BT's HazardLayer, not STVL) was drawn at a target, which would hold it lethal."""
    t_from = min(removed.values(), default=0.0) - 2.0
    if any(c.get("hz", 0) > 0 for t, c in series if t >= t_from):
        return "INVALID", "a known hazard (HazardLayer) was drawn at a target: STVL alone was not under test"
    parts, ok = [], []
    if in_name not in removed:
        return "NOT TESTABLE", f"{in_name} was never marked in the costmap (r_max too short, or no LiDAR return)"
    _, gone = decay_times(series, in_name, removed[in_name])
    good = gone is not None and gone <= in_max_s
    ok.append(good)
    parts.append(f"in-view {in_name} " + (f"gone in {gone:.1f} s" if gone is not None else "NEVER gone")
                 + f" (limit {in_max_s:g} s)")
    if out_name in removed:
        persist, gone = decay_times(series, out_name, removed[out_name])
        good = persist is not None and persist >= out_min_s and gone is not None and gone <= out_max_s
        ok.append(good)
        parts.append(f"out-of-view {out_name} persisted " + ("n/a" if persist is None else f"{persist:.1f} s")
                     + ", " + (f"gone at {gone:.1f} s" if gone is not None else "NEVER gone")
                     + f" (want {out_min_s:g}..{out_max_s:g} s)")
    else:
        parts.append(f"out-of-view {out_name} never marked: half not tested")
    verdict = "FAIL" if not all(ok) else ("PASS" if out_name in removed else "PARTIAL")
    return verdict, " | ".join(parts)


# ---- S9: the watcher's heading and leg-state lines against the SITL change times

WATCH_LINE = re.compile(r"^\[watch\] (\d+(?:\.\d+)?) (heading|leg) (\S+)\s*(.*)$")


def parse_watch(text):
    """[(t, 'heading'|'leg', value, why)] from an s9-watch log."""
    return [(float(m.group(1)), m.group(2), m.group(3), m.group(4))
            for m in (WATCH_LINE.match(line.strip()) for line in text.splitlines()) if m]


def legs_after(events, t0, limit=8):
    """The leg-state changes from t0 on, as 'STATE(why)@+secs', for a verdict that has to say what DID happen."""
    seen = [f"{v}{f'({why})' if why else ''}@{t - t0:+.1f}" for t, k, v, why in events if k == "leg" and t >= t0]
    return " ".join(seen[:limit]) + (" ..." if len(seen) > limit else "") if seen else "none"


def score_s9(events, t_off, t_on, task1_text, bt_text, tick_tol_s=0.5):
    """(verdict, detail). Spec 10.2 S9: DEGRADED within about one tick of the heading going NaN,
    no leg FAILED, back out of DEGRADED after SIM_GPS_HDG 1, and the mission completes.
    t_off/t_on are the epochs SITL acknowledged SIM_GPS_HDG 0 and 1."""
    nan = next((t for t, k, v, _ in events if k == "heading" and v == "NaN" and t >= t_off - 0.5), None)
    if nan is None:
        return "FAIL", ("heading never went NaN after SIM_GPS_HDG 0, so there was nothing for the leg to degrade on "
                        "(telemetry_bridge blanks it only for GLOBAL_POSITION_INT.hdg 65535, and ArduPilot sends "
                        f"ahrs.yaw_sensor); legs after the injection: {legs_after(events, t_off)}")
    deg = next(((t, why) for t, k, v, why in events
                if k == "leg" and v == "DEGRADED" and t >= nan - tick_tol_s), None)
    if deg is None:
        return "FAIL", f"heading NaN at +{nan - t_off:.1f} s but no leg went DEGRADED (a straight or exempt leg never degrades)"
    lag = deg[0] - nan
    failed = bt_text.count("leg FAILED") + sum(1 for _, k, v, _ in events if k == "leg" and v == "FAILED")
    resumed = next((t for t, k, v, _ in events
                    if k == "leg" and v not in ("DEGRADED", "FAILED") and t > deg[0]), None)
    m = re.search(r"\[result\] outcome (\d+)", task1_text)
    bits = [f"NaN at +{nan - t_off:.1f} s -> DEGRADED +{lag:.2f} s (limit {tick_tol_s:g}; {deg[1] or 'no reason given'})",
            f"leg FAILED x{failed}",
            "resumed " + ("never" if resumed is None else f"{resumed - t_on:+.1f} s from HDG 1"),
            f"mission outcome {'none' if m is None else m.group(1)} (0 = SUCCESS)"]
    ok = lag <= tick_tol_s and failed == 0 and resumed is not None and m is not None and m.group(1) == "0"
    return ("PASS" if ok else "FAIL"), " | ".join(bits)


# ------------------------------------------------------------------ ROS probes (container)

def _latched():
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _spin(node, seconds, done=lambda: False):
    import rclpy
    t_end = time.monotonic() + seconds
    while rclpy.ok() and not done() and time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)


@contextlib.contextmanager
def _ros_node(name):
    """An rclpy node for the length of a with-block, shut down however the block ends."""
    import rclpy
    rclpy.init()
    node = rclpy.create_node(name)
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_s6_precond(a):
    """Is the platform in the LiDAR's navigation cloud? Spec 10.2 S6 precondition. Counts, per
    nav cloud, the points within --margin-m of the element's footprint (the cloud is in the
    levelled body frame, so each is put on the world with the boat's TRUE pose) and prints the
    cluster health JSON beside it. Exit 0 = clustered, 1 = not, 2 = no cloud or pose."""
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String

    rects = Task1Judge(C.load(a.course), echo=False).shapes.get(a.element)
    if rects is None:
        print(f"[precond] no element {a.element!r} in course {a.course}")
        return 2
    st = {"pose": None, "counts": [], "health": None}

    def on_odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        st["pose"] = (p.x, p.y, yaw_of_quat(q.x, q.y, q.z, q.w))

    def on_cloud(m):
        if st["pose"] is not None:
            st["counts"].append(count_near_rects(
                [body_to_world(p, st["pose"]) for p in cloud_xyz(m)], rects, a.margin_m))

    with _ros_node("nav_checks_precond") as node:
        node.create_subscription(Odometry, "/sim/crusader/odometry", on_odom, 10)
        node.create_subscription(PointCloud2, "/crsd/nav/obstacle_cloud", on_cloud, qos_profile_sensor_data)
        node.create_subscription(String, "/crsd/lidar_cluster_health",
                                 lambda m: st.update(health=json.loads(m.data)), 10)
        _spin(node, a.seconds)
    counts, health = st["counts"], st["health"] or {}
    keys = ("n_in", "n_water", "n_clusters", "n_nav", "levelled", "nav_cloud")
    print("[precond] health: " + (" ".join(f"{k}={health[k]}" for k in keys if k in health) or "none yet"),
          flush=True)
    if not counts:
        print("[precond] no navigation cloud with a ground-truth pose arrived (is lidar_cluster_node up?)")
        return 2
    best = max(counts)
    clustered = best >= a.min_points
    print(f"[precond] {a.element}: up to {best} nav-cloud point(s) within {a.margin_m:g} m of its footprint "
          f"over {len(counts)} cloud(s) -> " + ("CLUSTERED" if clustered else "NOT clustered"), flush=True)
    return 0 if clustered else 1


def run_n2(a):
    """map -> base_footprint (TF) against frames_core.to_local of /crsd/pose, per pose, over
    --seconds. Spec 10.1 N2. Each pose is looked up at its own stamp (nav_frames_node stamps the
    transform with the pose's), a little later than it arrived, so TF has had time to get here."""
    from crusader_msgs.msg import LatLonHead
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    buf = Buffer()
    st = {"datum": None, "pending": [], "errors": [], "misses": 0, "nan": 0}

    def on_pose(m):
        if not math.isfinite(m.heading):
            st["nan"] += 1                      # no heading, no TF: by design, not an error
        elif st["datum"] is not None and len(st["pending"]) < 500:
            st["pending"].append((time.monotonic(), m.header.stamp, m.latitude, m.longitude))

    def drain():
        keep = []
        for t_rx, stamp, lat, lon in st["pending"]:
            try:
                tr = buf.lookup_transform("map", "base_footprint", Time.from_msg(stamp)).transform.translation
                st["errors"].append(tf_error_m((tr.x, tr.y), lat, lon, st["datum"]))
            except TransformException:
                if time.monotonic() - t_rx > a.lookup_s:
                    st["misses"] += 1
                else:
                    keep.append((t_rx, stamp, lat, lon))
        st["pending"] = keep

    with _ros_node("nav_checks_n2") as node:
        listener = TransformListener(buf, node)     # keeps its subscriptions alive
        node.create_subscription(LatLonHead, "/crsd/datum",
                                 lambda m: st.update(datum=(m.latitude, m.longitude)), _latched())
        node.create_subscription(LatLonHead, "/crsd/pose", on_pose, 10)
        node.create_timer(0.1, drain)
        _spin(node, a.seconds)
        del listener
    if st["datum"] is None:
        print("[n2] no /crsd/datum (is nav_frames_node running?)")
    verdict, detail = score_n2(st["errors"], st["misses"], a.limit_m, a.min_samples)
    print(f"[n2] {verdict} | {detail} | {st['nan']} pose(s) with NaN heading skipped", flush=True)
    return 0 if verdict == "PASS" else 1


def _hazard_xyr(h):
    """(x, y, radius) of a crusader_msgs/Hazard: the circle, or a polygon's centroid and
    farthest vertex."""
    if h.kind == 0 or not h.polygon_x:
        return h.x, h.y, h.radius_m
    cx, cy = sum(h.polygon_x) / len(h.polygon_x), sum(h.polygon_y) / len(h.polygon_y)
    return cx, cy, max(math.hypot(x - cx, y - cy) for x, y in zip(h.polygon_x, h.polygon_y))


def run_twin_params(a):
    """Write a Nav2 params file for a STVL-ONLY twin of the boat's planner, namespaced --ns: the
    installed nav2_params.yaml with the planner and costmap keys renamed to /<ns>/... and the layer
    list cut to stvl_layer + inflation_layer. Everything else (every STVL and inflation value) is
    the boat's own, so the twin's costmap is what STVL alone makes of the same cloud. N3 uses it:
    the real costmap also holds the BT's hazards (HazardLayer), and bt_runner keeps publishing the
    tracks it last held when the tracker goes quiet, so a buoy removed from the world stays lethal
    there whatever STVL does."""
    import copy
    import os

    import yaml
    from ament_index_python.packages import get_package_share_directory

    src = os.path.join(get_package_share_directory("crusader_nav"), "config", "nav2_params.yaml")
    with open(src, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    ns = a.ns.strip("/")
    cm = copy.deepcopy(cfg["global_costmap"]["global_costmap"])
    cm["ros__parameters"]["plugins"] = ["stvl_layer", "inflation_layer"]
    out = {f"/{ns}/planner_server": copy.deepcopy(cfg["planner_server"]),
           f"/{ns}/global_costmap/global_costmap": cm}
    with open(a.out, "w", encoding="utf-8") as f:
        yaml.safe_dump(out, f)
    print(f"[twin] {a.out}: /{ns}/planner_server + /{ns}/global_costmap/global_costmap, plugins "
          f"{cm['ros__parameters']['plugins']}, from {src}")
    return 0


def run_n3_watch(a):
    """Watch the costmap (/global_costmap/costmap, or <--ns>/global_costmap/costmap for the
    STVL-only twin of twin-params) at the --in and --out elements and print one line per costmap
    message: lethal cells within --radius-m of each, and `hz`, the known hazards (HazardLayer) near
    either (always 0 on the twin, which has no HazardLayer). Prints `[n3] ready ...` once both are marked for --mark-hold-s (or after --mark-wait-s
    with whatever is marked), then `[n3] done` --after-ready-s later. The caller removes the
    entities between the two and scores the log with score-n3."""
    from crusader_msgs.msg import HazardArray
    from map_msgs.msg import OccupancyGridUpdate
    from nav_msgs.msg import OccupancyGrid

    where = {e["name"]: (float(e["x"]), float(e["y"])) for e in C.load(a.course)["elements"]}
    names = [a.in_name, a.out_name]
    missing = [n for n in names if n not in where]
    if missing:
        print(f"[n3] no element {missing} in course {a.course}")
        return 2
    st = {"grid": None, "hazards": [], "since": {}, "t0": time.time(), "t_ready": None}
    prefix = "/" + a.ns.strip("/") if a.ns.strip("/") else ""
    print("[n3] costmap " + prefix + "/global_costmap/costmap" + (" (STVL only)" if prefix else ""), flush=True)
    print("[n3] targets " + " ".join(f"{n}@({where[n][0]:g},{where[n][1]:g})" for n in names)
          + f" radius {a.radius_m:g} m", flush=True)

    def sample():
        g, cells = st["grid"]
        w, h, res = g.info.width, g.info.height, g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        now = time.time()
        counts = {n: lethal_count(cells, w, h, res, ox, oy, *where[n], a.radius_m) for n in names}
        hz = sum(hazards_near(st["hazards"], *where[n], 1.5) for n in names)
        print(f"[n3] t={now:.3f} " + " ".join(f"{n}={c}" for n, c in counts.items()) + f" hz={hz}", flush=True)
        for n, c in counts.items():
            st["since"][n] = (st["since"].get(n) or now) if c > 0 else None
        if st["t_ready"] is None:
            held = {n: st["since"].get(n) is not None and now - st["since"][n] >= a.mark_hold_s for n in names}
            if all(held.values()) or now - st["t0"] >= a.mark_wait_s:
                st["t_ready"] = now
                print("[n3] ready " + " ".join(f"{n}={'marked' if held[n] else 'NOT'}" for n in names),
                      flush=True)

    def on_grid(m):
        st["grid"] = (m, list(m.data))
        sample()

    def on_update(u):
        if st["grid"] is None:
            return
        g, cells = st["grid"]
        for row in range(u.height):
            at = (u.y + row) * g.info.width + u.x
            cells[at:at + u.width] = list(u.data[row * u.width:(row + 1) * u.width])
        sample()

    with _ros_node("nav_checks_n3") as node:
        node.create_subscription(OccupancyGrid, prefix + "/global_costmap/costmap", on_grid, _latched())
        node.create_subscription(OccupancyGridUpdate, prefix + "/global_costmap/costmap_updates", on_update, 10)
        if not prefix:              # the twin has no HazardLayer: nothing to count
            node.create_subscription(HazardArray, "/crsd/nav/hazards",
                                     lambda m: st.update(hazards=[_hazard_xyr(h) for h in m.hazards]), _latched())
        _spin(node, a.mark_wait_s + a.after_ready_s + 30.0,
              lambda: st["t_ready"] is not None and time.time() - st["t_ready"] >= a.after_ready_s)
    if st["grid"] is None:
        print(f"[n3] no {prefix}/global_costmap/costmap arrived (is planner_server active?)")
        return 2
    print("[n3] done", flush=True)
    return 0


def run_s9_watch(a):
    """Print every change of the heading's validity (/crsd/pose) and of the leg state
    (/crsd/nav/leg_status), with the epoch, for --seconds. The scorer reads these lines."""
    from crusader_msgs.msg import LatLonHead
    from std_msgs.msg import String

    last = {"heading": None, "leg": None}

    def emit(kind, key, value, why=""):
        if last[kind] != key:
            last[kind] = key
            print(f"[watch] {time.time():.3f} {kind} {value} {why}".rstrip(), flush=True)

    def on_pose(m):
        ok = math.isfinite(m.heading)
        emit("heading", ok, "finite" if ok else "NaN")

    def on_leg(m):
        d = json.loads(m.data)
        emit("leg", (d.get("state"), d.get("why")), d.get("state"), d.get("why", ""))

    with _ros_node("nav_checks_s9") as node:
        node.create_subscription(LatLonHead, "/crsd/pose", on_pose, 10)
        node.create_subscription(String, "/crsd/nav/leg_status", on_leg, 10)
        _spin(node, a.seconds)
    return 0


# ------------------------------------------------------------------ scorers (host)

def _read(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


def _report(verdict, detail):
    """The scorers' one line, and their exit status (0 only for PASS)."""
    print(f"{verdict} | {detail}")
    return 0 if verdict == "PASS" else 1


def run_score_n3(a):
    removed = {k: float(v) for k, v in (kv.split("=", 1) for kv in a.removed)}
    return _report(*score_n3(parse_n3(_read(a.log)), removed, a.in_name, a.out_name))


def run_score_s9(a):
    return _report(*score_s9(parse_watch(_read(a.watch)), a.t_off, a.t_on,
                             _read(a.task1), _read(a.bt), a.tick_tol_s))


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("s6-precond", help="is the platform in the LiDAR navigation cloud?")
    p.add_argument("--course", required=True)
    p.add_argument("--element", required=True)
    p.add_argument("--seconds", type=float, default=8.0)
    p.add_argument("--min-points", type=int, default=2, help="points that count as clustered")
    p.add_argument("--margin-m", type=float, default=0.6, help="reach of a point from the footprint")
    p.set_defaults(run=run_s6_precond)

    p = sub.add_parser("n2", help="TF map->base_footprint against /crsd/pose")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--limit-m", type=float, default=0.05)
    p.add_argument("--min-samples", type=int, default=20)
    p.add_argument("--lookup-s", type=float, default=1.0, help="how long a pose waits for its TF")
    p.set_defaults(run=run_n2)

    p = sub.add_parser("twin-params", help="params file for a STVL-only twin planner, for N3")
    p.add_argument("--ns", required=True, help="the twin's namespace, e.g. n3")
    p.add_argument("--out", required=True, help="where to write the YAML")
    p.set_defaults(run=run_twin_params)

    p = sub.add_parser("n3-watch", help="costmap series at two buoys, for N3")
    p.add_argument("--ns", default="", help="watch <ns>/global_costmap (the twin) instead of /global_costmap")
    p.add_argument("--course", required=True)
    p.add_argument("--in", dest="in_name", required=True, help="element removed IN the clearing frustum")
    p.add_argument("--out", dest="out_name", required=True, help="element removed BEYOND the frustum's max_z")
    p.add_argument("--radius-m", type=float, default=1.0)
    p.add_argument("--mark-hold-s", type=float, default=2.0)
    p.add_argument("--mark-wait-s", type=float, default=30.0)
    p.add_argument("--after-ready-s", type=float, default=45.0)
    p.set_defaults(run=run_n3_watch)

    p = sub.add_parser("s9-watch", help="heading validity and leg-state changes, with epochs")
    p.add_argument("--seconds", type=float, required=True)
    p.set_defaults(run=run_s9_watch)

    p = sub.add_parser("score-n3", help="score an n3-watch log (pure python)")
    p.add_argument("--log", required=True)
    p.add_argument("--in", dest="in_name", required=True)
    p.add_argument("--out", dest="out_name", required=True)
    p.add_argument("--removed", nargs="*", default=[], metavar="NAME=EPOCH")
    p.set_defaults(run=run_score_n3)

    p = sub.add_parser("score-s9", help="score an s9 run (pure python)")
    p.add_argument("--watch", required=True)
    p.add_argument("--t-off", type=float, required=True)
    p.add_argument("--t-on", type=float, required=True)
    p.add_argument("--bt", required=True)
    p.add_argument("--task1", required=True)
    p.add_argument("--tick-tol-s", type=float, default=0.5)
    p.set_defaults(run=run_score_s9)
    return ap.parse_args(argv)


def main(argv=None):
    a = parse_args(argv)
    return a.run(a)


if __name__ == "__main__":
    sys.exit(main())
