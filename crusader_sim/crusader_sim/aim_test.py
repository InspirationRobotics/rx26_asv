"""aim_test — the cannon's aim on its own: the water through the window's MIDDLE.

    python3 -m crusader_sim.aim_test --window lr [--quick] [--tol 0.05]

Runs in: the crsd-sim container, on task3_aim_lr / task3_aim_ul brought up whole
(gz_sim_up.sh: cannon_aim_node, dock_slot_node, wall_range_node, task3_world), with
crusader_sim.aim_pin in WSL holding the boat where this asks. `sim aimtest lr|ul`
(crusader_sim/scripts/aim_test.sh) does all of it.

For every pose of the grid - the LiDAR's distance to the back wall, the boat's
offset from the bay's centreline, its heading off square - it:
  1. pins the boat there (aim_pin) and waits for it to settle;
  2. waits for the camera to see the window (dock/observations);
  3. aims at it the way SprayUntilHit does: /crsd/water_cannon with the camera's
     point and the window named, fire true, 10 Hz, the newest point each time.
     cannon_aim_node does the rest (NO trims: the camera's bearing, the LiDAR's
     distance, the window's known height);
  4. reads where the water crosses the face from the referee (/sim/task3/status
     water.miss_m, + right and + high of the window's centre) while it runs.
One row a pose: the median miss of the shot. PASS = every pose within --tol (5 cm)
of the middle, right/left and up/down. The table also goes to /tmp/aim_test.csv.

What it can and cannot show: the sim's water is the aim's own model (the same exit
speed, task3_world), so a miss here is the AIM's - the camera, the LiDAR, the
geometry, the servos, the hull's attitude - and not the stream's. The stream's
speed is the water's to measure (docs/T3_cannon_cal.md, `cal speed`).
"""
import argparse
import json
import math
import os
import statistics
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from crusader_msgs.msg import Attitude, DockObservation

from crusader_common import config as crsd_config
from crusader_fcu import cannon_aim_core as ca
from crusader_sim.operator_tools import arm, set_mode

PIN_DIR = os.path.expanduser("~/robotx_ws/.aimtest")      # = aim_pin's, through the mount
POSE_FILE = os.path.join(PIN_DIR, "pose.json")
ACK_FILE = os.path.join(PIN_DIR, "pinned.json")

# the Task 3 courses' dock: faces south at y = 19.035, the deck edge (the back wall
# the LiDAR measures) at y = 19.0, the middle bay on x = 0
DECK_EDGE_Y = 19.0
FACE_Y = 19.035            # the faces' front (task3_world FACE_U)
DRAFT_M = 0.24             # the model's origin (hull-bottom datum) under the water
LIDAR_XY = (0.32, 0.05)    # lidar_cluster_node lidar_x / lidar_y
SLOT = {"ul": 0, "lr": 1}
# each window's TRUE centre on the middle bay: (east of the bay's centreline, above
# the water) - world.py WINDOW_SLOTS on a face 0.8 m up (task3_world DECK_Z + 0.5)
WINDOW_TRUTH = {"ul": (-0.22, 1.05), "lr": (0.23, 0.81)}

# Inside the 1.5 m slip a 0.6 m hull has 0.45 m each side: +-8 cm and +-6 deg keep
# every corner clear of the fingers. The distances: where the camera (~0.22 m up)
# sees at least half of the window, and the water (4.85 m/s) still reaches it.
GRID = {
    "lr": {"dist": (1.25, 1.6, 1.9), "lat": (-0.08, 0.0, 0.08), "dh": (-6.0, 0.0, 6.0)},
    # the upper-left: half in view from ~1.65 m, in reach to ~1.78 m (2026-10-18)
    "ul": {"dist": (1.7, 1.75), "lat": (-0.08, 0.0, 0.08), "dh": (-6.0, 0.0, 6.0)},
}
QUICK = {"lat": (0.0,), "dh": (0.0, 6.0)}


def pose_for(dist, lat, dh):
    """Gazebo pose (x east, y north, ENU yaw deg) of the model origin for the LiDAR
    `dist` from the back wall (perpendicular), the origin `lat` LEFT of the middle
    bay's centreline, and the bow `dh` deg left of straight in."""
    yaw = 90.0 + dh
    r = math.radians(yaw)
    lx = LIDAR_XY[0] * math.cos(r) - LIDAR_XY[1] * math.sin(r)
    ly = LIDAR_XY[0] * math.sin(r) + LIDAR_XY[1] * math.cos(r)
    return -lat, DECK_EDGE_Y - dist - ly, yaw


def quat_to_R(w, x, y, z):
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def truth_body(pos, q, name):
    """The window's true centre in the body frame (hull datum) of the hull at Gazebo's
    pose `pos`, `q` (w, x, y, z): R^T (window - pos), the whole rotation."""
    we, wz = WINDOW_TRUTH[name]
    R = quat_to_R(*q)
    d = (we - pos[0], FACE_Y - pos[1], wz - pos[2])
    return tuple(R[0][i] * d[0] + R[1][i] * d[1] + R[2][i] * d[2] for i in range(3))


def ned_roll_pitch(q):
    """The hull's roll and pitch as the autopilot gives them (NED: + starboard down,
    + bow up), from Gazebo's quaternion (FLU body in ENU)."""
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_flu = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))   # + = bow DOWN
    return roll, -pitch_flu


def cannon_params():
    """The aim's own model, as task3_world builds it (the same file)."""
    d = dict(crsd_config.node_params("cannon_aim_node"))
    cam = crsd_config.node_params("target_tracker")
    for k in ("cam_x", "cam_y", "cam_z", "cam_yaw_deg", "cam_pitch_deg"):
        d[k] = cam.get(k, 0.0)
    return ca.CannonParams.from_dict(d)


class Tester(Node):
    def __init__(self):
        super().__init__("aim_test")
        self.cannon_pub = self.create_publisher(String, "/crsd/water_cannon", 10)
        self.mode_pub = self.create_publisher(String, "/crsd/set_mode", 10)
        self.reset_cli = self.create_client(Trigger, "/crsd/autonomy_drop_reset")
        self.obs = None
        self.obs_t = -1e18
        self.status = None
        self.cannon = None
        self.create_subscription(DockObservation, "dock/observations", self._on_obs, 10)
        self.create_subscription(String, "/sim/task3/status", self._on_status, 10)
        self.create_subscription(String, "/crsd/cannon_status", self._on_cannon, 10)
        self.att = (0.0, 0.0)
        self.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)
        self.odom = None                   # (pos, quat w x y z): Gazebo's truth
        self.create_subscription(Odometry, "/sim/crusader/odometry", self._on_odom, 10)

    def _on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.odom = ((p.x, p.y, p.z), (q.w, q.x, q.y, q.z))

    def _on_att(self, m):
        self.att = (float(m.roll), float(m.pitch))

    def _on_obs(self, m):
        self.obs, self.obs_t = m, time.monotonic()

    def _on_status(self, m):
        try:
            self.status = json.loads(m.data)
        except ValueError:
            pass

    def _on_cannon(self, m):
        try:
            self.cannon = json.loads(m.data)
        except ValueError:
            pass

    def spin_for(self, s):
        t_end = time.monotonic() + s
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def reset_latch(self):
        if not self.reset_cli.wait_for_service(timeout_sec=10.0):
            return False, "no /crsd/autonomy_drop_reset"
        fut = self.reset_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        r = fut.result() if fut.done() else None
        return (bool(r.success), r.message) if r is not None else (False, "no answer")

    def window(self, slot):
        """The newest camera point of window `slot`: the one nearest the middle of
        the picture (the bay in front), or None."""
        if self.obs is None or time.monotonic() - self.obs_t > 0.5:
            return None
        best = None
        for b in self.obs.bays:
            for w in b.windows:
                if int(w.index) == slot and w.has_position and math.isfinite(w.x):
                    if best is None or abs(w.y) < abs(best[1]):
                        best = (float(w.x), float(w.y), float(w.z))
        return best

    def aim(self, p, name):
        d = {"fire": p is not None, "frame_id": "camera_link"}
        if p is not None:
            d.update(x=p[0], y=p[1], z=p[2], window=name.upper())
        self.cannon_pub.publish(String(data=json.dumps(d)))


def pin(seq, x, y, yaw):
    os.makedirs(PIN_DIR, exist_ok=True)
    with open(POSE_FILE + ".tmp", "w", encoding="utf-8") as f:
        json.dump({"seq": seq, "x": x, "y": y, "yaw_deg": yaw}, f)
    os.replace(POSE_FILE + ".tmp", POSE_FILE)


def pinned_seq():
    try:
        with open(ACK_FILE, encoding="utf-8") as f:
            return json.load(f).get("seq")
    except (OSError, ValueError):
        return None


def release():
    try:
        with open(POSE_FILE, "w", encoding="utf-8") as f:
            json.dump({"release": True}, f)
    except OSError:
        pass


def shoot(t: Tester, slot, name, max_s=6.0, want_samples=6):
    """One shot: aim and fire until want_samples readings of the miss (after the
    first 0.6 s of water), or max_s. Returns (misses, wheres, cannon status)."""
    misses, wheres, diag, whys = [], [], [], []
    t0 = time.monotonic()
    water_t = None
    last_status = None
    while time.monotonic() - t0 < max_s and len(misses) < want_samples:
        t.aim(t.window(slot), name)
        t.spin_for(0.1)
        if t.cannon and t.cannon.get("why"):
            whys.append(t.cannon["why"])
        s = t.status
        if s is None or s is last_status:
            continue
        last_status = s
        if s.get("water_on"):
            water_t = water_t or time.monotonic()
            w = s.get("water") or {}
            if time.monotonic() - water_t >= 0.6:
                wheres.append(w.get("where", "?"))
                if w.get("miss_m") is not None:
                    misses.append(tuple(w["miss_m"]))
                c = t.cannon or {}
                if c.get("target_body"):
                    diag.append({"target": c["target_body"], "pan": c.get("pan_deg"),
                                 "tilt": c.get("tilt_deg"), "jpan": w.get("pan_deg"),
                                 "jtilt": w.get("tilt_deg"), "att": t.att, "odom": t.odom})
    for _ in range(5):
        t.aim(None, name)
        t.spin_for(0.1)
    return misses, wheres, t.cannon, diag, (whys[-1] if whys else "")


def med(xs):
    xs = [x for x in xs if x is not None]
    return statistics.median(xs) if xs else float("nan")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--window", choices=sorted(SLOT), default="lr")
    ap.add_argument("--tol", type=float, default=0.05, help="m, each axis (default 0.05)")
    ap.add_argument("--quick", action="store_true", help="the centreline only, 0 and +6 deg")
    ap.add_argument("--out", default="/tmp/aim_test.csv")
    a = ap.parse_args(argv)
    slot, name = SLOT[a.window], a.window
    g = dict(GRID[name])
    if a.quick:
        g.update(QUICK)

    ok, why = arm()
    print(f"[aim] {why}", flush=True)
    rclpy.init()
    t = Tester()
    if not set_mode(t, t.mode_pub, "MANUAL"):
        print("[aim] MANUAL never confirmed: is telemetry_bridge up?")
        return 2
    ok, why = t.reset_latch()
    print(f"[aim] MANUAL, drop latch reset: {why}", flush=True)

    cp = cannon_params()
    rows, seq = [], 0
    for dist in g["dist"]:
        for lat in g["lat"]:
            for dh in g["dh"]:
                seq += 1
                x, y, yaw = pose_for(dist, lat, dh)
                pin(seq, x, y, yaw)
                t0 = time.monotonic()
                while pinned_seq() != seq and time.monotonic() - t0 < 10.0:
                    t.spin_for(0.1)
                t.spin_for(2.0)                           # settle; fresh frames
                truth = (t.status or {}).get("berth") or {}
                row = {"dist": dist, "lat": lat, "dh": dh,
                       "true_dist": truth.get("lidar_to_deck_edge_m"),
                       "true_lat": truth.get("centreline_left_m"),
                       "true_dh": truth.get("yaw_off_square_deg")}
                t1 = time.monotonic()
                while t.window(slot) is None and time.monotonic() - t1 < 4.0:
                    t.spin_for(0.1)
                if t.window(slot) is None:
                    row.update(result="NOT SEEN", n=0)
                    rows.append(row)
                    print(f"[aim] {seq:2d} LiDAR {dist:.2f} m, {lat:+.2f} m, {dh:+.0f} deg: "
                          f"the camera does not see the {name.upper()} window", flush=True)
                    continue
                misses, wheres, cs, diag, why = shoot(t, slot, name)
                cs = cs or {}
                row.update(n=len(misses), aim_from=cs.get("aim_from"),
                           target_dist=cs.get("dist_m"), target_up=cs.get("height_m"))
                if diag:
                    # where the miss comes from: the aim point against the truth,
                    # the commanded angles against the ideal ones, the joints
                    # against the command, the hull's attitude
                    od = [d["odom"] for d in diag if d["odom"] is not None]
                    if od:
                        tbs = [truth_body(o[0], o[1], name) for o in od]
                        tb = [med(v[k] for v in tbs) for k in range(3)]
                        trp = [ned_roll_pitch(o[1]) for o in od]
                        troll, tpitch = med(v[0] for v in trp), med(v[1] for v in trp)
                    else:
                        tb, troll, tpitch = [float("nan")] * 3, float("nan"), float("nan")
                    tgt = [med(d["target"][k] for d in diag) for k in range(3)]
                    ideal = ca.solve(tb, cp, troll, tpitch)
                    row.update(
                        tgt_err_x=round(tgt[0] - tb[0], 3), tgt_err_y=round(tgt[1] - tb[1], 3),
                        tgt_err_z=round(tgt[2] - tb[2], 3),
                        pan_cmd_err=round(med(d["pan"] for d in diag) - ideal.pan_deg, 2),
                        tilt_cmd_err=round(med(d["tilt"] for d in diag) - ideal.tilt_deg, 2),
                        pan_joint_err=round(med(d["jpan"] for d in diag) - med(d["pan"] for d in diag), 2),
                        tilt_joint_err=round(med(d["jtilt"] for d in diag) - med(d["tilt"] for d in diag), 2),
                        roll_deg=round(math.degrees(med(d["att"][0] for d in diag)), 2),
                        pitch_deg=round(math.degrees(med(d["att"][1] for d in diag)), 2),
                        true_roll_deg=round(math.degrees(troll), 2),
                        true_pitch_deg=round(math.degrees(tpitch), 2))
                if misses:
                    dx = statistics.median(m[0] for m in misses)
                    dz = statistics.median(m[1] for m in misses)
                    good = abs(dx) <= a.tol and abs(dz) <= a.tol
                    row.update(miss_right=round(dx, 3), miss_high=round(dz, 3),
                               result="PASS" if good else "MISS")
                else:
                    row.update(result=("NO WATER ON THE FACE" if not wheres else wheres[-1].upper())
                               + (f" ({why})" if why and why not in ("FIRE", "spraying") else ""))
                rows.append(row)
                print(f"[aim] {seq:2d} LiDAR {dist:.2f} m, {lat:+.2f} m, {dh:+.0f} deg "
                      f"(true {row['true_dist']}, {row['true_lat']}, {row['true_dh']}): "
                      + (f"{row['miss_right'] * 100:+5.1f} cm right {row['miss_high'] * 100:+5.1f} cm high"
                         if "miss_right" in row else "-")
                      + f"  [{row['result']}] n={row['n']} aim from {row.get('aim_from')}", flush=True)
                if "tgt_err_x" in row:
                    print(f"[aim]      aim point off the truth: {row['tgt_err_x'] * 100:+.1f} fwd "
                          f"{row['tgt_err_y'] * 100:+.1f} left {row['tgt_err_z'] * 100:+.1f} up cm | "
                          f"cmd - ideal: pan {row['pan_cmd_err']:+.2f} tilt {row['tilt_cmd_err']:+.2f} deg | "
                          f"joint - cmd: pan {row['pan_joint_err']:+.2f} tilt {row['tilt_joint_err']:+.2f} deg | "
                          f"roll {row['roll_deg']:+.2f} pitch {row['pitch_deg']:+.2f} deg "
                          f"(true {row['true_roll_deg']:+.2f} {row['true_pitch_deg']:+.2f})", flush=True)
                t.spin_for(1.0)                           # the last water clears
    release()

    keys = ["dist", "lat", "dh", "true_dist", "true_lat", "true_dh", "miss_right", "miss_high",
            "n", "aim_from", "target_dist", "target_up", "tgt_err_x", "tgt_err_y", "tgt_err_z",
            "pan_cmd_err", "tilt_cmd_err", "pan_joint_err", "tilt_joint_err", "roll_deg",
            "pitch_deg", "true_roll_deg", "true_pitch_deg", "result"]
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(",".join(keys) + "\n")
        for r in rows:
            f.write(",".join("" if r.get(k) is None else str(r.get(k)) for k in keys) + "\n")
    hit = [r for r in rows if "miss_right" in r]
    passed = [r for r in rows if r["result"] == "PASS"]
    print(f"\n[aim] {name.upper()} window: {len(passed)} of {len(rows)} poses within "
          f"+-{a.tol * 100:.0f} cm of the middle", flush=True)
    if hit:
        worst = max(hit, key=lambda r: max(abs(r["miss_right"]), abs(r["miss_high"])))
        rms = math.sqrt(sum(r["miss_right"] ** 2 + r["miss_high"] ** 2 for r in hit) / len(hit))
        print(f"[aim] worst {worst['miss_right'] * 100:+.1f} cm right {worst['miss_high'] * 100:+.1f} cm high "
              f"(LiDAR {worst['dist']} m, {worst['lat']:+.2f} m, {worst['dh']:+.0f} deg); "
              f"rms {rms * 100:.1f} cm; table {a.out}", flush=True)
    t.destroy_node()
    rclpy.shutdown()
    return 0 if rows and len(passed) == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
