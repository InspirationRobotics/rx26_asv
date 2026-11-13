"""task1_score — a Task 1 Core run scored the way the handbook scores it (Section 4, 325 points).

    python3 -m crusader_sim.task1_score record --out FILE
        keeps the LAST safe_passage report the boat sent (/crsd/robocommand/report) in FILE,
        rewritten on every report, until killed
    python3 -m crusader_sim.task1_score score --course task1_sg --log RUN.log --report FILE
                                             [--json] [--plot PNG]
        the 325-point breakdown of one run: RUN.log is task1_goal's output (the judge's verdict
        is at its end), FILE is what `record` kept; --plot draws the true track (FILE.track.csv,
        also written by `record`) over the true field, with the reported buoys

Runs in: the crsd-sim container. Sim-only, never on the boat.

WHY. task1_judge grades the PATH (sides, circles, contact). Core tier also pays for the REPORT:
the ENTRY and the EXIT (10 each), and every flashing GREEN and RED at its true position (10 each,
up to 4 of each) are 100 of the 325 points, and the 100-point bonus needs all of it. A run can
drive perfectly and still lose most of that on a report built from duplicated or mislabelled
tracks, which nothing else in the sim looks at.

THE TABLE (handbook section-4-scoring-and-awards, Safe Passage, Core Tier):
    heartbeat (the run reported TASK_SAFE_PASSAGE)        25
    reports correct ENTRY / EXIT                          10 + 10
    circles ENTRY clockwise / EXIT counter-clockwise      25 + 25
    reports correct GREEN / RED locations, 10 each, <= 4  40 + 40
    after the ENTRY circle, ALL greens to port            25
    after the ENTRY circle, ALL reds to starboard         25
    bonus: every item above in full, and no contact       100
  and the cross-cutting -10 per strike of a course element, shown on its own line.

REPORT MATCHING. The handbook gives no position tolerance. A reported buoy is credited to the
TRUE buoy nearest it when that is within REPORT_TOL_M and its state is that buoy's; each true
buoy is credited once. 1.5 m is under the field's smallest centre-to-centre spacing (1.86 m,
task1_sg.yaml), so a credit can never belong to the neighbour. Everything else reported as red
or green is a false report: not scored by the table, listed because a judge will see it.
"""
import argparse
import json
import math
import os
import re
import sys
import time

from crusader_sim import course as C

REPORT_TOL_M = 1.5
REPORT_TOPIC = "/crsd/robocommand/report"
STATE_OF = {"flash_red": "BEACON_STATE_FLASHING_RED",
            "flash_green": "BEACON_STATE_FLASHING_GREEN",
            "flash_blue": "BEACON_STATE_FLASHING_BLUE",
            "steady_blue": "BEACON_STATE_STEADY_BLUE",
            "off": "BEACON_STATE_OFF"}


# ------------------------------------------------------------------ record

def record(out_path):
    """The last safe_passage report in out_path, and the TRUE track (/sim/crusader/odometry,
    5 Hz: t, x, y, yaw) in out_path + '.track.csv', for `plot`."""
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from crusader_msgs.msg import RoboCommandReport

    rclpy.init()
    node = Node("task1_score_record")
    n = [0]
    track = open(out_path + ".track.csv", "w", encoding="utf-8")
    track.write("t,x,y,yaw\n")
    last_t = [None]

    def on_odom(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if last_t[0] is not None and t - last_t[0] < 0.2:
            return
        last_t[0] = t
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        track.write(f"{t:.2f},{p.x:.3f},{p.y:.3f},{yaw:.4f}\n")
        track.flush()

    node.create_subscription(Odometry, "/sim/crusader/odometry", on_odom, 10)

    def on_report(msg):
        if msg.kind != "safe_passage":
            return
        n[0] += 1
        try:
            body = json.loads(msg.body_json)
        except ValueError:
            return
        tmp = out_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"wall_t": time.time(), "n_reports": n[0], "body": body}, f)
        os.replace(tmp, out_path)

    node.create_subscription(RoboCommandReport, REPORT_TOPIC, on_report, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        track.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# -------------------------------------------------------------------- plot

def plot(course_name, track_csv, report, png, title):
    """The true track over the true field, the reported buoys on top. matplotlib (Agg)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    course = C.load(course_name)
    colour = {"flash_red": "red", "flash_green": "green", "flash_blue": "blue",
              "steady_blue": "navy", "off": "black"}
    fig, ax = plt.subplots(figsize=(7, 9))
    xs, ys = [], []
    if os.path.isfile(track_csv):
        with open(track_csv, encoding="utf-8") as f:
            next(f, None)
            for line in f:
                p = line.split(",")
                if len(p) == 4:
                    xs.append(float(p[1]))
                    ys.append(float(p[2]))
    if xs:
        ax.plot(xs, ys, "-", color="0.4", lw=1.2, label="boat (truth)")
        ax.plot(xs[0], ys[0], "k^", ms=9, label="start")
        for i in range(0, len(xs), 25):            # a dot every ~5 s of sim time
            ax.plot(xs[i], ys[i], ".", color="0.6", ms=3)
    for name, x, y, beacon, *_ in C.buoys(course):
        ax.add_patch(plt.Circle((x, y), 0.22, color=colour.get(beacon, "grey"), zorder=3))
        ax.annotate(name, (x, y), textcoords="offset points", xytext=(6, 4), fontsize=7)
    body = (report or {}).get("body") or {}
    o = course["origin"]
    for rb in body.get("buoys", []):
        pos = rb.get("position") or {}
        if "latitude" in pos:
            e, n = latlon_to_enu(pos["latitude"], pos["longitude"], o)
            st = rb.get("state", "")
            c = ("red" if "RED" in st else "green" if "GREEN" in st else
                 "blue" if "BLUE" in st else "black")
            ax.plot(e, n, "x", color=c, ms=7, mew=1.5)
    ax.plot([], [], "kx", label="reported buoy (colour = reported state)")
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title(title, fontsize=9)
    ax.legend(loc="lower right", fontsize=7)
    fig.tight_layout()
    fig.savefig(png, dpi=110)


# ------------------------------------------------------------------- score

def latlon_to_enu(lat, lon, origin):
    """Inverse of course.enu_to_latlon: the sim's own metres-per-degree."""
    north = (lat - origin["lat"]) * C.EARTH_M_PER_DEG
    east = (lon - origin["lon"]) * C.EARTH_M_PER_DEG * math.cos(math.radians(origin["lat"]))
    return east, north


def parse_judge(text):
    """{buoys: {name: verdict}, circles: {name: verdict}, contacts: str, pass: bool} from the
    judge's verdict lines in a task1_goal log; None when the log has no verdict."""
    if "[judge] VERDICT:" not in text:
        return None
    head = re.search(r"\[judge\] VERDICT: (PASS|FAIL).*contacts (.*)$", text, re.M)
    buoys = dict(re.findall(r"^\[judge\]\s+buoy (\S+): (.*)$", text, re.M))
    circles = dict(re.findall(r"^\[judge\]\s+circle (\S+): (.*)$", text, re.M))
    return {"pass": head.group(1) == "PASS", "contacts": head.group(2).strip(),
            "buoys": buoys, "circles": circles}


def match_report(report_body, truth, origin):
    """Credit each reported red/green to the true buoy nearest it (REPORT_TOL_M, state must
    match, one credit per true buoy). Returns (credited names, false reports)."""
    credited, false = set(), []
    for rb in (report_body or {}).get("buoys", []):
        state = rb.get("state", "")
        if state not in (STATE_OF["flash_red"], STATE_OF["flash_green"]):
            continue
        pos = rb.get("position") or {}
        if "latitude" not in pos:
            continue
        e, n = latlon_to_enu(pos["latitude"], pos["longitude"], origin)
        name, x, y, beacon = min(truth, key=lambda b: math.hypot(b[1] - e, b[2] - n))[:4]
        d = math.hypot(x - e, y - n)
        if d <= REPORT_TOL_M and STATE_OF[beacon] == state and name not in credited:
            credited.add(name)
        else:
            false.append({"state": state.replace("BEACON_STATE_", ""), "nearest": name,
                          "nearest_is": beacon, "d_m": round(d, 2)})
    return credited, false


def anchor_ok(report_body, key, truth_xy, origin):
    pos = (report_body or {}).get(key)
    if not pos or truth_xy is None:
        return False, None
    e, n = latlon_to_enu(pos["latitude"], pos["longitude"], origin)
    d = math.hypot(truth_xy[0] - e, truth_xy[1] - n)
    return d <= REPORT_TOL_M, round(d, 2)


def score(course_name, log_text, report):
    course = C.load(course_name)
    origin = course["origin"]
    truth = C.buoys(course)
    by_beacon = lambda s: [b for b in truth if b[3] == s]
    entry = next(iter(by_beacon("flash_blue")), None)
    exit_ = next(iter(by_beacon("steady_blue")), None)
    greens, reds = by_beacon("flash_green"), by_beacon("flash_red")
    body = (report or {}).get("body")
    judge = parse_judge(log_text) or {"pass": False, "contacts": "no verdict", "buoys": {},
                                      "circles": {}}
    s = {}
    s["heartbeat"] = 25 if "TASK_SAFE_PASSAGE" in log_text else 0
    e_ok, e_d = anchor_ok(body, "entry_position", entry and entry[1:3], origin)
    x_ok, x_d = anchor_ok(body, "exit_position", exit_ and exit_[1:3], origin)
    s["report_entry"] = 10 if e_ok else 0
    s["report_exit"] = 10 if x_ok else 0
    circ = judge["circles"]
    entry_circled = entry is not None and circ.get(entry[0], "").startswith("correct")
    s["circle_entry_cw"] = 25 if entry_circled else 0
    s["circle_exit_ccw"] = 25 if exit_ and circ.get(exit_[0], "").startswith("correct") else 0
    credited, false = match_report(body, truth, origin)
    g_rep = sum(1 for b in greens if b[0] in credited)
    r_rep = sum(1 for b in reds if b[0] in credited)
    s["report_greens"] = 10 * min(4, g_rep)
    s["report_reds"] = 10 * min(4, r_rep)
    jb = judge["buoys"]
    side_ok = lambda bs: bool(bs) and all(jb.get(b[0], "").startswith("correct") for b in bs)
    s["nav_greens_port"] = 25 if entry_circled and side_ok(greens) else 0
    s["nav_reds_starboard"] = 25 if entry_circled and side_ok(reds) else 0
    contacts = judge["contacts"]
    no_contact = contacts == "none"
    full = (s["heartbeat"] == 25 and e_ok and x_ok and entry_circled and s["circle_exit_ccw"]
            and g_rep >= min(4, len(greens)) and r_rep >= min(4, len(reds))
            and s["nav_greens_port"] and s["nav_reds_starboard"] and no_contact)
    s["bonus"] = 100 if full else 0
    total = sum(s.values())
    n_contacts = 0 if no_contact else len(re.findall(r"\w+", contacts)) if contacts != "no verdict" else 0
    return {
        "course": course_name, "total": total, "max": 325, "items": s,
        "strike_penalty": -10 * n_contacts, "contacts": contacts,
        "judge_pass": judge["pass"],
        "report": {"entry_d_m": e_d, "exit_d_m": x_d,
                   "greens": f"{g_rep}/{len(greens)}", "reds": f"{r_rep}/{len(reds)}",
                   "false_reports": false,
                   "n_reports": (report or {}).get("n_reports", 0)},
        "sides": {b[0]: jb.get(b[0], "no verdict") for b in greens + reds},
        "circles": {k: v for k, v in circ.items()},
    }


def format_score(r):
    s = r["items"]
    rep = r["report"]
    lines = [f"[score] {r['total']}/325  ({r['course']}; judge {'PASS' if r['judge_pass'] else 'FAIL'})",
             f"[score]   heartbeat {s['heartbeat']}  report ENTRY {s['report_entry']} "
             f"(off {rep['entry_d_m']} m)  report EXIT {s['report_exit']} (off {rep['exit_d_m']} m)",
             f"[score]   circle ENTRY cw {s['circle_entry_cw']}  circle EXIT ccw {s['circle_exit_ccw']}",
             f"[score]   report greens {s['report_greens']} ({rep['greens']})  "
             f"reds {s['report_reds']} ({rep['reds']})  false reports {len(rep['false_reports'])}",
             f"[score]   greens to port {s['nav_greens_port']}  reds to starboard "
             f"{s['nav_reds_starboard']}  bonus {s['bonus']}",
             f"[score]   strikes {r['contacts']} ({r['strike_penalty']})"]
    for f in rep["false_reports"]:
        lines.append(f"[score]   false report: {f['state']} {f['d_m']} m from {f['nearest']} "
                     f"({f['nearest_is']})")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    rec = sub.add_parser("record")
    rec.add_argument("--out", required=True)
    sc = sub.add_parser("score")
    sc.add_argument("--course", required=True)
    sc.add_argument("--log", required=True)
    sc.add_argument("--report", default="")
    sc.add_argument("--json", action="store_true", help="one JSON line instead of text")
    sc.add_argument("--plot", default="", help="also draw the track and the report to this PNG")
    a = ap.parse_args(argv)
    if a.cmd == "record":
        record(a.out)
        return 0
    with open(a.log, encoding="utf-8", errors="replace") as f:
        text = f.read()
    report = None
    if a.report and os.path.isfile(a.report):
        with open(a.report, encoding="utf-8") as f:
            report = json.load(f)
    r = score(a.course, text, report)
    print(json.dumps(r) if a.json else format_score(r))
    if a.plot:
        try:
            plot(a.course, a.report + ".track.csv", report, a.plot,
                 f"{os.path.basename(a.log)}: {r['total']}/325, judge "
                 f"{'PASS' if r['judge_pass'] else 'FAIL'}")
        except ImportError as e:        # the crsd-sim image has no matplotlib; the WSL host does
            print(f"[score]   no plot: {e}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
