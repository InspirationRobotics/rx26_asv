"""task1_judge — an independent Task 1 referee, from the boat's TRUE path.

    python3 -m crusader_sim.task1_judge --course task1_core     # standalone, Ctrl-C for the verdict
    (task1_goal runs one alongside every mission and prints its verdict)

Runs in: the crsd-sim container. Reads only /sim/crusader/odometry (ground truth)
and the course file — nothing the boat's stack computes — so it can disagree
with the tree, which is the point: the tree's own "passed correctly" counter is
the tree grading itself.

Rules (handbook 3.3.2:19-33):
    flashing RED   kept to STARBOARD      flashing GREEN   kept to PORT
    ENTRY (flashing BLUE) circled CLOCKWISE,  EXIT (steady BLUE) COUNTER-clockwise
    no contact with any buoy
Gates are each red paired with its nearest green (<= 15 m). A gate is judged when
the boat's path crosses the segment between them; "correct" = red on the
starboard side at the crossing. A circle is >= 330 deg of unwrapped bearing
swept around the buoy while within circle_radius_m of it.
"""
import argparse
import json
import math

from crusader_sim import course as C

CONTACT_M = 0.55          # boat half-beam (0.3) + buoy half-width (0.22) + slack
CIRCLE_DEG = 330.0


def _cross(ax, ay, bx, by):
    return ax * by - ay * bx


def _segments_intersect(p1, p2, q1, q2):
    d1 = _cross(q2[0] - q1[0], q2[1] - q1[1], p1[0] - q1[0], p1[1] - q1[1])
    d2 = _cross(q2[0] - q1[0], q2[1] - q1[1], p2[0] - q1[0], p2[1] - q1[1])
    d3 = _cross(p2[0] - p1[0], p2[1] - p1[1], q1[0] - p1[0], q1[1] - p1[1])
    d4 = _cross(p2[0] - p1[0], p2[1] - p1[1], q2[0] - p1[0], q2[1] - p1[1])
    return (d1 * d2 < 0) and (d3 * d4 < 0)


class Task1Judge:
    def __init__(self, course, circle_radius_m=8.0):
        self.buoys = C.buoys(course)
        reds = [b for b in self.buoys if b[3] == "flash_red"]
        greens = [b for b in self.buoys if b[3] == "flash_green"]
        self.gates = []
        for r in reds:
            g = min(greens, key=lambda g: math.hypot(g[1] - r[1], g[2] - r[2]), default=None)
            if g is not None and math.hypot(g[1] - r[1], g[2] - r[2]) <= 15.0:
                self.gates.append({"red": r[0], "green": g[0], "r": (r[1], r[2]),
                                   "g": (g[1], g[2]), "result": None})
        self.circles = {}
        for b in self.buoys:
            if b[3] in ("flash_blue", "steady_blue"):
                self.circles[b[0]] = {"want": "cw" if b[3] == "flash_blue" else "ccw",
                                      "xy": (b[1], b[2]), "swept": 0.0, "last": None,
                                      "done": None}
        self.radius = circle_radius_m
        self.contacts = set()
        self.prev = None
        self.events = []

    def update(self, x, y):
        p = (x, y)
        if self.prev is not None and p != self.prev:
            for gate in self.gates:
                if gate["result"] is None and _segments_intersect(self.prev, p, gate["r"], gate["g"]):
                    # travel direction d; red is to STARBOARD iff cross(d, red - p) < 0
                    d = (p[0] - self.prev[0], p[1] - self.prev[1])
                    side = _cross(d[0], d[1], gate["r"][0] - p[0], gate["r"][1] - p[1])
                    gate["result"] = "correct" if side < 0 else "WRONG WAY"
                    self._say(f"gate {gate['red']}/{gate['green']}: {gate['result']}")
        for name, c in self.circles.items():
            dx, dy = x - c["xy"][0], y - c["xy"][1]
            if math.hypot(dx, dy) <= self.radius:
                a = math.atan2(dy, dx)
                if c["last"] is not None:
                    c["swept"] += (a - c["last"] + math.pi) % (2 * math.pi) - math.pi
                c["last"] = a
                deg = math.degrees(c["swept"])
                if c["done"] is None and abs(deg) >= CIRCLE_DEG:
                    got = "ccw" if deg > 0 else "cw"
                    c["done"] = "correct" if got == c["want"] else f"WRONG ({got})"
                    self._say(f"circled {name} {got}: {c['done']}")
            else:
                c["last"] = None
        for b in self.buoys:
            if math.hypot(x - b[1], y - b[2]) < CONTACT_M and b[0] not in self.contacts:
                self.contacts.add(b[0])
                self._say(f"CONTACT with {b[0]}")
        self.prev = p

    def _say(self, s):
        self.events.append(s)
        print(f"[judge] {s}", flush=True)

    def verdict(self):
        gates_ok = sum(1 for g in self.gates if g["result"] == "correct")
        circles = {n: (c["done"] or f"not circled ({math.degrees(c['swept']):.0f} deg)")
                   for n, c in self.circles.items()}
        ok = (gates_ok == len(self.gates) and not self.contacts
              and all(c["done"] == "correct" for c in self.circles.values()))
        return {"pass": ok, "gates_correct": gates_ok, "gates": len(self.gates),
                "gates_detail": {f"{g['red']}/{g['green']}": g["result"] or "not crossed"
                                 for g in self.gates},
                "circles": circles, "contacts": sorted(self.contacts)}


def format_verdict(v):
    lines = [f"[judge] VERDICT: {'PASS' if v['pass'] else 'FAIL'} — gates "
             f"{v['gates_correct']}/{v['gates']} correct, contacts {v['contacts'] or 'none'}"]
    lines += [f"[judge]   gate {k}: {r}" for k, r in v["gates_detail"].items()]
    lines += [f"[judge]   circle {k}: {r}" for k, r in v["circles"].items()]
    return "\n".join(lines)


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String

    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--course", default="task1_core")
    a = ap.parse_args()
    judge = Task1Judge(C.load(a.course))
    rclpy.init()
    node = Node("task1_judge")
    pub = node.create_publisher(String, "/sim/task1_judge", 10)
    node.create_subscription(
        Odometry, "/sim/crusader/odometry",
        lambda m: judge.update(m.pose.pose.position.x, m.pose.pose.position.y), 10)
    node.create_timer(1.0, lambda: pub.publish(String(data=json.dumps(judge.verdict()))))
    print(f"[judge] {len(judge.gates)} gates, circles {list(judge.circles)}; Ctrl-C for the verdict")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print(format_verdict(judge.verdict()))


if __name__ == "__main__":
    main()
