"""sim_verdict — the Task 3 referee so far, once: `sim verdict` (crusader_sim/scripts/sim.sh).

    python3 -m crusader_sim.sim_verdict

Runs in: the crsd-sim container. Reads task3_world's /sim/task3/status (the lights,
the water) and its latched /sim/task3/verdict (every report as RoboCommand judged it).
"""
import json
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    rclpy.init()
    n = rclpy.create_node("sim_verdict")
    got = {}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(String, "/sim/task3/verdict", lambda m: got.update(v=m.data), latched)
    n.create_subscription(String, "/sim/task3/status", lambda m: got.update(s=m.data), 10)
    t0 = time.time()
    while time.time() - t0 < 3.0 and not ("v" in got and "s" in got):
        rclpy.spin_once(n, timeout_sec=0.1)
    if "s" in got:
        s = json.loads(got["s"])
        print(f"lights {s.get('lights')} | window {s.get('window')} {s.get('colour')} | "
              f"water on: {s.get('water_on')} | sprayed {s.get('sprayed_s')} s")
    if "v" in got:
        v = json.loads(got["v"])
        print("VERDICT so far: " + ("PASS" if v.get("pass") else "not passed")
              + ("" if v.get("complete") else " (not finished)"))
        for line in v.get("lines", []):
            print("  " + line)
    elif "s" not in got:
        print("no referee: is a Task 3 course up (sim up)?")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
