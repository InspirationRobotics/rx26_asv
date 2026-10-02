"""Print the standalone judge's latest verdict (/sim/task1_judge, 1 Hz JSON) through format_verdict."""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from crusader_sim.task1_judge import format_verdict

rclpy.init()
n = Node("judge_verdict")
got = []
n.create_subscription(String, "/sim/task1_judge", lambda m: got.append(m.data), 10)
t = time.time()
while not got and time.time() - t < 8:
    rclpy.spin_once(n, timeout_sec=0.2)
if not got:
    print("no verdict on /sim/task1_judge")
else:
    v = json.loads(got[-1])
    print(format_verdict(v))
    print("PASS" if v.get("pass") else "NOT PASS")
