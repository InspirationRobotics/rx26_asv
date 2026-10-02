"""The RC pilot's stand-in for the rehearsal (the sim has no RC hand): arm over the TOOLING port 14550 and
select a mode through /crsd/set_mode, exactly what task1_goal did before lake mode took that job away.
NOT the panel and NOT lake_goal: the point of the rehearsal is that those two never do this.

    python3 pilot_standin.py arm-guided | guided | hold | status
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from crusader_msgs.msg import FcuStatus
from crusader_sim.operator_tools import arm, set_mode

cmd = sys.argv[1]
if cmd == "arm-guided":
    ok, why = arm()
    print("[pilot] " + why, flush=True)
    if not ok:
        sys.exit(2)
rclpy.init()
n = Node("pilot_standin")
if cmd in ("arm-guided", "guided", "hold"):
    name = "HOLD" if cmd == "hold" else "GUIDED"
    pub = n.create_publisher(String, "/crsd/set_mode", 10)
    print("[pilot] mode %s %s" % (name, "confirmed" if set_mode(n, pub, name) else "NOT confirmed"), flush=True)
else:
    got = []
    n.create_subscription(FcuStatus, "/crsd/fcu_status", lambda m: got.append((m.mode, m.armed)), 10)
    t = time.time()
    while not got and time.time() - t < 5:
        rclpy.spin_once(n, timeout_sec=0.2)
    print("[pilot] fcu", got[-1] if got else None, flush=True)
