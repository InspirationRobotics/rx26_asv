"""fcu_watch — one JSON line a second: the flight mode, armed, the drop latch.

    python3 -u -m crusader_sim.fcu_watch

Runs in: the crsd-sim container (rclpy). `tx keys` (crusader_sim.tx_keys, in WSL)
reads it through a `docker exec` pipe to show the autopilot's side of the switches.
"""
import json
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from crusader_msgs.msg import FcuStatus


def main():
    rclpy.init()
    n = rclpy.create_node("fcu_watch")
    st = {"mode": None, "armed": None, "dropped": None, "t": 0.0}

    def on_status(m):
        st.update(mode=m.mode, armed=bool(m.armed), t=time.monotonic())

    n.create_subscription(FcuStatus, "/crsd/fcu_status", on_status, 10)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(Bool, "/crsd/autonomy_drop", lambda m: st.update(dropped=bool(m.data)), latched)
    last = 0.0
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        now = time.monotonic()
        if now - last >= 1.0:
            last = now
            print(json.dumps({"mode": st["mode"], "armed": st["armed"], "dropped": st["dropped"],
                              "stale": now - st["t"] > 3.0}), flush=True)


if __name__ == "__main__":
    main()
