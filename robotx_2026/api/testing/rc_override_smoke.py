"""rc_override_smoke — bench node for the G1 autonomy-drop validation.

PROPS OFF / boat on stands. This node publishes a continuous, obviously-inert RC
override (neutral 1500 on steer/throttle channels) through the sanctioned
/crsd/rc_override path, and loudly reports the moment the autonomy-drop latch
strips it. Used with docs/G1_bench_procedure.md:

  expected observable behavior when the drop switch is flipped (or RC is turned
  off): Mission Planner's RC monitor shows the override vanish within one
  control cycle (<=100 ms at 10 Hz), and this node logs "OVERRIDE BLOCKED".

It also serves as the reference implementation of the OverrideGuard pattern for
dp_hold and any Level-2-generated override mechanism.
"""
import rclpy
from rclpy.node import Node

from robotx_2026.api.common.node_main import run_node
from robotx_2026.api.common.override_guard import OverrideGuard

NEUTRAL = 1500
RATE_HZ = 10.0


class RcOverrideSmoke(Node):
    def __init__(self):
        super().__init__("rc_override_smoke")
        self.declare_parameter("channels", [1, 3])   # steer, throttle by default
        self.guard = OverrideGuard(self)
        self._was_allowed = None
        self.create_timer(1.0 / RATE_HZ, self._cycle)
        self.get_logger().info("bench override running — PROPS OFF, boat on stands")

    def _cycle(self):
        allowed = self.guard.allowed
        if allowed != self._was_allowed:
            if allowed:
                self.get_logger().info("override ACTIVE (neutral values)")
            else:
                self.get_logger().error("OVERRIDE BLOCKED — autonomy-drop in effect")
            self._was_allowed = allowed
        if not allowed:
            return                    # one-cycle stop: nothing computed, nothing sent
        ch = [0] * 18
        for c in self.get_parameter("channels").value:
            ch[c - 1] = NEUTRAL
        self.guard.publish_override(ch)


def main(args=None):
    run_node(RcOverrideSmoke, args=args)


if __name__ == "__main__":
    main()
