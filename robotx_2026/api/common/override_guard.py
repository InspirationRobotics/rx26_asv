"""OverrideGuard — defense-in-depth client for RC-override-producing nodes.

telemetry_bridge already enforces the autonomy-drop latch at the wire, so a node
that ignores this guard still cannot reach the Pixhawk after a trip. The guard
exists so well-behaved nodes (dp_hold, future Level-2 mechanisms) also STOP
COMPUTING overrides within one control cycle, log the drop, and can transition
their own state machines (e.g. dp_hold -> idle) instead of publishing into a
void.

Usage inside a node:
    self.guard = OverrideGuard(self)
    ...
    def control_cycle(self):
        if not self.guard.allowed:
            self.enter_idle()        # node-specific safe state
            return
        self.guard.publish_override(channels_18)

Wiring note for the existing dp_hold on the Jetson: replace its direct override
path with guard.publish_override(), and delete any node-local MAVLink connection
— /crsd/rc_override via telemetry_bridge is the only sanctioned path.
"""
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool

from interfaces.msg import RcChannels


class OverrideGuard:
    def __init__(self, node):
        self._node = node
        self._dropped = True         # fail-safe until first latched msg arrives
        latched_qos = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._sub = node.create_subscription(
            Bool, "/crsd/autonomy_drop", self._cb, latched_qos)
        self._pub = node.create_publisher(RcChannels, "/crsd/rc_override", 10)

    def _cb(self, msg: Bool):
        was = self._dropped
        self._dropped = msg.data
        if self._dropped and not was:
            self._node.get_logger().error(
                "autonomy-drop tripped — this node must stop overriding NOW")
        elif was and not self._dropped:
            self._node.get_logger().warn("autonomy-drop cleared — overrides re-enabled")

    @property
    def allowed(self) -> bool:
        return not self._dropped

    def publish_override(self, channels):
        """channels: iterable of us values, up to 18; 0 = release that channel.
        Silently refuses while dropped (bridge would refuse anyway)."""
        if self._dropped:
            return False
        msg = RcChannels()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        ch = list(channels)[:18]
        msg.channels = ch + [0] * (18 - len(ch))
        self._pub.publish(msg)
        return True
