"""DetectionInput — prefer LiDAR-fused detections when available, fall back to
camera-only transparently.

This is what makes the fusion node OPTIONAL and additive. A consumer wires its
detection callback through DetectionInput instead of subscribing to one topic;
it then receives:
  * /crsd/detections_fused  while the fusion node is alive (LiDAR-refined range), or
  * /crsd/detections_body   when it is not (camera-only bench runs, or fusion down).

Both streams carry frame="body" DetectionArray, so the consumer callback is
identical either way. Selection rule (no duplication, correct latency):
  - a fused message marks fusion "alive" for `fusion_timeout_s` and is delivered;
  - a body message is delivered ONLY when fusion is not alive — so while fusion
    runs the body stream is suppressed (no double-processing), and within one
    timeout of the fusion node dying the camera stream takes back over.

Every source switch is logged at INFO so a silent downgrade to camera-only is
visible (CLAUDE.md: fail loudly; a silently-degraded perception path is a
safety/scoring issue, not a nuisance).

Split like the perception cores: SourceSelector is the ROS-free decision logic
(unit-tested), DetectionInput is the thin rclpy subscription wrapper.
"""
import time

FUSED_TOPIC = "/crsd/detections_fused"
BODY_TOPIC = "/crsd/detections_body"


class SourceSelector:
    """Pure fused-vs-camera arbitration — no ROS, no clock (caller passes `now`).

    Returns whether each incoming message should be delivered to the consumer,
    and the label of the source currently driving, so switches can be logged.
    """
    def __init__(self, fusion_timeout_s: float = 1.0):
        self.timeout = fusion_timeout_s
        self.last_fused = None           # `now` of the most recent fused msg
        self.source = None               # last driving source: "fused" | "camera"

    def fusion_alive(self, now) -> bool:
        return (self.last_fused is not None
                and (now - self.last_fused) <= self.timeout)

    def on_fused(self, now) -> bool:
        """Fused msg arrived: always delivered; marks fusion alive."""
        self.last_fused = now
        self.source = "fused"
        return True

    def on_body(self, now) -> bool:
        """Body msg arrived: delivered only when fusion is not alive."""
        if self.fusion_alive(now):
            return False                 # fused stream is driving; drop the duplicate
        self.source = "camera"
        return True


class DetectionInput:
    def __init__(self, node, callback, *, fused_topic=FUSED_TOPIC,
                 body_topic=BODY_TOPIC, fusion_timeout_s=1.0, qos=10):
        """node: the owning rclpy Node. callback: fn(DetectionArray) -> None."""
        from interfaces.msg import DetectionArray

        self._node = node
        self._cb = callback
        self._sel = SourceSelector(fusion_timeout_s)
        node.create_subscription(DetectionArray, fused_topic, self._on_fused, qos)
        node.create_subscription(DetectionArray, body_topic, self._on_body, qos)

    def _on_fused(self, msg):
        prev = self._sel.source
        if self._sel.on_fused(time.monotonic()):
            self._log_switch(prev)
            self._cb(msg)

    def _on_body(self, msg):
        prev = self._sel.source
        if self._sel.on_body(time.monotonic()):
            self._log_switch(prev)
            self._cb(msg)

    def _log_switch(self, prev):
        if self._sel.source != prev:
            self._node.get_logger().info(f"detection input source: {self._sel.source}")
