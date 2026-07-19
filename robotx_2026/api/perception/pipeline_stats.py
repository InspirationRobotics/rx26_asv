"""PipelineStats — perception health accounting (fps, latency budget, drop counts).

Phase-2 budget (plan): capture -> published BODY position <= 100 ms, >= 15 fps
end-to-end at G2. The node publishes these numbers every second and WARNs when
over budget — quiet degradation of perception corrupts objective-1 metrics, so
health is a first-class output, not a debug print.

No ROS imports; unit-tested.
"""
from collections import deque


class PipelineStats:
    def __init__(self, window: int = 60, latency_budget_s: float = 0.100,
                 fps_floor: float = 15.0):
        self.window = window
        self.latency_budget_s = latency_budget_s
        self.fps_floor = fps_floor
        self._samples = deque(maxlen=window)   # (publish_t, latency_s)
        self.frames = 0
        self.dropped_no_depth = 0              # detections without valid depth

    def record_frame(self, capture_t: float, publish_t: float,
                     dropped_no_depth: int = 0):
        self._samples.append((publish_t, publish_t - capture_t))
        self.frames += 1
        self.dropped_no_depth += dropped_no_depth

    @property
    def fps(self):
        if len(self._samples) < 2:
            return 0.0
        span = self._samples[-1][0] - self._samples[0][0]
        return (len(self._samples) - 1) / span if span > 0 else 0.0

    @property
    def p95_latency_s(self):
        if not self._samples:
            return None
        lats = sorted(l for _, l in self._samples)
        return lats[min(len(lats) - 1, int(0.95 * len(lats)))]

    @property
    def healthy(self):
        p95 = self.p95_latency_s
        return (p95 is not None and p95 <= self.latency_budget_s
                and self.fps >= self.fps_floor)

    def snapshot(self):
        p95 = self.p95_latency_s
        return {
            "fps": round(self.fps, 1),
            "p95_latency_ms": round(p95 * 1000, 1) if p95 is not None else None,
            "frames": self.frames,
            "dropped_no_depth": self.dropped_no_depth,
            "healthy": self.healthy,
        }
