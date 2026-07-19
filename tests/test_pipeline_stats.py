from robotx_2026.api.perception.pipeline_stats import PipelineStats


def feed(stats, n, fps, latency_s, t0=0.0):
    dt = 1.0 / fps
    for i in range(n):
        t = t0 + i * dt
        stats.record_frame(t, t + latency_s)
    return stats


def test_healthy_at_budget():
    s = feed(PipelineStats(), 60, fps=20.0, latency_s=0.05)
    assert s.fps > 19
    assert s.p95_latency_s <= 0.05 + 1e-9
    assert s.healthy


def test_unhealthy_when_slow_fps():
    s = feed(PipelineStats(), 60, fps=8.0, latency_s=0.05)
    assert not s.healthy


def test_unhealthy_when_over_latency_budget():
    s = feed(PipelineStats(), 60, fps=20.0, latency_s=0.150)
    assert not s.healthy


def test_drop_counting_and_snapshot():
    s = PipelineStats()
    s.record_frame(0.0, 0.05, dropped_no_depth=2)
    s.record_frame(0.05, 0.10, dropped_no_depth=1)
    snap = s.snapshot()
    assert snap["dropped_no_depth"] == 3
    assert snap["frames"] == 2
    assert set(snap) == {"fps", "p95_latency_ms", "frames",
                         "dropped_no_depth", "healthy"}


def test_empty_stats_not_healthy():
    assert not PipelineStats().healthy
