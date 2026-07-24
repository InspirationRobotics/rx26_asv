"""Pure-logic tests for DetectionInput's fused-vs-camera arbitration (ROS-free)."""
from robotx_2026.api.common.detection_input import SourceSelector


def test_camera_only_when_no_fusion():
    # no fused msg ever -> every body msg is delivered (camera-only bench run)
    s = SourceSelector(fusion_timeout_s=1.0)
    assert s.on_body(now=0.0) is True
    assert s.on_body(now=0.5) is True
    assert s.source == "camera"


def test_fused_suppresses_body_while_alive():
    s = SourceSelector(fusion_timeout_s=1.0)
    assert s.on_fused(now=10.0) is True        # fused delivered, fusion now alive
    assert s.on_body(now=10.1) is False        # body suppressed (no duplicate)
    assert s.on_body(now=10.9) is False        # still within timeout
    assert s.source == "fused"


def test_body_resumes_after_fusion_times_out():
    s = SourceSelector(fusion_timeout_s=1.0)
    s.on_fused(now=10.0)
    assert s.on_body(now=11.5) is True         # >1 s since last fused -> camera takes over
    assert s.source == "camera"


def test_fusion_recovers_after_camera_fallback():
    s = SourceSelector(fusion_timeout_s=1.0)
    s.on_fused(now=0.0)
    s.on_body(now=2.0)                          # fell back to camera
    assert s.source == "camera"
    assert s.on_fused(now=2.1) is True          # fusion back -> drives again
    assert s.source == "fused"
    assert s.on_body(now=2.2) is False          # body suppressed again


def test_timeout_boundary_inclusive():
    s = SourceSelector(fusion_timeout_s=1.0)
    s.on_fused(now=0.0)
    assert s.fusion_alive(now=1.0) is True      # exactly at timeout still alive
    assert s.fusion_alive(now=1.0001) is False


def test_never_alive_before_any_fused():
    s = SourceSelector(fusion_timeout_s=1.0)
    assert s.fusion_alive(now=100.0) is False
