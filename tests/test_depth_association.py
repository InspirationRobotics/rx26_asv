import numpy as np

from robotx_2026.api.perception.depth_association import (
    CameraModel, associate, bbox_median_depth, project_to_body)

CAM = CameraModel(fx=800.0, fy=800.0, cx=640.0, cy=360.0)   # 1280x720-ish


def depth_frame(value_mm=10000, w=1280, h=720):
    return np.full((h, w), value_mm, dtype=np.uint16)


def test_center_detection_is_dead_ahead():
    # bbox centered on cx at 10 m -> body x ~ 0, y ~ 10
    dets = associate([("buoy_flash_red", 0.9, (600, 320, 680, 400))],
                     depth_frame(10000), CAM)
    assert len(dets) == 1
    d = dets[0]
    assert abs(d.x) < 1e-6
    assert abs(d.y - 10.0) < 1e-6


def test_offset_detection_projects_starboard():
    # bbox center at u = cx + 400 px at 10 m -> x = 400*10/800 = 5 m starboard
    dets = associate([("buoy_off", 0.9, (1000, 320, 1080, 400))],
                     depth_frame(10000), CAM)
    assert abs(dets[0].x - 5.0) < 1e-6


def test_mount_offset_applied():
    dets = associate([("misc", 0.5, (600, 320, 680, 400))],
                     depth_frame(10000), CAM, mount_offset=(0.1, 0.3))
    assert abs(dets[0].x - 0.1) < 1e-6
    assert abs(dets[0].y - 10.3) < 1e-6


def test_invalid_depth_drops_detection_not_zero_range():
    # all-zero depth region: detection must be DROPPED, never emitted at range 0
    dets = associate([("buoy_off", 0.9, (600, 320, 680, 400))],
                     depth_frame(0), CAM)
    assert dets == []


def test_median_robust_to_holes():
    d = depth_frame(10000)
    d[330:390, 610:670:2] = 0                  # stripe of invalid pixels
    raw = bbox_median_depth(d, (600, 320, 680, 400))
    assert raw == 10000.0


def test_median_uses_central_region():
    # bbox edges see the background (30 m), center sees the buoy (10 m):
    # shrink must keep the estimate on the buoy
    d = depth_frame(30000)
    d[340:380, 620:660] = 10000
    raw = bbox_median_depth(d, (610, 330, 670, 390), shrink=0.25)
    assert raw == 10000.0


def test_radius_scales_with_range():
    near = associate([("buoy_off", 0.9, (600, 320, 680, 400))],
                     depth_frame(5000), CAM)[0]
    far = associate([("buoy_off", 0.9, (600, 320, 680, 400))],
                    depth_frame(20000), CAM)[0]
    assert far.radius > near.radius


def test_project_to_body_signs():
    bx, by = project_to_body(u=CAM.cx - 160, v=360, z_m=8.0, cam=CAM)
    assert bx < 0                               # left of center = port = negative
    assert by == 8.0
