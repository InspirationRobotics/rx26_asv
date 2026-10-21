"""oak_pipeline — the one OAK-D RGB+depth pipeline, built in one place.

camera_node builds its device from this; tools/oak_record.py keeps a standalone copy so it can
run on a laptop. Depth is aligned to the RGB camera's field of view, which is what makes the
RGB intrinsics valid on the depth image after a plain ratio of the two frame sizes.

Geometry ported from the prequal-proven gate_navigator in InspirationRobotics/robotx_2026.

SAFE TO IMPORT ANYWHERE: every `import depthai` is inside a function, so this module still
imports in a container with no device SDK (the `asv` build, the import smoke, unit tests off
the boat).
"""
from crusader_perception import oak_controls

# The OAK-D's stereo engine refuses an input wider than this (measured on the boat): at
# isp_denominator=1 the camera opens at 1920x1200, then StereoDepth logs "Maximum supported
# input image width for stereo is 1280. Skipping frame!" every frame, depth never arrives, and
# the detector receives NOTHING from a camera that is plainly running. Checked up front so that
# is one sentence at startup instead of a per-frame device log nobody reads.
STEREO_MAX_WIDTH = 1280

SENSOR_WIDTH = 1920
SENSOR_HEIGHT = 1200

# Stereo settings. Constants, not parameters: nobody has tuned them, and a range should mean
# the same thing in every recording. If you need to, change them here, once.
SUBPIXEL = True             # finer far-range depth
LR_CHECK = True             # left/right check; rejects occlusions
SYNC_THRESHOLD_MS = 50      # max RGB/depth gap the device Sync will pair


# The device-bound queue that carries CameraControl messages to CAM_A after
# the pipeline is running. Without it every exposure setting is frozen at build
# time, which on a dock at dusk means a rebuild and a restart to try a number.
CONTROL_STREAM = "control"


def output_size(isp_denominator):
    """The frame size the whole pipeline runs at. 1/3 gives 640x400, which is
    the stereo-native size and needs no aspect-ratio cropping."""
    return SENSOR_WIDTH // isp_denominator, SENSOR_HEIGHT // isp_denominator


def build_rgbd(isp_denominator, fps, controls=None, rgb_isp_denominator=None):
    """RGB + depth, aligned and paired on-device. Returns (pipeline, w, h) where w, h are the
    RGB frame's size.

    The Sync node is the point: RGB and depth leave the device already matched to within
    sync_threshold_ms, so no consumer guesses which depth frame belongs to which image; an
    unmatched pair never leaves the device.

    controls is the camera profile as {parameter: value} — see oak_controls, which owns the
    table of what exists and how each reaches depthai. It is applied to CAM_A's initialControl
    here and re-applied through the control queue on every change, so the picture at startup
    and after tuning come from the same code. None leaves depthai's defaults.

    APPLIED TO CAM_A ONLY, deliberately: the stereo pair's exposure is a depth-quality
    decision, and darkening it to fix a colour problem would spend range on the water. Stereo
    needs left and right to match EACH OTHER, which they do by being left alone together.

    rgb_isp_denominator SCALES THE COLOUR CAMERA SEPARATELY FROM THE STEREO PAIR (default
    isp_denominator, so existing callers are unchanged), because the two paths want opposite
    things:

      * Moving both from 1/3 to 1/2 (640x400 -> 960x600) HALVED the frame rate on the boat
        (27.8 -> 13.6 fps) while inference barely moved (det 20-25 -> 24-32 ms); the rest is
        stereo and USB.
      * Depth gains nothing from that resolution, but the LED classifier cuts a 64x64 crop from
        the raw frame, so every pixel the sensor keeps is one the crop need not invent.
      * Beyond 1280 the stereo engine refuses (STEREO_MAX_WIDTH), so a full-resolution COLOUR
        frame is unreachable while the two share one scale.

    The cost is that `depth[v, u]` stops being legal, which this module's header exists to
    protect. The caller pays it: buoy_model scales detection boxes into depth space and keeps
    the bearing maths on RGB coordinates with RGB intrinsics. Depth is still ALIGNED to CAM_A
    (same field of view), so a plain ratio is the whole correction.
    """
    from datetime import timedelta

    import depthai as dai

    depth_width, depth_height = output_size(isp_denominator)
    rgb_den = isp_denominator if rgb_isp_denominator is None \
        else int(rgb_isp_denominator)
    width, height = output_size(rgb_den)

    if depth_width > STEREO_MAX_WIDTH:
        raise ValueError(
            f"isp_denominator={isp_denominator} puts the stereo pair at "
            f"{depth_width}px wide; the OAK-D's stereo engine refuses anything "
            f"over {STEREO_MAX_WIDTH} and drops EVERY frame, so the node would "
            "run and publish nothing. Raise isp_denominator (it scales depth), "
            "and use rgb_isp_denominator to get a bigger COLOUR frame.")

    pipeline = dai.Pipeline()

    rgb = pipeline.create(dai.node.ColorCamera)
    rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
    rgb.setIspScale(1, rgb_den)
    rgb.setInterleaved(False)
    rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    rgb.setFps(fps)

    if controls:
        # Refusals are logged by the caller, not raised. A setter missing from
        # one depthai build should cost that one control, not the camera — and
        # this is the path that decides whether the boat can see at all.
        refused = oak_controls.apply(dai, rgb.initialControl, controls)
        if refused:
            raise RuntimeError(
                f"depthai {dai.__version__} refused: {'; '.join(refused)}")

    # Exposure is settable while the camera runs, not only at build time. The
    # numbers that matter here are the ones you find by watching the picture
    # change, and a camera you have to rebuild and restart to retune is a
    # camera nobody retunes — they live with the dark frame instead.
    control_in = pipeline.create(dai.node.XLinkIn)
    control_in.setStreamName(CONTROL_STREAM)
    control_in.out.link(rgb.inputControl)

    # OAK-D LR: the stereo pair are colour sensors too, so they are ColorCamera
    # nodes scaled identically to CAM_A.
    left = pipeline.create(dai.node.ColorCamera)
    left.setCamera("left")
    left.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
    left.setIspScale(1, isp_denominator)
    left.setFps(fps)

    right = pipeline.create(dai.node.ColorCamera)
    right.setCamera("right")
    right.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
    right.setIspScale(1, isp_denominator)
    right.setFps(fps)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setLeftRightCheck(LR_CHECK)
    stereo.setSubpixel(SUBPIXEL)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)      # depth in RGB pixels
    # Depth is emitted at the STEREO size and aligned into CAM_A's field of
    # view. Same view, fewer pixels — which is what makes the caller's ratio a
    # correct mapping rather than an approximation.
    stereo.setOutputSize(depth_width, depth_height)
    left.isp.link(stereo.left)
    right.isp.link(stereo.right)

    sync = pipeline.create(dai.node.Sync)
    sync.setSyncThreshold(timedelta(milliseconds=SYNC_THRESHOLD_MS))
    rgb.isp.link(sync.inputs["rgb"])
    stereo.depth.link(sync.inputs["depth"])

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("rgbd")
    sync.out.link(xout.input)

    return pipeline, width, height


def rgb_intrinsics(device, width, height):
    """(fx, fy, cx, cy) for the RGB camera AT THE PIPELINE SIZE.

    Scale matters: intrinsics read at 1920x1200 are wrong by 3x for a 640x400
    frame, and the error is a plausible-looking range rather than an exception.
    Ask the calibration for the size actually in use.
    """
    import depthai as dai

    matrix = device.readCalibration().getCameraIntrinsics(
        dai.CameraBoardSocket.CAM_A, width, height)
    return (float(matrix[0][0]), float(matrix[1][1]),
            float(matrix[0][2]), float(matrix[1][2]))


def usb_warning(device):
    """A sentence about the USB link if it negotiated below SuperSpeed, else
    None. USB2 cannot carry this pipeline at 30fps; it degrades to a fraction of
    the rate, which reads as a perception bug three layers downstream."""
    import depthai as dai

    speed = device.getUsbSpeed()
    if speed in (dai.UsbSpeed.SUPER, dai.UsbSpeed.SUPER_PLUS):
        return None
    return (f"link negotiated {speed.name}, not SUPER — expect dropped frames. "
            "Check the cable and the port (blue/SS).")