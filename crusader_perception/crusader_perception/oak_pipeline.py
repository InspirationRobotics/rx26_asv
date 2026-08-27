"""oak_pipeline — the one OAK-D RGB+depth pipeline, built in one place.

Two nodes in this package need the identical device configuration, and they need
it identical for a reason that is easy to miss: depth is aligned to the RGB
camera and emitted at the same size, which is the only thing that makes the RGB
intrinsics valid on the depth image and `depth[v, u]` legal for a (u, v) taken
off an RGB detection box. Two copies of this builder would drift — a different
ISP scale, a different setOutputSize — and the result is not a crash but ranges
that are quietly wrong.

Geometry ported from the prequal-proven gate_navigator in
InspirationRobotics/robotx_2026.

SAFE TO IMPORT ANYWHERE. Every `import depthai` is inside a function, so this
module — and any node that imports it at module scope — still imports in a
container with no device SDK: the `asv` build, the import smoke, unit tests off
the boat. Only calling a function here needs the hardware SDK present.
"""
SENSOR_WIDTH = 1920
SENSOR_HEIGHT = 1200

# Stereo settings. CONSTANTS, not parameters, and that is a simplification with
# teeth: all three OAK-D nodes have to configure the device identically or a
# range measured under one does not mean the same thing under another. As
# parameters they were three values duplicated across three sections of
# crusader_params.yaml, kept equal by a check in check_config.py. Here they are
# equal by construction and there is nothing left to check.
#
# Nobody has ever tuned them. If you need to, change them here — once.
SUBPIXEL = True             # finer far-range depth
LR_CHECK = True             # left/right check; rejects occlusions
SYNC_THRESHOLD_MS = 50      # max RGB/depth gap the device Sync will pair


def output_size(isp_denominator):
    """The frame size the whole pipeline runs at. 1/3 gives 640x400, which is
    the stereo-native size and needs no aspect-ratio cropping."""
    return SENSOR_WIDTH // isp_denominator, SENSOR_HEIGHT // isp_denominator


def build_rgbd(isp_denominator, fps, ae_compensation=0):
    """RGB + depth, aligned and paired on-device. Returns (pipeline, w, h).

    The Sync node is the point of the whole arrangement: RGB and depth leave the
    device already matched to within sync_threshold_ms, so no consumer has to
    guess which depth frame belongs to which image. A pair that cannot be
    matched never leaves the device.

    ae_compensation biases auto-exposure in EV steps, -9..9, negative darker.
    It is a TRAINING CONTRACT, not a preference: the LED classifier learned
    colours off footage shot at one bias, and a different one hands it colours
    it never saw. tools/oak_record.py takes the same flag and defaults to -3,
    which is what the current training set was shot at.

    Why it matters more than it sounds. A lit LED is self-luminous; the white
    hull beside it is reflecting the sun. Stopping down darkens the hull without
    darkening the LED, so the LED's colour separates from the hull's. Measured on
    the val split, median LED saturation minus the same buoy's hull is red +28,
    blue +15 and GREEN +6 — and an unlit buoy is also +6, which is why green is
    the class that fails. Getting green to blue's +15 is an exposure change, not
    an algorithm change.

    APPLIED TO CAM_A ONLY, deliberately. The stereo pair's exposure is a depth
    quality decision, not a colour one, and darkening them to fix a colour
    problem would spend range on the water for nothing. What stereo needs is for
    left and right to match EACH OTHER, which they do by being left alone
    together.

    0 disables it and leaves depthai's default AE untouched.
    """
    from datetime import timedelta

    import depthai as dai

    width, height = output_size(isp_denominator)
    pipeline = dai.Pipeline()

    rgb = pipeline.create(dai.node.ColorCamera)
    rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
    rgb.setIspScale(1, isp_denominator)
    rgb.setInterleaved(False)
    rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    rgb.setFps(fps)

    if ae_compensation:
        # Guarded rather than assumed: the call is missing on some depthai
        # builds, and a pipeline that fails to build takes the camera down.
        # Losing the exposure bias costs colour accuracy; losing the camera
        # costs everything. A silent skip would be worse than either, so it
        # raises with the version named — the caller logs it and carries on.
        try:
            rgb.initialControl.setAutoExposureCompensation(int(ae_compensation))
        except Exception as e:
            raise RuntimeError(
                f"depthai {dai.__version__} rejected "
                f"setAutoExposureCompensation({int(ae_compensation)}): {e}. "
                "Set ae_compensation to 0 to run without it — the classifier "
                "will then see a different exposure from its training set."
            ) from e

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
    stereo.setOutputSize(width, height)
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