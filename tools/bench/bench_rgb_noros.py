#!/usr/bin/env python3
"""bench_rgb_noros — floor test: one camera, no stereo, no ROS.

The cheapest thing the OAK-D can do, and the closest match to robotx_2026's
`oak_view.py` (which is 1 camera, ISP 1/2, 15fps, and nothing else). Whatever
this reports is the ceiling every other configuration is measured against: no
later script can beat it, and a script that falls far below it has found a cost
worth naming.

    python3 bench_rgb_noros.py                     # 640x400 @ 30, XLink only
    python3 bench_rgb_noros.py --isp 2 --fps 15    # the robotx_2026 settings
    python3 bench_rgb_noros.py --cv                # + host-side NV12->BGR cost

--cv is the honest comparison with anything that touches pixels: `getCvFrame()`
is a real colorspace conversion on the host CPU, not an accessor, and every
consumer of this camera pays it. Run both ways; the gap is that conversion.

These four bench_* scripts duplicate their pipelines on purpose. Each one has to
be readable and runnable on its own to be worth anything as evidence — a shared
helper would put the variable under test in a file you are not looking at.
"""
import argparse
import time

import depthai as dai


def build(args):
    pipeline = dai.Pipeline()

    rgb = pipeline.create(dai.node.ColorCamera)
    rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
    rgb.setIspScale(1, args.isp)
    rgb.setInterleaved(False)
    rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    rgb.setFps(args.fps)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("rgb")
    rgb.isp.link(xout.input)

    return pipeline


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--isp", type=int, default=3,
                    help="ISP downscale 1/N of 1920x1200 (default 3 -> 640x400)")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--cv", action="store_true",
                    help="also call getCvFrame() and report its cost")
    args = ap.parse_args()

    print(f"== rgb only, no ROS | isp=1/{args.isp} fps={args.fps:g} "
          f"cv={args.cv} for {args.seconds:g}s")

    with dai.Device(build(args)) as device:
        print(f"   usb={device.getUsbSpeed().name} mxid={device.getMxId()}")
        queue = device.getOutputQueue("rgb", maxSize=4, blocking=False)

        frames = 0
        cv_seconds = 0.0
        shape = None
        # Discard the first second: the first frames arrive while the device is
        # still ramping, and counting them understates a healthy pipeline.
        warmup_until = time.perf_counter() + 1.0
        while time.perf_counter() < warmup_until:
            queue.tryGet()

        start = time.perf_counter()
        while time.perf_counter() - start < args.seconds:
            message = queue.tryGet()
            if message is None:
                time.sleep(0.001)
                continue
            if args.cv:
                t0 = time.perf_counter()
                frame = message.getCvFrame()
                cv_seconds += time.perf_counter() - t0
                shape = frame.shape
            frames += 1
        elapsed = time.perf_counter() - start

    print(f"   frames={frames}  elapsed={elapsed:.1f}s  "
          f"RATE={frames / elapsed:.1f} fps")
    if args.cv and frames:
        print(f"   getCvFrame: {1000.0 * cv_seconds / frames:.2f} ms/frame "
              f"({100.0 * cv_seconds / elapsed:.0f}% of one core)  shape={shape}")


if __name__ == "__main__":
    main()
