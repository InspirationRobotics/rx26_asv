#!/usr/bin/env python3
"""bench_stereo_noros — three cameras + aligned depth, still no ROS.

Same pipeline `oakd_publisher` builds, with every stage that costs something
behind a flag, and nothing published anywhere. This answers the only question
that matters first: does the DEVICE produce 30 synced pairs a second? If it does
not, no ROS-side work can help, and the fix is in the flags below.

    python3 bench_stereo_noros.py                    # the node's settings
    python3 bench_stereo_noros.py --no-subpixel      # drop the expensive mode
    python3 bench_stereo_noros.py --no-subpixel --no-lr-check
    python3 bench_stereo_noros.py --isp 4            # 480x300
    python3 bench_stereo_noros.py --no-sync          # rgb and depth separately

--no-sync is the sharpest tool here. With the Sync node the output rate is
min(rgb, depth) by construction, so a slow depth stream silently throttles
colour and both look equally bad. Split apart, the two rates are reported
independently and the slow half names itself.
"""
import argparse
import time
from datetime import timedelta

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

    left = pipeline.create(dai.node.ColorCamera)
    left.setCamera("left")
    left.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
    left.setIspScale(1, args.isp)
    left.setFps(args.fps)

    right = pipeline.create(dai.node.ColorCamera)
    right.setCamera("right")
    right.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
    right.setIspScale(1, args.isp)
    right.setFps(args.fps)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setLeftRightCheck(not args.no_lr_check)
    stereo.setSubpixel(not args.no_subpixel)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(1920 // args.isp, 1200 // args.isp)
    left.isp.link(stereo.left)
    right.isp.link(stereo.right)

    if args.no_sync:
        x_rgb = pipeline.create(dai.node.XLinkOut)
        x_rgb.setStreamName("rgb")
        rgb.isp.link(x_rgb.input)
        x_depth = pipeline.create(dai.node.XLinkOut)
        x_depth.setStreamName("depth")
        stereo.depth.link(x_depth.input)
    else:
        sync = pipeline.create(dai.node.Sync)
        sync.setSyncThreshold(timedelta(milliseconds=args.sync_ms))
        rgb.isp.link(sync.inputs["rgb"])
        stereo.depth.link(sync.inputs["depth"])
        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("rgbd")
        sync.out.link(xout.input)

    return pipeline


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--isp", type=int, default=3)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--sync-ms", type=int, default=50)
    ap.add_argument("--no-subpixel", action="store_true")
    ap.add_argument("--no-lr-check", action="store_true")
    ap.add_argument("--no-sync", action="store_true",
                    help="separate rgb/depth streams; report each rate")
    args = ap.parse_args()

    print(f"== stereo, no ROS | isp=1/{args.isp} fps={args.fps:g} "
          f"subpixel={not args.no_subpixel} lr_check={not args.no_lr_check} "
          f"sync={not args.no_sync} for {args.seconds:g}s")

    with dai.Device(build(args)) as device:
        print(f"   usb={device.getUsbSpeed().name} mxid={device.getMxId()}")

        if args.no_sync:
            queues = {"rgb": device.getOutputQueue("rgb", 4, False),
                      "depth": device.getOutputQueue("depth", 4, False)}
        else:
            queues = {"rgbd": device.getOutputQueue("rgbd", 4, False)}

        counts = {name: 0 for name in queues}
        incomplete = 0

        warmup_until = time.perf_counter() + 1.0
        while time.perf_counter() < warmup_until:
            for queue in queues.values():
                queue.tryGet()

        start = time.perf_counter()
        while time.perf_counter() - start < args.seconds:
            idle = True
            for name, queue in queues.items():
                message = queue.tryGet()
                if message is None:
                    continue
                idle = False
                counts[name] += 1
                if name == "rgbd":
                    names = {n for n, _ in message}
                    if names != {"rgb", "depth"}:
                        incomplete += 1
            if idle:
                time.sleep(0.001)
        elapsed = time.perf_counter() - start

    print(f"   elapsed={elapsed:.1f}s")
    for name, count in counts.items():
        print(f"   {name:6s} frames={count:5d}  RATE={count / elapsed:.1f} fps")
    if not args.no_sync:
        print(f"   incomplete groups={incomplete}")
        print("   NOTE: a synced rate is min(rgb, depth). Re-run with --no-sync "
              "to see which stream is the slow one.")


if __name__ == "__main__":
    main()
