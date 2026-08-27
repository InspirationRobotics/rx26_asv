#!/usr/bin/env python3
"""bench_stereo_ros — the full `oakd_publisher` pipeline, instrumented.

What the node does, minus params, launch, and the executor, plus a per-stage
timing breakdown. Compare against bench_stereo_noros: same device work, so the
difference is the ROS tax on 1.28 MB per synced pair.

    python3 bench_stereo_ros.py                  # publish both topics
    python3 bench_stereo_ros.py --no-publish     # everything except the write
    python3 bench_stereo_ros.py --rgb-only       # publish colour, skip depth
    python3 bench_stereo_ros.py --no-subpixel

Read the breakdown, not just the rate. `publish=` far above `build=` means the
DDS write is blocking and the fix is transport-side. Both small while the rate
stays low means the device never produced the frames, and bench_stereo_noros
will say the same. --rgb-only isolates the 512 kB depth image, which is 40% of
the bytes and the half that cannot be JPEG-compressed later.

As with bench_rgb_ros: no executor, no timers, flat loop — and re-measure with
`ros2 topic hz` running in the consuming container, since a live subscriber is
what the fusion node will actually be.
"""
import argparse
import time
from datetime import timedelta

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

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

    sync = pipeline.create(dai.node.Sync)
    sync.setSyncThreshold(timedelta(milliseconds=args.sync_ms))
    rgb.isp.link(sync.inputs["rgb"])
    stereo.depth.link(sync.inputs["depth"])

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("rgbd")
    sync.out.link(xout.input)

    return pipeline


def image_msg(frame, encoding, stamp):
    frame = np.ascontiguousarray(frame)
    message = Image()
    message.header.stamp = stamp
    message.header.frame_id = "bench"
    message.height, message.width = frame.shape[:2]
    message.encoding = encoding
    message.is_bigendian = 0
    message.step = int(frame.strides[0])
    message.data = frame.tobytes()
    return message


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--isp", type=int, default=3)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--sync-ms", type=int, default=50)
    ap.add_argument("--no-subpixel", action="store_true")
    ap.add_argument("--no-lr-check", action="store_true")
    ap.add_argument("--no-publish", action="store_true")
    ap.add_argument("--rgb-only", action="store_true",
                    help="publish colour only; measures depth's share")
    args, ros_args = ap.parse_known_args()

    print(f"== stereo + ROS | isp=1/{args.isp} fps={args.fps:g} "
          f"subpixel={not args.no_subpixel} publish={not args.no_publish} "
          f"rgb_only={args.rgb_only} for {args.seconds:g}s")

    rclpy.init(args=ros_args)
    node = rclpy.create_node("bench_stereo_ros")
    pub_rgb = node.create_publisher(Image, "/bench/rgb", qos_profile_sensor_data)
    pub_depth = node.create_publisher(Image, "/bench/depth",
                                      qos_profile_sensor_data)

    with dai.Device(build(args)) as device:
        print(f"   usb={device.getUsbSpeed().name}")
        queue = device.getOutputQueue("rgbd", maxSize=4, blocking=False)

        frames = 0
        incomplete = 0
        t_cv = t_msg = t_pub = 0.0

        warmup_until = time.perf_counter() + 1.0
        while time.perf_counter() < warmup_until:
            queue.tryGet()

        start = time.perf_counter()
        while time.perf_counter() - start < args.seconds:
            group = queue.tryGet()
            if group is None:
                time.sleep(0.001)
                continue

            messages = {name: message for name, message in group}
            if "rgb" not in messages or "depth" not in messages:
                incomplete += 1
                continue

            t0 = time.perf_counter()
            rgb_frame = messages["rgb"].getCvFrame()
            depth_frame = messages["depth"].getFrame()
            t1 = time.perf_counter()

            stamp = node.get_clock().now().to_msg()
            rgb_image = image_msg(rgb_frame, "bgr8", stamp)
            depth_image = (None if args.rgb_only
                           else image_msg(depth_frame, "16UC1", stamp))
            t2 = time.perf_counter()

            if not args.no_publish:
                pub_rgb.publish(rgb_image)
                if depth_image is not None:
                    pub_depth.publish(depth_image)
            t3 = time.perf_counter()

            t_cv += t1 - t0
            t_msg += t2 - t1
            t_pub += t3 - t2
            frames += 1
        elapsed = time.perf_counter() - start

    node.destroy_node()
    rclpy.try_shutdown()

    print(f"   frames={frames}  incomplete={incomplete}  elapsed={elapsed:.1f}s  "
          f"RATE={frames / elapsed:.1f} fps")
    if frames:
        ms = 1000.0 / frames
        print(f"   per frame: getCvFrame+getFrame={t_cv * ms:.2f}ms  "
              f"build={t_msg * ms:.2f}ms  publish={t_pub * ms:.2f}ms  "
              f"total={(t_cv + t_msg + t_pub) * ms:.2f}ms")
        budget = 1000.0 / args.fps
        print(f"   frame budget at {args.fps:g} fps = {budget:.1f}ms "
              f"-> host work is {(t_cv + t_msg + t_pub) * ms / budget:.0%} of it")


if __name__ == "__main__":
    main()
