#!/usr/bin/env python3
"""bench_rgb_ros — one camera, published as a ROS Image. The ROS tax, alone.

Same device pipeline as bench_rgb_noros, plus message construction and a
publish. Run them back to back and the difference is exactly what wrapping one
stream in ROS costs — no stereo, no depth, no Sync in the way.

    python3 bench_rgb_ros.py                   # publish to /bench/rgb
    python3 bench_rgb_ros.py --no-publish      # build messages, never publish
    python3 bench_rgb_ros.py --isp 2 --fps 15

--no-publish is the important one. Everything except the DDS write still
happens: getCvFrame, the contiguity copy, tobytes, the Image message. If rates
match with and without it, the transport is free and the cost is host-side
conversion. If --no-publish is fast and publishing is slow, the write is
blocking and the fix is transport-side (socket buffers, shared memory, fewer
bytes) — not Python.

Deliberately NO executor and NO timers: a flat loop, so the number measures the
camera and the publish path rather than callback scheduling.

Watch for a subscriber changing the result. With nobody subscribed, DDS can
discard cheaply; the honest number for the fusion node is the one measured with
`ros2 topic hz /bench/rgb` running in the container that will consume it.
"""
import argparse
import time

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

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("rgb")
    rgb.isp.link(xout.input)

    return pipeline


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--isp", type=int, default=3)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--topic", default="/bench/rgb")
    ap.add_argument("--no-publish", action="store_true")
    args, ros_args = ap.parse_known_args()

    print(f"== rgb + ROS | isp=1/{args.isp} fps={args.fps:g} "
          f"publish={not args.no_publish} topic={args.topic} "
          f"for {args.seconds:g}s")

    rclpy.init(args=ros_args)
    node = rclpy.create_node("bench_rgb_ros")
    publisher = node.create_publisher(Image, args.topic, qos_profile_sensor_data)

    with dai.Device(build(args)) as device:
        print(f"   usb={device.getUsbSpeed().name}")
        queue = device.getOutputQueue("rgb", maxSize=4, blocking=False)

        frames = 0
        t_cv = t_msg = t_pub = 0.0

        warmup_until = time.perf_counter() + 1.0
        while time.perf_counter() < warmup_until:
            queue.tryGet()

        start = time.perf_counter()
        while time.perf_counter() - start < args.seconds:
            message = queue.tryGet()
            if message is None:
                time.sleep(0.001)
                continue

            t0 = time.perf_counter()
            frame = np.ascontiguousarray(message.getCvFrame())
            t1 = time.perf_counter()

            image = Image()
            image.header.stamp = node.get_clock().now().to_msg()
            image.header.frame_id = "bench"
            image.height, image.width = frame.shape[:2]
            image.encoding = "bgr8"
            image.is_bigendian = 0
            image.step = int(frame.strides[0])
            image.data = frame.tobytes()
            t2 = time.perf_counter()

            if not args.no_publish:
                publisher.publish(image)
            t3 = time.perf_counter()

            t_cv += t1 - t0
            t_msg += t2 - t1
            t_pub += t3 - t2
            frames += 1
        elapsed = time.perf_counter() - start

    node.destroy_node()
    rclpy.try_shutdown()

    print(f"   frames={frames}  elapsed={elapsed:.1f}s  "
          f"RATE={frames / elapsed:.1f} fps")
    if frames:
        ms = 1000.0 / frames
        print(f"   per frame: getCvFrame={t_cv * ms:.2f}ms  "
              f"build={t_msg * ms:.2f}ms  publish={t_pub * ms:.2f}ms  "
              f"total={(t_cv + t_msg + t_pub) * ms:.2f}ms")


if __name__ == "__main__":
    main()
