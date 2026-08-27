#!/usr/bin/env python3
"""oak_view.py — forward the camera stream to a laptop as MJPEG over HTTP.

Subscribes to a camera topic and re-serves it at http://<JETSON_IP>:8080 so
anyone on the boat's network can see what the camera sees in a browser. Field
bring-up tool: the one question it answers is "is the camera alive and pointing
where I think?".

    python3 tools/oak_view.py --topic /oak/rgb        # what oakd_publisher emits
    python3 tools/oak_view.py --topic /oak/rgb/image_raw/compressed
    python3 tools/oak_view.py --port 8081

This does NOT open the camera. The OAK-D allows a single client and
`crusader_perception`'s node is it — that is the whole reason this reads a topic
instead of talking to depthai directly. Any number of these can run at once, and
starting one can never take the camera away from perception.

REACH FOR `buoy_detector`'s OWN VIEW FIRST. It serves the same port with boxes,
ranges and depth-sample patches drawn on, which answers "is the camera alive"
*and* "is the detector working". This script is for the case where you want a
raw topic — a bare `oakd_publisher` run — and they cannot both bind 8080.

TOPIC NAMES: `oakd_publisher` publishes `oak/rgb` (raw `bgr8`) and `oak/depth`.
The DEFAULT here is instead the `/compressed` variant of `depthai_ros_driver`'s
naming, kept because those bytes are already a JPEG and go straight to the
browser with no decode, no re-encode and no cv2/numpy on this side. Against our
own node you must therefore pass `--topic /oak/rgb`, and that path needs cv2.

REQUIREMENTS: rclpy + sensor_msgs, i.e. any container with ROS 2 on the path —
including `asv`. Raw (uncompressed) topics additionally need cv2 + numpy to
encode each frame; the script says so plainly rather than failing obscurely.

Run it from a container started with `--network host` (ours are), or DDS will not
see the publisher.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

import mjpeg_server                       # sibling module; see its docstring

DEFAULT_TOPIC = "/oak/rgb/image_raw/compressed"
DEFAULT_PORT = 8080
JPEG_QUALITY = 70


class CameraRelay(Node):

    def __init__(self, topic, buffer):
        super().__init__("oak_view")
        self.buf = buffer
        self.frames = 0
        self.compressed = topic.endswith("/compressed")

        # Sensor-data QoS (BEST_EFFORT). Image publishers use it, and a RELIABLE
        # subscription to a BEST_EFFORT publisher matches nothing at all: the
        # topic lists fine, `ros2 topic hz` shows the publisher, and this node
        # silently receives zero frames. That mismatch is the single most likely
        # reason for a black page here.
        if self.compressed:
            self.create_subscription(CompressedImage, topic, self._compressed_cb,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(Image, topic, self._raw_cb,
                                     qos_profile_sensor_data)

        self.get_logger().info(f"subscribed to {topic}")
        self.create_timer(5.0, self._health)

    def _health(self):
        if self.frames == 0:
            self.get_logger().warn(
                "no frames yet — is oakd_publisher running? check "
                "`ros2 topic list` and that this container has --network host",
                throttle_duration_sec=10.0)

    def _compressed_cb(self, msg: CompressedImage):
        # msg.data is already a JPEG (image_transport's compressed plugin) —
        # hand it to the browser untouched.
        self.frames += 1
        self.buf.put(bytes(msg.data))

    def _raw_cb(self, msg: Image):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.get_logger().error(
                "raw Image topic needs cv2 + numpy to encode. Either subscribe "
                "to the compressed topic (add '/compressed' to --topic, no "
                "extra deps) or install opencv-python here.")
            raise SystemExit(2)

        buf = np.frombuffer(msg.data, dtype=np.uint8)
        enc = msg.encoding.lower()
        if enc in ("bgr8", "rgb8"):
            frame = buf.reshape(msg.height, msg.width, 3)
            if enc == "rgb8":
                frame = frame[:, :, ::-1]          # cv2 encodes BGR
        elif enc == "mono8":
            frame = buf.reshape(msg.height, msg.width)
        else:
            self.get_logger().error(
                f"unsupported encoding {msg.encoding!r} — use the compressed topic")
            raise SystemExit(2)

        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            self.frames += 1
            self.buf.put(jpg.tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--topic", default=DEFAULT_TOPIC,
                    help=f"camera topic (default {DEFAULT_TOPIC}); a topic "
                         "ending in /compressed needs no image libraries")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    buffer = mjpeg_server.FrameBuffer()
    node = CameraRelay(args.topic, buffer)

    server = mjpeg_server.start(buffer, args.port, args.topic, "oak_view")
    print(f"Open http://<JETSON_IP>:{args.port} in a browser (Ctrl+C to stop).",
          file=sys.stderr)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
