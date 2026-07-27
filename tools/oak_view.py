"""oak_view.py — standalone OAK-D camera check (field bring-up, no ROS).

One camera user at a time: stop perception_node first. Run inside the container,
then open http://<JETSON_IP>:8080 in a browser to confirm the camera streams
before anything moves

  python3 tools/oak_view.py

For detections + distances, use the ROS pipeline (perception_node + its health
topic); this tool is deliberately dependency-light for "is the camera alive?".
"""
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import depthai as dai

PORT = 8080


def build_pipeline():
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
    cam.setIspScale(1, 2)
    cam.setFps(15)
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("video")
    cam.isp.link(xout.input)
    return pipeline


def main():
    device = dai.Device(build_pipeline())
    q = device.getOutputQueue("video", maxSize=4, blocking=False)
    print(f"Camera up. Open http://<JETSON_IP>:{PORT} in a browser (Ctrl+C to stop).")

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    frame = q.get().getCvFrame()
                    ok, jpg = cv2.imencode(".jpg", frame,
                                           [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if not ok:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    try:
        HTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
