#!/usr/bin/env python3
"""collect_footage — buoy training-data capture for the G2 model retrain.

Retraining the buoy detector on Crusader's OWN buoys is a prerequisite for
trusting objective-1 metrics (CLAUDE.md), not a nice-to-have. This tool captures
timestamped RGB frames from the OAK-D LR plus a JSONL sidecar with GPS position
(read from MAVProxy's rebroadcast, UDP only — never the Pixhawk serial), so
frames can later be bucketed by range/lighting/geometry for labeling.

Run inside the asv container during bench/water days:

    python3 collect_footage.py --out /root/footage/$(date +%Y%m%d) \
        --interval 0.5 --mav udp:127.0.0.1:14551

Labeling conventions follow the RX24 practice (one dir per session, sidecar per
session); classes are the 5 LED buoy states (OFF, FLASH_RED, FLASH_GREEN,
FLASH_BLUE, SOLID_BLUE) + dock features.
"""
import argparse
import json
import os
import threading
import time


def gps_thread(endpoint, state, stop):
    try:
        from pymavlink import mavutil
    except ImportError:
        print("pymavlink missing — frames will be captured without GPS tags")
        return
    if not (endpoint.startswith("udp") or endpoint.startswith("tcp")):
        raise ValueError("GPS endpoint must be udp/tcp (MAVProxy rebroadcast only)")
    conn = mavutil.mavlink_connection(endpoint)
    conn.wait_heartbeat()
    while not stop.is_set():
        msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg:
            state["fix"] = {"lat": msg.lat / 1e7, "lon": msg.lon / 1e7,
                            "hdg": msg.hdg / 100.0 if msg.hdg != 65535 else None}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True)
    ap.add_argument("--interval", type=float, default=0.5, help="s between frames")
    ap.add_argument("--mav", default="udp:127.0.0.1:14551")
    ap.add_argument("--max-frames", type=int, default=0, help="0 = until ctrl-c")
    args = ap.parse_args()

    import cv2
    import depthai as dai
    from rx26_asv.api.perception.oakd_guard import assert_usb_super

    os.makedirs(args.out, exist_ok=True)
    print(f"USB: {assert_usb_super()}")

    state, stop = {}, threading.Event()
    t = threading.Thread(target=gps_thread, args=(args.mav, state, stop), daemon=True)
    t.start()

    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("rgb")
    cam.video.link(xout.input)

    sidecar = open(os.path.join(args.out, "frames.jsonl"), "a")
    n = 0
    try:
        with dai.Device(pipeline) as dev:
            q = dev.getOutputQueue("rgb", maxSize=2, blocking=False)
            last = 0.0
            while args.max_frames == 0 or n < args.max_frames:
                frame = q.get().getCvFrame()
                now = time.time()
                if now - last < args.interval:
                    continue
                last = now
                name = f"frame_{int(now * 1000)}.jpg"
                cv2.imwrite(os.path.join(args.out, name), frame)
                sidecar.write(json.dumps({"file": name, "t": now,
                                          "gps": state.get("fix")}) + "\n")
                sidecar.flush()
                n += 1
                if n % 50 == 0:
                    print(f"{n} frames  (gps: {'yes' if state.get('fix') else 'NO'})")
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        sidecar.close()
        print(f"done: {n} frames in {args.out}")


if __name__ == "__main__":
    main()
