#!/usr/bin/env python3
"""
record_oakd_lr.py -- collect CV training data from an OAK-D LR (3x OV9782).

Design notes (why it works the way it does):

  * The SENSOR runs at your deployment frame rate (--sensor-fps, default 15),
    NOT at the save rate. Running the sensor at 2 FPS would let auto-exposure
    stretch to ~500 ms, so every frame would have motion blur and rolling-
    shutter skew your model will never see at inference time. Set --sensor-fps
    to whatever the deployed pipeline will actually run at.

  * Frames are then SAMPLED down to --save-fps (default 2) using device
    timestamps, so you get temporally diverse training images instead of 15
    near-duplicates per second.

  * All three cameras are software-synced by device timestamp into triplets.
    The center camera (CAM_A) is the clock; for each kept center frame the
    nearest-in-time frame from CAM_B and CAM_C is chosen. The worst-case skew
    inside each triplet is logged to frames.csv so you can throw out bad rows.

Outputs, per run, in <out-dir>/<session-name>/:
    CAM_A.avi, CAM_B.avi, CAM_C.avi   one video per camera, frame N of each
                                      file belongs to the same triplet
    frames.csv                        provenance: index, seq nums, device
                                      timestamps, inter-camera skew
    meta.json                         exact capture config + device info

Examples:
    # 3 cameras, sensor at 15 FPS, keep 2 FPS, run until Ctrl+C
    python3 record_oakd_lr.py

    # match a 30 FPS deployment, 20 minutes, full 1920x1200, lossless-ish
    python3 record_oakd_lr.py --sensor-fps 30 --duration 1200 --isp-scale 1 1

    # cap exposure at 8 ms so nothing in the dataset is motion blurred
    python3 record_oakd_lr.py --max-exposure-us 8000

    # exposure compensation defaults to -3 EV; override or disable it
    python3 record_oakd_lr.py --ae-compensation -1
    python3 record_oakd_lr.py --ae-compensation 0

    python3 record_oakd_lr.py --list-cameras
"""

from __future__ import annotations

import argparse
import csv
import json
import signal
import sys
import time
from collections import deque
from datetime import datetime, timedelta
from pathlib import Path

import cv2
import depthai as dai

# ---------------------------------------------------------------------------
# OAK-D LR camera layout. Baselines: B<->A 5 cm, A<->C 10 cm, B<->C 15 cm.
# ---------------------------------------------------------------------------
SOCKETS = {
    "CAM_A": (dai.CameraBoardSocket.CAM_A, "center"),
    "CAM_B": (dai.CameraBoardSocket.CAM_B, "left"),
    "CAM_C": (dai.CameraBoardSocket.CAM_C, "right"),
}

RESOLUTIONS = {
    "1200p": dai.ColorCameraProperties.SensorResolution.THE_1200_P,  # 1920x1200
    "800p": dai.ColorCameraProperties.SensorResolution.THE_800_P,  # 1280x800
    "720p": dai.ColorCameraProperties.SensorResolution.THE_720_P,
}

CONTAINERS = {"MJPG": ".avi", "XVID": ".avi", "mp4v": ".mp4", "avc1": ".mp4"}

_stop = False


def _handle_sigint(signum, frame):
    global _stop
    if _stop:  # second Ctrl+C: give up immediately
        sys.exit(130)
    _stop = True
    print("\n[record] stopping, flushing writers...", flush=True)


def frame_ts(pkt) -> float:
    """Device-clock timestamp in seconds. Same clock domain for all 3 cameras,
    which is what makes the triplet sync meaningful."""
    try:
        ts = pkt.getTimestampDevice()
    except Exception:
        ts = pkt.getTimestamp()
    if isinstance(ts, timedelta):
        return ts.total_seconds()
    return float(ts)


# ---------------------------------------------------------------------------
# Pipeline
# ---------------------------------------------------------------------------
def build_pipeline(args, cam_names):
    pipeline = dai.Pipeline()
    for name in cam_names:
        socket, _ = SOCKETS[name]
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(socket)
        cam.setResolution(RESOLUTIONS[args.resolution])
        cam.setIspScale(args.isp_scale[0], args.isp_scale[1])
        cam.setFps(args.sensor_fps)
        cam.setInterleaved(False)

        if args.ae_compensation:
            # Biases the AE target darker (negative) or brighter (positive).
            # -3 pulls exposure down ~ a stop and a half: shorter shutter, less
            # motion blur, and highlights stop clipping -- but shadows get
            # noisier, so check a test clip before committing to a long run.
            try:
                cam.initialControl.setAutoExposureCompensation(
                    args.ae_compensation)
            except Exception as e:
                print(f"[record] warn: --ae-compensation unsupported here ({e})")

        if args.max_exposure_us:
            # Bounds auto-exposure so the dataset never contains frames blurrier
            # than what the deployed camera can produce.
            try:
                cam.initialControl.setAutoExposureLimit(args.max_exposure_us)
            except Exception as e:  # older depthai
                print(f"[record] warn: --max-exposure-us unsupported here ({e})")
        if args.manual_exposure:
            # Manual exposure disables AE entirely, so compensation/limit are
            # moot -- warned about once in main().
            iso, exp_us = args.manual_exposure
            cam.initialControl.setManualExposure(int(exp_us), int(iso))

        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName(name)
        # Only isp frames leave the device; no encoder, so pixels are untouched.
        cam.isp.link(xout.input)
    return pipeline


def make_writer(path: Path, codec: str, fps: float, size, quality: int):
    fourcc = cv2.VideoWriter_fourcc(*codec)
    writer = cv2.VideoWriter(str(path), fourcc, fps, size)
    if not writer.isOpened():
        raise RuntimeError(
            f"OpenCV could not open a writer for {path} with codec {codec!r}. "
            f"Try --codec MJPG (most portable) or install a build of OpenCV "
            f"with the needed backend."
        )
    try:  # honored by the MJPG encoder; harmless elsewhere
        writer.set(cv2.VIDEOWRITER_PROP_QUALITY, quality)
    except Exception:
        pass
    return writer


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> int:
    args = parse_args()

    if args.list_cameras:
        with dai.Device() as dev:
            print(f"Device: {dev.getDeviceName()}  USB: {dev.getUsbSpeed().name}")
            for f in dev.getConnectedCameraFeatures():
                print(f"  {f.socket.name:6s} {f.sensorName:10s} "
                      f"{f.width}x{f.height}")
        return 0

    cam_names = args.cameras
    if args.save_fps > args.sensor_fps:
        print(f"[record] error: --save-fps ({args.save_fps}) exceeds --sensor-fps "
              f"({args.sensor_fps}); the sensor cannot supply that many frames.")
        return 2

    session = args.session_name or datetime.now().strftime("session_%Y%m%d_%H%M%S")
    out_dir = Path(args.out_dir).expanduser() / session
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.manual_exposure and (args.ae_compensation or args.max_exposure_us):
        print("[record] note: --manual-exposure turns auto-exposure off, so "
              "--ae-compensation/--max-exposure-us have no effect.")

    signal.signal(signal.SIGINT, _handle_sigint)
    signal.signal(signal.SIGTERM, _handle_sigint)

    pipeline = build_pipeline(args, cam_names)
    interval = 1.0 / args.save_fps

    with dai.Device(pipeline) as device:
        usb = device.getUsbSpeed().name
        print(f"[record] device: {device.getDeviceName()}  link: {usb}")
        if usb not in ("SUPER", "SUPER_PLUS"):
            print(f"[record] WARNING: link is {usb}, not USB3. Three 1200p "
                  f"streams will drop frames. Check the cable/port.")

        present = {f.socket.name for f in device.getConnectedCameraFeatures()}
        missing = [n for n in cam_names if n not in present]
        if missing:
            print(f"[record] WARNING: requested {missing} but device reports "
                  f"only {sorted(present)}")

        queues = {n: device.getOutputQueue(n, maxSize=8, blocking=False)
                  for n in cam_names}
        buffers = {n: deque(maxlen=args.buffer_frames) for n in cam_names}

        master = "CAM_A" if "CAM_A" in cam_names else cam_names[0]
        others = [n for n in cam_names if n != master]

        writers: dict[str, cv2.VideoWriter] = {}
        sizes: dict[str, tuple] = {}
        csv_path = out_dir / "frames.csv"
        csv_file = open(csv_path, "w", newline="")
        csv_w = csv.writer(csv_file)
        csv_w.writerow(
            ["index", "wall_clock_iso", "master_ts_s", "max_skew_ms"]
            + [f"{n}_{f}" for n in cam_names for f in ("seq", "ts_s")]
        )

        saved = 0
        seen = 0
        next_ts = None
        t_start = time.monotonic()
        last_emit = t_start
        last_report = t_start

        print(f"[record] sensor {args.sensor_fps} FPS -> saving {args.save_fps} "
              f"FPS  cameras={cam_names}")
        print(f"[record] writing to {out_dir}")
        print("[record] Ctrl+C to stop.")

        try:
            while not _stop:
                if args.duration and time.monotonic() - t_start >= args.duration:
                    print("[record] duration reached.")
                    break

                # ---- drain every queue into its buffer -------------------
                # Bounded per pass: a burst must never replace more than half a
                # buffer before the emit step below gets a chance to run, or we
                # would silently evict frames that still had a triplet pending.
                got_any = False
                budget = max(1, args.buffer_frames // 2)
                for n, q in queues.items():
                    for _ in range(budget):
                        pkt = q.tryGet()
                        if pkt is None:
                            break
                        buffers[n].append(pkt)
                        got_any = True
                        if n == master:
                            seen += 1
                if not got_any:
                    time.sleep(0.002)

                # ---- emit every synced triplet we can form ---------------
                while buffers[master]:
                    m = buffers[master][0]
                    m_ts = frame_ts(m)

                    # A neighbour frame at or after m_ts must exist, otherwise
                    # a closer match could still arrive.
                    if not all(
                        buffers[n] and frame_ts(buffers[n][-1]) >= m_ts
                        for n in others
                    ):
                        break

                    buffers[master].popleft()

                    if next_ts is not None and m_ts < next_ts:
                        continue  # between sample points, drop it

                    picks = {master: m}
                    for n in others:
                        picks[n] = min(buffers[n],
                                       key=lambda p: abs(frame_ts(p) - m_ts))
                    ts_of = {n: frame_ts(p) for n, p in picks.items()}
                    skew_ms = (max(ts_of.values()) - min(ts_of.values())) * 1000.0

                    if args.max_skew_ms and skew_ms > args.max_skew_ms:
                        print(f"[record] skip: triplet skew {skew_ms:.1f} ms "
                              f"> {args.max_skew_ms} ms")
                    else:
                        for n in cam_names:
                            img = picks[n].getCvFrame()
                            if n not in writers:
                                h, w = img.shape[:2]
                                sizes[n] = (w, h)
                                writers[n] = make_writer(
                                    out_dir / f"{n}{CONTAINERS[args.codec]}",
                                    args.codec, args.save_fps, (w, h),
                                    args.quality)
                            writers[n].write(img)
                        row = [saved, datetime.now().isoformat(timespec="milliseconds"),
                               f"{m_ts:.6f}", f"{skew_ms:.3f}"]
                        for n in cam_names:
                            row += [picks[n].getSequenceNum(), f"{ts_of[n]:.6f}"]
                        csv_w.writerow(row)
                        saved += 1
                        last_emit = time.monotonic()

                    # Advance on an ideal grid so the AVERAGE rate is exactly
                    # --save-fps. Anchoring to m_ts instead would snap to the
                    # sensor grid and quantize low (15 FPS -> 1.875, not 2.0).
                    # If we fall behind (stall, dropped frames), resync rather
                    # than burst-saving to catch up.
                    next_ts = m_ts + interval if next_ts is None \
                        else next_ts + interval
                    if next_ts <= m_ts:
                        next_ts = m_ts + interval

                    for n in others:  # older neighbours can never be reused
                        while buffers[n] and frame_ts(buffers[n][0]) < ts_of[n]:
                            buffers[n].popleft()

                now = time.monotonic()
                if now - last_report >= 5.0:
                    el = now - t_start
                    print(f"[record] {saved} frames saved | {seen} seen from "
                          f"{master} ({seen / el:.1f} FPS in) | {el:.0f}s")
                    last_report = now
                if now - last_emit > 5.0 and saved:
                    print("[record] WARNING: no synced triplet in 5 s -- a "
                          "camera may have stalled.")
                    last_emit = now
        finally:
            for w in writers.values():
                w.release()
            csv_file.close()
            elapsed = time.monotonic() - t_start
            meta = {
                "session": session,
                "created": datetime.now().astimezone().isoformat(),
                "cameras": cam_names,
                "sensor_fps": args.sensor_fps,
                "save_fps": args.save_fps,
                "resolution": args.resolution,
                "isp_scale": list(args.isp_scale),
                "frame_size": {n: list(s) for n, s in sizes.items()},
                "codec": args.codec,
                "jpeg_quality": args.quality,
                "ae_compensation": args.ae_compensation,
                "max_exposure_us": args.max_exposure_us,
                "manual_exposure": args.manual_exposure,
                "max_skew_ms": args.max_skew_ms,
                "master_camera": master,
                "frames_saved": saved,
                "master_frames_seen": seen,
                "elapsed_s": round(elapsed, 2),
                "effective_save_fps": round(saved / elapsed, 3) if elapsed else 0,
                "depthai_version": dai.__version__,
                "opencv_version": cv2.__version__,
            }
            (out_dir / "meta.json").write_text(json.dumps(meta, indent=2))
            print(f"[record] done: {saved} frames per camera in {elapsed:.1f}s "
                  f"({meta['effective_save_fps']} FPS effective)")
            print(f"[record] {out_dir}")
    return 0


def parse_args():
    p = argparse.ArgumentParser(
        description="Record synchronized low-rate training data from an OAK-D LR.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--cameras", nargs="+", default=["CAM_A", "CAM_B", "CAM_C"],
                   choices=list(SOCKETS), metavar="CAM",
                   help="sockets to record (default: all three)")
    p.add_argument("--sensor-fps", type=float, default=15.0,
                   help="sensor frame rate -- set this to your DEPLOYMENT frame "
                        "rate so exposure and motion blur match (default: 15)")
    p.add_argument("--save-fps", type=float, default=2.0,
                   help="frames actually written to disk (default: 2)")
    p.add_argument("--duration", type=float, default=0,
                   help="stop after N seconds (default: run until Ctrl+C)")
    p.add_argument("--out-dir", default="recordings")
    p.add_argument("--session-name", default=None,
                   help="subfolder name (default: session_<timestamp>)")
    p.add_argument("--resolution", default="1200p", choices=list(RESOLUTIONS))
    p.add_argument("--isp-scale", nargs=2, type=int, default=[1, 2],
                   metavar=("NUM", "DEN"),
                   help="downscale the ISP output, e.g. 1 2 -> 960x600 "
                        "(default), 1 1 -> full 1920x1200")
    p.add_argument("--codec", default="MJPG", choices=list(CONTAINERS),
                   help="MJPG (default) codes each frame independently, so "
                        "extracting training images later is lossless-ish and "
                        "frame-exact; mp4v is smaller but temporally compressed")
    p.add_argument("--quality", type=int, default=95,
                   help="MJPG quality 0-100 (default: 95)")
    p.add_argument("--ae-compensation", type=int, default=-3,
                   choices=range(-9, 10), metavar="[-9..9]",
                   help="auto-exposure compensation in EV steps; negative is "
                        "darker/shorter shutter (default: -3, 0 disables)")
    p.add_argument("--max-exposure-us", type=int, default=0,
                   help="cap auto-exposure, e.g. 8000 for 8 ms, to keep motion "
                        "blur out of the dataset (default: uncapped)")
    p.add_argument("--manual-exposure", nargs=2, type=int, default=None,
                   metavar=("ISO", "EXP_US"),
                   help="lock exposure entirely, e.g. --manual-exposure 400 6000")
    p.add_argument("--max-skew-ms", type=float, default=0,
                   help="drop triplets whose inter-camera timestamp spread "
                        "exceeds this (default: 0 = keep all, just log skew)")
    p.add_argument("--buffer-frames", type=int, default=16,
                   help="per-camera sync buffer depth (default: 16)")
    p.add_argument("--list-cameras", action="store_true",
                   help="print the device's cameras and exit")
    return p.parse_args()


if __name__ == "__main__":
    sys.exit(main())