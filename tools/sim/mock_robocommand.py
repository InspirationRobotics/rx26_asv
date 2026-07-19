#!/usr/bin/env python3
"""Mock RoboCommand server — drives Mission-4 testing without the real link,
the boat armed, or water. The comms listener and task-stack logic are exercised
byte-identically to competition conditions.

Wire framing: 4-byte big-endian length prefix, then one serialized
robocommand.Envelope (see ../proto/robocommand.proto). If the generated proto
classes aren't importable, falls back to JSON payloads with the same framing so
early integration can proceed — the listener must treat a parse failure as a
malformed-message event (which is itself a test case).

Usage:
    mock_robocommand.py --port 9500 --script scenario.jsonl     # scripted
    mock_robocommand.py --port 9500                             # interactive REPL

Script file: one JSON object per line:
    {"t": 5.0,  "type": "assistance_request", "request_id": "A1",
     "domain": "surface", "latitude": 32.70, "longitude": -117.25}
    {"t": 40.0, "type": "clearance", "request_id": "A1"}
    {"t": 60.0, "type": "keep_out_zone", "zone_id": "K1",
     "polygon": [[32.701,-117.251],[32.702,-117.251],[32.702,-117.250]]}
    {"t": 90.0, "type": "moving_object", "object_id": "M1", "latitude": 32.7,
     "longitude": -117.25, "heading_deg": 90, "speed_mps": 1.5,
     "system_type": "usv"}
    {"t": 120.0,"type": "all_clear", "ref_id": "K1"}
`t` is seconds from client connect. Inbound VehicleStatus frames are logged with
receive timestamps so ack-timing compliance can be scored from the log alone.
"""
import argparse
import json
import socket
import struct
import sys
import threading
import time

try:
    import robocommand_pb2 as pb  # protoc output from ../proto/robocommand.proto
    HAVE_PROTO = True
except ImportError:
    pb = None
    HAVE_PROTO = False


def frame(payload: bytes) -> bytes:
    return struct.pack(">I", len(payload)) + payload


def encode_event(ev: dict) -> bytes:
    """dict -> wire bytes (proto Envelope if available, else JSON)."""
    if not HAVE_PROTO:
        return frame(json.dumps(ev).encode())
    env = pb.Envelope()
    t = ev["type"]
    if t == "assistance_request":
        m = env.assistance_request
        m.request_id, m.domain = ev["request_id"], ev.get("domain", "surface")
        m.latitude, m.longitude = ev["latitude"], ev["longitude"]
    elif t == "keep_out_zone":
        m = env.keep_out_zone
        m.zone_id = ev["zone_id"]
        for lat, lon in ev["polygon"]:
            p = m.polygon.add()
            p.latitude, p.longitude = lat, lon
    elif t == "moving_object":
        m = env.moving_object
        m.object_id = ev["object_id"]
        m.latitude, m.longitude = ev["latitude"], ev["longitude"]
        m.heading_deg, m.speed_mps = ev["heading_deg"], ev["speed_mps"]
        m.system_type = ev.get("system_type", "usv")
    elif t == "all_clear":
        env.all_clear.ref_id = ev["ref_id"]
    elif t == "clearance":
        env.clearance.request_id = ev["request_id"]
    else:
        raise ValueError(f"unknown event type: {t}")
    return frame(env.SerializeToString())


def reader_loop(conn: socket.socket, log):
    """Log inbound VehicleStatus frames with timestamps (ack-compliance scoring)."""
    buf = b""
    while True:
        try:
            chunk = conn.recv(4096)
        except OSError:
            return
        if not chunk:
            return
        buf += chunk
        while len(buf) >= 4:
            n = struct.unpack(">I", buf[:4])[0]
            if len(buf) < 4 + n:
                break
            payload, buf = buf[4:4 + n], buf[4 + n:]
            ts = time.time()
            if HAVE_PROTO:
                env = pb.Envelope()
                try:
                    env.ParseFromString(payload)
                    desc = str(env).replace("\n", " ")
                except Exception as e:
                    desc = f"<malformed: {e}>"
            else:
                try:
                    desc = json.dumps(json.loads(payload))
                except Exception:
                    desc = f"<malformed json: {payload[:60]!r}>"
            log(f"RX {ts:.3f} {desc}")


def serve(port, script_events, logfile):
    logf = open(logfile, "a") if logfile else None

    def log(line):
        print(line, flush=True)
        if logf:
            logf.write(line + "\n")
            logf.flush()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    log(f"mock_robocommand listening on :{port} "
        f"({'protobuf' if HAVE_PROTO else 'JSON-fallback'} framing)")

    while True:
        conn, addr = srv.accept()
        log(f"client connected: {addr}")
        threading.Thread(target=reader_loop, args=(conn, log), daemon=True).start()
        t0 = time.time()
        try:
            if script_events is not None:
                for ev in script_events:
                    delay = t0 + ev["t"] - time.time()
                    if delay > 0:
                        time.sleep(delay)
                    conn.sendall(encode_event(ev))
                    log(f"TX {time.time():.3f} {json.dumps(ev)}")
                log("script complete; holding connection open (ctrl-c to exit)")
                while True:
                    time.sleep(3600)
            else:
                print("REPL — paste one JSON event per line (see --help for shapes):")
                for line in sys.stdin:
                    line = line.strip()
                    if not line:
                        continue
                    ev = json.loads(line)
                    conn.sendall(encode_event(ev))
                    log(f"TX {time.time():.3f} {json.dumps(ev)}")
        except (BrokenPipeError, ConnectionResetError):
            log("client disconnected")
        finally:
            conn.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=9500)
    ap.add_argument("--script", help="JSONL scenario file (see docstring)")
    ap.add_argument("--log", default="mock_robocommand.log")
    args = ap.parse_args()

    events = None
    if args.script:
        with open(args.script) as f:
            events = sorted((json.loads(l) for l in f if l.strip()),
                            key=lambda e: e["t"])
    serve(args.port, events, args.log)


if __name__ == "__main__":
    main()
