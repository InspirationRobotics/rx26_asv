"""gcs_wire — the framing between gcs_bridge (the boat) and gcs_app (the laptop).

    frame = u32 header_len | u32 blob_len | UTF-8 JSON header | blob     (big-endian)

The header is a JSON object whose "t" names the frame; the blob carries a JPEG or packed
points, never JSON. Standard library only: the laptop runs it with no ROS.

  boat -> laptop  state (2 Hz), targets, plan, path, nodes, tree (on change), log, radio,
                  ocs (as they happen), result {id, ok, msg, data}, cam {w, h, stamp} + JPEG,
                  lidar {n, scale} + int16 x,y pairs, file {id, seq, last} + bytes
  laptop -> boat  req {id, op, ...}
"""
import json
import socket
import struct

PORT = 14559
_HEAD = struct.Struct(">II")
MAX_HEADER = 1 << 20
MAX_BLOB = 16 << 20


def encode(header, blob=b""):
    head = json.dumps(header, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(head) > MAX_HEADER or len(blob) > MAX_BLOB:
        raise ValueError(f"frame {header.get('t')!r} too large: {len(head)} + {len(blob)}")
    return _HEAD.pack(len(head), len(blob)) + head + bytes(blob)


def _exactly(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(min(n - len(buf), 1 << 16))
        if not chunk:
            raise ConnectionError("closed by the other end")
        buf += chunk
    return bytes(buf)


def read_frame(sock):
    """(header dict, blob) from a blocking socket. Raises ConnectionError on close or on a
    frame that cannot be ours, after which the connection is useless."""
    head_len, blob_len = _HEAD.unpack(_exactly(sock, _HEAD.size))
    if head_len > MAX_HEADER or blob_len > MAX_BLOB:
        raise ConnectionError(f"frame of {head_len} + {blob_len} bytes: not gcs_wire")
    try:
        header = json.loads(_exactly(sock, head_len).decode("utf-8"))
    except ValueError as e:
        raise ConnectionError(f"undecodable frame header: {e}") from e
    if not isinstance(header, dict) or "t" not in header:
        raise ConnectionError("frame header without a type")
    return header, _exactly(sock, blob_len) if blob_len else b""


def finite(v, digits=None):
    """A float the header can carry: None for NaN, inf or None (blanks over guesses)."""
    if v is None:
        return None
    v = float(v)
    if v != v or v in (float("inf"), float("-inf")):
        return None
    return round(v, digits) if digits is not None else v


def keepalive(sock):
    """TCP keepalive on, so a laptop that vanished off the WiFi is noticed in seconds."""
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    for name, value in (("TCP_KEEPIDLE", 5), ("TCP_KEEPINTVL", 2), ("TCP_KEEPCNT", 3)):
        if hasattr(socket, name):
            sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, name), value)
