"""gcs_wire framing over a real socket pair. Standard library only."""
import os
import socket
import sys
import threading
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from crusader_groundstation import gcs_wire  # noqa: E402


class Framing(unittest.TestCase):

    def setUp(self):
        self.a, self.b = socket.socketpair()

    def tearDown(self):
        self.a.close()
        self.b.close()

    def test_round_trip_with_blob(self):
        self.a.sendall(gcs_wire.encode({"t": "cam", "id": 3, "w": 320}, b"\xff\xd8jpeg"))
        header, blob = gcs_wire.read_frame(self.b)
        self.assertEqual(header, {"t": "cam", "id": 3, "w": 320})
        self.assertEqual(blob, b"\xff\xd8jpeg")

    def test_frames_split_across_writes(self):
        frame = gcs_wire.encode({"t": "state", "x": list(range(500))}, b"z" * 70000)

        def dribble():
            for i in range(0, len(frame), 999):
                self.a.sendall(frame[i:i + 999])
        threading.Thread(target=dribble).start()
        header, blob = gcs_wire.read_frame(self.b)
        self.assertEqual(len(header["x"]), 500)
        self.assertEqual(len(blob), 70000)

    def test_close_and_garbage_raise(self):
        self.a.sendall(b"\x7f\xff\xff\xff\x00\x00\x00\x00")
        with self.assertRaises(ConnectionError):
            gcs_wire.read_frame(self.b)
        a, b = socket.socketpair()
        a.sendall(gcs_wire.encode({"t": "x"})[:6])
        a.close()
        with self.assertRaises(ConnectionError):
            gcs_wire.read_frame(b)
        b.close()

    def test_header_must_be_typed_json(self):
        body = b'{"no_type": 1}'
        self.a.sendall(len(body).to_bytes(4, "big") + (0).to_bytes(4, "big") + body)
        with self.assertRaises(ConnectionError):
            gcs_wire.read_frame(self.b)

    def test_nan_is_refused_and_finite_blanks_it(self):
        with self.assertRaises(ValueError):
            gcs_wire.encode({"t": "state", "v": float("nan")})
        self.assertIsNone(gcs_wire.finite(float("nan")))
        self.assertIsNone(gcs_wire.finite(float("inf")))
        self.assertIsNone(gcs_wire.finite(None))
        self.assertEqual(gcs_wire.finite(1.23456, 2), 1.23)


if __name__ == "__main__":
    unittest.main()
