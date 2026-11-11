"""recorder.FrameWriter off-boat: frames are saved at the rate asked for.

    python -m pytest crusader_groundstation/test/test_recorder.py

Needs nothing outside the standard library (and pytest to run it that way).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from crusader_groundstation.recorder import FrameWriter  # noqa: E402


def feed(writer, times):
    for t in times:
        writer.put(b"jpeg", now=t)
    return writer.count


def camera(fps, start, stop):
    n = int(round((stop - start) * fps))
    return [start + i / fps for i in range(n)]


def test_saves_the_asked_rate_from_a_faster_camera(tmp_path):
    # The OAK-D at ~4.7 fps, recorded at 1 Hz for 30 s: one frame per second, not 26.
    w = FrameWriter(str(tmp_path / "cam"), 1.0)
    assert feed(w, camera(4.7, 1000.0, 1030.0)) == 30
    assert len(os.listdir(tmp_path / "cam")) == 30


def test_three_hz_is_three_hz(tmp_path):
    w = FrameWriter(str(tmp_path / "cam"), 3.0)
    assert feed(w, camera(10.3, 50.0, 110.0)) in (179, 180, 181)


def test_never_more_than_the_camera_sends(tmp_path):
    # A 0.5 fps camera recorded at 1 Hz: every frame, no more.
    w = FrameWriter(str(tmp_path / "cam"), 1.0)
    assert feed(w, camera(0.5, 0.0, 20.0)) == 10


def test_no_burst_after_a_stall(tmp_path):
    # 5 fps, a 6 s gap, then 5 fps again: the first frame back is saved and the
    # next save waits a full period instead of catching up on the gap.
    w = FrameWriter(str(tmp_path / "cam"), 1.0)
    before = camera(5.0, 0.0, 3.0)
    after = camera(5.0, 9.0, 9.9)
    assert feed(w, before) == 3
    assert feed(w, after) == 4                          # only 9.0 saved in 9.0-9.8
