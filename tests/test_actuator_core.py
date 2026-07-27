"""Unit tests for the Maestro wire-protocol encoding (actuator_core).

A wrong Pololu frame drives the Mission-3 water cannon / launcher to the wrong
position, so the quarter-microsecond / 7-bit split is pinned with vectors and a
round-trip decode. ROS-free — runs under plain pytest."""
from robotx_2026.api.actuators.actuator_core import (
    MAESTRO_SET_TARGET, maestro_target_bytes)


def _decode(frame: bytes):
    """Inverse of maestro_target_bytes: (channel, target_us)."""
    assert frame[0] == MAESTRO_SET_TARGET
    channel = frame[1]
    quarter_us = frame[2] | (frame[3] << 7)
    return channel, quarter_us // 4


def test_frame_shape_and_command_byte():
    frame = maestro_target_bytes(3, 1500)
    assert len(frame) == 4
    assert frame[0] == 0x84
    assert frame[1] == 3
    # the two payload bytes are 7-bit only (high bit never set)
    assert frame[2] < 0x80 and frame[3] < 0x80


def test_known_vectors():
    # 1500 us -> 6000 quarter-us -> 0x1770 -> low7=112, high7=46
    assert maestro_target_bytes(0, 1500) == bytes([0x84, 0, 112, 46])
    # 2000 us -> 8000 quarter-us -> low7=64, high7=62
    assert maestro_target_bytes(0, 2000) == bytes([0x84, 0, 64, 62])
    # 1000 us -> 4000 quarter-us -> low7=32, high7=31
    assert maestro_target_bytes(0, 1000) == bytes([0x84, 0, 32, 31])


def test_round_trips_across_range_and_channels():
    for channel in (0, 5, 17):
        for us in (500, 1000, 1500, 1900, 2000, 2500):
            ch, back = _decode(maestro_target_bytes(channel, us))
            assert ch == channel and back == us


def test_float_target_is_coerced():
    assert maestro_target_bytes(1, 1500.0) == maestro_target_bytes(1, 1500)
