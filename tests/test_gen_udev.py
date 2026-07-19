"""Tests for the boat-config -> udev-rules generator, using the REAL device
paths from both boats' configs (crusader.json / barco.json) — including the
JetPack controller-rename case the generator must be immune to."""
import importlib.util
import sys
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    "gen_udev_rules",
    Path(__file__).parent.parent / "tools" / "udev" / "gen_udev_rules.py")
gen = importlib.util.module_from_spec(spec)
sys.modules["gen_udev_rules"] = gen
spec.loader.exec_module(gen)


# real paths, verbatim from the two boat configs
CRUSADER_TEENSY = ("/devices/platform/bus@0/3610000.usb/usb1/1-2/1-2.2/"
                   "1-2.2:1.0/tty/ttyACM0")
CRUSADER_GPS = ("/devices/platform/bus@0/3610000.usb/usb1/1-2/1-2.1/"
                "1-2.1:1.0/ttyUSB0/tty/ttyUSB0")
BARCO_TEENSY = "/devices/platform/3610000.xhci/usb1/1-2/1-2.2/1-2.2:1.0/tty/ttyACM0"
BARCO_WEBCAM = ("/devices/platform/3610000.xhci/usb1/1-2/1-2.2/1-2.2.3/"
                "1-2.2.3:1.0/video4linux/video0")


def test_parse_tty_interface_chain():
    info = gen.parse_devpath(CRUSADER_TEENSY)
    assert info == {"kernels": "1-2.2:1.0", "subsystem": "tty"}


def test_parse_ttyusb_nested_path():
    # ttyUSB paths nest an extra level (ttyUSB0/tty/ttyUSB0) — chain must still
    # resolve to the interface, not the tty node
    info = gen.parse_devpath(CRUSADER_GPS)
    assert info == {"kernels": "1-2.1:1.0", "subsystem": "tty"}


def test_controller_rename_immunity():
    # SAME physical port, different JetPack controller prefix (xhci vs bus@0):
    # the extracted match key must be identical — this is the whole point
    a = gen.parse_devpath(CRUSADER_TEENSY)
    b = gen.parse_devpath(BARCO_TEENSY)
    assert a == b


def test_parse_video4linux():
    info = gen.parse_devpath(BARCO_WEBCAM)
    assert info == {"kernels": "1-2.2.3:1.0", "subsystem": "video4linux"}


def test_garbage_path_fails_loudly():
    with pytest.raises(gen.DevpathError):
        gen.parse_devpath("/devices/platform/serial8250/tty/ttyS0")  # not USB
    with pytest.raises(gen.DevpathError):
        gen.parse_devpath("/devices/platform/bus@0/usb1/1-2/1-2.2:1.0/hidraw/hidraw0")


def test_generate_rules_and_resolved_config():
    config = {
        "teensy": {"port": CRUSADER_TEENSY, "rate": 115200},
        "gps": {"port": CRUSADER_GPS, "rate": 115200},
        "oakd_lr": {"id": "18443010012C48F500"},
        "harbor_pos": {"one_blast": [1, 2]},
    }
    rules, resolved = gen.generate(config, source_name="test.json")
    assert 'KERNELS=="1-2.2:1.0", SYMLINK+="crsd-teensy"' in rules
    assert 'KERNELS=="1-2.1:1.0", SYMLINK+="crsd-gps"' in rules
    assert rules.isascii()                       # udev files must stay ASCII-safe
    # resolved config: ports rewritten to symlinks, rates kept
    assert resolved["teensy"]["port"] == "/dev/crsd-teensy"
    assert resolved["teensy"]["rate"] == 115200
    assert resolved["teensy"]["port_chain"] == "1-2.2:1.0"
    # non-port entries pass through untouched
    assert resolved["oakd_lr"] == {"id": "18443010012C48F500"}
    assert resolved["harbor_pos"] == {"one_blast": [1, 2]}


def test_symlink_name_sanitization():
    assert gen.symlink_name("ball_launcher") == "crsd-ball-launcher"
