"""oakd_guard — OAK-D LR USB3 assertion (fail loudly, never degrade silently).

The OAK-D LR must enumerate at USB3 ("SUPER"); at "HIGH" (USB2) stereo throughput
collapses and the perception pipeline silently underperforms — which corrupts
objective-1 metrics rather than failing an episode. So the camera node MUST call
assert_usb_super() before publishing a single frame, and preflight runs the same
check from the CLI:

    python3 -m rx26_asv.api.perception.oakd_guard
"""
import sys


class UsbSpeedError(RuntimeError):
    pass


def get_usb_speed() -> str:
    import depthai as dai
    with dai.Device() as dev:
        return str(dev.getUsbSpeed())


def assert_usb_super() -> str:
    """Returns the speed string on success; raises UsbSpeedError otherwise."""
    speed = get_usb_speed()
    if "SUPER" not in speed:
        raise UsbSpeedError(
            f"OAK-D enumerated at {speed}, need SUPER (USB3). Reseat/replace the "
            "cable or port — do NOT run perception at USB2 speeds.")
    return speed


def main():
    try:
        print(f"OAK-D USB speed: {assert_usb_super()} — OK")
    except ImportError:
        print("depthai not installed (run inside the crusader container)")
        sys.exit(2)
    except UsbSpeedError as e:
        print(f"FAIL: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"FAIL: device error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
