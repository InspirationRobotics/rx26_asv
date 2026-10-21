"""fcu_health: the FcuHealth fields from the latest fresh MAVLink samples. No ROS, no pymavlink.

Every input is the cached sample or None when its stream is stale; a stale stream is a
blank (NaN, 0 or TRI_UNKNOWN), never its last value.
"""
import math

TRI_UNKNOWN, TRI_NO, TRI_YES = 0, 1, 2

PREARM_CHECK = 0x10000000          # MAV_SYS_STATUS_PREARM_CHECK
EKF_NEEDED = 1 | 2 | 16            # attitude, horizontal velocity, absolute horizontal position
EKF_BAD = 128 | 1024 | 32768       # constant-position mode, uninitialised, GPS glitch

NAN = float("nan")


def tri(value):
    return TRI_UNKNOWN if value is None else (TRI_YES if value else TRI_NO)


def battery(sys_status, battery_status):
    """(volts, percent) from SYS_STATUS (voltage mV, remaining %), else BATTERY_STATUS."""
    for sample in (sys_status, battery_status):
        if sample is None:
            continue
        mv, pct = sample["voltage_mv"], sample["remaining_pct"]
        volts = mv / 1000.0 if 0 < mv < 65535 else NAN
        percent = float(pct) if 0 <= pct <= 100 else NAN
        if not (math.isnan(volts) and math.isnan(percent)):
            return volts, percent
    return NAN, NAN


def prearm(sys_status):
    if sys_status is None or not sys_status["present"] & PREARM_CHECK:
        return TRI_UNKNOWN
    return tri(sys_status["health"] & PREARM_CHECK)


def ekf_ok(flags):
    if flags is None:
        return TRI_UNKNOWN
    return tri((flags & EKF_NEEDED) == EKF_NEEDED and not flags & EKF_BAD)


def gps(gps_raw):
    """(fix_type, sats, hdop); sats 255 and eph 65535 are MAVLink's 'unknown'."""
    if gps_raw is None:
        return 0, 0, NAN
    sats = gps_raw["sats"] if gps_raw["sats"] != 255 else 0
    hdop = gps_raw["eph"] / 100.0 if gps_raw["eph"] != 65535 else NAN
    return gps_raw["fix_type"], sats, hdop


def estop(rc, channel, threshold):
    """The e-stop switch below `threshold` (a lost RC link reads 0, so it counts too)."""
    if rc is None or len(rc) < channel:
        return TRI_UNKNOWN
    return tri(rc[channel - 1] < threshold)


def build(sys_status, battery_status, gps_raw, ekf_flags, rc, estop_channel, estop_threshold):
    """FcuHealth's numeric fields as a dict (the text fields are the bridge's)."""
    volts, percent = battery(sys_status, battery_status)
    fix, sats, hdop = gps(gps_raw)
    return {
        "battery_v": volts, "battery_pct": percent,
        "gps_fix_type": fix, "gps_sats": sats, "gps_hdop": hdop,
        "ekf_ok": ekf_ok(ekf_flags), "ekf_flags": ekf_flags or 0,
        "prearm_ok": prearm(sys_status),
        "estop_active": estop(rc, estop_channel, estop_threshold),
    }
