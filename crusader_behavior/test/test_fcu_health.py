"""fcu_health and the GUIDED setpoint check: blanks for stale streams, never guesses."""
import math
import unittest

from crusader_behavior import fcu_health as fh
from crusader_behavior.guided_hs_core import setpoint_error


def sys_status(mv=12600, pct=80, present=fh.PREARM_CHECK, health=fh.PREARM_CHECK):
    return {"voltage_mv": mv, "remaining_pct": pct, "present": present, "health": health}


class FcuHealthTest(unittest.TestCase):

    def test_everything_stale_is_blank(self):
        h = fh.build(None, None, None, None, None, 7, 1200)
        self.assertTrue(math.isnan(h["battery_v"]) and math.isnan(h["battery_pct"]))
        self.assertTrue(math.isnan(h["gps_hdop"]))
        self.assertEqual((h["gps_fix_type"], h["gps_sats"], h["ekf_flags"]), (0, 0, 0))
        for k in ("ekf_ok", "prearm_ok", "estop_active"):
            self.assertEqual(h[k], fh.TRI_UNKNOWN, k)

    def test_battery_from_sys_status_then_battery_status(self):
        self.assertEqual(fh.battery(sys_status(), None), (12.6, 80.0))
        volts, pct = fh.battery(sys_status(mv=65535, pct=-1), {"voltage_mv": 12000, "remaining_pct": 50})
        self.assertEqual((volts, pct), (12.0, 50.0))

    def test_prearm_needs_the_bit_present(self):
        self.assertEqual(fh.prearm(sys_status()), fh.TRI_YES)
        self.assertEqual(fh.prearm(sys_status(health=0)), fh.TRI_NO)
        self.assertEqual(fh.prearm(sys_status(present=0)), fh.TRI_UNKNOWN)

    def test_ekf(self):
        self.assertEqual(fh.ekf_ok(1 | 2 | 16 | 8), fh.TRI_YES)
        self.assertEqual(fh.ekf_ok(1 | 2), fh.TRI_NO)
        self.assertEqual(fh.ekf_ok(1 | 2 | 16 | 128), fh.TRI_NO)

    def test_gps_unknown_markers(self):
        self.assertEqual(fh.gps({"fix_type": 6, "sats": 18, "eph": 70}), (6, 18, 0.7))
        fix, sats, hdop = fh.gps({"fix_type": 0, "sats": 255, "eph": 65535})
        self.assertEqual((fix, sats), (0, 0))
        self.assertTrue(math.isnan(hdop))

    def test_estop_and_rc_loss(self):
        rc = [1500] * 18
        self.assertEqual(fh.estop(rc, 7, 1200), fh.TRI_NO)
        rc[6] = 0                                   # a lost link reads 0
        self.assertEqual(fh.estop(rc, 7, 1200), fh.TRI_YES)
        self.assertEqual(fh.estop([1500] * 4, 7, 1200), fh.TRI_UNKNOWN)


class SetpointCheckTest(unittest.TestCase):

    def test_refuses_what_is_not_a_place(self):
        self.assertEqual(setpoint_error(32.92, -117.03), "")
        for lat, lon in ((float("nan"), 1.0), (1.0, float("inf")), (0.0, 0.0), (91.0, 0.0)):
            self.assertNotEqual(setpoint_error(lat, lon), "", (lat, lon))


if __name__ == "__main__":
    unittest.main()
