"""pump_core: every refusal, the burst it sends, the watchdog.

No ROS, no pymavlink:   python crusader_fcu/test/test_pump_core.py
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from crusader_fcu import pump_core as pc   # noqa: E402

P = pc.PumpParams(servo_channel=10, rc_channel=10, on_pwm=2000, off_pwm=1000,
                  max_burst_s=1.0, min_gap_s=1.0, allow_disarmed=False,
                  estop_channel=7, estop_threshold=1200)


def rc(ch7=1498, ch10=988, ch9=1498):
    r = [1500] * 18
    r[6], r[8], r[9] = ch7, ch9, ch10
    return r


def ok_inputs(now=100.0, **kw):
    d = dict(now=now, rc=rc(), armed=True, output_pwm=1000)
    d.update(kw)
    return pc.PumpInputs(**d)


class TestRefusals(unittest.TestCase):

    def setUp(self):
        self.g = pc.PumpGate(P)

    def test_all_clear(self):
        self.assertEqual(self.g.check_burst(0.3, ok_inputs()), (True, "ok"))

    def refused(self, duration=0.3, gate=None, **kw):
        ok, why = (gate or self.g).check_burst(duration, ok_inputs(**kw))
        self.assertFalse(ok)
        return why

    def test_no_pump_path(self):
        g = pc.PumpGate(pc.PumpParams(**dict(P.__dict__, servo_channel=0)))
        self.assertIn("no pump path", self.refused(gate=g))
        self.assertFalse(g.enabled)

    def test_burst_length(self):
        self.assertIn("outside", self.refused(duration=1.5))
        self.assertIn("outside", self.refused(duration=0.0))
        self.assertIn("outside", self.refused(duration=-0.2))

    def test_rc_stale(self):
        self.assertIn("RC not fresh", self.refused(rc=None))

    def test_estop(self):
        self.assertIn("e-stop", self.refused(rc=rc(ch7=994)))

    def test_rc_lost_reads_zero(self):
        self.assertIn("e-stop engaged or RC lost", self.refused(rc=rc(ch7=0)))

    def test_pilot_switch_on(self):
        self.assertIn("pilot's pump switch is ON", self.refused(rc=rc(ch10=2012)))

    def test_disarmed_and_unknown(self):
        self.assertEqual(self.refused(armed=False), "disarmed")
        self.assertIn("unknown", self.refused(armed=None))

    def test_disarmed_allowed_on_the_bench(self):
        g = pc.PumpGate(pc.PumpParams(**dict(P.__dict__, allow_disarmed=True)))
        self.assertTrue(g.check_burst(0.3, ok_inputs(armed=False))[0])

    def test_output_not_reported(self):
        self.assertIn("SERVO_OUTPUT_RAW", self.refused(output_pwm=None))

    def test_output_already_on(self):
        self.assertIn("already ON", self.refused(output_pwm=1990))

    def test_running_and_gap(self):
        self.g.start_burst(0.3, 100.0)
        self.assertIn("still running", self.refused(now=100.1))
        self.assertIn("too soon", self.refused(now=100.9))
        self.assertTrue(self.g.check_burst(0.3, ok_inputs(now=101.4))[0])

    def test_latched_refuses_everything(self):
        self.g.latched = "test"
        self.assertIn("latched off", self.refused())
        self.assertFalse(self.g.enabled)


class TestCommands(unittest.TestCase):

    def test_one_cycle_twice_the_burst(self):
        p = pc.repeat_servo_params(10, 2000, 0.3)
        self.assertEqual(p[0], 10.0)
        self.assertEqual(p[1], 2000.0)
        self.assertEqual(p[2], 1.0)              # ONE cycle: never negative/forever
        self.assertAlmostEqual(p[3], 0.6)         # on for half the cycle
        self.assertEqual(len(p), 7)

    def test_set_servo(self):
        self.assertEqual(pc.set_servo_params(10, 1000), (10.0, 1000.0, 0, 0, 0, 0, 0))

    def test_command_ids(self):
        self.assertEqual(pc.MAV_CMD_DO_SET_SERVO, 183)
        self.assertEqual(pc.MAV_CMD_DO_REPEAT_SERVO, 211)

    def test_is_on_either_polarity(self):
        self.assertTrue(P.is_on(1990))
        self.assertFalse(P.is_on(1010))
        self.assertFalse(P.is_on(0))
        inv = pc.PumpParams(**dict(P.__dict__, on_pwm=1000, off_pwm=2000))
        self.assertTrue(inv.is_on(1010))

    def test_servo_raw(self):
        class M:
            port = 0
            servo10_raw = 1995
        self.assertEqual(pc.servo_raw(M(), 10), 1995)
        self.assertEqual(pc.servo_raw(M(), 11), 0)
        M.port = 1
        self.assertIsNone(pc.servo_raw(M(), 10))

    def test_result_codes_match_the_message(self):
        here = os.path.dirname(os.path.abspath(__file__))
        msg = open(os.path.join(here, "..", "..", "crusader_msgs", "msg",
                                "PumpState.msg")).read()
        for name, val in (("NONE", pc.RESULT_NONE), ("SENT", pc.RESULT_SENT),
                          ("ACCEPTED", pc.RESULT_ACCEPTED),
                          ("REJECTED", pc.RESULT_REJECTED),
                          ("REFUSED", pc.RESULT_REFUSED)):
            self.assertIn(f"RESULT_{name}={val}", msg)

    def test_from_dict_needs_every_key(self):
        d = dict(pump_servo_channel=10, pump_rc_channel=10, pump_on_pwm=2000,
                 pump_off_pwm=1000, pump_max_burst_s=1.0, pump_min_gap_s=1.0,
                 pump_allow_disarmed=False, estop_channel=7, estop_threshold=1200)
        self.assertEqual(pc.PumpParams.from_dict(d).servo_channel, 10)
        del d["pump_off_pwm"]
        with self.assertRaises(KeyError):
            pc.PumpParams.from_dict(d)


class TestWatchdog(unittest.TestCase):

    def test_quiet_when_off(self):
        g = pc.PumpGate(P)
        self.assertEqual(g.watchdog(ok_inputs(rc=rc(ch7=994))), "")

    def test_estop_with_pump_on_sends_off_once_a_second(self):
        g = pc.PumpGate(P)
        why = g.watchdog(ok_inputs(now=10.0, rc=rc(ch7=994, ch10=2012), output_pwm=2012))
        self.assertIn("e-stop", why)
        self.assertEqual(g.watchdog(ok_inputs(now=10.5, rc=rc(ch7=994, ch10=2012),
                                              output_pwm=2012)), "")
        self.assertIn("e-stop", g.watchdog(ok_inputs(now=11.1, rc=rc(ch7=994, ch10=2012),
                                                     output_pwm=2012)))
        self.assertEqual(g.latched, "")          # the pilot's pump: not a fault

    def test_rc_lost_with_pump_on(self):
        g = pc.PumpGate(P)
        self.assertIn("RC lost", g.watchdog(ok_inputs(rc=None, output_pwm=2012)))

    def test_pilot_squirting_is_left_alone(self):
        g = pc.PumpGate(P)
        g.start_burst(0.3, 100.0)
        self.assertEqual(g.watchdog(ok_inputs(now=101.0, rc=rc(ch10=2012),
                                              output_pwm=2012)), "")

    def test_burst_that_did_not_stop_latches(self):
        g = pc.PumpGate(P)
        g.start_burst(0.3, 100.0)
        self.assertEqual(g.watchdog(ok_inputs(now=100.35, output_pwm=2000)), "")
        why = g.watchdog(ok_inputs(now=100.7, output_pwm=2000))
        self.assertIn("TRIM", why)
        self.assertTrue(g.latched)
        self.assertFalse(g.check_burst(0.3, ok_inputs(now=110.0))[0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
