"""rc_heartbeat_core: the autopilot's RC-receiver report trips the kill even with frozen PWM."""
import unittest

from crusader_behavior.rc_heartbeat_core import RcHeartbeatCore, WatchdogConfig


def core(now=0.0):
    return RcHeartbeatCore(WatchdogConfig(heartbeat_timeout=1.0, min_valid_pwm=900,
                                          require_armed=True, link_timeout=3.0,
                                          enable_latch=False, reset_low=1300, reset_high=1700),
                           now=now)


def feed(c, t, armed=True, receiver_lost=False):
    """One bridge cycle: frozen-but-valid ch7, FcuStatus, and the receiver report."""
    c.note_rc(t, True, 1500)
    c.note_fcu(t, armed)
    c.note_receiver(t, receiver_lost)


class RcHeartbeatCoreTest(unittest.TestCase):

    def test_receiver_lost_kills_despite_valid_pwm(self):
        c = core()
        feed(c, 0.1)
        self.assertFalse(c.tick(0.1).kill_active)
        feed(c, 0.2, receiver_lost=True)
        d = c.tick(0.2)
        self.assertTrue(d.kill_active and d.disarm_request)

    def test_recovers_when_the_receiver_is_back(self):
        c = core()
        feed(c, 0.1, receiver_lost=True)
        self.assertTrue(c.tick(0.1).kill_active)
        feed(c, 0.2)
        self.assertFalse(c.tick(0.2).kill_active)

    def test_disarmed_boat_is_not_killed(self):
        c = core()
        feed(c, 0.1, armed=False, receiver_lost=True)
        self.assertFalse(c.tick(0.1).kill_active)

    def test_a_stale_receiver_report_does_not_count(self):
        c = core()
        feed(c, 0.1, receiver_lost=True)
        for t in (1.0, 2.0, 3.0, 3.5):             # PWM and FcuStatus keep coming, the report stops
            c.note_rc(t, True, 1500)
            c.note_fcu(t, True)
        self.assertFalse(c.tick(3.5).kill_active)


if __name__ == "__main__":
    unittest.main()
