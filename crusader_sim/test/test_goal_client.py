"""goal_client.run_goal against stand-in futures: no ROS. A stop (Ctrl-C) must cancel the goal
and wait for the tree to finish it; a cancel the server never confirms is said, not hidden."""
import os
import signal
import sys
import threading
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from crusader_sim import goal_client as G  # noqa: E402


class Fut:
    def __init__(self):
        self._v, self._done = None, False

    def done(self):
        return self._done

    def result(self):
        return self._v

    def resolve(self, v):
        self._v, self._done = v, True


def result_msg(outcome=0, detail="ok"):
    return NS(result=NS(outcome=outcome, detail=detail, elapsed_s=61.0))


class Handle:
    """cancel_goal_async resolves the result as CANCELLED (3) on the next spin, as bt_runner
    does, unless `stuck`."""

    def __init__(self, accepted=True, stuck=False):
        self.accepted, self.stuck, self.cancels = accepted, stuck, 0
        self.res = Fut()
        self.cancel_pending = False

    def get_result_async(self):
        return self.res

    def cancel_goal_async(self):
        self.cancels += 1
        self.cancel_pending = not self.stuck
        f = Fut()
        f.resolve(NS(return_code=0))
        return f

    def tick(self):
        if self.cancel_pending:
            self.cancel_pending = False
            self.res.resolve(result_msg(3, "cancelled"))


class Client:
    def __init__(self, handle, accept_at=1):
        self.handle, self.accept_at, self.goals = handle, accept_at, []
        self.fut, self.spins = Fut(), 0

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        return self.fut

    def spin(self, _t):
        self.spins += 1
        if self.spins == self.accept_at and not self.fut.done():
            self.fut.resolve(self.handle)
        self.handle.tick()


def ticking_clock():
    t = [0.0]

    def clock():
        t[0] += 1.0
        return t[0]
    return clock


class RunGoal(unittest.TestCase):

    def run_it(self, handle, stop=None, accept_at=1, on_spin=None, **kw):
        c = Client(handle, accept_at)
        lines = []
        stop = stop or threading.Event()

        def spin(t):
            c.spin(t)
            if on_spin:
                on_spin(c, c.spins, stop)
        out = G.run_goal(c, NS(g=1), spin, stop, say=lines.append,
                         names={0: "SUCCESS", 3: "CANCELLED"}, **kw)
        return out, c, lines

    def test_result_without_a_stop(self):
        h = Handle()
        out, _, lines = self.run_it(h, on_spin=lambda c, n, s: n == 4 and h.res.resolve(result_msg(0)))
        self.assertEqual((out.status, out.cancelled, h.cancels), ("result", False, 0))
        self.assertIn("[result] outcome SUCCESS  ok  (61 s)", lines)

    def test_a_stop_cancels_once_and_waits_for_the_trees_answer(self):
        h = Handle()
        out, _, lines = self.run_it(h, on_spin=lambda c, n, s: n == 3 and s.set())
        self.assertEqual((h.cancels, out.status, out.cancelled), (1, "result", True))
        self.assertEqual(out.result.outcome, 3)
        self.assertTrue(any("CANCELLING" in line for line in lines))

    def test_a_stop_before_acceptance_cancels_the_moment_the_goal_exists(self):
        h = Handle()
        stop = threading.Event()
        stop.set()
        out, _, _ = self.run_it(h, stop=stop, accept_at=3)
        self.assertEqual((h.cancels, out.result.outcome), (1, 3))

    def test_an_unconfirmed_cancel_is_reported_not_hidden(self):
        h = Handle(stuck=True)
        out, _, lines = self.run_it(h, on_spin=lambda c, n, s: n == 2 and s.set(),
                                    cancel_wait_s=5.0, clock=ticking_clock())
        self.assertEqual((out.status, h.cancels), ("cancel_unconfirmed", 1))
        self.assertTrue(any("not confirmed" in line for line in lines))

    def test_rejected_and_never_answered(self):
        out, _, _ = self.run_it(Handle(accepted=False))
        self.assertEqual(out.status, "rejected")
        out, _, _ = self.run_it(Handle(), accept_at=10 ** 6, accept_wait_s=5.0,
                                clock=ticking_clock())
        self.assertEqual(out.status, "not_accepted")

    def test_feedback_prints_changes_only(self):
        lines = []
        fb = G.feedback_printer(lines.append)
        msg = NS(feedback=NS(elapsed_s=12.0, running_node="CircleBuoy", current_task="SAFE_PASSAGE",
                             warning=""))
        fb(msg)
        fb(msg)
        fb(NS(feedback=NS(elapsed_s=13.0, running_node="CircleBuoy", current_task="SAFE_PASSAGE",
                          warning="no fresh pose")))
        self.assertEqual(len(lines), 2)
        self.assertIn("CircleBuoy", lines[0])
        self.assertIn("WARNING no fresh pose", lines[1])

    def test_outcome_names_from_the_result_class(self):
        cls = type("Result", (), {"OUTCOME_SUCCESS": 0, "OUTCOME_FAULT": 5, "outcome": 0})
        self.assertEqual(G.outcome_names(cls), {0: "SUCCESS", 5: "FAULT"})

    def test_install_stop_turns_sigint_and_sigterm_into_the_event(self):
        old = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
        try:
            for sig in (signal.SIGINT, signal.SIGTERM):
                stop = G.install_stop()
                os.kill(os.getpid(), sig)
                self.assertTrue(stop.wait(2.0), f"signal {sig} did not set the stop event")
        finally:
            for s, h in old.items():
                signal.signal(s, h)


if __name__ == "__main__":
    unittest.main()
