"""goal_client — the SafePassage goal-sending helpers task1_goal and lake_goal share.

Runs in: wherever ROS 2 is (crsd-sim for the sim's task1_goal, `asv` for the lake's
lake_goal). This module itself imports no ROS: the caller passes the action client's
futures through and a `spin` callable, so the cancel logic is testable on a plain python3
(test/test_lake.py) and exists ONCE for both tools.

WHY A SHARED CANCEL. A SafePassage goal outlives the process that sent it: the action server
(bt_runner) keeps ticking the tree when its client dies, so a Ctrl-C on the sender used to
leave the boat driving a mission nobody was watching (gz_task1.sh's header says so: "the tree
itself keeps its goal"). run_goal() therefore turns SIGINT/SIGTERM into an explicit
cancel_goal_async, WAITS for the server's cancel response and the goal's result, and only then
returns. On the real boat that is the difference between an ABORT button that stops the tree
and one that merely stops a script.
"""
import signal
import threading
import time

TIERS = {"core": 0, "advanced": 1, "disruptive": 2}


def feedback_printer(say=print):
    """A feedback callback: one line per phase change, per 5 % of progress, or per new
    warning — the tree sends feedback every tick and 0.1 % steps drown the phases. The
    line format is what task1_panel's TREE_RE parses."""
    last = [None]

    def fb(msg):
        f = msg.feedback
        key = (f.phase, int(f.progress * 20), f.buoys_resolved, f.plan_version, f.warning)
        if key != last[0]:
            say(f"[tree] {f.phase:12s} {f.progress * 100:5.1f}%  buoys {f.buoys_resolved}/"
                f"{f.buoys_known}  plan v{f.plan_version}  {f.warning}")
            last[0] = key
    return fb


def result_lines(r):
    """The two `[result]` lines the panel's mission parser reads."""
    return (f"[result] outcome {r.outcome}  {r.detail}\n"
            f"         classified {r.buoys_classified}, passed correctly "
            f"{r.buoys_passed_correctly}, {r.elapsed_s:.0f} s")


def install_stop(signals=(signal.SIGINT, signal.SIGTERM)):
    """An Event that SIGINT/SIGTERM set. Call it AFTER rclpy.init(): rclpy installs its own
    SIGINT handler (it shuts the context down, which kills a spin), and this replaces it,
    so the signal reaches run_goal() as a request to cancel instead."""
    stop = threading.Event()
    for s in signals:
        signal.signal(s, lambda *_: stop.set())
    return stop


class GoalRun:
    """What happened to one goal. status: result (the server answered) | rejected |
    not_accepted (no answer to the goal in accept_wait_s) | cancel_unconfirmed (we asked
    for a cancel and the server never finished the goal). `cancelled` = a cancel request
    was sent because of a stop signal."""

    def __init__(self, status, result=None, cancelled=False):
        self.status, self.result, self.cancelled = status, result, cancelled


def run_goal(client, goal, spin, stop, say=print, accept_wait_s=15.0, cancel_wait_s=15.0,
             clock=time.monotonic):
    """Send `goal`, print its feedback, wait for the result; if `stop` is set, CANCEL the
    goal and wait for the server to finish it. Returns a GoalRun.

    client  an rclpy ActionClient (send_goal_async -> future of a handle with .accepted,
            .get_result_async(), .cancel_goal_async())
    spin    spin(timeout_s): service the node once, e.g. lambda t: rclpy.spin_once(node, timeout_sec=t)
    stop    a threading.Event, set by a signal handler (install_stop) or a test

    A stop that arrives before the server has accepted the goal is held: the goal is
    cancelled the moment the handle exists, so a Ctrl-C in that window cannot leave a goal
    behind. Cancelling is never skipped, and never retried: one request, then a bounded
    wait for the result (cancel_wait_s) so the caller can say plainly if it never came."""
    fut = client.send_goal_async(goal, feedback_callback=feedback_printer(say))
    t0 = clock()
    while not fut.done():
        if clock() - t0 > accept_wait_s:
            say("[operator] the server never answered the goal")
            return GoalRun("not_accepted")
        spin(0.1)
    handle = fut.result()
    if not handle.accepted:
        say("[operator] goal REJECTED")
        return GoalRun("rejected")
    res_fut = handle.get_result_async()
    cancelled, t_cancel = False, None
    while not res_fut.done():
        if stop.is_set() and not cancelled:
            say("[operator] stop signal: CANCELLING the goal")
            handle.cancel_goal_async()
            cancelled, t_cancel = True, clock()
        if cancelled and clock() - t_cancel > cancel_wait_s:
            say("[operator] the cancel was not confirmed in %.0f s: the tree may still be running" % cancel_wait_s)
            return GoalRun("cancel_unconfirmed", None, True)
        spin(0.1)
    r = res_fut.result().result
    say(result_lines(r))
    return GoalRun("result", r, cancelled)
