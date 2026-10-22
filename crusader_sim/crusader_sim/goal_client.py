"""goal_client — sending a RunMission goal from a script: task1_goal and nav_goal share it.

This module imports no ROS: the caller passes the action client's futures through and a
`spin` callable, so the cancel logic is tested on a plain python3 (test/test_goal_client.py)
and exists once.

WHY A SHARED CANCEL. A goal outlives the process that sent it: bt_runner keeps ticking the tree
when its client dies, so a Ctrl-C on the sender would leave the boat driving a mission nobody is
watching. run_goal() turns SIGINT/SIGTERM into an explicit cancel_goal_async, WAITS for the
server's answer and the goal's result, and only then returns.
"""
import os
import signal
import threading
import time


def tree_name(tree):
    """A built-in tree's name from a name, a file name or a path: bt_runner takes only the
    name (crusader_bt/behavior_trees/<name>.xml)."""
    name = os.path.basename(str(tree))
    return name[:-4] if name.endswith(".xml") else name


def feedback_printer(say=print):
    """A feedback callback: one line whenever the running node, the task or the warning
    changes. The tree sends feedback twice a second; most of it says the same thing."""
    last = [None]

    def fb(msg):
        f = msg.feedback
        key = (f.running_node, f.current_task, f.warning)
        if key != last[0]:
            say(f"[tree] {f.elapsed_s:6.1f} s  {f.running_node or '-':24s} task {f.current_task or '-'}"
                + (f"  WARNING {f.warning}" if f.warning else ""))
            last[0] = key
    return fb


def outcome_names(result_cls):
    """{code: "SUCCESS", ...} from RunMission.Result's OUTCOME_* constants."""
    return {getattr(result_cls, k): k[len("OUTCOME_"):] for k in dir(result_cls)
            if k.startswith("OUTCOME_")}


def result_line(r, names=None):
    name = (names or {}).get(r.outcome, str(r.outcome))
    return f"[result] outcome {name}  {r.detail}  ({r.elapsed_s:.0f} s)"


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
             clock=time.monotonic, names=None):
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
    say(result_line(r, names))
    return GoalRun("result", r, cancelled)
