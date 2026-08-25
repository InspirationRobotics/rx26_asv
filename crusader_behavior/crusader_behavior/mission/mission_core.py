"""mission_core — what state and task the boat is in. Pure: no ROS, no clock.

mission_planner is the thin wrapper that feeds this freshness-checked telemetry
and publishes the answer. The split follows the fleet's standing convention that
safety-relevant logic belongs in a core you can read and exercise without rclpy
in the way — and this is safety-relevant in a specific sense: the answer is
relayed to RoboCommand as our claim about whether the boat is driving itself.

WHAT "AUTONOMOUS" MEANS ON A HULL. The autopilot mode is the whole answer:
GUIDED (companion computer driving) or AUTO (flying a stored mission) and armed.
Anything else -- MANUAL, HOLD, disarmed -- is STATE_MANUAL. There is no
in-between, which is why the boat's rule is so much shorter than the aircraft's.

NEVER STATE_UNKNOWN. It is the proto zero value; the OCS validator refuses it,
and it would mean "the reporter forgot" rather than "the boat does not know".
When the truth is not knowable this returns None and the caller sends nothing --
a gap is visible on the OCS as rising silence, a fabricated state is not.
"""
from __future__ import annotations

#: Modes in which an armed hull is driving itself.
#:
#: DEFINED HERE, and imported by pixhawk_led_status_node rather than the other
#: way round. This module has to stay importable with no rclpy present -- that
#: is what lets the rule be exercised on a bare laptop -- and the LED node is
#: not. One list either way: a mode that counts as autonomy for the status
#: light counts as autonomy for RoboCommand, and two lists would drift.
AUTO_MODES = ("AUTO", "GUIDED")

AUTO = "STATE_AUTO"
MANUAL = "STATE_MANUAL"
KILLED = "STATE_KILLED"

#: The proto's RxTask names. TASK_UNKNOWN is absent deliberately -- it is the
#: zero value and the handbook forbids sending it.
TASK_NONE = "TASK_NONE"
TASKS = (
    TASK_NONE,
    "TASK_SAFE_PASSAGE",
    "TASK_INFRA_SURVEY_REPAIR",
    "TASK_COORDINATED_LOGISTICS",
    "TASK_DYNAMIC_INCIDENT",
)


def robot_state(mode, armed, killed=False):
    """(state, reason). state is None when it cannot be answered truthfully.

    mode:   FcuStatus.mode, or None if the stream is stale.
    armed:  FcuStatus.armed, or None if the stream is stale.
    killed: the RC watchdog's kill latch.
    """
    if killed:
        # Checked first and unconditionally: a killed boat is killed whatever
        # the autopilot last said its mode was.
        return KILLED, "RC kill active"
    if mode is None or armed is None:
        return None, "fcu_status stale -- mode and armed both unknown"
    if not armed:
        return MANUAL, "disarmed"
    m = (mode or "").upper()
    if m in AUTO_MODES:
        return AUTO, "armed in %s" % m
    return MANUAL, "armed in %s, which is not an autonomy mode" % m


def decide(mode, armed, killed=False, *, mission_active=False,
           mission_name="", task=TASK_NONE, bench_auto=False):
    """The planner's whole answer: (state, task, reason, mission_active).

    bench_auto forces STATE_AUTO with no autopilot involved, for producing
    rc-test logs off the water. It is reported in the reason so it can never be
    mistaken for a real mode on a page or in a log, and it still cannot override
    a kill -- a killed boat reads KILLED whatever the bench flag says.
    """
    if task not in TASKS:
        # A task the proto does not have would be refused by the OCS validator
        # anyway; failing here names the planner instead of the wire.
        raise ValueError("unknown task %r; expected one of %s" % (task, TASKS))

    state, reason = robot_state(mode, armed, killed)
    if bench_auto and state != KILLED:
        return AUTO, task, "BENCH_AUTO -- not from the autopilot (%s)" % reason, \
            mission_active
    return state, task, reason, mission_active
