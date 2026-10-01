"""lifecycle_core - what to do about a lifecycle node, as pure decisions. No rclpy.

docs/nav2_avoidance_spec.md section 3.6 (note of 2026-10-01). nav_lifecycle.py is
the node; this file is every decision it makes, so the decisions can be tested
without ROS.

The state ids are lifecycle_msgs/State: 1 unconfigured, 2 inactive, 3 active,
4 finalized, 10..15 the transitional states, and 0 here for "no answer" (the
service was down, or the call timed out). The transition ids are
lifecycle_msgs/Transition: 1 configure, 3 activate.

WHY NOT NAV2'S LIFECYCLE MANAGER. Humble's lifecycle_manager calls change_state
with no timeout. When Fast DDS has not yet matched the server's response writer
with the brand-new client's response reader (rmw_fastrtps answers "failed to
send response ... client will not receive response" after 100 ms), the reply is
dropped and the manager waits for it forever; planner_server then stays
`inactive` and every planned leg holds and FAILs. Nothing there retries. This
policy times out every call, asks again, and after a few timeouts in a row
builds new clients (fresh discovery).
"""

# lifecycle_msgs/State primary and transitional ids
STATE_UNKNOWN = 0                 # not a ROS id: "no answer"
STATE_UNCONFIGURED = 1
STATE_INACTIVE = 2
STATE_ACTIVE = 3
STATE_FINALIZED = 4
STATE_ERROR_PROCESSING = 15

STATE_NAMES = {
    STATE_UNKNOWN: "unknown",
    STATE_UNCONFIGURED: "unconfigured",
    STATE_INACTIVE: "inactive",
    STATE_ACTIVE: "active",
    STATE_FINALIZED: "finalized",
    10: "configuring",
    11: "cleaning up",
    12: "shutting down",
    13: "activating",
    14: "deactivating",
    STATE_ERROR_PROCESSING: "error processing",
}

# lifecycle_msgs/Transition ids
TRANSITION_CONFIGURE = 1
TRANSITION_ACTIVATE = 3

# What to do next
ACTION_CONFIGURE = "configure"    # call change_state(TRANSITION_CONFIGURE)
ACTION_ACTIVATE = "activate"      # call change_state(TRANSITION_ACTIVATE)
ACTION_NONE = "none"              # nothing to do: it is active, or cannot be revived
ACTION_WAIT = "wait"              # not an answer to act on yet: ask again next period

TRANSITION_FOR_ACTION = {
    ACTION_CONFIGURE: TRANSITION_CONFIGURE,
    ACTION_ACTIVATE: TRANSITION_ACTIVATE,
}

DEFAULT_RECREATE_AFTER_TIMEOUTS = 3
DEFAULT_LOG_REPEAT_S = 30.0

# The node's parameters and their defaults. A parameter's TYPE is its default's
# type: rcl will not take the int 3 for a double, so the YAML must write 3.0.
DEFAULT_PARAMS = {
    "node_names": ["planner_server"],
    "check_period_s": 1.0,
    "call_timeout_s": 3.0,
    "recreate_after_timeouts": DEFAULT_RECREATE_AFTER_TIMEOUTS,
}


def params_error(params):
    """Why `params` (a parameter set, or the YAML section) cannot configure the
    node, or None. Pure: nav_lifecycle checks the values it was given, and
    tools/scripts/check_config.py checks the YAML with the same function.

    The node ignores a key it does not declare and dies at startup on a wrong
    type, so both are reported here, by name.
    """
    unknown = sorted(set(params) - set(DEFAULT_PARAMS))
    if unknown:
        return f"unknown parameters {unknown}: the node would ignore them"
    for key, value in params.items():
        want = type(DEFAULT_PARAMS[key])
        if type(value) is not want:
            return (f"{key} = {value!r} is a {type(value).__name__}, the node declares "
                    f"a {want.__name__}" + (" (write 3.0, not 3)" if want is float else ""))
    names = params.get("node_names", DEFAULT_PARAMS["node_names"])
    if not names or not all(isinstance(n, str) and n and n.strip("/") == n for n in names):
        return f"node_names {names!r} must be non-empty names without a leading slash"
    return None


def state_name(state_id):
    """A word for a state id; an id outside lifecycle_msgs is shown as such."""
    if state_id is None:
        return STATE_NAMES[STATE_UNKNOWN]
    return STATE_NAMES.get(state_id, f"state {state_id}")


def next_action(state_id):
    """The one thing to do about a node last seen in `state_id`.

    None or 0 is no answer: the next period asks again. A transitional state
    (10..15) is the node mid-transition: wait, it will land somewhere. Finalized
    is terminal; the process exits, and a respawn comes back unconfigured, which
    is a new observation. An id outside lifecycle_msgs is not acted on either.
    """
    if state_id == STATE_UNCONFIGURED:
        return ACTION_CONFIGURE
    if state_id == STATE_INACTIVE:
        return ACTION_ACTIVATE
    if state_id in (STATE_ACTIVE, STATE_FINALIZED):
        return ACTION_NONE
    return ACTION_WAIT


class TimeoutPolicy:
    """Consecutive call timeouts, and when to build new service clients.

    note_timeout() is True on the call that reaches `limit` and starts a new
    count, so a persistent silence gives one "recreate" per `limit` timeouts and
    not one per timeout. Any answer resets the count: the clients work.
    """

    def __init__(self, limit=DEFAULT_RECREATE_AFTER_TIMEOUTS):
        if limit < 1:
            raise ValueError(f"limit {limit} < 1: every timeout would recreate")
        self.limit = limit
        self.consecutive = 0

    def note_timeout(self):
        self.consecutive += 1
        if self.consecutive >= self.limit:
            self.consecutive = 0
            return True
        return False

    def note_answer(self):
        self.consecutive = 0


class RepeatGate:
    """Let a condition through once, then at most every `period_s` seconds.

    allow(key, now) is True the first time a key is seen and then no more often
    than period_s. forget(key) makes the next allow() True again, which is how a
    condition that went away and came back is told as news, not as a repeat.
    """

    def __init__(self, period_s=DEFAULT_LOG_REPEAT_S):
        self.period_s = period_s
        self._last = {}

    def allow(self, key, now):
        last = self._last.get(key)
        if last is not None and now - last < self.period_s:
            return False
        self._last[key] = now
        return True

    def forget(self, key):
        self._last.pop(key, None)
