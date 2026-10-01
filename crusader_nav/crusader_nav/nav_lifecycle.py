"""nav_lifecycle - brings Nav2 lifecycle nodes to ACTIVE and keeps them there.

docs/nav2_avoidance_spec.md section 3.6 (note of 2026-10-01). It replaces
nav2_lifecycle_manager, whose Humble 1.1.20 change_state call has no timeout and
hung forever when Fast DDS dropped one reply (lifecycle_core.py has the whole
story). Every decision is in lifecycle_core.py; this file only makes the calls.

Calls (per managed node, every `check_period_s`, one at a time):
  /<node>/get_state      lifecycle_msgs/GetState     what state is it in?
  /<node>/change_state   lifecycle_msgs/ChangeState  configure, then activate
Publishes nothing. State changes are logged at INFO, repeated conditions once
and then at most every 30 s.

NEVER BLOCKS. Every call is call_async with a deadline of `call_timeout_s`; a
call that has not answered by then is dropped, counted, and asked again on the
next period. After `recreate_after_timeouts` in a row the two clients are
destroyed and built again, which is a fresh discovery. A node that is busy in a
transition (planner_server sits in `activating` until TF map -> base_footprint
exists) does not answer either, so this also fires while it waits; harmless.

IT NEVER STOPS. A planner_server that dies is configured and activated again
when nav.launch.py respawns it.

NO BOND. Nav2's nodes create a bond to "the lifecycle manager" on activate
(nav2_util::LifecycleNode::createBond). Humble 1.1.20 has no parameter to turn
that off, and with no manager it never connects. Measured 2026-10-01: nothing
follows the "Creating bond" line (no log, no deactivate) over 50 s of idle, past
the bond's 10 s connect timeout, so it is left alone.
Parameters are the `nav_lifecycle` section of nav2_params.yaml.
"""
import time

from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config

from crusader_nav import lifecycle_core as lc

PARAM_SPEC = {
    "node_names": dict(read_only=True, description="lifecycle nodes to configure and activate"),
    "check_period_s": dict(read_only=True, lo=0.1, hi=60.0,
                           description="period of the get_state check [s]"),
    "call_timeout_s": dict(read_only=True, lo=0.5, hi=60.0,
                           description="a service call unanswered after this is dropped [s]"),
    "recreate_after_timeouts": dict(read_only=True, lo=1, hi=100,
                                    description="consecutive timeouts before new clients"),
}

GET_STATE = "get_state"


class _Call:
    """One service call in flight. `kind` is get_state or a lifecycle_core ACTION."""
    __slots__ = ("kind", "client", "future", "deadline")

    def __init__(self, kind, client, future, deadline):
        self.kind, self.client, self.future, self.deadline = kind, client, future, deadline


class ManagedNode:
    """One lifecycle node: its two clients, the call in flight, what was last seen."""

    def __init__(self, owner, name, call_timeout_s, recreate_after, gate):
        self._owner = owner
        self.name = name
        self._timeout_s = call_timeout_s
        self._policy = lc.TimeoutPolicy(recreate_after)
        self._gate = gate
        self._call = None
        self._state = None                    # last state id seen; None = nothing seen
        self._make_clients()

    # ---- logging: one condition once, then at most every 30 s ----
    # Two methods and not one with a `level` argument: rclpy raises "Logger severity
    # cannot be changed between calls" when ONE call site logs at two severities.

    def _info(self, kind, text, now):
        if self._gate.allow((self.name, kind), now):
            self._owner.get_logger().info(f"{self.name}: {text}")

    def _warn(self, kind, text, now):
        if self._gate.allow((self.name, kind), now):
            self._owner.get_logger().warning(f"{self.name}: {text}")

    # ---- clients ----

    def _make_clients(self):
        self._get = self._owner.create_client(GetState, f"/{self.name}/get_state")
        self._change = self._owner.create_client(ChangeState, f"/{self.name}/change_state")

    def _recreate_clients(self):
        self._owner.destroy_client(self._get)
        self._owner.destroy_client(self._change)
        self._make_clients()

    def _ready(self):
        return self._get.service_is_ready() and self._change.service_is_ready()

    # ---- the period ----

    def tick(self, now):
        """Once per check period: handle a late call, else ask get_state."""
        if self._call is not None:
            if now < self._call.deadline or self._call.future.done():
                return                        # waiting, or answered and its callback is queued
            self._on_timeout(now)
            return                            # the next period asks again
        if not self._ready():
            self._on_unavailable(now)
            return
        self._send(GET_STATE, self._get, GetState.Request(), now)

    def _send(self, kind, client, request, now):
        call = _Call(kind, client, client.call_async(request), now + self._timeout_s)
        self._call = call
        call.future.add_done_callback(lambda fut: self._on_done(call, fut))

    def _on_unavailable(self, now):
        """No server: down, respawning, or not discovered yet. Keep checking."""
        if self._state is not None:
            self._owner.get_logger().warning(
                f"{self.name}: gone (was {lc.state_name(self._state)}); waiting for it")
            self._state = None                # whatever comes back is news
        self._info("unavailable",
                   "service not available (process down or still starting); will keep checking",
                   now)

    def _on_timeout(self, now):
        call, self._call = self._call, None
        call.client.remove_pending_request(call.future)
        self._warn("timeout",
                   f"no answer to {call.kind} in {self._timeout_s:g} s (a lost reply, or the "
                   "node is still inside a transition); asking again", now)
        if self._policy.note_timeout():
            self._recreate_clients()
            self._warn("recreate",
                       f"{self._policy.limit} calls in a row unanswered: service clients "
                       "recreated", now)

    # ---- answers ----

    def _on_done(self, call, future):
        if self._call is not call:
            return                            # it timed out and was dropped
        self._call = None
        now = time.monotonic()
        try:
            result = future.result()
        except Exception as e:                # the service call itself failed
            self._warn("call-failed", f"{call.kind} failed: {e}", now)
            return
        self._policy.note_answer()
        self._gate.forget((self.name, "unavailable"))
        if call.kind == GET_STATE:
            self._on_state(result.current_state.id, now)
        elif not result.success:
            self._warn(f"refused-{call.kind}",
                       f"the node refused to {call.kind}; will try again", now)

    def _on_state(self, state_id, now):
        previous, self._state = self._state, state_id
        if state_id != previous:
            self._owner.get_logger().info(
                f"{self.name} active (was {lc.state_name(previous)})"
                if state_id == lc.STATE_ACTIVE else
                f"{self.name}: {lc.state_name(previous)} -> {lc.state_name(state_id)}")
        action = lc.next_action(state_id)
        if action in lc.TRANSITION_FOR_ACTION:
            self._info(f"ask-{action}", f"asking it to {action}", now)
            request = ChangeState.Request()
            request.transition.id = lc.TRANSITION_FOR_ACTION[action]
            self._send(action, self._change, request, now)


class NavLifecycle(Node):

    def __init__(self):
        super().__init__("nav_lifecycle")
        p = declare_from_config(self, lc.DEFAULT_PARAMS, PARAM_SPEC)
        err = lc.params_error(p)
        if err:
            raise ValueError(f"nav_lifecycle: {err}")
        gate = lc.RepeatGate()
        self._managed = [
            ManagedNode(self, name, p["call_timeout_s"],
                        p["recreate_after_timeouts"], gate)
            for name in p["node_names"]]
        self.create_timer(p["check_period_s"], self._tick)
        self.get_logger().info(
            f"managing {', '.join(m.name for m in self._managed)}: check every "
            f"{p['check_period_s']:g} s, call timeout {p['call_timeout_s']:g} s, new clients "
            f"after {p['recreate_after_timeouts']} timeouts in a row")

    def _tick(self):
        now = time.monotonic()
        for managed in self._managed:
            managed.tick(now)


def main(args=None):
    run_node(NavLifecycle, args)


if __name__ == "__main__":
    main()
