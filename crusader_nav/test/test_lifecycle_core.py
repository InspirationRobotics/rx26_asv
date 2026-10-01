"""lifecycle_core: what nav_lifecycle does about a node in each lifecycle state.

stdlib only, no ROS:   python crusader_nav/test/test_lifecycle_core.py
(pytest collects it too.)

The ids are written out from lifecycle_msgs/State and Transition, not imported,
so this runs where ROS does not and pins the numbers the node sends.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from crusader_nav import lifecycle_core as lc   # noqa: E402

UNCONFIGURED, INACTIVE, ACTIVE, FINALIZED = 1, 2, 3, 4
TRANSITIONAL = {10: "configuring", 11: "cleaning up", 12: "shutting down",
                13: "activating", 14: "deactivating", 15: "error processing"}


class NextActionTest(unittest.TestCase):

    def test_unconfigured_is_configured(self):
        self.assertEqual(lc.next_action(UNCONFIGURED), "configure")

    def test_inactive_is_activated(self):
        self.assertEqual(lc.next_action(INACTIVE), "activate")

    def test_active_is_left_alone(self):
        self.assertEqual(lc.next_action(ACTIVE), "none")

    def test_finalized_is_left_alone(self):
        """Terminal: a respawn is a new process, seen as unconfigured."""
        self.assertEqual(lc.next_action(FINALIZED), "none")

    def test_every_transitional_state_waits(self):
        for state_id in TRANSITIONAL:
            self.assertEqual(lc.next_action(state_id), "wait", state_id)

    def test_no_answer_waits(self):
        self.assertEqual(lc.next_action(None), "wait")
        self.assertEqual(lc.next_action(lc.STATE_UNKNOWN), "wait")

    def test_an_id_outside_lifecycle_msgs_waits(self):
        for state_id in (5, 9, 16, 99, -1):
            self.assertEqual(lc.next_action(state_id), "wait", state_id)

    def test_whole_id_range_is_covered_with_one_of_four_actions(self):
        allowed = {lc.ACTION_CONFIGURE, lc.ACTION_ACTIVATE, lc.ACTION_NONE, lc.ACTION_WAIT}
        for state_id in [None] + list(range(-1, 20)):
            self.assertIn(lc.next_action(state_id), allowed, state_id)

    def test_transition_ids_are_lifecycle_msgs_ids(self):
        """configure = 1, activate = 3; the other actions send nothing."""
        self.assertEqual(lc.TRANSITION_FOR_ACTION,
                         {lc.ACTION_CONFIGURE: 1, lc.ACTION_ACTIVATE: 3})
        for action in (lc.ACTION_NONE, lc.ACTION_WAIT):
            self.assertNotIn(action, lc.TRANSITION_FOR_ACTION)

    def test_bring_up_walks_unconfigured_to_active(self):
        """Following the actions from unconfigured reaches active in two steps."""
        state, sent = UNCONFIGURED, []
        step_to = {1: INACTIVE, 3: ACTIVE}       # what change_state(id) does
        for _ in range(4):
            action = lc.next_action(state)
            if action not in lc.TRANSITION_FOR_ACTION:
                break
            sent.append(lc.TRANSITION_FOR_ACTION[action])
            state = step_to[sent[-1]]
        self.assertEqual((state, sent), (ACTIVE, [1, 3]))


class StateNameTest(unittest.TestCase):

    def test_primary_and_transitional_names(self):
        self.assertEqual([lc.state_name(i) for i in (1, 2, 3, 4)],
                         ["unconfigured", "inactive", "active", "finalized"])
        for state_id, name in TRANSITIONAL.items():
            self.assertEqual(lc.state_name(state_id), name)

    def test_no_answer_and_foreign_ids(self):
        self.assertEqual(lc.state_name(None), "unknown")
        self.assertEqual(lc.state_name(0), "unknown")
        self.assertEqual(lc.state_name(42), "state 42")


class TimeoutPolicyTest(unittest.TestCase):

    def test_default_is_three(self):
        self.assertEqual(lc.TimeoutPolicy().limit, 3)

    def test_recreate_on_the_third_consecutive_timeout(self):
        p = lc.TimeoutPolicy(3)
        self.assertEqual([p.note_timeout() for _ in range(3)], [False, False, True])

    def test_an_answer_resets_the_count(self):
        p = lc.TimeoutPolicy(3)
        p.note_timeout()
        p.note_timeout()
        p.note_answer()
        self.assertEqual(p.consecutive, 0)
        self.assertEqual([p.note_timeout() for _ in range(3)], [False, False, True])

    def test_recreating_starts_a_new_count(self):
        """A long silence recreates once per `limit` timeouts, not once per timeout."""
        p = lc.TimeoutPolicy(3)
        self.assertEqual([p.note_timeout() for _ in range(9)],
                         [False, False, True] * 3)

    def test_limit_one_recreates_every_time(self):
        p = lc.TimeoutPolicy(1)
        self.assertEqual([p.note_timeout() for _ in range(3)], [True, True, True])

    def test_a_limit_below_one_is_refused(self):
        for bad in (0, -1):
            with self.assertRaises(ValueError):
                lc.TimeoutPolicy(bad)


class RepeatGateTest(unittest.TestCase):

    def test_first_time_is_allowed_then_held_for_the_period(self):
        g = lc.RepeatGate(30.0)
        self.assertTrue(g.allow("down", 100.0))
        self.assertFalse(g.allow("down", 100.1))
        self.assertFalse(g.allow("down", 129.9))
        self.assertTrue(g.allow("down", 130.0))      # 30 s since the last one told
        self.assertFalse(g.allow("down", 131.0))

    def test_keys_are_independent(self):
        g = lc.RepeatGate(30.0)
        self.assertTrue(g.allow(("planner_server", "timeout"), 0.0))
        self.assertTrue(g.allow(("planner_server", "down"), 0.0))
        self.assertTrue(g.allow(("other", "timeout"), 0.0))

    def test_forgetting_makes_the_next_one_news(self):
        g = lc.RepeatGate(30.0)
        g.allow("down", 0.0)
        self.assertFalse(g.allow("down", 1.0))
        g.forget("down")
        self.assertTrue(g.allow("down", 2.0))
        g.forget("never seen")                       # harmless

    def test_default_period_is_thirty_seconds(self):
        self.assertEqual(lc.RepeatGate().period_s, 30.0)


class ParamsTest(unittest.TestCase):

    def test_the_defaults_are_valid_and_manage_planner_server(self):
        self.assertIsNone(lc.params_error(lc.DEFAULT_PARAMS))
        self.assertEqual(lc.DEFAULT_PARAMS["node_names"], ["planner_server"])
        self.assertEqual(lc.DEFAULT_PARAMS["check_period_s"], 1.0)
        self.assertEqual(lc.DEFAULT_PARAMS["call_timeout_s"], 3.0)
        self.assertEqual(lc.DEFAULT_PARAMS["recreate_after_timeouts"], 3)

    def test_a_partial_section_is_valid(self):
        self.assertIsNone(lc.params_error({"call_timeout_s": 2.0}))

    def test_an_int_for_a_double_is_named(self):
        """rcl would kill the node at declare: say which key, and the fix."""
        err = lc.params_error({"call_timeout_s": 3})
        self.assertIn("call_timeout_s", err)
        self.assertIn("3.0", err)

    def test_a_float_for_the_count_is_refused(self):
        self.assertIn("recreate_after_timeouts",
                      lc.params_error({"recreate_after_timeouts": 3.0}))

    def test_a_bool_is_not_a_number(self):
        self.assertIsNotNone(lc.params_error({"recreate_after_timeouts": True}))

    def test_an_unknown_key_is_named(self):
        err = lc.params_error({"bond_timeout": 4.0})
        self.assertIn("bond_timeout", err)

    def test_node_names_must_be_names(self):
        for bad in ([], [""], ["/planner_server"], ["planner_server", ""]):
            self.assertIn("node_names", lc.params_error({"node_names": bad}), bad)
        self.assertIsNone(lc.params_error({"node_names": ["planner_server", "other"]}))

    def test_node_names_must_be_a_list_of_strings(self):
        self.assertIsNotNone(lc.params_error({"node_names": "planner_server"}))


if __name__ == "__main__":
    unittest.main()
