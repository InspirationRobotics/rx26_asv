"""target_tracker_core: colour-family association and colour voting.

stdlib only, no ROS:   python crusader_world_model/test/test_target_tracker_core.py

The scenario this pins is Task 1 Disruptive. A RoboBuoy's side beacon is OFF
(only the top beacon, which the UAV sees, is lit) and in lower tiers the beacon
flashes 1 s on / 1 s off. The detector names the beacon STATE, so one buoy is
red_buoy on one frame and black_buoy on the next, which without colour voting is
two tracks a metre apart (a phantom obstacle beside a real gate buoy).

The expected values are worked out from the rule (a lit colour needs
colour_min_votes votes AND colour_min_ratio of the LIT votes, dark votes
excluded), not read back from the code under test.
"""
import ast
import dataclasses
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "..", "crusader_common"))

from crusader_world_model import target_tracker_core as core   # noqa: E402

# A level boat at the origin facing north: a detection at body (x, y) lands at
# a fixed world point, so the tests place buoys by body offset and never touch
# the projection maths (which crusader_common.geo owns).
BOAT = core.BoatState(east=0.0, north=0.0, roll=0.0, pitch=0.0, yaw=0.0)
SPOT = (10.0, 0.0)


def make_tracker(**overrides):
    return core.TargetTracker(core.TrackerParams(**overrides))


class Feeder:
    """Drives one tracker at 10 Hz, one camera cycle per call."""

    def __init__(self, tracker):
        self.tracker = tracker
        self.now = 0.0

    def cycle(self, *sightings):
        """sightings: (x, y, label) in body metres. Returns the track list."""
        self.now += 0.1
        cams = [core.CameraDetection(x, y, 0.0, label, 0.9)
                for x, y, label in sightings]
        return self.tracker.update(cams, [], BOAT, self.now)

    def see(self, label, n, at=SPOT):
        """n consecutive sightings of one label at one spot."""
        return self.repeat(n, (*at, label))

    def repeat(self, n, *sightings):
        """The same sightings in n consecutive cycles."""
        tracks = self.tracker.tracks
        for _ in range(n):
            tracks = self.cycle(*sightings)
        return tracks

    def alternate(self, first, second, n, at=SPOT):
        """first, second, first, second ... at one spot: n pairs. A flashing
        beacon seen at 10 Hz, or a detector torn between two colours."""
        tracks = self.tracker.tracks
        for _ in range(n):
            self.cycle((*at, first))
            tracks = self.cycle((*at, second))
        return tracks


def one_label(tracks):
    assert len(tracks) == 1, f"expected ONE track, got {len(tracks)}"
    return tracks[0].label


class AssociationTest(unittest.TestCase):

    def test_red_and_black_at_one_spot_are_one_track_resolving_red(self):
        f = Feeder(make_tracker())
        tracks = f.alternate("red_buoy", "black_buoy", 10)   # a flashing red buoy
        self.assertEqual(len(tracks), 1)
        t = tracks[0]
        self.assertEqual(t.label, "red_buoy")
        self.assertEqual(t.label_votes, {"red_buoy": 10, "black_buoy": 10})
        self.assertEqual(t.hits, 20)
        self.assertTrue(t.confirmed(f.tracker.p))

    def test_dark_first_then_red_still_one_track(self):
        """The track is born on a dark frame; the lit frames must join it."""
        f = Feeder(make_tracker())
        f.see("black_buoy", 3)
        self.assertEqual(one_label(f.see("red_buoy", 6)), "red_buoy")

    def test_two_family_buoys_beyond_the_radius_stay_two_tracks(self):
        f = Feeder(make_tracker())                   # assoc_radius_m = 3.0
        tracks = f.repeat(8, (10.0, 0.0, "red_buoy"), (10.0, 3.5, "black_buoy"))
        self.assertEqual(len(tracks), 2)
        self.assertEqual(sorted(t.label for t in tracks),
                         ["red_buoy", "unknown_buoy"])   # red voted, black only

    def test_a_six_metre_gate_resolves_red_and_green_separately(self):
        f = Feeder(make_tracker())
        tracks = f.repeat(8, (10.0, -3.0, "red_buoy"), (10.0, 3.0, "green_buoy"))
        self.assertEqual(sorted(t.label for t in tracks),
                         ["green_buoy", "red_buoy"])

    def test_family_colours_inside_the_radius_do_merge(self):
        """The documented price of colour voting: assoc_radius_m must stay
        below the narrowest gate, because red and green are no longer kept
        apart by their labels."""
        f = Feeder(make_tracker())
        f.see("red_buoy", 4)
        tracks = f.see("green_buoy", 4, at=(10.0, 2.0))
        self.assertEqual(len(tracks), 1)

    def test_non_family_labels_with_different_names_never_merge(self):
        f = Feeder(make_tracker())
        tracks = f.repeat(4, (*SPOT, "yellow_buoy"), (*SPOT, "black_target_boat"))
        self.assertEqual(sorted(t.label for t in tracks),
                         ["black_target_boat", "yellow_buoy"])

    def test_family_label_never_merges_with_a_non_family_label(self):
        f = Feeder(make_tracker())
        f.see("yellow_buoy", 4)
        tracks = f.see("red_buoy", 4)
        self.assertEqual(len(tracks), 2)

    def test_non_family_track_keeps_the_plain_majority(self):
        f = Feeder(make_tracker())
        self.assertEqual(one_label(f.see("yellow_buoy", 1)), "yellow_buoy")

    def test_same_non_family_label_still_merges(self):
        f = Feeder(make_tracker())
        self.assertEqual(one_label(f.see("yellow_buoy", 5)), "yellow_buoy")

    def test_a_lidar_only_track_is_still_compatible_with_any_label(self):
        """The empty-label rule is unchanged: no votes yet, so anything fits."""
        t = core.Track(id=1, x=0.0, y=0.0, z=0.0)
        obs = core.Observation(0.0, 0.0, 0.0, "red_buoy", 0.9, core.SOURCE_CAMERA)
        self.assertTrue(core._label_ok(t, obs, core.TrackerParams()))


class VotingTest(unittest.TestCase):

    def test_red_and_green_fifty_fifty_is_one_unknown_track(self):
        f = Feeder(make_tracker())
        tracks = f.alternate("red_buoy", "green_buoy", 10)
        self.assertEqual(one_label(tracks), "unknown_buoy")
        self.assertEqual(f.tracker.stats["colour_unknown"], 1)

    def test_black_only_is_unknown(self):
        f = Feeder(make_tracker())
        self.assertEqual(one_label(f.see("black_buoy", 30)), "unknown_buoy")

    def test_four_red_votes_are_below_the_minimum(self):
        f = Feeder(make_tracker())
        self.assertEqual(one_label(f.see("red_buoy", 4)), "unknown_buoy")

    def test_the_fifth_red_vote_resolves_it(self):
        f = Feeder(make_tracker())
        f.see("red_buoy", 4)
        self.assertEqual(one_label(f.see("red_buoy", 1)), "red_buoy")

    def test_dark_votes_are_not_votes_against_the_lit_colour(self):
        f = Feeder(make_tracker())
        f.see("black_buoy", 40)                      # 5 lit vs 40 dark: red
        self.assertEqual(one_label(f.see("red_buoy", 5)), "red_buoy")

    def test_a_stray_wrong_colour_does_not_flip_a_clear_winner(self):
        f = Feeder(make_tracker())
        f.alternate("red_buoy", "black_buoy", 12)
        self.assertEqual(one_label(f.see("green_buoy", 1)), "red_buoy")  # 12/13

    def test_an_unknown_track_resolves_once_one_colour_pulls_ahead(self):
        f = Feeder(make_tracker())
        f.see("red_buoy", 5)
        self.assertEqual(one_label(f.see("green_buoy", 5)), "unknown_buoy")
        self.assertEqual(one_label(f.see("red_buoy", 10)), "red_buoy")  # 15/20

    def test_a_dead_heat_is_unknown_even_with_a_lax_ratio(self):
        f = Feeder(make_tracker(colour_min_ratio=0.5))
        f.see("red_buoy", 5)
        self.assertEqual(one_label(f.see("green_buoy", 5)), "unknown_buoy")

    def test_the_two_blue_states_vote_as_distinct_lit_colours(self):
        f = Feeder(make_tracker())
        self.assertEqual(one_label(f.see("flashing_blue_buoy", 6)),
                         "flashing_blue_buoy")

    def test_label_votes_stay_complete(self):
        f = Feeder(make_tracker())
        f.see("black_buoy", 3)
        f.see("red_buoy", 2)
        self.assertEqual(f.tracker.tracks[0].label_votes,
                         {"black_buoy": 3, "red_buoy": 2})

    def test_vote_thresholds_re_resolve_the_whole_history_live(self):
        f = Feeder(make_tracker())
        f.see("red_buoy", 4)
        self.assertEqual(one_label(f.tracker.snapshot(BOAT)), "unknown_buoy")
        f.tracker.p.colour_min_votes = 3             # a [DYN] parameter
        self.assertEqual(one_label(f.tracker.snapshot(BOAT)), "red_buoy")

    def test_the_unknown_label_carries_no_colour_word(self):
        """bt_runner beaconFromLabel() matches these substrings, in this order
        of precedence, so the unknown label must hold none of them or the UAV's
        colour would be overridden by a colour the boat does not have."""
        label = core.TrackerParams().colour_unknown_label.lower()
        for word in ("red", "green", "blue", "off", "black", "flash"):
            self.assertNotIn(word, label)


class OakVocabularyTest(unittest.TestCase):
    """buoy_model labels: [flash_|off_]<colour>_<shape>, or a bare <shape>."""

    def test_bare_and_flash_red_diamond_are_one_track_resolving_red(self):
        f = Feeder(make_tracker())
        tracks = f.alternate("flash_red_diamond", "diamond", 10)
        self.assertEqual(one_label(tracks), "flash_red_diamond")

    def test_flash_and_solid_of_one_colour_pool_their_votes(self):
        f = Feeder(make_tracker())
        f.see("red_diamond", 3)
        self.assertEqual(one_label(f.see("flash_red_diamond", 2)), "red_diamond")

    def test_dark_diamond_only_is_unknown_diamond(self):
        f = Feeder(make_tracker())
        f.see("diamond", 10)
        self.assertEqual(one_label(f.see("off_diamond", 10)), "unknown_diamond")
        self.assertEqual(f.tracker.stats["colour_unknown"], 1)

    def test_red_green_diamond_split_is_unknown(self):
        f = Feeder(make_tracker())
        tracks = f.alternate("flash_red_diamond", "flash_green_diamond", 10)
        self.assertEqual(one_label(tracks), "unknown_diamond")

    def test_diamond_never_merges_with_another_shape_or_the_buoy_list(self):
        f = Feeder(make_tracker())
        f.see("diamond", 5)
        tracks = f.see("red_buoy", 5)
        self.assertEqual(len(tracks), 2)
        tracks = f.see("blue_circle", 5)
        self.assertEqual(len(tracks), 3)


class DisableTest(unittest.TestCase):

    def test_disabled_restores_one_track_per_label(self):
        f = Feeder(make_tracker(colour_vote_enable=False))
        tracks = f.repeat(5, (*SPOT, "red_buoy"), (*SPOT, "black_buoy"))
        self.assertEqual(sorted(t.label for t in tracks),
                         ["black_buoy", "red_buoy"])

    def test_disabled_reports_the_plain_majority(self):
        f = Feeder(make_tracker(colour_vote_enable=False))
        self.assertEqual(one_label(f.see("red_buoy", 1)), "red_buoy")


class TurnSmearTest(unittest.TestCase):
    """Mid-turn, a far camera sighting is placed metres off its buoy (the frame and the pose
    are not from one instant): it is dropped instead of becoming a ghost track."""

    @staticmethod
    def run_at(tracker, yaw_rate, x, n=5):
        boat = dataclasses.replace(BOAT, yaw_rate=yaw_rate)
        tracks = []
        for k in range(n):
            cams = [core.CameraDetection(x, 0.0, 0.0, "red_buoy", 0.9)]
            tracks = tracker.update(cams, [], boat, 0.1 * (k + 1))
        return tracks

    def test_far_sighting_mid_turn_is_dropped(self):
        t = make_tracker()                       # 1.0 m at 0.2 s: past 10 m at 0.5 rad/s
        self.assertEqual(self.run_at(t, 0.5, 20.0), [])
        self.assertEqual(t.stats["turn_dropped"], 1)
        self.assertEqual(self.run_at(t, -0.5, 20.0), [])   # either way round

    def test_near_sighting_mid_turn_is_kept(self):
        self.assertEqual(len(self.run_at(make_tracker(), 0.5, 5.0)), 1)

    def test_far_sighting_steady_is_kept(self):
        self.assertEqual(len(self.run_at(make_tracker(), 0.0, 20.0)), 1)
        self.assertEqual(len(self.run_at(make_tracker(), 0.04, 20.0)), 1)   # 0.16 m

    def test_zero_limit_is_off(self):
        t = make_tracker(turn_smear_max_m=0.0)
        self.assertEqual(len(self.run_at(t, 2.0, 30.0)), 1)


class RangeAwareTest(unittest.TestCase):
    """range_sigma_k > 0: the gate, the position weight and the colour vote follow the
    stereo error, sigma(r) = k r^2 (0.0036 = the sim's depth model). The field these pin
    is the 2026-11-10 Singapore one: buoys 1.86 m apart at the tightest."""

    K = 0.0036

    def ranged(self, **kw):
        return make_tracker(range_sigma_k=self.K, **kw)

    def test_off_by_default(self):
        self.assertEqual(core.TrackerParams().range_sigma_k, 0.0)
        self.assertEqual(core.TrackerParams().vote_ambiguity_m, 0.0)

    def test_close_neighbours_stay_apart_and_keep_their_colours(self):
        # 6 m out: sigma 0.13 m, gate 0.9 m (the floor); the buoys are 1.86 m apart
        f = Feeder(self.ranged())
        tracks = f.repeat(10, (6.0, 0.93, "red_buoy"), (6.0, -0.93, "green_buoy"))
        self.assertEqual(sorted(t.label for t in tracks), ["green_buoy", "red_buoy"])

    def test_the_fixed_three_metre_gate_merges_the_same_pair(self):
        # the failure this exists for: the default gate holds both in one track
        f = Feeder(make_tracker())
        tracks = f.repeat(10, (6.0, 0.93, "red_buoy"), (6.0, -0.93, "green_buoy"))
        tracks = f.repeat(10, (6.0, 0.93, "red_buoy"))
        self.assertLessEqual(len(tracks), 2)
        f2 = Feeder(make_tracker())
        f2.see("red_buoy", 5, at=(6.0, 0.93))
        self.assertEqual(len(f2.see("green_buoy", 5, at=(6.0, -0.93))), 1)

    def test_far_scatter_is_one_track_not_duplicates(self):
        # 20 m out: sigma 1.44 m, gate clamps to assoc_radius_m 3.0; sightings 1.2 m apart
        f = Feeder(self.ranged())
        for k in range(12):
            f.cycle((20.0 + (1.2 if k % 2 else -1.2), 0.0, "red_buoy"))
        self.assertEqual(len(f.tracker.tracks), 1)

    def test_a_fixed_small_gate_splits_the_same_scatter(self):
        f = Feeder(make_tracker(assoc_radius_m=1.5))
        for k in range(12):
            f.cycle((20.0 + (1.2 if k % 2 else -1.2), 0.0, "red_buoy"))
        self.assertEqual(len(f.tracker.tracks), 2)

    def test_a_close_sighting_corrects_a_buoy_placed_from_far(self):
        # placed from 20 m (sigma 1.44, w 0.48), then one sighting from 5 m, 1.0 m away
        # (sigma 0.10 floor -> 0.09: w 100): gain 100 / 100.48 = 0.995
        f = Feeder(self.ranged())
        f.see("red_buoy", 1, at=(20.0, 0.0))
        t = f.tracker.tracks[0]
        t.x, t.y = 0.0, 20.0                     # pin the world position for the arithmetic
        boat = core.BoatState(east=0.0, north=14.0, roll=0.0, pitch=0.0, yaw=0.0)
        f.tracker.update([core.CameraDetection(5.0, 0.0, 0.0, "red_buoy", 0.9)], [], boat, 1.0)
        self.assertEqual(len(f.tracker.tracks), 1)    # associated, not a new track
        self.assertAlmostEqual(t.y, 19.0, delta=0.05)

    def test_a_far_sighting_barely_moves_a_buoy_fixed_from_close(self):
        f = Feeder(self.ranged())
        f.see("red_buoy", 20, at=(5.0, 0.0))     # info capped at 4 / 0.1^2 = 400
        t = f.tracker.tracks[0]
        y0 = t.y
        f.cycle((5.0, 0.0, "red_buoy"))
        # the same buoy seen from 20 m, 1.0 m off along the line of sight
        boat = core.BoatState(east=0.0, north=-15.0, roll=0.0, pitch=0.0, yaw=0.0)
        f.tracker.update([core.CameraDetection(21.0, 0.0, 0.0, "red_buoy", 0.9)], [], boat, 9.0)
        self.assertEqual(len(f.tracker.tracks), 1)    # associated, not a new track
        self.assertLess(abs(t.y - y0), 0.01)     # w 0.40 vs 400: 0.1 % of 1.0 m

    def test_an_ambiguous_sighting_does_not_vote(self):
        f = Feeder(self.ranged(vote_ambiguity_m=1.0))
        f.repeat(6, (6.0, 0.93, "red_buoy"), (6.0, -0.93, "green_buoy"))
        before = {t.label: dict(t.label_votes) for t in f.tracker.tracks}
        f.cycle((6.0, 0.2, "green_buoy"))        # 0.73 / 1.13 m from the two: ambiguous
        after = {t.label: dict(t.label_votes) for t in f.tracker.tracks}
        self.assertEqual(before, after)
        self.assertEqual(f.tracker.stats["votes_withheld"], 1)

    def test_a_clear_sighting_still_votes(self):
        f = Feeder(self.ranged(vote_ambiguity_m=1.0))
        f.repeat(6, (6.0, 0.93, "red_buoy"), (6.0, -0.93, "green_buoy"))
        f.cycle((6.0, 0.95, "red_buoy"))
        red = next(t for t in f.tracker.tracks if t.label == "red_buoy")
        self.assertEqual(red.label_votes["red_buoy"], 7)


class WiringTest(unittest.TestCase):
    """The node cannot be imported without rclpy, so its wiring is checked from the source:
    a name in _CORE_PARAMS that TrackerParams lacks is a TypeError at node start, and a YAML
    key the node does not declare is ignored by ROS without a word."""

    NODE = os.path.join(HERE, "..", "crusader_world_model", "target_tracker_node.py")
    YAML = os.path.join(HERE, "..", "..", "crusader_bringup", "config",
                        "crusader_params.yaml")

    @classmethod
    def setUpClass(cls):
        with open(cls.NODE, encoding="utf-8") as fh:
            tree = ast.parse(fh.read())
        lit = {}
        for node in tree.body:
            if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
                try:
                    lit[node.targets[0].id] = ast.literal_eval(node.value)
                except ValueError:
                    pass
        cls.core_params, cls.params = lit["_CORE_PARAMS"], lit["PARAMS"]

    def test_every_core_param_is_a_trackerparams_field(self):
        fields = {f.name for f in dataclasses.fields(core.TrackerParams)}
        self.assertEqual(sorted(set(self.core_params) - fields), [])

    def test_every_core_param_is_declared(self):
        self.assertEqual(sorted(set(self.core_params) - set(self.params)), [])

    def test_yaml_keys_are_declared(self):
        try:
            import yaml
        except ImportError:
            self.skipTest("PyYAML not installed")
        with open(self.YAML, encoding="utf-8") as fh:
            tt = yaml.safe_load(fh)["target_tracker"]["ros__parameters"]
        self.assertEqual(sorted(set(tt) - set(self.params)), [])

    def test_colour_defaults_match_the_core(self):
        defaults = core.TrackerParams()
        for name in self.core_params:
            if name.startswith("colour_"):
                v = self.params[name]
                self.assertEqual(tuple(v) if isinstance(v, list) else v,
                                 getattr(defaults, name), name)


class ForgetColoursTest(unittest.TestCase):
    """bt_runner_node asks for this at every mission start: the reds and greens are re-dealt."""

    def test_a_red_buoy_forgets_its_colour_and_keeps_its_place(self):
        f = Feeder(make_tracker())
        before = f.see("red_buoy", 40)[0]
        tid, x, y, hits = before.id, before.x, before.y, before.hits
        self.assertEqual(f.tracker.forget_colours(), 1)
        t = f.tracker.tracks[0]
        self.assertEqual((t.id, t.x, t.y, t.hits), (tid, x, y, hits))
        self.assertEqual(t.label, "unknown_buoy")

    def test_this_run_s_colour_is_not_outvoted_by_the_last_run_s(self):
        f = Feeder(make_tracker())
        f.see("red_buoy", 40)
        f.tracker.forget_colours()
        self.assertEqual(one_label(f.see("green_buoy", 5)), "green_buoy")

    def test_a_shape_family_keeps_its_family(self):
        f = Feeder(make_tracker())
        f.see("flash_red_diamond", 20)
        f.tracker.forget_colours()
        self.assertEqual(f.tracker.tracks[0].label_votes, {"diamond": 1})
        self.assertEqual(one_label(f.see("green_diamond", 5)), "green_diamond")

    def test_a_track_outside_the_colour_family_keeps_its_label(self):
        f = Feeder(make_tracker())
        f.see("dock_sign", 10)
        self.assertEqual(f.tracker.forget_colours(), 0)
        self.assertEqual(f.tracker.tracks[0].label_votes, {"dock_sign": 10})


if __name__ == "__main__":
    unittest.main()
