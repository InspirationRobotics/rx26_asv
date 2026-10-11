"""Offline tests for sitl_params: the SITL parameter file built from the boat's baseline + the sim overlay.

No ROS, no sim, no SITL. Needs only python3:

    cd Boat/rx26_asv/crusader_sim; python -m unittest discover -s test -p "test_sitl_params.py" -v
"""
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from crusader_sim import sitl_params                                          # noqa: E402

OVERLAY = os.path.join(os.path.dirname(HERE), "config", "sitl_overlay.parm")


def _generated(baseline=None, overlay=OVERLAY):
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "gz.parm")
        sitl_params.build(out, baseline=baseline, overlay=overlay)
        with open(out, encoding="utf-8") as f:
            lines = [ln.split() for ln in f if ln.strip() and not ln.startswith("#")]
    return lines


class SitlParams(unittest.TestCase):
    def test_sim_boat_is_sysid_2_like_the_real_boat(self):
        # Ekko's Cube is sysid 1 on the shared RFD link; the real Crusader is 2.
        lines = [v for k, v in _generated() if k == "SYSID_THISMAV"]
        self.assertEqual(lines, ["2"], "exactly one SYSID_THISMAV line, and it is 2")

    def test_overlay_replaces_baseline_value_without_a_duplicate_line(self):
        lines = _generated()
        names = [k for k, _ in lines]
        self.assertEqual(len(names), len(set(names)), "a param appears twice in the SITL file")
        self.assertEqual(dict(lines)["SR0_EXTRA1"], "10")        # overlay beats the baseline's 4

    def test_a_baseline_only_param_is_still_kept(self):
        self.assertIn("WP_RADIUS", dict(_generated()))


if __name__ == "__main__":
    unittest.main()
