"""param_save on the real crusader_params.yaml (a copy): edits keep the file's
comments and every other line, and read back.

    python crusader_groundstation/test/test_param_save.py
"""
import os
import sys
import tempfile
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from crusader_groundstation import param_save as ps   # noqa: E402

REAL = os.path.join(HERE, "..", "..", "crusader_bringup", "config", "crusader_params.yaml")


class TestFormat(unittest.TestCase):

    def test_values(self):
        self.assertEqual(ps.format_value(True), "true")
        self.assertEqual(ps.format_value(3), "3")
        self.assertEqual(ps.format_value(150.0), "150.0")
        self.assertEqual(ps.format_value(0.1), "0.1")
        self.assertEqual(ps.format_value("a b"), '"a b"')
        with self.assertRaises(ValueError):
            ps.format_value(float("nan"))


class TestEdit(unittest.TestCase):

    def setUp(self):
        with open(REAL, encoding="utf-8") as f:
            self.text = f.read().replace("\r\n", "\n")

    def test_change_keeps_the_comment_and_every_other_line(self):
        new, how = ps.apply(self.text, "cannon_aim_node", {"tilt_trim_lr_deg": 12.5})
        self.assertEqual(how, {"tilt_trim_lr_deg": "changed"})
        a, b = self.text.split("\n"), new.split("\n")
        self.assertEqual(len(a), len(b))
        diff = [(x, y) for x, y in zip(a, b) if x != y]
        self.assertEqual(len(diff), 1)
        self.assertIn("tilt_trim_lr_deg: 12.5", diff[0][1])
        self.assertIn("# [DYN] lower-right", diff[0][1])
        self.assertEqual(yaml.safe_load(new)["cannon_aim_node"]["ros__parameters"]["tilt_trim_lr_deg"], 12.5)

    def test_a_parameter_the_file_lacks_is_added_to_its_section(self):
        new, how = ps.apply(self.text, "bt_runner_node", {"strafe.kd_fwd": 150.0})
        self.assertEqual(how, {"strafe.kd_fwd": "added"})
        d = yaml.safe_load(new)
        self.assertEqual(d["bt_runner_node"]["ros__parameters"]["strafe.kd_fwd"], 150.0)
        self.assertEqual(len(new.split("\n")), len(self.text.split("\n")) + 1)
        # ...and nothing else in the file moved
        old = yaml.safe_load(self.text)
        old["bt_runner_node"]["ros__parameters"]["strafe.kd_fwd"] = 150.0
        self.assertEqual(old, d)

    def test_same_value_is_left_alone(self):
        new, how = ps.apply(self.text, "telemetry_bridge", {"pump_servo_channel": 10})
        self.assertEqual(how, {"pump_servo_channel": "same"})
        self.assertEqual(new, self.text)

    def test_no_section(self):
        with self.assertRaises(KeyError):
            ps.apply(self.text, "no_such_node", {"x": 1.0})

    def test_save_writes_and_backs_up(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "crusader_params.yaml")
            with open(p, "w", encoding="utf-8", newline="") as f:
                f.write(self.text)
            how = ps.save(p, "cannon_aim_node", {"pan_trim_ul_deg": -2.0}, os.path.join(d, "bk"))
            self.assertEqual(how, {"pan_trim_ul_deg": "changed"})
            with open(p, encoding="utf-8") as f:
                self.assertIn("pan_trim_ul_deg: -2.0", f.read())
            self.assertEqual(len(os.listdir(os.path.join(d, "bk"))), 1)


if __name__ == "__main__":
    unittest.main()
