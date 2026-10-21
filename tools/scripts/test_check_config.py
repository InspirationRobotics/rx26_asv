"""check_config's params-file rules, against edited copies of the real file. No ROS.

    python3 -m pytest tools/scripts/test_check_config.py
"""
import contextlib
import importlib.util
import io
import pathlib
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("check_config", HERE / "check_config.py")
cc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cc)
SHIPPED = cc.PARAMS_YAML.read_text(encoding="utf-8")


class ParamsFile(unittest.TestCase):

    def failures(self, text):
        with tempfile.TemporaryDirectory() as d:
            path = pathlib.Path(d) / "crusader_params.yaml"
            path.write_text(text, encoding="utf-8")
            cc.PARAMS_YAML, cc.failures[:] = path, []
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    cc.check_params_yaml()
            finally:
                cc.PARAMS_YAML = HERE.parents[1] / "crusader_bringup" / "config" / "crusader_params.yaml"
        return " | ".join(detail for _check, detail in cc.failures)

    def edited(self, old, new):
        self.assertIn(old, SHIPPED)
        return SHIPPED.replace(old, new, 1)

    def test_the_shipped_file_passes(self):
        self.assertEqual(self.failures(SHIPPED), "")

    def test_a_misspelt_key_fails(self):
        text = self.edited("    fcu_sysid: 2", "    fcu_sysd: 2")
        self.assertIn("telemetry_bridge.fcu_sysd: not declared", self.failures(text))

    def test_an_int_for_a_double_fails(self):
        text = self.edited("    lidar_x: 0.32", "    lidar_x: 1")
        self.assertIn("lidar_node.lidar_x is integer, declared double", self.failures(text))

    def test_a_cpp_parameter_is_typed_too(self):
        text = self.edited("    cam_x: 0.37\n    cam_y: 0.0\n    cam_yaw_deg",
                           "    cam_x: 0.37\n    cam_y: 0\n    cam_yaw_deg")
        self.assertIn("is integer, declared double", self.failures(text))

    def test_a_shared_value_set_again_fails(self):
        text = self.edited("    rfd_endpoint:", "    pose_timeout_s: 1.0\n    rfd_endpoint:")
        self.assertIn("rfd_link_node.pose_timeout_s sets the /** value again", self.failures(text))

    def test_the_pose_age_must_match_the_stream_timeouts(self):
        text = self.edited("    pose_timeout_s: 1.0", "    pose_timeout_s: 1.5")
        self.assertIn("/**.pose_timeout_s = 1.5", self.failures(text))


if __name__ == "__main__":
    unittest.main()
