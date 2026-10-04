"""lake_rig: the datum a START RIG uses, the environment it hands lake_rig_up.sh, what counts as
"up", and the one-at-a-time runner. No ROS, no real script: the runner is a fake."""
import os
import subprocess
import sys
import tempfile
import threading
import time
import unittest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "..", "crusader_common"))

from crusader_groundstation import lake_rig  # noqa: E402

BOAT = (32.9238460, -117.0385582)


def north_of(p, metres):
    return (p[0] + metres / 111320.0, p[1])


class ChooseDatum(unittest.TestCase):
    def test_no_field_takes_the_boat(self):
        lat, lon, why = lake_rig.choose_datum(BOAT, None)
        self.assertEqual((lat, lon), BOAT)
        self.assertIn("no field in progress", why)

    def test_a_near_field_keeps_its_datum_so_the_panel_restores_it(self):
        field = north_of(BOAT, 120.0)
        lat, lon, why = lake_rig.choose_datum(BOAT, field)
        self.assertEqual((lat, lon), field)
        self.assertIn("field in progress", why)
        self.assertIn("120 m", why)

    def test_a_far_field_is_another_site(self):
        lat, lon, why = lake_rig.choose_datum(BOAT, north_of(BOAT, 5000.0))
        self.assertEqual((lat, lon), BOAT)
        self.assertIn("5.0 km", why)

    def test_new_datum_here_wins_over_a_near_field(self):
        lat, lon, why = lake_rig.choose_datum(BOAT, north_of(BOAT, 10.0), new_here=True)
        self.assertEqual((lat, lon), BOAT)
        self.assertIn("new datum", why)

    def test_the_threshold_is_inclusive_and_configurable(self):
        field = north_of(BOAT, 50.0)
        self.assertEqual(lake_rig.choose_datum(BOAT, field, reuse_m=60.0)[:2], field)
        self.assertEqual(lake_rig.choose_datum(BOAT, field, reuse_m=40.0)[:2], BOAT)


class FieldDatum(unittest.TestCase):
    def write(self, text):
        f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        f.write(text)
        f.close()
        self.addCleanup(os.unlink, f.name)
        return f.name

    def test_reads_the_course_origin(self):
        try:
            import yaml  # noqa: F401
        except ImportError:
            self.skipTest("PyYAML not installed")
        p = self.write("name: autosave\norigin: {lat: 32.92384600, lon: -117.03855820}\nelements: []\n")
        self.assertEqual(lake_rig.field_datum(p), (32.923846, -117.0385582))

    def test_blanks_not_guesses(self):
        self.assertIsNone(lake_rig.field_datum("/nonexistent/autosave.yaml"))
        self.assertIsNone(lake_rig.field_datum(self.write("name: x\n")))
        self.assertIsNone(lake_rig.field_datum(self.write("origin: {lat: abc, lon: 1}\n")))
        self.assertIsNone(lake_rig.field_datum(self.write("origin: {lat: 95.0, lon: 1}\n")))
        self.assertIsNone(lake_rig.field_datum(self.write("origin: {lat: .nan, lon: 1}\n")))
        self.assertIsNone(lake_rig.field_datum(self.write(": : not yaml [\n")))


class RigEnv(unittest.TestCase):
    def test_sets_the_datum_and_only_the_switches_asked_for(self):
        base = {"PATH": "/bin", "POOL": "1", "PUBLISH": "1", "TREE": "x.xml", "NAV_MODE": "on",
                "LAKE_TUNING": "t.yaml", "LAKE_DATUM": "0,0"}
        env = lake_rig.rig_env(base, BOAT, pool=False, publish=False)
        self.assertEqual(env["PATH"], "/bin")
        self.assertEqual(env["LAKE_DATUM"], "32.923846000,-117.038558200")
        for k in ("POOL", "PUBLISH", "TREE", "NAV_MODE", "LAKE_TUNING"):
            self.assertNotIn(k, env)
        env = lake_rig.rig_env({}, BOAT, pool=True, publish=True)
        self.assertEqual((env["POOL"], env["PUBLISH"]), ("1", "1"))


class RigPids(unittest.TestCase):
    def setUp(self):
        self.proc = tempfile.mkdtemp()
        self.pids = os.path.join(self.proc, "pids")

    def fake_proc(self, pid, cmdline):
        os.makedirs(os.path.join(self.proc, str(pid)))
        with open(os.path.join(self.proc, str(pid), "cmdline"), "wb") as f:
            f.write(cmdline.replace(" ", "\0").encode())

    def test_only_live_processes_that_still_carry_their_name(self):
        self.fake_proc(101, "python3 -u -m crusader_sim.task1_panel --lake")
        self.fake_proc(102, "sleep 100")                       # a recycled pid
        with open(self.pids, "w") as f:
            f.write("task1_panel 101\nbt_runner 102\nrxl_link_node 103\ngarbage\n")
        self.assertEqual(lake_rig.rig_pids(self.pids, proc=self.proc), [("task1_panel", 101)])

    def test_no_pid_file_is_nothing_running(self):
        self.assertEqual(lake_rig.rig_pids(self.pids, proc=self.proc), [])


class Runner(unittest.TestCase):
    def rig(self, run):
        d = tempfile.mkdtemp()
        return lake_rig.LakeRig("/scripts", d, run=run)

    def wait_idle(self, rig):
        for _ in range(200):
            if not rig.view()["busy"]:
                return rig.view()
            time.sleep(0.01)
        self.fail("runner never finished")

    def test_start_runs_the_up_script_with_the_datum_and_keeps_the_result(self):
        seen = {}

        def run(cmd, **kw):
            seen.update(cmd=cmd, env=kw["env"], timeout=kw["timeout"])
            return subprocess.CompletedProcess(cmd, 0, stdout="=== plan ===\n\n  tree task1_global.xml\n")
        rig = self.rig(run)
        ok, msg = rig.start(BOAT, "the boat's position", pool=True, publish=False, base_env={})
        self.assertTrue(ok)
        self.assertIn("32.9238460, -117.0385582", msg)
        v = self.wait_idle(rig)
        self.assertEqual(seen["cmd"], ["bash", "/scripts/lake_rig_up.sh"])
        self.assertEqual(seen["env"]["POOL"], "1")
        self.assertNotIn("PUBLISH", seen["env"])
        self.assertTrue(v["last"]["ok"])
        self.assertEqual(v["last"]["tail"], ["=== plan ===", "  tree task1_global.xml"])
        self.assertEqual(v["last"]["datum"], [BOAT[0], BOAT[1]])

    def test_one_at_a_time(self):
        gate = threading.Event()

        def run(cmd, **kw):
            gate.wait(5)
            return subprocess.CompletedProcess(cmd, 0, stdout="")
        rig = self.rig(run)
        self.assertTrue(rig.start(BOAT, "x", False, False, base_env={})[0])
        ok, msg = rig.stop(base_env={})
        self.assertFalse(ok)
        self.assertIn("already starting", msg)
        gate.set()
        self.wait_idle(rig)
        self.assertTrue(rig.stop(base_env={})[0])
        self.wait_idle(rig)

    def test_a_failure_and_a_timeout_are_kept_with_their_output(self):
        rig = self.rig(lambda cmd, **kw: subprocess.CompletedProcess(cmd, 2, stdout="*** LAKE_DATUM bad\n"))
        rig.start(BOAT, "x", False, False, base_env={})
        v = self.wait_idle(rig)
        self.assertEqual((v["last"]["ok"], v["last"]["rc"]), (False, 2))
        self.assertIn("*** LAKE_DATUM bad", v["last"]["tail"])

        def slow(cmd, **kw):
            raise subprocess.TimeoutExpired(cmd, kw["timeout"], output=b"=== preflight ===\n")
        rig = self.rig(slow)
        rig.start(BOAT, "x", False, False, base_env={})
        v = self.wait_idle(rig)
        self.assertIsNone(v["last"]["rc"])
        self.assertEqual(v["last"]["tail"][0], "=== preflight ===")
        self.assertIn("did not finish", v["last"]["tail"][-1])

    def test_stop_runs_the_down_script_without_rig_variables(self):
        seen = {}

        def run(cmd, **kw):
            seen.update(cmd=cmd, env=kw["env"])
            return subprocess.CompletedProcess(cmd, 0, stdout="")
        rig = self.rig(run)
        rig.stop(base_env={"PUBLISH": "1", "HOME": "/root"})
        self.wait_idle(rig)
        self.assertEqual(seen["cmd"], ["bash", "/scripts/lake_rig_down.sh"])
        self.assertEqual(seen["env"], {"HOME": "/root"})


if __name__ == "__main__":
    unittest.main()
