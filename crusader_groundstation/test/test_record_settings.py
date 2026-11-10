"""record_settings off-boat: the settings file and the auto-record state machine.

    python -m pytest crusader_groundstation/test/test_record_settings.py

Needs nothing outside the standard library (and pytest to run it that way).
"""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from crusader_groundstation import record_settings as rs  # noqa: E402

RUN, WAIT, IDLE = rs.RUNNING, rs.WAITING, rs.IDLE


# ---- settings ---------------------------------------------------------------------

def test_defaults_when_missing(tmp_path):
    st = rs.Store(str(tmp_path / "nope" / "r.json"))
    assert st.get() == {"auto": True, "frame_hz": 1.0, "camera": True, "topics": []}
    assert st.warning == ""


def test_set_persists_and_reloads(tmp_path):
    path = str(tmp_path / "sub" / "r.json")
    saved, err = rs.Store(path).set({"auto": False, "frame_hz": 5, "topics": ["/a", "/a", "/b"]})
    assert err == "" and saved["topics"] == ["/a", "/b"] and saved["camera"] is True
    assert rs.Store(path).get() == saved
    assert not os.path.exists(path + ".tmp")


@pytest.mark.parametrize("bad", [
    {"frame_hz": 0.01}, {"frame_hz": 31}, {"frame_hz": "1"}, {"frame_hz": True},
    {"auto": "yes"}, {"camera": 1}, {"topics": "/a"}, {"topics": [1]}, {"topics": [""]}, [],
])
def test_invalid_refused_and_not_saved(tmp_path, bad):
    st = rs.Store(str(tmp_path / "r.json"))
    settings, err = st.set(bad)
    assert settings is None and err
    assert not os.path.exists(st.path) and st.get() == rs.DEFAULTS


@pytest.mark.parametrize("content", ["{not json", "[1, 2]", json.dumps({"frame_hz": 99})])
def test_corrupt_file_gives_defaults_and_warning(tmp_path, content):
    path = tmp_path / "r.json"
    path.write_text(content)
    st = rs.Store(str(path))
    assert st.get() == rs.DEFAULTS and st.warning


def test_store_default_is_not_shared(tmp_path):
    a = rs.Store(str(tmp_path / "a.json"))
    a.get()["topics"].append("/x")
    assert rs.Store(str(tmp_path / "b.json")).get()["topics"] == []


# ---- state machine ----------------------------------------------------------------

class Rig:
    """The bridge's side: carries out actions on a fake recorder."""

    def __init__(self, auto_on=True):
        self.auto, self.auto_on, self.rec, self.n, self.log = rs.AutoRecorder(), auto_on, None, 0, []

    def tick(self, t, state, name):
        for action, mission in self.auto.update(t, state, name, self.rec, self.auto_on):
            self.log.append((action, mission))
            if action == "stop":
                self.rec = None
            else:
                self.n += 1
                self.rec = f"s{self.n}"
                self.auto.started(self.rec, mission)


def test_starts_on_running_not_on_waiting():
    r = Rig()
    r.auto.mission_started("m", 0)
    r.tick(1, IDLE, None)
    r.tick(2, WAIT, "m")
    r.tick(3, WAIT, "m")
    assert r.log == [] and r.auto.active is None
    r.tick(4, RUN, "m")
    assert r.log == [("start", "m")] and r.auto.active == "m"
    r.tick(5, RUN, "m")
    assert r.log == [("start", "m")]


def test_stops_when_mission_leaves_running():
    r = Rig()
    r.auto.mission_started("m", 0)
    r.tick(1, RUN, "m")
    r.tick(2, IDLE, None)
    assert r.log == [("start", "m"), ("stop", "m")] and r.rec is None and r.auto.active is None
    r.tick(3, IDLE, None)
    assert len(r.log) == 2


def test_stops_when_bt_status_goes_stale():
    r = Rig()
    r.auto.mission_started("m", 0)
    r.tick(1, RUN, "m")
    r.tick(2, None, None)               # bt_runner gone: the cache aged out
    assert r.log[-1] == ("stop", "m")


def test_cancel_while_waiting_never_records():
    r = Rig()
    r.auto.mission_started("m", 0)
    r.tick(1, WAIT, "m")
    r.tick(2, IDLE, None)
    r.tick(3, RUN, "m")                 # a later run of the same name sent elsewhere
    assert r.log == []


def test_not_a_mission_tab_goal_never_records():
    r = Rig()                           # mission_started never called (Task 1 / Task 3 panel)
    r.tick(1, WAIT, "task1")
    r.tick(2, RUN, "task1")
    assert r.log == []


def test_other_mission_running_does_not_start():
    r = Rig()
    r.auto.mission_started("m", 0)
    r.tick(1, RUN, "other")
    assert r.log == []


def test_stale_idle_before_the_goal_shows_does_not_forget_it():
    r = Rig()
    r.auto.mission_started("m", 0)
    r.tick(0.5, IDLE, None)             # bt_status not yet updated
    r.tick(1, RUN, "m")
    assert r.log == [("start", "m")]


def test_unseen_start_is_forgotten():
    r = Rig()
    r.auto.mission_started("m", 0)
    r.tick(rs.PENDING_S + 1, IDLE, None)
    r.tick(rs.PENDING_S + 2, RUN, "m")
    assert r.log == []


def test_auto_off():
    r = Rig(auto_on=False)
    r.auto.mission_started("m", 0)
    r.tick(1, RUN, "m")
    assert r.log == [] and r.auto.active is None


def test_manual_recording_untouched():
    r = Rig()
    r.rec = "manual"
    r.auto.mission_started("m", 0)
    r.tick(1, RUN, "m")
    r.tick(2, IDLE, None)
    assert r.log == [] and r.rec == "manual"


def test_operator_stop_is_not_undone():
    r = Rig()
    r.auto.mission_started("m", 0)
    r.tick(1, RUN, "m")
    r.rec = None                        # the operator pressed Stop
    r.tick(2, RUN, "m")
    r.tick(3, RUN, "m")
    assert r.log == [("start", "m")] and r.auto.active is None
    r.tick(4, IDLE, None)
    assert r.log == [("start", "m")]


def test_operator_stop_then_own_recording_is_not_stopped():
    r = Rig()
    r.auto.mission_started("m", 0)
    r.tick(1, RUN, "m")
    r.rec = "mine"                      # stopped and restarted by hand between two ticks
    r.tick(2, IDLE, None)
    assert r.log == [("start", "m")] and r.rec == "mine"


def test_manual_start_takes_over_the_auto_recording():
    r = Rig()
    r.auto.mission_started("m", 0)
    r.tick(1, RUN, "m")
    r.auto.manual_start()               # the operator pressed Start (adds a bag, say)
    r.tick(2, IDLE, None)
    assert r.log == [("start", "m")] and r.rec == "s1"


def test_new_mission_while_previous_auto_recording_active_restarts_cleanly():
    r = Rig()
    r.auto.mission_started("a", 0)
    r.tick(1, RUN, "a")
    r.auto.mission_started("b", 2)
    r.tick(3, RUN, "b")                 # a's IDLE never seen
    assert r.log == [("start", "a"), ("stop", "a"), ("start", "b")]
    assert r.rec == "s2" and r.auto.active == "b"


def test_start_failure_is_not_retried_every_tick():
    auto = rs.AutoRecorder()
    auto.mission_started("m", 0)
    assert auto.update(1, RUN, "m", None, True) == [("start", "m")]
    auto.start_failed()
    assert auto.update(2, RUN, "m", None, True) == []
