"""params_yaml: the GCS's "Set + save" writes values into the params YAML in place."""
import os

import pytest
import yaml

from crusader_groundstation import params_yaml as py

DOC = """# header comment
/**:
  ros__parameters:
    use_sim_time: false

target_tracker:
  ros__parameters:
    use_lidar: true                       # [DYN] false = camera-only
    gate_m: 2.5                            # association gate
    mode: "buoy"
    roi: [0.15, 0.85, 0.65, 1.25]
    # a comment line inside the section
    max_tracks: 6

# ---- the next node ----
bt_runner_node:
  ros__parameters:
    dock_heading_deg: -1.0
"""


def test_value_replaced_and_comment_kept_at_its_column():
    new, done = py.set_values(DOC, "target_tracker", {"use_lidar": False})
    assert done == [("use_lidar", "changed")]
    old_line = next(ln for ln in DOC.split("\n") if "use_lidar" in ln)
    new_line = next(ln for ln in new.split("\n") if "use_lidar" in ln)
    assert new_line.startswith("    use_lidar: false")
    assert new_line.index("#") == old_line.index("#")
    assert yaml.safe_load(new)["target_tracker"]["ros__parameters"]["use_lidar"] is False


def test_missing_param_added_at_the_end_of_its_own_section():
    new, done = py.set_values(DOC, "target_tracker", {"min_hits": 3})
    assert done == [("min_hits", "added")]
    doc = yaml.safe_load(new)
    assert doc["target_tracker"]["ros__parameters"]["min_hits"] == 3
    assert "min_hits" not in doc["bt_runner_node"]["ros__parameters"]


def test_types_survive_the_round_trip():
    vals = {"gate_m": py.coerce(3, "double"), "mode": "dock", "roi": py.coerce([0, 1, 0.5, 1.5], "double_array"),
            "max_tracks": py.coerce(8.0, "integer")}
    new, _ = py.set_values(DOC, "target_tracker", vals)
    got = yaml.safe_load(new)["target_tracker"]["ros__parameters"]
    assert got["gate_m"] == 3.0 and isinstance(got["gate_m"], float)
    assert got["mode"] == "dock"
    assert got["roi"] == [0.0, 1.0, 0.5, 1.5] and all(isinstance(v, float) for v in got["roi"])
    assert got["max_tracks"] == 8 and isinstance(got["max_tracks"], int)


def test_small_doubles_stay_doubles():
    assert yaml.safe_load(py.fmt(1e-05)) == 1e-05
    assert isinstance(yaml.safe_load(py.fmt(70.0)), float)


def test_refusals():
    with pytest.raises(ValueError):
        py.set_values(DOC, "no_such_node", {"x": 1})
    with pytest.raises(ValueError):
        py.set_values(DOC, "target_tracker", {"a.b": 1})
    with pytest.raises(ValueError):
        py.coerce(1.5, "integer")
    with pytest.raises(ValueError):
        py.coerce("yes", "bool")
    with pytest.raises(ValueError):
        py.fmt([])


def test_save_writes_backs_up_and_keeps_mode(tmp_path):
    src = tmp_path / "crusader_params.yaml"
    src.write_text(DOC, encoding="utf-8")
    os.chmod(src, 0o664)
    link = tmp_path / "linked.yaml"
    try:
        os.symlink(src, link)
        target = link
    except (OSError, NotImplementedError):    # no symlinks for this user (Windows)
        target = src
    path, done, backup = py.save(str(target), "/target_tracker", {"use_lidar": False, "gate_m": 3},
                                 types={"use_lidar": "bool", "gate_m": "double"},
                                 backup_dir=str(tmp_path / "backups"), now=0)
    assert os.path.realpath(path) == os.path.realpath(src)
    assert os.path.dirname(backup) == str(tmp_path / "backups")
    assert dict(done) == {"use_lidar": "changed", "gate_m": "changed"}
    got = yaml.safe_load(src.read_text(encoding="utf-8"))["target_tracker"]["ros__parameters"]
    assert got["use_lidar"] is False and got["gate_m"] == 3.0
    assert len(os.listdir(tmp_path / "backups")) == 1
    assert (tmp_path / "backups" / os.listdir(tmp_path / "backups")[0]).read_text(encoding="utf-8") == DOC
    assert "# [DYN] false = camera-only" in src.read_text(encoding="utf-8")


def test_saving_the_same_value_writes_nothing(tmp_path):
    src = tmp_path / "p.yaml"
    src.write_text(DOC, encoding="utf-8")
    _, done, backup = py.save(str(src), "target_tracker", {"use_lidar": True, "mode": "buoy"},
                              types={"use_lidar": "bool", "mode": "string"},
                              backup_dir=str(tmp_path / "b"))
    assert done == [("use_lidar", "unchanged"), ("mode", "unchanged")]
    assert backup is None and not (tmp_path / "b").exists()


def test_an_equal_value_written_another_way_is_left_alone():
    doc = DOC.replace("gate_m: 2.5 ", "gate_m: 2.50").replace('mode: "buoy"', "mode: buoy")
    new, done = py.set_values(doc, "target_tracker", {"gate_m": 2.5, "mode": "buoy"})
    assert new == doc and dict(done) == {"gate_m": "unchanged", "mode": "unchanged"}
    _, done = py.set_values(DOC, "target_tracker", {"max_tracks": 6.0})     # 6 is not 6.0
    assert done == [("max_tracks", "changed")]


def test_crlf_file_stays_crlf(tmp_path):
    src = tmp_path / "p.yaml"
    src.write_bytes(DOC.replace("\n", "\r\n").encode())
    py.save(str(src), "target_tracker", {"use_lidar": False, "min_hits": 3},
            types={"use_lidar": "bool", "min_hits": "integer"}, backup_dir=False)
    raw = src.read_bytes().decode()
    assert raw.count("\n") == raw.count("\r\n") == DOC.count("\n") + 1
    got = yaml.safe_load(raw)["target_tracker"]["ros__parameters"]
    assert got["use_lidar"] is False and got["min_hits"] == 3


def test_backup_dir_is_outside_the_checkout():
    p = os.path.join(os.sep, "root", "robotx_ws", "src", "rx26_asv", "crusader_bringup", "config", "x.yaml")
    assert py.backup_dir_for(p) == os.path.join(os.sep, "root", "robotx_ws", "backups", "params")
