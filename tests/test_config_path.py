"""Config-path resolution across the source and installed layouts.

Every node loads crusader_params.yaml for its declaration defaults, so if this
path is wrong nothing starts. It was wrong: the module computed the path with
`parents[3]`, which lands on the package dir in the source tree but on
site-packages/ in the install space — lib/ and share/ are siblings there, so no
fixed parent count reaches the config. Tests all ran from the source tree, so
CI was green while every node on the boat died in __init__ with FileNotFoundError.

These tests pin the resolution ORDER rather than any one path, since the failure
was specifically about which layout gets assumed.
"""
from pathlib import Path

from rx26_asv.api.common import config as crsd_config


def test_default_path_resolves_to_a_real_file():
    """Whatever layout the tests run in, the resolved path must exist."""
    assert crsd_config.DEFAULT_CONFIG_PATH.is_file(), (
        f"config not found at {crsd_config.DEFAULT_CONFIG_PATH}")


def test_default_path_actually_loads():
    """Resolution is not enough — it has to parse and carry node sections."""
    cfg = crsd_config.load()
    assert "led_node" in cfg, "resolved a file, but not crusader_params.yaml"


def test_installed_share_dir_wins_when_ament_can_find_it(tmp_path):
    """In the install space the share/ copy must win over the source path."""
    share = tmp_path / "share" / "rx26_asv"
    (share / "config").mkdir(parents=True)
    installed = share / "config" / "crusader_params.yaml"
    installed.write_text("led_node:\n  ros__parameters:\n    port: /dev/null\n")

    resolved = crsd_config._resolve_config_path(lambda pkg: str(share))
    assert resolved == installed, (
        "ament-reported share dir must take priority over the source tree")


def test_falls_back_to_source_when_share_dir_has_no_config(tmp_path):
    """An empty/wrong share dir must not shadow a working source checkout."""
    resolved = crsd_config._resolve_config_path(lambda pkg: str(tmp_path))
    assert resolved == crsd_config._SOURCE_CONFIG_PATH


def test_falls_back_when_ament_raises():
    """Unsourced workspace raises PackageNotFoundError — degrade, don't crash."""
    def boom(pkg):
        raise RuntimeError("PackageNotFoundError: rx26_asv")

    assert crsd_config._resolve_config_path(boom) == crsd_config._SOURCE_CONFIG_PATH


def test_missing_file_error_names_both_layouts():
    """The bare errno message cost a debugging session; it must self-explain."""
    try:
        crsd_config.load(Path("/nonexistent/crusader_params.yaml"))
    except FileNotFoundError as e:
        msg = str(e)
        assert "installed layout" in msg and "source layout" in msg, msg
    else:
        raise AssertionError("expected FileNotFoundError")
