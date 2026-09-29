"""paths — find this package's config and the boat's params file.

Installed (colcon) first, then the source tree, so the generators work both from
`ros2 run` and from a bare `python3 -m crusader_sim.gen_world` in a checkout.
"""
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_PKG = os.path.dirname(_HERE)                 # .../rx26_asv/crusader_sim
_SRC_REPO = os.path.dirname(_SRC_PKG)             # .../rx26_asv


def _share(pkg):
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:
        return None


def sim_share():
    """crusader_sim's config/, courses/ live here."""
    s = _share("crusader_sim")
    if s and os.path.isdir(os.path.join(s, "config")):
        return s
    return _SRC_PKG


def default_hull_yaml():
    return os.path.join(sim_share(), "config", "crusader_hull.yaml")


def default_params_yaml():
    """The boat's params file — the one crusader_bringup installs, i.e. the one
    the nodes under test actually read."""
    s = _share("crusader_bringup")
    if s:
        p = os.path.join(s, "config", "crusader_params.yaml")
        if os.path.isfile(p):
            return p
    return os.path.join(_SRC_REPO, "crusader_bringup", "config", "crusader_params.yaml")


def courses_dir():
    return os.path.join(sim_share(), "courses")


def generated_dir():
    """Where generated models/worlds go. Outside the source tree on purpose:
    generated files are outputs, not things to commit."""
    d = os.environ.get("CRUSADER_SIM_GEN", os.path.expanduser("~/.cache/crusader_sim"))
    os.makedirs(d, exist_ok=True)
    return d
