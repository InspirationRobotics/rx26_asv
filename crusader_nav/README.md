# crusader_nav

Nav2 obstacle avoidance, planner side (`docs/nav2_avoidance_spec.md`). Pure Python: Nav2 is
needed to run `nav.launch.py`, not to build.

| Piece | What it does |
|---|---|
| `nav_frames_node` | Owns the datum: `/crsd/datum` (latched) and TF `map -> base_footprint` from `/crsd/pose`. `frames_core.py` says why there is one projection only |
| `nav_lifecycle` | Configures and activates `planner_server`, again after a respawn, every call with a timeout (its docstring: why not Nav2's lifecycle manager) |
| `config/nav2_params.yaml` | Nav2's parameters; `check_config.py` cross-checks them with `crusader_params.yaml` |
| `costmap_probe` | Console tool: lethal blobs in the costmap as lat/lon, range, bearing |

Before `bt_runner_node`, not in `core.launch.py` until it has run on the water:
`ros2 launch crusader_nav nav.launch.py datum_source:=param datum_lat:=<lat> datum_lon:=<lon>`.
`first_fix` is never respawned: a restart would move the map under a running mission.
