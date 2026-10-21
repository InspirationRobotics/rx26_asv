# crusader_common

Three small modules every node imports. No nodes, no vehicle policy, no package of ours.

| Module | What it is |
|---|---|
| `node_main.py` | `run_node` (SIGTERM runs `destroy_node`, exit 143), `declare_params` (plain ROS parameters: read-only unless named dynamic, ranges in the descriptor) and `on_param_set` |
| `stream_cache.py` | a value with a freshness budget: stale reads return nothing, never the last value |
| `geo.py` | lat/lon to local x/y, body to world rotations (yaw only, and full roll/pitch/yaw with its exact inverse). Read its header for the two body-axis conventions |

Parameters come from `crusader_bringup/config/crusader_params.yaml`, which the launch file
passes; no code here reads that file.
