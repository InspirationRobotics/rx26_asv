# `config/` — the ROS parameter source of truth

| File | What it is |
|---|---|
| `crusader_params.yaml` | Every ROS-side parameter for every node, in standard ROS 2 params format. Node code reads it for declaration defaults, so a code default cannot drift from the file the launch system loads. |

The ArduRover side of the boat's configuration is **not** here — it lives on the Pixhawk,
with the known-good baseline committed at `params/working_crusader.params` and enforced by
`tools/scripts/param_guard.py`.

## Parameter posture

Every parameter is declared with a `ParameterDescriptor` that marks it:

- **`[RO]` read_only** — safety or structural. `ros2 param set` is REJECTED. The change
  path is: edit this file, restart the node. No rebuild needed; params load at start.
- **`[DYN]` dynamic** — accepted at runtime, range-validated on set.

The whole **safety and telemetry** stack is `[RO]` — it is safety plumbing and none of it
has a test-day tuning knob. `[DYN]` appears only in the two perception/world-model nodes
that have not run on the boat yet (`lidar_cluster_node`, `target_tracker`, `map_server`),
and only on the gates a bench session exists to sweep: cluster thresholds, fusion and
association radii, decay timeouts. Their **structural** parameters — topic names, sensor
extrinsics, freshness budgets — stay `[RO]`, because a live-tunable mounting geometry
silently invalidates every position already published and there is no way to tell
afterwards which numbers produced which.

## Two hard format rules

`rcl_yaml_param_parser` is stricter than PyYAML and it fails CLOSED — an illegal file kills
every node in the launch at `rclpy.init()`, before any node code runs. This grounded the
whole stack (LEDs included) on 2026-07-29.

1. **No YAML anchors/aliases** (`&name` / `*name`). rcl is a token-level parser, not a
   document loader, and rejects them outright. Values that must stay equal across sections
   are written out literally and pinned by `tools/scripts/check_config.py`.
2. **Every top-level key must be a node name whose only child is `ros__parameters:`.**
   A bare scalar at top level produces "Cannot have a value before ros__parameters". This
   is why the `shared` block — documentation, not a real node — still carries a
   `ros__parameters:` level.

`tools/scripts/check_config.py` enforces both in CI, plus three cross-section guards that
catch the failures where nothing crashes:

- **A section for a node that no longer exists fails the build** — a stale parameter set
  reads as a capability the boat still has.
- **`shared.pose_timeout_s` is pinned across every section that carries a copy of it**, in
  both directions. A consumer trusting a pose for longer than `telemetry_bridge` vouches
  for it is the frozen-pose failure the value exists to prevent, and a comment saying "use
  the same number" cannot enforce itself.
- **Producer/consumer topic names are pinned against each other** (`buoy_detector` →
  `target_tracker`, `lidar_cluster_node` → `target_tracker`, `target_tracker` →
  `map_server`). A mismatch there starts both nodes cleanly, shows two plausible names in
  `ros2 topic list`, and delivers nothing — the most expensive kind of green.

## Change impact

| You changed | Re-run |
|---|---|
| any node's params | restart that node (no rebuild); `python3 tools/scripts/check_config.py` |
| `shared.pose_timeout_s` | update every literal copy the check names, or it fails |
| a topic name | update **both** sides — the check pins producer against consumer |
| added/removed a node section | update `CONFIG_DRIVEN_NODES` in `tools/scripts/check_config.py` |
| `target_tracker.cam_*` | every fused position shifts; see [crusader_world_model](../../crusader_world_model/README.md) |
