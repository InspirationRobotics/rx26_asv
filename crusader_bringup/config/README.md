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

Everything in the file is currently `[RO]`. That is not an oversight: the whole stack is
safety plumbing, and none of it has a test-day tuning knob.

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

`tools/scripts/check_config.py` enforces both in CI, plus the reverse guard: a section for a
node that no longer exists fails the build, because a stale parameter set reads as a
capability the boat still has.

## Change impact

| You changed | Re-run |
|---|---|
| any node's params | restart that node (no rebuild); `python3 tools/scripts/check_config.py` |
| `shared.pose_timeout_s` | update every literal copy the test names, or the test fails |
| added/removed a node section | update `CONFIG_DRIVEN_NODES` in `tools/scripts/check_config.py` |
