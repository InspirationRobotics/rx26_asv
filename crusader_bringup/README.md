# `crusader_bringup` — how the boat is started

Launch files and the single-source-of-truth parameter file. **Ships no code.**

| Path | What |
|---|---|
| `launch/core.launch.py` | The always-on stack: `telemetry_bridge`, `pixhawk_led_status_node`, `led_node`, `rc_watchdog` |
| `config/crusader_params.yaml` | Every ROS parameter for every node ([config/README.md](config/README.md)) |

```bash
ros2 launch crusader_bringup core.launch.py
```

## This package is the build entry point

It `exec_depend`s on every package we ship, so:

```bash
colcon build --packages-up-to crusader_bringup
```

builds the whole stack — and keeps doing so when a package is added, which an explicit
`--packages-select` list does not. `rebuild.sh`, `install_container.sh` and CI all use
that form.

The corollary: **a new package must be added to this `package.xml`'s exec_depends**, or it
silently stops being built by every one of those paths.

## The params file lives here, and one node reads it two ways

`config/crusader_params.yaml` is installed to `share/crusader_bringup/config/`. Two things
find it, and they must agree:

1. `launch/core.launch.py` passes the path explicitly as `parameters=[...]` — this is what
   sets each node's live values.
2. `crusader_common/config.py` resolves the same file via ament to supply *declaration
   defaults*, so a code default can never drift from the file the launch loads.

If those two ever point at different files, a node's declared default and its launched
value can disagree, and nothing will tell you until behaviour differs on the water. CI
asserts the ament resolution lands in `crusader_bringup`.

`$CRUSADER_PARAMS` overrides the lookup for bench work or a second boat.

## Adding a node to the launch

After it has run on the boat — not when it compiles. Add the params section, the
`exec_depend`, the `Node(...)` entry, and the node name to `CONFIG_DRIVEN_NODES` in
`tools/scripts/check_config.py` (which fails CI if the YAML and that list disagree in
either direction).
