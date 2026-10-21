# crusader_bringup

How the boat starts; no code. `launch/core.launch.py` is the always-on stack (its docstring
lists the nodes and why each is there); `config/crusader_params.yaml` is what this boat sets
(its header has the rules, and `tools/scripts/check_config.py` enforces them).

    ros2 launch crusader_bringup core.launch.py

It `exec_depend`s on every package we ship, so `colcon build --packages-up-to crusader_bringup`
(rebuild.sh, install_container.sh, CI) is the whole stack. A new package must be added to
`package.xml`, or every one of those paths silently stops building it.
