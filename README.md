# RobotX 2026: Crusader

Team Inspiration's code for the 2026 RobotX competition's ASV, **Crusader**: a holonomic
4×T200 boat on ArduRover/Pixhawk with a Jetson Orin Nano running ROS 2 Humble in the `asv`
container. Before your first day, follow [setup/README.md](setup/README.md); before writing
code, read [Standards](#standards) and the [safety constraints](#safety-constraints).

## Packages

All but `crusader_sim` (laptop only) build on the boat with
`colcon build --packages-up-to crusader_bringup`. Each package README says what it runs and
on which topics.

| Package | Contains |
|---|---|
| [`crusader_msgs`](crusader_msgs/README.md) | the typed contracts: messages and the RunMission action |
| [`crusader_common`](crusader_common/README.md) | `declare_params`, `run_node`, `StreamCache`, `geo`. No nodes |
| [`crusader_behavior`](crusader_behavior/README.md) | `telemetry_bridge` (the only MAVLink speaker), `rc_watchdog`, `led_status_node`, `led_node` |
| [`crusader_perception`](crusader_perception/README.md) | `camera_node` (the OAK-D: buoy or dock model), `lidar_node` (the MID360: clusters, the Nav2 cloud, proximity, the Task 3 wall) |
| [`crusader_world_model`](crusader_world_model/README.md) | `target_tracker`: detections and clusters to earth-anchored tracks |
| [`crusader_bt`](crusader_bt/README.md) | `bt_runner_node`: runs the built-in tree a RunMission goal names |
| [`crusader_link`](crusader_link/README.md) | `rfd_link_node`: the RFD900 mesh to Ekko, the UAV (TUNNEL v3) |
| [`crusader_nav`](crusader_nav/README.md), [`crusader_nav_layers`](crusader_nav_layers/README.md) | Nav2 avoidance: the datum and TF, `planner_server`, the known-hazard costmap layer |
| [`crusader_groundstation`](crusader_groundstation/README.md) | `gcs_bridge` on the boat, `gcs_app` on the laptop, `ocs_client` to the OCS |
| [`crusader_bringup`](crusader_bringup/README.md) | `core.launch.py` and `crusader_params.yaml`; the build entry point |
| [`crusader_sim`](crusader_sim/README.md) | Gazebo Harmonic + ArduRover SITL with the boat's own nodes |

Beside them: `tools/` (config guards, preflight, param_guard, rebuild, squirt_cal, SITL
checks, udev, systemd), `scripts/` (MAVProxy and Livox start), `setup/` (per machine),
`params/` (the ArduRover baseline), `firmware/` (the LED Arduino), `docs/` (an archive).

The repo is one source dir in a colcon workspace (`~/robotx_ws/src/rx26_asv` on the Jetson),
never the workspace: the root is not a package, so colcon finds every package below it.

## Running

| What | How |
|---|---|
| The always-on stack | `ros2 launch crusader_bringup core.launch.py` (systemd `crsd-ros` on the boat) |
| The ground station | `tools/scripts/BOAT_GUI.cmd` on the laptop, then http://localhost:8090 |
| Anything else (camera_node, target_tracker, bt_runner_node, ocs_client) | the Nodes tab |
| Nav2 avoidance | `ros2 launch crusader_nav nav.launch.py datum_source:=param ...` |
| A mission | the Mission tab, or `ros2 action send_goal /crsd/run_mission crusader_msgs/action/RunMission "{name: task1_advanced}"` |
| Rebuild after any edit | `tools/scripts/rebuild.sh` on the Jetson host |
| Before every session | `tools/scripts/preflight.py` |

MAVProxy owns the Pixhawk serial link and is started by systemd outside ROS
(`scripts/start_mavproxy.sh`); nothing else opens `/dev/crsd-pixhawk`. The MID360 driver runs
in the separate `livox` container and publishes `/livox/lidar`.

## Safety constraints

1. **One Pixhawk owner.** MAVProxy holds the serial link; `telemetry_bridge` consumes its UDP
   rebroadcast and refuses a serial endpoint. No other node opens a MAVLink connection.
2. **The RC e-stop is the only safety path.** The SB switch kills the motors in any mode.
   WiFi, SSH and the ground station are conveniences, never safety mechanisms.
3. **Moving and squirting are gated.** `bt_runner_node`'s `publish_setpoints` (Gate G1) and
   `fire_pump` (Gate G7) are false in the params file.
4. **Steering inversion is fixed in `SERVOx_*`, never `RCx_REVERSED`**: autonomous modes
   bypass RC settings. `COMPASS_USE=0` stays.
5. **Never loosen `ARMING_*` or safety params to make testing easier.** `param_guard.py`
   hard-fails on the protected set.
6. **Fail loudly, and keep silence silent.** A dead stream stops being republished rather than
   being replayed with a fresh timestamp.

## Standards

1. `core.launch.py` holds only what has run on the boat.
2. A node's logic lives in a ROS-free `*_core.py` (or header) with tests; the node is a thin
   wrapper. Safety rules in particular belong in the core.
3. A node declares every parameter, with its default, in code (`PARAMS` +
   `crusader_common.node_main.declare_params`); `crusader_params.yaml` lists only what this
   boat sets. `tools/scripts/check_config.py` fails a key no node declares or of the wrong type.
4. Imports are absolute. `crusader_common` imports no package of ours.
5. A file starts with what it is for; comments say why, not what. A package README stays under
   15 lines.
6. CI (`.github/workflows/ci.yml`): check_config, shell and Python syntax, the header-only C++
   tests, then a full `colcon build`, every entry point importing, `colcon test` (every
   built-in tree loads) and `pytest`.

## Dependencies

Hardware: Jetson Orin Nano; Pixhawk (ArduRover 4.6.3, `FRAME_TYPE=2` OmniX); 4×T200; RTK GPS
with dual-antenna heading (compass off by design); ELRS RC; Ubiquiti 5.8 GHz and an RFD900
mesh to Ekko; OAK-D LR; Livox MID360; the LED mast (RED e-stop, YELLOW manual, GREEN
autonomous).

Software: JetPack 6; the `asv` image (`Dockerfile`: ROS 2 Humble, CUDA/TensorRT, depthai,
ultralytics, MAVProxy, pymavlink) on `ultralytics/ultralytics:latest-jetson-jetpack6`. The
laptop needs Python 3.10+ only for the ground station.

Versions (each `package.xml`): minor for a confirmed change in one area, major for core
changes; `1.0.0` at competition freeze.
