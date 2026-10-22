# crusader_sim

Crusader in Gazebo Harmonic: real ArduRover SITL on the boat's parameters, simulated MID360 and
OAK-D, the RobotX courses (`courses/*.yaml`), and the boat's own nodes on its params file
(telemetry_bridge, lidar_node, camera_node, rfd_link_node, target_tracker, gcs_bridge,
bt_runner_node; Nav2 when the `crsd-sim:nav2` image has it).

| Run | What it does |
|---|---|
| `scripts/GZ_SIM_UP.cmd [course]`, `GZ_SIM_DOWN.cmd` | Windows: the whole sim, the ground station (`BOAT_GUI.cmd --boat 127.0.0.1`), then Task 1 |
| `scripts/gz_sim_up.sh` | WSL, with flags: its header lists them (`--no-uav`, `--tuning`, `--radio-endpoint`, `NAV_MODE`, `SIM_DETECTOR`, `TREE`) |
| `scripts/gz_task1.sh [course]` | arm, GUIDED, RunMission (`TREE`, default `task1_advanced`), then `task1_judge`'s verdict on ground truth. `gz_nav_test.sh`, `gz_nav_extra.sh`: the Nav2 avoidance runs |

Sim-only nodes: `livox_shim` (gz cloud to `/livox/lidar`), `sim_camera` (`oak/rgb`, `oak/depth`,
and oracle or real-YOLO detections), `sim_uav` (Ekko's TUNNEL v3 map of the course), `sim_transmitter` (RC).
