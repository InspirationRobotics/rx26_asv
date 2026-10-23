# crusader_perception

| Node | In | Out |
|---|---|---|
| `camera_node` | the OAK-D (`source: oak`) or `oak/rgb` + `oak/depth` (`source: topic`) | `/crsd/oak/detections` (model `buoy`), `/dock/observations` (model `dock`), `/crsd/camera/image/compressed`, `/crsd/camera/raw/compressed`, `/crsd/camera/health` |
| `lidar_node` | `/livox/lidar`, `/crsd/attitude`, `/crsd/pose`, `crsd/avoidance_enable` | `crsd/lidar_clusters`, `/crsd/nav/obstacle_cloud`, `crsd/obstacle_distance`, `/crsd/wall_range` (`wall_enable`), `/crsd/lidar/preview`, `/crsd/lidar/health` |

`model` (`none | buoy | dock`) and the 17 camera controls (`oak_controls.py`) are live
parameters: `ros2 param set /camera_node model dock`. The pictures are encoded only while
something subscribes.

Positions are REP-103 (x forward, y left, z up): detections in `camera_link`, LiDAR in
`base_link`. The maths is in the `*_core.py` modules, which need no ROS and have the tests.
camera_node needs depthai and ultralytics (the `asv` image); `core.launch.py` starts it, without
respawn, so the Nodes tab can stop it to free the OAK-D.
