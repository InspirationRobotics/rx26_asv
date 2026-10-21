# crusader_world_model

`target_tracker`: camera detections (`/crsd/oak/detections`), LiDAR clusters
(`crsd/lidar_clusters`) and `/crsd/pose` in; earth-anchored tracks out on `/crsd/world_targets`
(TrackedTargetArray, nearest first), which `bt_runner_node` (confirmed tracks only) and the
ground station's map read. Not in `core.launch.py`: start it from the Nodes tab.

Three stages, all in the ROS-free `target_tracker_core.py`: fuse each camera box with the LiDAR
cluster on its bearing, associate to a track within `assoc_radius_m`, vote the colour. Every
parameter and what to tune is in the node's PARAMS.

If the map fills with anonymous targets, check `use_lidar` first. Off (the default) is
camera-only, right in a pool or beside a dock, where an unlabelled cluster is the wall. Open
water: `use_lidar` true with lidar_node `r_max` 25-40. The camera orientation (`cam_yaw_deg`,
`cam_pitch_deg`) is assumed, not measured: everything fused is provisional until it is.
