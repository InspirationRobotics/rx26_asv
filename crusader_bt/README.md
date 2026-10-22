# crusader_bt

Cognition: `bt_runner_node` serves `/crsd/run_mission` (RunMission) and ticks the built-in
tree the goal names, `behavior_trees/<name>.xml` (`task1_advanced`, `task1_disruptive`,
`task3_core`, `task3_advanced`, `task3_disruptive`, ...). Goals never carry XML. A tier belongs
to the tree, not the goal: one tree per task per tier. Each top-level file includes the body its
tiers share (`lib/common/`) once, then its own `lib/<name>.xml`.

Topics: in `/crsd/pose`, `/crsd/fcu_status`, `/crsd/world_targets`, `/crsd/passage_plan`,
`dock/observations`, `/crsd/ocs_command`, `/crsd/datum`; out `/crsd/bt_status` (JSON),
`/crsd/robocommand/report`, `/crsd/current_task`, `/crsd/guided_setpoint` (only with
`publish_setpoints`), `/crsd/nav/hazards`, `/crsd/nav/leg_status`.

The geometry is ROS-free and header-only (`nav_math`, `dock_math`, `fire_math`, `path_math`,
`global_passage`); `colcon test` runs its tests and loads every built-in tree with
`bt_tree_check`. Check a tree by hand: `ros2 run crusader_bt bt_tree_check behavior_trees/*.xml`.
