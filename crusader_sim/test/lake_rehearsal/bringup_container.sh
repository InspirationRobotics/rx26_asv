#!/usr/bin/env bash
# bringup_container.sh — in crsd-sim, after lake_rig_up.sh: what the rig really started and what the running nodes say.
# Read-only: it asks, it changes nothing. The rehearsal's `bringup` scenario (reh_run.sh) runs it, then bringup_check.py.
source /opt/ros/humble/setup.bash
source /root/robotx_ws/install/setup.bash
echo "--- the bt_runner's own parameters (ros2 param get): the overlay, and the nav mode"
for k in nav_orbit_radius_m nav_orbit_points nav_orbit_tolerance_m nav_soft_m nav_mode tree_file publish_setpoints; do
  printf '  %-26s' "$k"; ros2 param get /bt_runner_node "$k" --no-daemon 2>&1 | head -1
done
echo "--- target_tracker"
printf '  %-26s' "use_lidar"; ros2 param get /target_tracker use_lidar --no-daemon 2>&1 | head -1
echo "--- nav / planner processes (zombies and this grep excluded)"
found=0
for p in $(pgrep -f 'planner_server|nav_frames_node|nav_lifecycle|crusader_nav|nav2_|ros2 launch crusader_nav'); do
  [ -s /proc/$p/cmdline ] && { echo "  pid $p: $(tr '\0' ' ' < /proc/$p/cmdline | cut -c1-110)"; found=1; }
done
[ "$found" = 0 ] && echo "  none"
echo "--- nodes"
ros2 node list --no-daemon 2>/dev/null | sort | sed 's/^/  /'
echo "--- lifecycle: is there a /planner_server?"
ros2 lifecycle get /planner_server --no-daemon 2>&1 | head -1 | sed 's/^/  /'
