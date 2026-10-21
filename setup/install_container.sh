#!/usr/bin/env bash
# setup/install_container.sh — build and smoke-test the stack inside `asv`.
#
# The one build path: tools/scripts/rebuild.sh runs this from the host. It installs
# NOTHING: runtime dependencies live in the image (Dockerfile), pinned, because an
# unpinned `pip install` from here once resolved a dependency out from under the stack.
#
#     docker exec -it asv bash /root/robotx_ws/src/rx26_asv/setup/install_container.sh
set -euo pipefail

# This repo is ONE package source inside a colcon workspace, not the workspace root.
REPO="$(cd "$(dirname "$0")/.." && pwd)"      # …/src/rx26_asv
WS="${WS:-$(cd "$REPO/../.." && pwd)}"        # …/robotx_ws
if [[ ! -d "$WS/src" ]]; then
  echo "ERROR: '$WS' is not a colcon workspace (no src/). Clone this repo to" >&2
  echo "       <workspace>/src/rx26_asv, or run with WS=/path/to/workspace." >&2
  exit 1
fi

echo "== [1/3] dependency guard (the image supplies these) =="
missing=0
for mod in yaml serial pymavlink numpy cv2 torch; do
  python3 -c "import $mod" 2>/dev/null || { echo "   MISSING: $mod" >&2; missing=1; }
done
if [[ "$missing" -ne 0 ]]; then
  echo "ERROR: rebuild the image (docker build -t asv .) rather than pip-installing here." >&2
  exit 1
fi

echo "== [2/3] colcon build =="
# --packages-up-to crusader_bringup: the workspace may hold other sources whose build is
# not ours, and bringup exec_depends on every package we ship. ROS is sourced explicitly:
# a non-interactive shell never reads ~/.bashrc, and ament_cmake then dies with
# "No module named 'ament_package'" in a package that did not change.
cd "$WS"
set +u; source /opt/ros/humble/setup.bash; set -u
colcon build --symlink-install --packages-up-to crusader_bringup
# --symlink-install leaves a dangling link for every installed file since deleted, and a
# deleted tree would still be offered as a mission.
find install/crusader_*/share -xtype l -print -delete
set +u; source install/setup.bash; set -u

echo "== [3/3] smoke =="
python3 "$REPO/tools/scripts/check_entry_points.py"
ros2 run crusader_bt bt_tree_check install/crusader_bt/share/crusader_bt/behavior_trees/*.xml
test -f install/crusader_bringup/share/crusader_bringup/config/crusader_params.yaml
# camera_node imports depthai only when it opens the camera; check it here, not on the boat.
python3 -c "import depthai; print('depthai', depthai.__version__)" \
  || echo "WARNING: no depthai: camera_node cannot open the OAK-D. Rebuild the asv image."
echo "== done. Restart the nodes (or the launch) for changes to take effect. =="
