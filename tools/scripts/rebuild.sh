#!/usr/bin/env bash
# The one blessed rebuild path for Crusader code changes.
# The container COPIES files at build time — every edit requires this before it
# takes effect. "The change did nothing" almost always means this was skipped.
set -euo pipefail

CONTAINER="${CRSD_CONTAINER:-asv}"
# Colcon WORKSPACE root inside the container (not the repo root — this repo is
# cloned to $WS/src/rx26_asv alongside any other package sources).
WS="${WS:-/root/robotx_ws}"

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
  echo "ERROR: '$CONTAINER' container is not running." >&2
  exit 1
fi

echo "== colcon build inside $CONTAINER =="
# --packages-up-to crusader_bringup, not a bare build: the workspace may hold
# other package sources (the robotx_2026 boat repo, the livox container's
# sources) whose build state is not ours to change and whose build failure must
# not block ours. crusader_bringup exec_depends on every package we ship, so
# "up-to" is the whole stack — and it stays correct when a package is added,
# which an explicit --packages-select list does not.
#
# ROS IS SOURCED EXPLICITLY, not left to `bash -lc`. A non-interactive login
# shell reads /etc/profile and ~/.bash_profile; ROS setup conventionally lands
# in ~/.bashrc, which such a shell never reads. scripts/start_livox.sh carries
# the same comment because the same assumption cost a debugging session there
# ("ros2: command not found" from a container where `docker exec -it ... bash`
# then `ros2` works perfectly by hand).
#
# It bites differently here and worse. colcon is usually still on PATH, so the
# build STARTS — and then ament_cmake's own cmake scripts invoke /usr/bin/python3
# directly, that interpreter has no ROS site-packages on its path, and the
# failure surfaces as:
#     ModuleNotFoundError: No module named 'ament_package'
# from a template script nobody has heard of, in a package that did not change.
# It reads like a broken toolchain rather than an unsourced shell.
SETUPS="${CRSD_SETUP:-/opt/ros/humble/setup.bash $WS/install/setup.bash}"
SOURCE_CMD=""
for s in $SETUPS; do
  # Sourced only if present, so one list covers the pre-first-build case (no
  # install/ yet) and every later one. `set -u` is off inside the sourced
  # scripts' scope because ROS setup files reference unset vars by design.
  SOURCE_CMD="$SOURCE_CMD if [ -f $s ]; then set +u; . $s; set -u; fi;"
done

if ! docker exec "$CONTAINER" bash -c \
     "$SOURCE_CMD python3 -c 'import ament_package' 2>/dev/null"; then
  echo "ERROR: 'ament_package' is not importable inside '$CONTAINER' even after" >&2
  echo "       sourcing: $SETUPS" >&2
  echo "       Without it, EVERY ament_cmake package (crusader_msgs," >&2
  echo "       crusader_bringup) fails at cmake configure time." >&2
  echo >&2
  echo "  What is actually there:" >&2
  docker exec "$CONTAINER" bash -c \
    "$SOURCE_CMD echo '    python3: '\$(command -v python3); \
     echo '    colcon:  '\$(command -v colcon); \
     echo '    AMENT_PREFIX_PATH='\$AMENT_PREFIX_PATH; \
     echo '    PYTHONPATH='\$PYTHONPATH; \
     ls -d /opt/ros/*/ 2>/dev/null | sed 's/^/    distro: /'" >&2 || true
  echo >&2
  echo "  If a distro is listed above but ament_package is still missing, the" >&2
  echo "  package itself is absent from the image — install it in the container:" >&2
  echo "      docker exec $CONTAINER apt-get update" >&2
  echo "      docker exec $CONTAINER apt-get install -y python3-ament-package" >&2
  echo "  If NO distro is listed, this container has no ROS at all and" >&2
  echo "  CRSD_SETUP must name the right setup.bash." >&2
  exit 1
fi

docker exec "$CONTAINER" bash -c \
  "$SOURCE_CMD cd $WS && colcon build --symlink-install --packages-up-to crusader_bringup"

echo "== import smoke test =="
# Fail loudly if any package doesn't import — a silently-inactive mechanism is a
# safety issue on this boat, not a nuisance. Every package with code is named
# here: a build that succeeds while an import fails is the exact gap this closes.
docker exec "$CONTAINER" bash -c \
  "$SOURCE_CMD cd $WS && { set +u; . install/setup.bash; set -u; } && python3 -c '
import crusader_common, crusader_fcu, crusader_behavior
import crusader_perception, crusader_world_model, crusader_groundstation
from crusader_msgs.msg import Attitude, FcuStatus, LatLonHead, RcChannels
from crusader_msgs.msg import Detection3D, Detection3DArray
from crusader_msgs.msg import Cluster3D, Cluster3DArray
from crusader_msgs.msg import TrackedTarget, TrackedTargetArray
print(\"import ok\")'"

echo "== done. Restart affected nodes/launch for changes to take effect. =="
