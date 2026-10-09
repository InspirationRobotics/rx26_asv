#!/usr/bin/env bash
# cal.sh ARGS - cannon_cal.py with ROS sourced: the water cannon's aim
# calibration (docs/T3_cannon_cal.md). Runs INSIDE the asv container:
#
#   docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/cal.sh node
#
# On the Jetson, once:  alias cal='docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/cal.sh'
# then:                 cal node | cal deg 0 20 | cal point 1.5 0.3 0.1 | cal show ...
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"
exec python3 "$SCRIPT_DIR/cannon_cal.py" "$@"
