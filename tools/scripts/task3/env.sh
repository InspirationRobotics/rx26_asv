# env.sh - sourced by the other tools/scripts/task3 scripts: ROS, the workspace,
# and where things are. Runs INSIDE the asv container.
#
# The scripts live in the repo; their LOGS do not. They go to /root/robotx_ws/t3tools
# (= ~/robotx_ws/t3tools on the Jetson), outside src/, so a run never dirties
# the checkout and the logs survive a re-clone.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /root/robotx_ws/install/setup.bash

PARAMS=/root/robotx_ws/install/crusader_bringup/share/crusader_bringup/config/crusader_params.yaml
TREES=/root/robotx_ws/install/crusader_bt/share/crusader_bt/behavior_trees
LOGDIR=/root/robotx_ws/t3tools
LOG=$LOGDIR/bt.log            # bt_runner_node's output (tree.sh truncates it)
GOLOG=$LOGDIR/go.log          # the action client's feedback (go.sh -d)
mkdir -p "$LOGDIR"
