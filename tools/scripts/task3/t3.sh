#!/usr/bin/env bash
# t3.sh - Task 3 in short words. Runs INSIDE the asv container. On the Jetson,
# ~/.bash_aliases has:
#
#   alias t3='docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/t3.sh'
#   alias slip='docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/slip.sh'
#   alias cal='docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/cal.sh'
#
# so:  t3 start | t3 go | t3 steps | t3 log | slip | cal ...   (t3 alone: this list)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"

usage() {
  cat <<'EOF'
t3 start [TREE]  start the Task 3 launch (leave it running; Ctrl-C ends it)
                 TREE: slip (the default: dock in the slip on the LiDAR)
                       fire (cannon_fire: no slip, the wall and the camera)
                       hold (cannon_hold), cannon (the full Gazebo tree)
t3 go            start the run (checks, resets the autonomy drop, sends the goal)
t3 steps         the TASK3 step lines: APPROACHING, IN THE SLIP, DOCKED, ...
t3 log           the tree's log, live: the useful lines only
slip             what the LiDAR makes of the slip, once a second (no tree)
cal ...          the cannon's aim calibration (cal alone: its help)
EOF
}

cmd="${1:-help}"
[ $# -gt 0 ] && shift
case "$cmd" in
  start)
    tree="${1:-slip}"
    [ $# -gt 0 ] && shift
    case "$tree" in
      slip) tree=cannon_slip ;;
      fire) tree=cannon_fire ;;
      hold) tree=cannon_hold ;;
    esac
    exec ros2 launch crusader_bringup task3.launch.py tree:="$tree" "$@"
    ;;
  go)
    exec "$SCRIPT_DIR/go.sh" "${@:-86400}"
    ;;
  steps)
    exec ros2 topic echo /crsd/task3_events --qos-durability transient_local \
      --qos-reliability reliable --field data
    ;;
  log)
    f=$(ls -t /root/.ros/log/bt_runner_node_*.log 2>/dev/null | head -1)
    [ -n "$f" ] || { echo "no tree log yet: start it with  t3 start"; exit 1; }
    echo "following $f (Ctrl-C to stop)"
    tail -n 40 -F "$f" | grep --line-buffered -E \
      "TASK3|slot:|strafe:|the LiDAR has the slip|waiting for the LiDAR|code, window|decoded|FIRE|HIT|REFUSED|WARN|ERROR"
    ;;
  slip) exec "$SCRIPT_DIR/slip.sh" "$@" ;;
  cal) exec "$SCRIPT_DIR/cal.sh" "$@" ;;
  *) usage ;;
esac
