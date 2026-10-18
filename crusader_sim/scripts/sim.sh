#!/usr/bin/env bash
# sim.sh - the Gazebo sim in short words. Runs in: WSL2 Ubuntu-22.04. With the
# aliases (sim_aliases.sh) it is just `sim ...`; docs/T3_sim_commands.md has the
# whole list, side by side with the water's.
#
#   sim up [COURSE] [--no-gui]   the boat on the water: Gazebo, the autopilot, the
#                                transmitter and the boat's own stack, NOT the Task 3
#                                launch or a tree (that is `t3 start`, as on the
#                                Jetson). COURSE default task3_slip.
#   sim down                     everything off
#   sim wind M/S [TOWARD] [--gust F] [--swing DEG]   wind pushing toward compass TOWARD
#                                (0 = into the slip on the Task 3 courses; gusts default
#                                0.3; --swing: the direction wanders that many deg)
#   sim current M/S [TOWARD] [--vary F] [--swing DEG]   a water current flowing toward
#                                TOWARD, its speed (--vary) and direction (--swing) wandering
#   sim weather rough | light    both at once, varying (6 m/s across + 0.15 m/s in; 3 + 0.05)
#   sim calm                     no wind, no current
#   sim weather                  what it is set to, and what it is now
#   sim verdict                  the Task 3 referee: lights, water, reports so far
#   sim log                      LIVE: RoboCommand (every report and what it said) and
#                                the light sequence (ARMING, FIRE, GREEN, GAP, CODE)
#   sim robocommand | sim lights   ...only the reports, or only the lights
#   sim wp [left|middle|right] [M]   ONE waypoint M (5) m off that bay, uploaded as QGC's
#                                Plan view does (sim wp LAT LON: anywhere); t3 go wp uses it
#   sim speed [X]                the sim's real-time factor (show, or set X)
#   sim aimtest lr|ul [--quick]  the cannon's aim alone: the boat pinned at a grid of
#                                places/headings, a shot each, the miss from the
#                                window's middle (its own bring-up; aim_test.sh)
#
# There is no transmitter in the sim: `tx` is the pilot's hand (tx.sh).
HERE="$(cd "$(dirname "$0")" && pwd)"
WIN_SRC="$(cd "$HERE/../.." && pwd)"
CONTAINER="${RX26_CONTAINER:-crsd-sim}"
export GZ_PARTITION=crusader_sim
export PYTHONPATH="$WIN_SRC/crusader_sim${PYTHONPATH:+:$PYTHONPATH}"

cmd="${1:-help}"
[ $# -gt 0 ] && shift
case "$cmd" in
  up)
    course=task3_slip; gui=()
    for a in "$@"; do
      case "$a" in --no-gui) gui=(--no-gui) ;; -*) echo "sim up: unknown flag $a" >&2; exit 2 ;; *) course="$a" ;; esac
    done
    tr -d '\r' < "$WIN_SRC/crusader_sim/scripts/gz_sim_up.sh" > /tmp/gz_sim_up.sh
    RX26_WIN_SRC="$WIN_SRC" bash /tmp/gz_sim_up.sh "$course" --boat "${gui[@]}" || exit $?
    # as on the water for a Task 3 GUIDED approach (docs/T3_running.md): the boat's
    # 2.0 parks GUIDED 2 m short of every point
    docker exec -i "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash && python3 -m crusader_sim.sitl_param_set WP_RADIUS 0.3" >/dev/null 2>&1 \
      && echo "  WP_RADIUS 0.3 (a Task 3 GUIDED approach, as on the water)" \
      || echo "  *** WP_RADIUS not set: GUIDED will stop ~2 m short of the waypoint"
    ;;
  down)
    tr -d '\r' < "$WIN_SRC/crusader_sim/scripts/gz_sim_down.sh" > /tmp/gz_sim_down.sh
    exec bash /tmp/gz_sim_down.sh
    ;;
  wind|current|calm)
    exec python3 -m crusader_sim.weather "$cmd" "$@"
    ;;
  weather)
    # sim weather            what it is set to, and what it is now
    # sim weather rough|light   both, varying (crusader_sim.weather PRESETS)
    exec python3 -m crusader_sim.weather "${1:-show}"
    ;;
  verdict)
    exec docker exec -i "$CONTAINER" bash -lc       "source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash && python3 -m crusader_sim.sim_verdict"
    ;;
  log|robocommand|lights)
    case "$cmd" in
      log) pat="\[referee\]" ;;
      robocommand) pat="\[referee\] (RoboCommand|UAV|VERDICT)" ;;
      lights) pat="\[referee\] lights" ;;
    esac
    echo "following the referee (Ctrl-C to stop)"
    exec docker exec -i "$CONTAINER" bash -c "tail -n 50 -F /tmp/task3_world.log | grep --line-buffered -E '$pat' | sed -u -E 's/^\[[A-Z]+\] \[[0-9.]+\] \[task3_world\]: //'"
    ;;
  wp)
    course="$(cat "$HOME/.cache/crusader_sim/current_course" 2>/dev/null)"
    [ -n "$course" ] || { echo "no course up: sim up first" >&2; exit 1; }
    if [ $# -ge 2 ] && [[ "$1" =~ ^-?[0-9]+\.[0-9]+$ ]] && [[ "$2" =~ ^-?[0-9]+\.[0-9]+$ ]]; then
      args="--latlon $1 $2"
    else
      args="--course $course ${1:-right} ${2:-5}"
    fi
    exec docker exec -i "$CONTAINER" bash -lc       "source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash && python3 -m crusader_sim.upload_wp $args"
    ;;
  aimtest)
    tr -d '\r' < "$HERE/aim_test.sh" > /tmp/aim_test.sh
    RX26_SIM_SCRIPTS="$HERE" exec bash /tmp/aim_test.sh "$@"
    ;;
  speed)
    if [ $# -gt 0 ]; then
      gz service -s /world/crusader_sim/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean \
        --timeout 5000 --req "max_step_size: 0.001, real_time_factor: $1" >/dev/null && echo "real-time factor set to $1"
    fi
    timeout 5 gz topic -e -t /world/crusader_sim/stats -n 3 | grep real_time_factor
    ;;
  *)
    sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'
    ;;
esac
