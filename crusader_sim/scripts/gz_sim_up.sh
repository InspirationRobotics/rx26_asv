#!/usr/bin/env bash
# gz_sim_up.sh — the whole Gazebo sim, in order. Runs in: WSL2 Ubuntu-22.04.
#
#     bash crusader_sim/scripts/gz_sim_up.sh [course] [--no-gui] [--no-uav] [--no-rig]
#
#     course     a name in crusader_sim/courses/ (default task1_core)
#     --no-gui   Gazebo server only (the sim runs the same; you just can't watch)
#     --no-uav   no Ekko stand-in: the boat is on its own camera (Core tier)
#     --no-rig   stop after Gazebo + SITL (for check_motion, or your own nodes)
#
# From Windows, double-click crusader_sim/scripts/GZ_SIM_UP.cmd instead.
#
# ORDER, and why:
#   1. sync       Windows checkout -> ~/robotx_ws/src (the team's sync_to_wsl.sh)
#   2. generate   model (hull yaml + the boat's params), world (course), SITL params
#   3. gazebo     server first: SITL's JSON backend needs something to talk to
#   4. transmitter  before SITL, so the receiver has a signal from boot
#   5. SITL       the team's start_sitl.sh — same binary, same MAVProxy port map
#   6. e-stop     cycle SB e-stop -> run: ArduPilot latches the e-stop it read at
#                 boot, and a pilot clears it by moving the switch
#   7. rig        the boat's nodes + sim shims in the crsd-sim container
set -uo pipefail

COURSE=task1_core; GUI=1; UAV_ARG=""; RIG=1
for a in "$@"; do
  case "$a" in
    --no-gui) GUI=0 ;;
    --no-uav) UAV_ARG="--no-uav" ;;
    --no-rig) RIG=0 ;;
    -*) echo "unknown flag $a" >&2; exit 2 ;;
    *) COURSE="$a" ;;
  esac
done

WIN_SRC="${RX26_WIN_SRC:-/mnt/c/Users/Chaser/Documents/dev/RobotX_2026/Boat/rx26_asv}"
WS_SRC="${RX26_WSL_SRC:-$HOME/robotx_ws/src/rx26_asv}"
SIM="$WS_SRC/crusader_sim"
GEN="${CRUSADER_SIM_GEN:-$HOME/.cache/crusader_sim}"
APGZ="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
CONTAINER="${RX26_CONTAINER:-crsd-sim}"
IMAGE="${RX26_IMAGE:-crsd-sim:humble}"

export PATH="$HOME/.local/bin:$PATH"
export PYTHONPATH="$SIM${PYTHONPATH:+:$PYTHONPATH}"
export GZ_PARTITION=crusader_sim
export GZ_SIM_SYSTEM_PLUGIN_PATH="$APGZ/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$GEN/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

step() { printf '\n=== %s ===\n' "$1"; }
die()  { printf '\n*** FAILED: %s\n' "$1" >&2; exit 2; }

step "0/7  clean slate"
bash "$SIM/scripts/gz_sim_down.sh" --keep-container >/dev/null 2>&1 || true

step "1/7  Windows -> WSL"
[ -d "$WIN_SRC" ] || die "no Windows checkout at $WIN_SRC (set RX26_WIN_SRC)"
mkdir -p "$WS_SRC"
tr -d '\r' < "$WIN_SRC/tools/sitl/sync_to_wsl.sh" > /tmp/rx26_sync.sh
RX26_WIN_SRC="$WIN_SRC" RX26_WSL_SRC="$WS_SRC" bash /tmp/rx26_sync.sh sync | tail -1 | sed 's/^/  /'
# meshes are BINARY: the sync above strips \r from every file it copies, which
# would corrupt a .glb/.stl, so they are copied byte-for-byte here instead
if [ -d "$WIN_SRC/crusader_sim/meshes" ]; then
  mkdir -p "$SIM/meshes" && cp -u "$WIN_SRC/crusader_sim/meshes/"* "$SIM/meshes/" 2>/dev/null
  echo "  meshes: $(ls "$SIM/meshes" | wc -l) file(s)"
fi

step "2/7  generate  (course: $COURSE)"
cd /tmp
python3 -m crusader_sim.gen_crusader --out "$GEN/models" | sed 's/^/  /' || die "gen_crusader"
WORLD_INFO="$(python3 -m crusader_sim.gen_world "$SIM/courses/$COURSE.yaml" --out "$GEN/worlds")" \
  || die "gen_world (is there a courses/$COURSE.yaml?)"
WORLD="$(echo "$WORLD_INFO" | head -1)"
HOME_LL="$(echo "$WORLD_INFO" | awk '/^home /{print $2}')"
echo "  world $WORLD   home $HOME_LL"
python3 -m crusader_sim.sitl_params --out "$GEN/crusader_gz.parm" | sed 's/^/  /' || die "sitl_params"

step "3/7  Gazebo"
nohup gz sim -s -r "$WORLD" > /tmp/gz_server.log 2>&1 &
sleep 5
pgrep -f "gz sim -s" >/dev/null || die "gz server exited — see /tmp/gz_server.log"
echo "  server up (log /tmp/gz_server.log)"
if [ "$GUI" = 1 ]; then
  nohup gz sim -g > /tmp/gz_gui.log 2>&1 &
  echo "  GUI starting on the Windows desktop (WSLg)"
fi

step "4/7  transmitter"
nohup python3 -m crusader_sim.sim_transmitter > /tmp/sim_tx.log 2>&1 &
sleep 1; echo "  RC -> udp 5501 (log /tmp/sim_tx.log)"

step "5/7  ArduRover SITL (JSON, lockstep with Gazebo)"
mkdir -p "$GEN/sitl"
SITL_FRAME=rover SITL_HOME="$HOME_LL,0,0" \
SITL_EXTRA_ARGS="--model JSON --use-dir $GEN/sitl -w --add-param-file=$GEN/crusader_gz.parm" \
  bash "$WS_SRC/tools/sitl/start_sitl.sh" > /tmp/start_sitl.out 2>&1
grep -E "ArduRover V|WARN|ERROR|endpoints" /tmp/start_sitl.out | sed 's/^/  /'
pgrep -f "bin/ardurover" >/dev/null || die "SITL did not start — /tmp/sitl.log"

step "6/7  SB e-stop -> run"
python3 -m crusader_sim.sim_transmitter set estop on >/dev/null; sleep 1.5
python3 -m crusader_sim.sim_transmitter set estop off >/dev/null; sleep 1
echo "  cleared"

if [ "$RIG" = 0 ]; then
  echo; echo "  up without the ROS rig. Try: python3 -m crusader_sim.check_motion"
  exit 0
fi

step "7/7  the ROS rig in $CONTAINER"
if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
  echo "  creating $CONTAINER from $IMAGE (--net=host --ipc=host)"
  docker run -d --name "$CONTAINER" --net=host --ipc=host \
    -v "$HOME/robotx_ws:/root/robotx_ws" "$IMAGE" sleep infinity >/dev/null \
    || die "docker run $IMAGE (build it: see crusader_sim/docker/crsd-sim.Dockerfile)"
fi
docker start "$CONTAINER" >/dev/null || die "cannot start $CONTAINER"
if ! docker exec "$CONTAINER" test -d /root/robotx_ws/install/crusader_sim; then
  # plain build, like the team's rig: mixing --symlink-install into an install/
  # a plain build made is a colcon error on the next build
  echo "  first run: colcon build (a few minutes)"
  docker exec "$CONTAINER" bash -lc \
    "cd /root/robotx_ws && source /opt/ros/humble/setup.bash && colcon build 2>&1 | tail -5" \
    | sed 's/^/  /'
else
  # the sim package only, every run (seconds): its nodes run from install/
  docker exec "$CONTAINER" bash -lc \
    "cd /root/robotx_ws && source install/setup.bash && colcon build --packages-select crusader_sim 2>&1 | tail -1" \
    | sed 's/^/  /'
fi
docker exec "$CONTAINER" bash -lc \
  "bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_rig_up.sh $COURSE $UAV_ARG" 2>&1 | sed 's/^/  /'

cat <<EOF

=== ready ===
  Gazebo GUI      on the desktop (or: gz sim -g, with GZ_PARTITION=crusader_sim)
  ground station  http://localhost:8090     behaviour tree  http://localhost:8085
  QGroundControl  auto-connects on udp 14550 (Windows)
  start Task 1:   docker exec -it $CONTAINER bash -lc 'python3 -m crusader_sim.task1_goal --course $COURSE'
  drive MANUAL:   ros2 topic pub /crsd/rc_override ... (in $CONTAINER), mode MANUAL
  stop:           bash $SIM/scripts/gz_sim_down.sh
EOF
