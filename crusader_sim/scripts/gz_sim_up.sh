#!/usr/bin/env bash
# gz_sim_up.sh — the whole Gazebo sim, in order. Runs in: WSL2 Ubuntu-22.04.
#
#     bash crusader_sim/scripts/gz_sim_up.sh [course | --course-file PATH] [--no-gui] [--no-uav] [--no-rig]
#                                            [--recreate-container]
#
#     NAV_MODE=off|shadow|on  (environment) the tree's Nav2 planning; default on
#                 when the image has Nav2. docs/nav2_avoidance_spec.md.
#     RX26_IMAGE  the image a NEW container is made from. Default: crsd-sim:nav2 when
#                 it exists, else crsd-sim:humble (no Nav2).
#     --recreate-container
#                 remove the crsd-sim container if it was made from a different image
#                 than RX26_IMAGE and make a new one. Only after the sim is down.
#                 An existing container keeps its old image otherwise, and the rig
#                 then runs nav_mode off with a banner.
#
#     course     a name in crusader_sim/courses/ (default task1_core)
#     --course-file PATH
#                a course YAML from anywhere (the Task 1 panel writes one). It
#                is copied in as courses/<its stem>.yaml after the sync, so the
#                container's colcon build installs it and every node finds it
#                by name, exactly like a checked-in course
#     --no-gui   Gazebo server only (the sim runs the same; you just can't watch)
#     --no-uav   no Ekko stand-in: the boat is on its own camera (Core tier)
#     --no-rig   stop after Gazebo + SITL (for check_motion, or your own nodes)
#
# From Windows, double-click crusader_sim/scripts/GZ_SIM_UP.cmd instead.
#
# ORDER, and why:
#   1. sync       Windows checkout -> ~/robotx_ws/src (gz_sync.sh: the team's
#                 sync_to_wsl.sh plus the binary meshes), then --course-file
#   2. generate   model (hull yaml + the boat's params), world (course), SITL params
#   3. gazebo     server first: SITL's JSON backend needs something to talk to
#   4. transmitter  before SITL, so the receiver has a signal from boot
#   5. SITL       the team's start_sitl.sh — same binary, same MAVProxy port map
#   6. e-stop     cycle SB e-stop -> run: ArduPilot latches the e-stop it read at
#                 boot, and a pilot clears it by moving the switch
#   7. rig        the boat's nodes + sim shims in the crsd-sim container
set -uo pipefail

COURSE=task1_core; COURSE_FILE=""; GUI=1; UAV_ARG=""; RIG=1; RECREATE=0
while [ $# -gt 0 ]; do
  case "$1" in
    --recreate-container) RECREATE=1 ;;
    --no-gui) GUI=0 ;;
    --no-uav) UAV_ARG="--no-uav" ;;
    --no-rig) RIG=0 ;;
    --course-file)
      [ $# -ge 2 ] || { echo "--course-file needs a path" >&2; exit 2; }
      COURSE_FILE="$2"; shift ;;
    -*) echo "unknown flag $1" >&2; exit 2 ;;
    *) COURSE="$1" ;;
  esac
  shift
done

WIN_SRC="${RX26_WIN_SRC:-/mnt/c/Users/Chaser/Documents/dev/RobotX_2026/Boat/rx26_asv}"
WS_SRC="${RX26_WSL_SRC:-$HOME/robotx_ws/src/rx26_asv}"
SIM="$WS_SRC/crusader_sim"
GEN="${CRUSADER_SIM_GEN:-$HOME/.cache/crusader_sim}"
APGZ="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
CONTAINER="${RX26_CONTAINER:-crsd-sim}"
# The Nav2 image when it has been built (docs/nav2_avoidance_spec.md 8.2), else the
# original. The choice only matters when a container is CREATED; see ensure_container.
if [ -n "${RX26_IMAGE:-}" ]; then IMAGE="$RX26_IMAGE"
elif docker image inspect crsd-sim:nav2 >/dev/null 2>&1; then IMAGE=crsd-sim:nav2
else IMAGE=crsd-sim:humble; fi

export PATH="$HOME/.local/bin:$PATH"
export PYTHONPATH="$SIM${PYTHONPATH:+:$PYTHONPATH}"
export GZ_PARTITION=crusader_sim
export GZ_SIM_SYSTEM_PLUGIN_PATH="$APGZ/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$GEN/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# RENDERING ON THE GPU. WSLg's Mesa defaults to llvmpipe on this machine —
# `glxinfo -B` says "llvmpipe ... Accelerated: no" — so every camera, LiDAR
# and GUI frame was rendered on the CPU, the sim fell to RTF ~0.2, SITL's
# HEARTBEAT went stale in wall-clock terms and the tree (rightly) refused to
# keep going. Naming the adapter selects the RTX through Mesa's d3d12 driver:
# "D3D12 (NVIDIA GeForce RTX 5060 Ti)". GZ_GPU=cpu forces software rendering
# back on — slow, but it works if a driver update ever breaks the GPU path.
GZ_GPU="${GZ_GPU:-NVIDIA}"
if [ "$GZ_GPU" = cpu ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
else
  export GALLIUM_DRIVER=d3d12
  export MESA_D3D12_DEFAULT_ADAPTER_NAME="$GZ_GPU"
fi

step() { printf '\n=== %s ===\n' "$1"; }
die()  { printf '\n*** FAILED: %s\n' "$1" >&2; exit 2; }

# Reuse, create or (only with --recreate-container) recreate the sim container.
# `docker start` never changes a container's image, so an existing crsd-sim stays on
# the image it was made from after crsd-sim:nav2 is built. IDs are compared, not tags
# (tags move). A recreate also drops the two CMake build dirs that look for Nav2:
# find_package results are cached per build dir and were made in the other image.
# FULL_BUILD=1 then makes the colcon step rebuild everything.
FULL_BUILD=0
ensure_container() {
  local want have
  want="$(docker image inspect -f '{{.Id}}' "$IMAGE" 2>/dev/null)"
  if [ -n "$want" ] && docker inspect "$CONTAINER" >/dev/null 2>&1; then
    have="$(docker inspect -f '{{.Image}}' "$CONTAINER")"
    if [ "$have" != "$want" ]; then
      if [ "$RECREATE" = 1 ]; then
        echo "  $CONTAINER is on an older image: recreating it from $IMAGE"
        docker rm -f "$CONTAINER" >/dev/null || die "cannot remove $CONTAINER"
        # the dirs are root-owned (the container builds as root), so they are removed
        # from INSIDE the new container below; an rm from this shell is Permission denied
        FULL_BUILD=1
      else
        echo "  *** $CONTAINER was made from an older image than $IMAGE and keeps it: Nav2 may be missing (the rig then runs nav_mode off and says so)."
        echo "      To switch, with the sim down: gz_sim_up.sh --recreate-container   (crusader_sim/README.md, 'Nav2 avoidance in the sim')"
      fi
    fi
  fi
  if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
    echo "  creating $CONTAINER from $IMAGE (--net=host --ipc=host)"
    docker run -d --name "$CONTAINER" --net=host --ipc=host \
      -v "$HOME/robotx_ws:/root/robotx_ws" "$IMAGE" sleep infinity >/dev/null \
      || die "docker run $IMAGE (build it: see crusader_sim/docker/crsd-sim.Dockerfile)"
    if [ "$FULL_BUILD" = 1 ]; then
      docker exec "$CONTAINER" bash -c 'cd /root/robotx_ws && rm -rf build/crusader_bt install/crusader_bt build/crusader_nav_layers install/crusader_nav_layers' \
        || die "cannot remove the old crusader_bt / crusader_nav_layers build dirs"
    fi
  fi
}

# checked BEFORE step 0: a bad path must not cost the user the sim that is running
if [ -n "$COURSE_FILE" ]; then
  [ -f "$COURSE_FILE" ] || die "--course-file $COURSE_FILE: no such file"
  COURSE_FILE="$(realpath "$COURSE_FILE")"
  COURSE="$(basename "$COURSE_FILE")"; COURSE="${COURSE%.*}"
  # the name crosses into docker exec and ros2 run unquoted
  [[ "$COURSE" =~ ^[A-Za-z0-9_.-]+$ ]] || die "--course-file: '$COURSE' is not a usable course name (letters, digits, _ . - only)"
fi

step "0/7  clean slate"
bash "$SIM/scripts/gz_sim_down.sh" --keep-container >/dev/null 2>&1 || true

step "1/7  Windows -> WSL"
# the WINDOWS copy: before the first sync the workspace may not have gz_sync.sh
[ -f "$WIN_SRC/crusader_sim/scripts/gz_sync.sh" ] \
  || die "no $WIN_SRC/crusader_sim/scripts/gz_sync.sh (set RX26_WIN_SRC to the Windows checkout)"
tr -d '\r' < "$WIN_SRC/crusader_sim/scripts/gz_sync.sh" > /tmp/gz_sync.sh
RX26_WIN_SRC="$WIN_SRC" RX26_WSL_SRC="$WS_SRC" bash /tmp/gz_sync.sh || die "Windows -> WSL sync"
if [ -n "$COURSE_FILE" ]; then
  # after the sync, so the sync cannot overwrite it with a checked-in namesake
  [ "$COURSE_FILE" -ef "$SIM/courses/$COURSE.yaml" ] \
    || cp "$COURSE_FILE" "$SIM/courses/$COURSE.yaml" || die "copying $COURSE_FILE into courses/"
  echo "  course file $COURSE_FILE -> courses/$COURSE.yaml"
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
ensure_container
docker start "$CONTAINER" >/dev/null || die "cannot start $CONTAINER"
# a full build when there is no install/ yet, after a recreate, or when the checkout
# has the Nav2 packages and install/ does not (the msgs changed under them too)
if [ "$FULL_BUILD" = 1 ] || ! docker exec "$CONTAINER" bash -c \
     'cd /root/robotx_ws && [ -d install/crusader_sim ] && { [ ! -d src/rx26_asv/crusader_nav ] || [ -d install/crusader_nav ]; }'; then
  # plain build, like the team's rig: mixing --symlink-install into an install/
  # a plain build made is a colcon error on the next build
  echo "  full colcon build (first run, new image or new packages: a few minutes)"
  docker exec "$CONTAINER" bash -lc \
    "cd /root/robotx_ws && source /opt/ros/humble/setup.bash && flock /root/robotx_ws/.colcon.lock colcon build 2>&1 | tail -5" \
    | sed 's/^/  /'
else
  # every package a sim run exercises, every run: nodes run from install/, and a stale
  # crusader_bt silently ignores nav_mode and runs legacy legs (the review of
  # 2026-10-01). Unchanged packages cost seconds; a changed crusader_bt about 40 s.
  # Only the packages the checkout has: --packages-select refuses an unknown name.
  docker exec "$CONTAINER" bash -lc \
    "cd /root/robotx_ws && source install/setup.bash && P=''; for p in crusader_msgs crusader_bt crusader_perception crusader_sim crusader_nav crusader_nav_layers crusader_groundstation; do [ -d src/rx26_asv/\$p ] && P=\"\$P \$p\"; done; flock /root/robotx_ws/.colcon.lock colcon build --packages-select \$P 2>&1 | tail -2" \
    | sed 's/^/  /'
fi
# docker exec does not inherit this shell's environment: TREE and NAV_MODE cross explicitly
docker exec -e TREE="${TREE:-}" -e NAV_MODE="${NAV_MODE:-}" "$CONTAINER" bash -lc \
  "bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_rig_up.sh $COURSE $UAV_ARG" 2>&1 | sed 's/^/  /'

if [ "$GUI" = 1 ]; then
  # The GUI sometimes segfaults at startup inside NVIDIA's WSL driver
  # (glXChooseFBConfig -> libnvwgf2umx.so, seen 2026-09-29) while the server
  # renders on the same GPU without trouble, and a relaunch has come up every
  # time. So check it and relaunch it; the third try renders the WINDOW on the
  # CPU (the sensors stay on the GPU) rather than show nothing.
  for try in 1 2 3; do
    pgrep -f "gz sim -g" >/dev/null && break
    cp /tmp/gz_gui.log "/tmp/gz_gui.crash$try.log" 2>/dev/null
    if [ "$try" -lt 3 ]; then
      echo "  GUI is not running (crashed at startup) — relaunching"
      nohup gz sim -g > /tmp/gz_gui.log 2>&1 &
    else
      echo "  GUI crashed twice — relaunching it with CPU rendering"
      GALLIUM_DRIVER=llvmpipe LIBGL_ALWAYS_SOFTWARE=1 nohup gz sim -g > /tmp/gz_gui.log 2>&1 &
    fi
    sleep 12
  done
  pgrep -f "gz sim -g" >/dev/null || echo "  *** no GUI (see /tmp/gz_gui.log); the sim runs on without it"
  # lock the GUI camera onto the boat: the course is ~50 m long and a 1 m boat
  # leaves the default view within seconds of GUIDED
  for _ in $(seq 1 30); do gz service -l 2>/dev/null | grep -qx /gui/follow && break; sleep 1; done
  gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
    --timeout 3000 --req 'data: "crusader"' >/dev/null 2>&1 \
  && gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
    --timeout 3000 --req 'x: -6, y: -5, z: 4' >/dev/null 2>&1 \
  && echo "  GUI camera following crusader"
fi

cat <<EOF

=== ready ===
  Gazebo GUI      on the desktop (or: gz sim -g, with GZ_PARTITION=crusader_sim)
  ground station  http://localhost:8090     behaviour tree  http://localhost:8085
  QGroundControl  auto-connects on udp 14550 (Windows)
  start Task 1:   bash $SIM/scripts/gz_task1.sh $COURSE
  drive MANUAL:   ros2 topic pub /crsd/rc_override ... (in $CONTAINER), mode MANUAL
  stop:           bash $SIM/scripts/gz_sim_down.sh
EOF
