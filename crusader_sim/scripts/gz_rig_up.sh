#!/usr/bin/env bash
# gz_rig_up.sh — the ROS half of the Gazebo sim. Runs in: the crsd-sim container.
#
#     bash crusader_sim/scripts/gz_rig_up.sh [course] [--no-uav] [--radio-endpoint EP] [--ocs-host IP]
#
#     --radio-endpoint EP  where rfd_link_node listens (a pymavlink endpoint). Default
#                        udpin:127.0.0.1:14555, which only sim_uav in this container can reach.
#                        udpin:0.0.0.0:14555 lets a radio relay on the Windows side (the real
#                        Ekko's RFD traffic, forwarded) reach it. Give --no-uav too when Ekko is
#                        the source: two transmitters make the plan alternate between them.
#     --ocs-host IP      also start the boat's crusader_groundstation ocs_client, dialling the
#                        Operator Control Station at IP (port 37564, the params file's ocs_port).
#                        Without it ocs_client is not started, as before.
#
# Started by gz_sim_up.sh (WSL) after Gazebo and SITL are up. Instead of
# bench_world_model (an invented buoy field and invented detections), it runs
# the simulator's sensors through the boat's own perception:
#
#     gz  --ros_gz_bridge-->  /sim/*  --livox_shim-->  /livox/lidar
#                                         --> lidar_node (the boat's) --> crsd/lidar_clusters
#                             /sim/*  --sim_camera-->  oak/rgb, oak/depth, crsd/oak/detections
#     sim_uav  --TUNNEL v3, udp 14555-->  rfd_link_node (the boat's): Ekko's map of the course
#     nav_frames_node + Nav2 planner_server (crusader_nav, when the image has Nav2)
#         --/crsd/nav/hazards, /crsd/nav/obstacle_cloud-->  bt_runner's planned legs
#
# NAV_MODE=off|shadow|on in the environment picks the tree's planning (default on
# when Nav2 is present, forced off with a banner when it is not).
#
# SIM_DETECTOR=truth|yolo in the environment picks sim_camera's boxes: truth (default) is the
# geometric oracle; yolo runs the team's REAL YOLO detector + LED classifier on the rendered
# frames (setup_yolo_venv.sh's header). It needs the venv from
# setup_yolo_venv.sh and falls back to truth, with a banner, when that is missing.
# SIM_CAMERA_ARGS adds ROS args to sim_camera, e.g. "-p yolo_imgsz:=1920 -p yolo_conf:=0.1".
#
# Everything downstream — telemetry_bridge, rfd_link_node, target_tracker, gcs_bridge,
# bt_runner — is the boat's code, unmodified, same params file with the sim's values layered on
# top from /tmp/sim_params.yaml. The laptop's ground station
# connects to it: BOAT_GUI.cmd --boat 127.0.0.1. TREE (a built-in tree name, default
# task1_global) is the mission gz_task1.sh sends.
# ROS's setup.bash reads unset variables, so strict mode goes on AFTER it
source /opt/ros/humble/setup.bash
source /root/robotx_ws/install/setup.bash
set -uo pipefail

COURSE=task1_core
UAV=1
RADIO_EP=udpin:127.0.0.1:14555   # sim_uav's udpout:127.0.0.1:14555 lands here
RADIO_EP_DEFAULT=$RADIO_EP
OCS_HOST=""
while [ $# -gt 0 ]; do
  case "$1" in
    --no-uav) UAV=0 ;;
    --radio-endpoint)
      [ $# -ge 2 ] || { echo "--radio-endpoint needs an endpoint" >&2; exit 2; }
      RADIO_EP="$2"; shift ;;
    --ocs-host)
      [ $# -ge 2 ] || { echo "--ocs-host needs an address" >&2; exit 2; }
      OCS_HOST="$2"; shift ;;
    -*) echo "unknown flag $1" >&2; exit 2 ;;
    *) COURSE="$1" ;;
  esac
  shift
done
# both values are words in a ros2 run line, so only the characters an endpoint or a host needs
[[ "$RADIO_EP" =~ ^[A-Za-z0-9_.:/@-]+$ ]] || { echo "*** --radio-endpoint '$RADIO_EP': letters, digits and _ . : / @ - only" >&2; exit 2; }
[[ -z "$OCS_HOST" || "$OCS_HOST" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "*** --ocs-host '$OCS_HOST': letters, digits and _ . - only" >&2; exit 2; }
WS=/root/robotx_ws
SRC=$WS/src/rx26_asv
CFG=$WS/install/crusader_bringup/share/crusader_bringup/config/crusader_params.yaml
# SIM_TUNING (gz_sim_up.sh --tuning): a params file of overrides, layered on top of $CFG for
# bt_runner and of nav2_params.yaml for Nav2. Empty = none, and the run is exactly as before.
BT_TUNE=(); NAV_TUNE=()
if [ -n "${SIM_TUNING:-}" ]; then
  [ -f "$SIM_TUNING" ] || { echo "*** FAILED: SIM_TUNING=$SIM_TUNING: no such file" >&2; exit 2; }
  BT_TUNE=(--params-file "$SIM_TUNING"); NAV_TUNE=("nav2_overlay:=$SIM_TUNING")
fi
SIMSHARE=$WS/install/crusader_sim/share/crusader_sim
TREE="${TREE:-task1_global}"       # sent by gz_task1.sh as the RunMission name
export GZ_PARTITION=crusader_sim
export RX26_SRC=$SRC

# The sim's own values. A bare `-p k:=v` is a /** override, and $CFG's node-named section beats
# it whatever the order on the line, so a key $CFG also sets (model, rfd_endpoint, ocs_host,
# publish_setpoints, nav_mode) silently kept the boat's value. They live in this node-named file
# instead, read after $CFG by every boat node; bt_runner's section is added once NAV_MODE is known.
SIM_PARAMS=/tmp/sim_params.yaml
cat > "$SIM_PARAMS" <<EOF
camera_node:
  ros__parameters:
    source: "topic"
    model: "none"
telemetry_bridge:
  ros__parameters:
    mav_endpoint: "udp:127.0.0.1:14551"
rfd_link_node:
  ros__parameters:
    rfd_endpoint: "$RADIO_EP"
ocs_client:
  ros__parameters:
    ocs_host: "$OCS_HOST"
EOF
BOAT_PARAMS=(--params-file "$CFG" --params-file "$SIM_PARAMS")

bash "$SRC/crusader_sim/scripts/gz_rig_down.sh" >/dev/null 2>&1
sleep 1

up() { local name=$1 log=$2; shift 2; nohup "$@" > "$log" 2>&1 &
       printf '  %-20s -> %s\n' "$name" "$log"; }

echo "=== sensors (Gazebo -> the boat's topics) ==="
up gz_bridge /tmp/gzb.log \
  ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SIMSHARE/config/gz_bridge.yaml"
up livox_shim /tmp/livox_shim.log ros2 run crusader_sim livox_shim
# the camera: the oracle detector, or the real YOLO. YOLO needs torch + ultralytics, which live
# in a venv (setup_yolo_venv.sh), so the node is started with THAT python; -m instead of
# `ros2 run` because the entry-point script's shebang is the system python
DETECTOR="${SIM_DETECTOR:-truth}"
case "$DETECTOR" in truth|yolo) ;;
  *) echo "*** SIM_DETECTOR must be truth or yolo (got '$DETECTOR')" >&2; exit 2 ;; esac
YOLO_PY=$WS/venvs/yolo/bin/python
if [ "$DETECTOR" = yolo ]; then
  YOLO_WHY=""
  if [ ! -x "$YOLO_PY" ]; then YOLO_WHY="no venv at $YOLO_PY"
  elif [ ! -f "$WS/models/sim_yolo/crusader_det_yolo26n.pt" ] || [ ! -f "$WS/models/sim_yolo/crusader_led_cls.pt" ]; then
    YOLO_WHY="no .pt files in $WS/models/sim_yolo"
  elif ! "$YOLO_PY" -c "import torch, ultralytics, rclpy, crusader_sim.yolo_detect" >/dev/null 2>&1; then
    YOLO_WHY="the venv does not import torch/ultralytics/rclpy/crusader_sim.yolo_detect"
  fi
  if [ -n "$YOLO_WHY" ]; then
    echo "  *** SIM_DETECTOR=yolo UNAVAILABLE ($YOLO_WHY): running the TRUTH detector. Fix: bash crusader_sim/scripts/setup_yolo_venv.sh"
    DETECTOR=truth
  fi
fi
export YOLO_CONFIG_DIR=/tmp/Ultralytics
CAM_CMD=(ros2 run crusader_sim sim_camera); CAM_ARGS=()
if [ "$DETECTOR" = yolo ]; then
  CAM_CMD=("$YOLO_PY" -u -m crusader_sim.sim_camera); CAM_ARGS=(-p detector:=yolo)
fi
up sim_camera /tmp/sim_camera.log   "${CAM_CMD[@]}" --ros-args -p course:="$COURSE" "${CAM_ARGS[@]}" ${SIM_CAMERA_ARGS:-}

echo "=== the boat's stack ==="
# the GCS Sensors tab's picture: the boat's camera_node on sim_camera's oak/rgb + oak/depth,
# model none (sim_camera already publishes the detections); it encodes only while watched
up camera_node /tmp/camera.log \
  ros2 run crusader_perception camera_node --ros-args "${BOAT_PARAMS[@]}"
up telemetry_bridge /tmp/tb.log \
  ros2 run crusader_behavior telemetry_bridge --ros-args "${BOAT_PARAMS[@]}"
up lidar_node /tmp/lidar.log \
  ros2 run crusader_perception lidar_node --ros-args "${BOAT_PARAMS[@]}"
# rfd_endpoint is the RFD900 (/dev/crsd-rfd) on the boat; here the UDP form sim_uav talks to.
# --radio-endpoint overrides it (a Windows-side radio relay needs udpin:0.0.0.0:14555)
up rfd_link_node /tmp/rfd.log \
  ros2 run crusader_link rfd_link_node --ros-args "${BOAT_PARAMS[@]}"
if [ "$RADIO_EP" != "$RADIO_EP_DEFAULT" ] && [ "$UAV" = 1 ]; then
  echo "  *** radio endpoint is $RADIO_EP but sim_uav is still on: if the real Ekko is the source, add --no-uav (two transmitters make the plan alternate)"
fi
if [ "$UAV" = 1 ]; then
  up sim_uav /tmp/sim_uav.log python3 -u -m crusader_sim.sim_uav --course "$COURSE"
else
  echo "  sim_uav              -- off (--no-uav): the boat is on its own camera"
fi
up target_tracker /tmp/tt.log \
  ros2 run crusader_world_model target_tracker --ros-args "${BOAT_PARAMS[@]}"

# Nav2 avoidance (docs/nav2_avoidance_spec.md 10.1). The nav stack goes up BEFORE
# bt_runner, which plans through it. NAV_MODE (env, default on): off = the legacy
# straight legs and no nav stack; shadow = plans and displays, drives legacy; on.
# An image without Nav2 (or without the two nav packages built) cannot plan, so it
# runs off with a banner rather than a tree that holds waiting for a planner.
echo "=== nav stack ==="
case "${NAV_MODE:-}" in ""|off|shadow|on) ;;
  *) echo "*** NAV_MODE must be off, shadow or on (got '$NAV_MODE')" >&2; exit 2 ;; esac
NAV_WHY=""
if ! ros2 pkg prefix nav2_planner >/dev/null 2>&1; then
  NAV_WHY="this image has no Nav2. Rebuild crsd-sim and recreate the container (docker/crsd-sim.Dockerfile's header)"
elif ! ros2 pkg prefix crusader_nav >/dev/null 2>&1 || ! ros2 pkg prefix crusader_nav_layers >/dev/null 2>&1; then
  NAV_WHY="Nav2 is here but crusader_nav / crusader_nav_layers are not built. In the container: colcon build --packages-up-to crusader_nav crusader_nav_layers crusader_bt"
fi
if [ -n "$NAV_WHY" ]; then
  echo "  *** AVOIDANCE OFF: $NAV_WHY"
  NAV_MODE=off
else
  NAV_MODE="${NAV_MODE:-on}"
fi
if [ "$NAV_MODE" = off ]; then
  [ -n "$NAV_WHY" ] || echo "  nav stack            -- not started (NAV_MODE=off: the legacy straight legs)"
else
  # the datum is the course origin, so the boat's map frame is the world's ENU frame
  read -r DLAT DLON < <(python3 -c "import sys; from crusader_sim import course as C; o=C.load(sys.argv[1])['origin']; print(o['lat'], o['lon'])" "$COURSE")
  if [ -z "${DLAT:-}" ] || [ -z "${DLON:-}" ]; then
    echo "  *** AVOIDANCE OFF: cannot read the origin of course '$COURSE'"
    NAV_MODE=off
  else
    up nav /tmp/nav.log ros2 launch crusader_nav nav.launch.py \
      datum_source:=param datum_lat:=$DLAT datum_lon:=$DLON "${NAV_TUNE[@]}"
  fi
fi

up gcs_bridge /tmp/gcs.log \
  ros2 run crusader_groundstation gcs_bridge --ros-args "${BOAT_PARAMS[@]}"
# the boat's heartbeat to the Operator Control Station, only when asked for: the OCS is a
# laptop on the team network and the sim has no business dialling it unless told where it is.
# $SIM_PARAMS gives ocs_host; $CFG the rest (ocs_port 37564, team_id, vehicle_id).
if [ -n "$OCS_HOST" ]; then
  up ocs_client /tmp/ocs_client.log \
    ros2 run crusader_groundstation ocs_client --ros-args "${BOAT_PARAMS[@]}"
else
  echo "  ocs_client           -- off (--ocs-host IP starts it)"
fi
sleep 6
cat >> "$SIM_PARAMS" <<EOF
bt_runner_node:
  ros__parameters:
    publish_setpoints: true
    default_timeout_s: 400.0
    nav_mode: "$NAV_MODE"
EOF
up bt_runner /tmp/bt.log \
  ros2 run crusader_bt bt_runner_node --ros-args "${BOAT_PARAMS[@]}" "${BT_TUNE[@]}"
sleep 6

# planner_server leaves 'activating' only once TF map -> base_footprint exists,
# i.e. a pose with a FINITE heading (spec 3.6). Say so if it has not, instead of
# letting the first planned leg find out 15 s into a run.
if [ "$NAV_MODE" != off ]; then
  t0=$SECONDS; state=""
  while [ $((SECONDS - t0)) -lt 40 ]; do
    state="$(timeout 8 ros2 lifecycle get /planner_server 2>/dev/null | head -1)"
    case "$state" in active*) break ;; esac
    sleep 2
  done
  case "$state" in
    active*) echo "  planner_server: $state" ;;
    *) echo "  *** planner_server is not active after 40 s (${state:-no such node}): nav_mode=$NAV_MODE legs will hold and FAIL at 15 s. See /tmp/nav.log" ;;
  esac
fi

echo
echo "=== up ==="
ros2 node list 2>/dev/null | sort | sed 's/^/  /'
echo "  tree: $(basename "$TREE")   course: $COURSE   nav_mode: $NAV_MODE   detector: $DETECTOR"
echo "  radio: $RADIO_EP   ocs_client: ${OCS_HOST:-off}   ground station: BOAT_GUI.cmd --boat 127.0.0.1"
timeout 5 ros2 topic echo /crsd/fcu_status --once 2>/dev/null \
  | grep -E '^mode|^armed' | tr '\n' ' ' | sed 's/^/  /'; echo
