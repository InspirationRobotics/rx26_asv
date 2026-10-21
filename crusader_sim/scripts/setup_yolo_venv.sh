#!/usr/bin/env bash
# setup_yolo_venv.sh — one-time setup for `SIM_DETECTOR=yolo`: the team's REAL
# YOLO detector + LED classifier running on the sim camera (sim_camera.py,
# crusader_sim/yolo_detect.py). Idempotent: run it again to repair or upgrade.
#
#     bash crusader_sim/scripts/setup_yolo_venv.sh [--force]
#
# Runs in: WSL2 Ubuntu-22.04 (it drives the crsd-sim container). From the Git Bash
# tool: tr -d '\r' < <this file> > /tmp/setup_yolo_venv.sh && bash /tmp/setup_yolo_venv.sh
#
# WHAT IT MAKES, and why it is where it is
#   ~/robotx_ws/venvs/yolo           (container: /root/robotx_ws/venvs/yolo)
#       a python venv built INSIDE the container with --system-site-packages, so
#       ROS (rclpy, crusader_msgs, ...) still imports, plus torch (CPU),
#       ultralytics and a numpy-1-compatible opencv-python-headless. ~/robotx_ws
#       is the bind mount, so the venv survives a container recreate (it only
#       needs the same image's /usr/bin/python3).
#   ~/robotx_ws/models/sim_yolo/     the two .pt files, copied from the
#       crusader_vision/runs folder on Windows. NEVER committed (.gitignore: *.pt).
#
# WHAT IT NEVER TOUCHES: the WSL host's python (~/.local, system). A numpy 2 or
# an opencv-python there breaks cv_bridge for every other ROS shell on this
# machine. Everything below is `docker exec` into the container or a file copy.
#
# numpy/opencv pinning: ultralytics declares opencv-python, whose newest wheels
# need numpy 2, and ROS Humble's compiled packages are built against numpy 1.
# So ultralytics goes in with --no-deps and its other requirements are named
# here; opencv comes from the HEADLESS wheel, pinned below 4.12 (the last line
# that still installs on numpy 1). `pip check`-style verification is at the end.
#
# GPU: not used. The container is created without --gpus, and the RTX 5060 Ti
# (Blackwell, sm_120) would need a cu128+ torch build on top. CPU torch does the
# 5 Hz inference this needs.
set -uo pipefail

VENV=/root/robotx_ws/venvs/yolo
ULTRALYTICS="${ULTRALYTICS_VERSION:-8.4.144}"   # what the .pt files were trained with
CONTAINER="${RX26_CONTAINER:-crsd-sim}"
MODELS_SRC="${YOLO_MODELS_SRC:-/mnt/c/Users/Chaser/Documents/dev/RobotX_2026/Boat/crusader_vision/runs}"
MODELS_DST_HOST="$HOME/robotx_ws/models/sim_yolo"
FORCE="${FORCE:-0}"; INSIDE=0     # FORCE also arrives as an env var: the container side is fed this script on stdin
for a in "$@"; do
  case "$a" in
    --force) FORCE=1 ;;
    --inside) INSIDE=1 ;;
    *) echo "unknown option $a" >&2; exit 2 ;;
  esac
done
die() { printf '\n*** FAILED: %s\n' "$1" >&2; exit 2; }

# ----------------------------------------------------------------- WSL side
if [ "$INSIDE" = 0 ]; then
  echo "=== models -> $MODELS_DST_HOST"
  mkdir -p "$MODELS_DST_HOST" || die "cannot make $MODELS_DST_HOST"
  for f in crusader_det_yolo26n.pt crusader_led_cls.pt summary.json; do
    src="$MODELS_SRC/$f"
    [ -f "$src" ] || die "$src is missing (set YOLO_MODELS_SRC to the crusader_vision/runs folder)"
    cmp -s "$src" "$MODELS_DST_HOST/$f" || cp "$src" "$MODELS_DST_HOST/$f" || die "copy $f"
  done
  ls -l "$MODELS_DST_HOST" | tail -n +2 | awk '{print "  " $5 "  " $9}'
  docker start "$CONTAINER" >/dev/null || die "cannot start $CONTAINER"
  echo "=== venv, inside $CONTAINER"
  # stream this file in: it works from a /tmp copy, with no sync to the workspace first
  exec docker exec -i -e FORCE="$FORCE" "$CONTAINER" bash -s -- --inside < "$0"
fi

# ----------------------------------------------------- inside the container
PY=/usr/bin/python3
[ -d /root/robotx_ws ] || die "/root/robotx_ws is not mounted: this must run in the crsd-sim container"

# `nice`: the sim may be running; this is a pip install, not a deadline
PIP() { nice -n 10 "$VENV/bin/python" -m pip install --no-input --disable-pip-version-check "$@" </dev/null; }

if [ "$FORCE" = 1 ]; then rm -rf "$VENV"; fi
if [ ! -x "$VENV/bin/python" ]; then
  echo "  creating $VENV"
  mkdir -p "$(dirname "$VENV")"
  if ! "$PY" -m venv --system-site-packages "$VENV" >/dev/null 2>&1; then
    # the image has no python3-venv's ensurepip: build the venv bare and bootstrap
    # pip from PyPI (into the venv, not the image)
    rm -rf "$VENV"
    "$PY" -m venv --system-site-packages --without-pip "$VENV" || die "python3 -m venv"
    "$PY" - <<'PYEOF' || die "cannot download get-pip.py"
import urllib.request
urllib.request.urlretrieve("https://bootstrap.pypa.io/get-pip.py", "/tmp/get-pip.py")
PYEOF
    "$VENV/bin/python" /tmp/get-pip.py --no-warn-script-location >/dev/null || die "get-pip.py"
  fi
fi
"$VENV/bin/python" -m pip --version || die "the venv has no pip"

echo "  numpy < 2 and the headless opencv first, so nothing below can replace them"
PIP "numpy>=1.24,<2" "opencv-python-headless>=4.8,<4.12" || die "numpy / opencv"
echo "  torch + torchvision (CPU wheels, a few hundred MB)"
PIP torch torchvision --index-url https://download.pytorch.org/whl/cpu \
  || die "torch (cpu wheel index unreachable?)"
echo "  ultralytics $ULTRALYTICS (--no-deps: its opencv-python requirement would pull numpy 2)"
# the version both .pt files were trained with (their checkpoints say 8.4.144)
PIP --no-deps "ultralytics==$ULTRALYTICS" || die "ultralytics"
PIP "numpy>=1.24,<2" "opencv-python-headless>=4.8,<4.12" matplotlib pillow requests scipy psutil     polars ultralytics-thop cloudpickle nvidia-ml-py || die "ultralytics requirements"

echo "=== verifying with the venv's own interpreter"
# ROS is sourced the way gz_rig_up.sh sources it; set +u: ROS's setup.bash reads unset variables
set +u
source /opt/ros/humble/setup.bash
[ -f /root/robotx_ws/install/setup.bash ] && source /root/robotx_ws/install/setup.bash
set -u
"$VENV/bin/python" - <<'PYEOF' || die "the venv does not import cleanly (see above)"
import numpy, cv2, torch, ultralytics, rclpy, sys
assert numpy.__version__.startswith("1."), "numpy %s: ROS needs numpy 1" % numpy.__version__
print("  python", sys.version.split()[0])
print("  numpy", numpy.__version__, "| cv2", cv2.__version__, "| torch", torch.__version__,
      "| ultralytics", ultralytics.__version__, "| rclpy ok")
from crusader_msgs.msg import Detection3D            # a ROS message package, via system-site-packages
print("  crusader_msgs ok (the rig's ROS packages are visible from the venv)")
PYEOF
echo
echo "  done. SIM_DETECTOR=yolo bash crusader_sim/scripts/gz_sim_up.sh <course> uses it."
echo "  venv   $VENV   (host: ~/robotx_ws/venvs/yolo)"
echo "  models /root/robotx_ws/models/sim_yolo   (host: ~/robotx_ws/models/sim_yolo)"
