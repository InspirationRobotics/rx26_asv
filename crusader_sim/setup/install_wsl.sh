#!/usr/bin/env bash
# install_wsl.sh — everything the Gazebo sim needs, in WSL2 Ubuntu-22.04.
#
# Runs in: WSL2 Ubuntu-22.04 (the same distro tools/sitl/SIM_UP.cmd targets).
#
#     # from Windows PowerShell — the apt stage needs root, and WSL's root login
#     # needs no password:
#     wsl -d Ubuntu-22.04 -u root -- bash <path>/install_wsl.sh apt
#     # everything else as your normal WSL user:
#     wsl -d Ubuntu-22.04 -- bash <path>/install_wsl.sh user
#
# Stages (idempotent — re-running a finished stage is a no-op or a rebuild):
#   apt   root   Gazebo Harmonic, ros-humble-ros-gzharmonic, ArduPilot + plugin
#                build deps. Adds the OSRF apt source.
#   user  user   ~/ardupilot at Rover-4.6.3 (the boat's EXACT release) built for
#                SITL; ~/ardupilot_gazebo (ArduPilotPlugin) built for Harmonic;
#                MAVProxy + pymavlink pinned to the asv image's version.
#
# WHY HUMBLE + HARMONIC, NOT JAZZY. The boat runs Humble. Running the sim on the
# same distro means the nodes under test are byte-for-byte what the boat runs,
# and the team's existing crsd-sim rig (Humble) talks to the bridge with no
# cross-distro DDS in between. Harmonic is the Gazebo that ardupilot_gazebo
# targets; OSRF ships ros_gz for Humble+Harmonic as ros-humble-ros-gzharmonic.
# That package CONFLICTS with ros-humble-ros-gz (Fortress) — this distro has
# neither installed, which is why it is safe here. If yours has Fortress, stop.
set -euo pipefail

stage="${1:-}"
AP_TAG="Rover-4.6.3"
PYMAVLINK_PIN="2.4.49"     # = Dockerfile's pin; the wire library must match the boat

apt_stage() {
  [[ $EUID -eq 0 ]] || { echo "apt stage needs root: wsl -d Ubuntu-22.04 -u root -- bash $0 apt" >&2; exit 1; }
  export DEBIAN_FRONTEND=noninteractive

  if dpkg -l ros-humble-ros-gz 2>/dev/null | grep -q '^ii'; then
    echo "ros-humble-ros-gz (Fortress) is installed and conflicts with ros-humble-ros-gzharmonic. Stopping." >&2
    exit 1
  fi

  if [[ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]]; then
    apt-get install -y --no-install-recommends curl lsb-release gnupg ca-certificates
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  fi
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

  # Unrelated third-party sources on a dev machine can fail `update` (an expired
  # ROS 1 key, a slow mirror). Tolerate that, then prove OUR source loaded.
  apt-get update || true
  apt-cache policy gz-harmonic | grep -q 'packages.osrfoundation.org' \
    || { echo "OSRF source did not load — gz-harmonic unavailable" >&2; exit 1; }

  apt-get install -y --no-install-recommends \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    build-essential ccache g++ gawk git make wget cmake pkg-config \
    python3-pip python3-dev python3-setuptools python3-wheel \
    python3-numpy python3-pyparsing python3-psutil python3-lxml python3-future \
    python3-pexpect python3-yaml python3-serial python3-scipy python3-opencv \
    python3-prompt-toolkit python3-matplotlib \
    libtool libxml2-dev libxslt1-dev \
    rapidjson-dev libopencv-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    libgz-sim8-dev \
    binutils
  echo "apt stage done: $(gz sim --versions 2>/dev/null | head -1 || echo 'gz not on PATH?')"
}

user_stage() {
  [[ $EUID -ne 0 ]] || { echo "run the user stage as your normal user, not root" >&2; exit 1; }
  export PATH="$HOME/.local/bin:$PATH"

  # --- MAVProxy / pymavlink, user site (no venv: a venv on PATH would hide
  #     rclpy from every ROS shell in this distro) ---
  # --no-deps IS LOAD-BEARING. A plain `pip install MAVProxy` drags numpy 2.x,
  # opencv-python 5 and matplotlib into ~/.local, which SHADOW the apt copies ROS
  # Humble is built against: cv_bridge then dies on "compiled using NumPy 1.x".
  # It did exactly that on 2026-09-28. numpy/cv2/matplotlib/serial come from apt
  # (apt stage); only the pure-python leaves are pip'd here.
  python3 -m pip install --user --no-deps "pymavlink==${PYMAVLINK_PIN}" MAVProxy \
    fastcrc pynmeagps defusedxml
  python3 -c "import numpy; assert numpy.__version__.startswith('1.'), numpy.__file__" \
    || { echo "numpy 2 is shadowing the apt numpy in ~/.local — ROS will break" >&2; exit 1; }

  # --- ArduPilot at the boat's exact release ---
  if [[ ! -d "$HOME/ardupilot/.git" ]]; then
    git clone https://github.com/ArduPilot/ardupilot.git "$HOME/ardupilot"
  fi
  cd "$HOME/ardupilot"
  git fetch --tags --quiet
  git checkout --quiet "$AP_TAG"
  git submodule update --init --recursive --quiet
  ./waf configure --board sitl > /tmp/ap_configure.log 2>&1 \
    || { tail -30 /tmp/ap_configure.log; exit 1; }
  ./waf rover > /tmp/ap_build.log 2>&1 || { tail -40 /tmp/ap_build.log; exit 1; }
  strings build/sitl/bin/ardurover | grep -oE 'ArduRover V[0-9.]+' | sort -u | head -1

  # --- ardupilot_gazebo (ArduPilotPlugin) for Harmonic ---
  if [[ ! -d "$HOME/ardupilot_gazebo/.git" ]]; then
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$HOME/ardupilot_gazebo"
  fi
  cd "$HOME/ardupilot_gazebo"
  export GZ_VERSION=harmonic
  mkdir -p build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo > /tmp/apgz_cmake.log 2>&1 \
    || { tail -30 /tmp/apgz_cmake.log; exit 1; }
  make -j"$(nproc)" > /tmp/apgz_build.log 2>&1 || { tail -40 /tmp/apgz_build.log; exit 1; }
  ls -1 "$HOME/ardupilot_gazebo/build/"*.so
  echo "user stage done."
}

case "$stage" in
  apt)  apt_stage ;;
  user) user_stage ;;
  *) echo "usage: $0 apt|user" >&2; exit 2 ;;
esac
