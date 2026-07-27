#!/usr/bin/env bash
# Install Gazebo Harmonic + ardupilot_gazebo in the crusader container (one-time).
#
# Companion to install_sitl.sh. That script pins ArduRover 4.6.3 because "Level-1
# param evaluation is only honest if SITL runs the same firmware logic as the
# Pixhawk" — the same argument applies here: the FDM must be reproducible, so
# Gazebo is pinned to the Harmonic LTS that ardupilot_gazebo targets.
#
# Adds the OmniX-capable FDM path. See docs/GAZEBO_BACKEND.md.
set -euo pipefail

GZ_VERSION="${GZ_VERSION:-harmonic}"
AP_GZ_DIR="${AP_GZ_DIR:-/root/ardupilot_gazebo}"

echo "=== Gazebo ${GZ_VERSION} ==="
if ! command -v gz >/dev/null 2>&1; then
  apt-get update
  apt-get install -y curl lsb-release gnupg
  curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list
  apt-get update
  apt-get install -y "gz-${GZ_VERSION}"
else
  echo "gz already present — skipping"
fi

echo "=== ardupilot_gazebo plugin ==="
apt-get install -y libgz-sim8-dev rapidjson-dev \
  libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl

if [[ -d "$AP_GZ_DIR" ]]; then
  echo "$AP_GZ_DIR exists — skipping clone (delete it to force reinstall)."
else
  git clone https://github.com/ArduPilot/ardupilot_gazebo "$AP_GZ_DIR"
fi

cd "$AP_GZ_DIR"
export GZ_VERSION
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j"$(nproc)"

echo
echo "OK. Next:"
echo "  python3 tools/sim/scenario_to_world.py --all"
echo "  SCENARIO=orchestrator/scenarios/mission1_transit.json docker/sitl/run_sitl_gazebo.sh"
