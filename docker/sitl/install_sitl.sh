#!/usr/bin/env bash
# Install ArduPilot Rover SITL inside the crusader container (one-time).
# Pinned to the firmware on the boat: ArduRover 4.6.3 — Level-1 param evaluation is
# only honest if SITL runs the same firmware logic as the Pixhawk.
set -euo pipefail

SITL_DIR="${SITL_DIR:-/root/ardupilot}"
TAG="Rover-4.6.3"

if [[ -d "$SITL_DIR" ]]; then
  echo "$SITL_DIR exists — skipping clone (delete it to force reinstall)."
else
  git clone --depth 1 --branch "$TAG" https://github.com/ArduPilot/ardupilot.git "$SITL_DIR"
  cd "$SITL_DIR"
  git submodule update --init --recursive --depth 1
fi

cd "$SITL_DIR"
# container is Ubuntu-based (jetson-jetpack6 image); prereqs script handles deps
Tools/environment_install/install-prereqs-ubuntu.sh -y || {
  echo "prereqs script failed — install python3-dev g++ pkg-config libtool by hand"; exit 1; }

./waf configure --board sitl
./waf rover

echo "OK. Launch with docker/sitl/run_sitl.sh"
