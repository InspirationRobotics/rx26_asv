# ============================================================================
# Crusader container image (Jetson Orin Nano, JetPack 6 / arm64).
#
# Builds the `asv` image the rest of the repo assumes: ROS 2 Humble + the build
# toolchain + MAVProxy/pymavlink/pyserial. That is the whole dependency list,
# because that is the whole stack — telemetry bridge, RC watchdog, LED nodes.
#
# NO CUDA, depthai, TensorRT, PCL or Livox here. Camera and LiDAR run in their
# OWN container; this image deliberately cannot see either device, which is what
# keeps a perception dependency from creeping back into the safety stack. If you
# find yourself wanting depthai in this file, the node you are writing belongs in
# the other container.
#
# The rx26_asv package itself is NOT copied/built here — it is bind-mounted
# at /root/robotx_ws and built at runtime by tools/scripts/rebuild.sh /
# setup/install_container.sh (the container copies files at build time, so a
# mounted+runtime build is the blessed path).
#
# Build (on the Jetson host):
#     docker build -t asv .
# ============================================================================
FROM arm64v8/ros:humble-ros-base

# ARG, not ENV: ENV persists into the running container, so every later `apt`
# run inside the boat's container would silently accept defaults instead of
# prompting. Build-time only is what we actually want.
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ----------------------------------------------------------------------------
# Build toolchain. ros-base alone cannot build the `interfaces` ament_cmake/
# rosidl package; ros-dev-tools brings colcon, rosdep and the rosidl generators.
# ----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-dev-tools \
        python3-colcon-common-extensions \
        python3-pip \
        build-essential cmake git \
        ros-humble-std-srvs \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------------------------
# Python deps. THE IMAGE IS THE ONLY PLACE RUNTIME DEPS ARE INSTALLED — nothing
# pip-installs into a running container (that state is undocumented and lost on
# `docker rm`).
#
# pymavlink is PINNED: it is the wire library for the only process that talks to
# the autopilot, and an upgrade that changes message parsing is not something to
# discover on the water.
# ----------------------------------------------------------------------------
RUN pip3 install --no-cache-dir \
        "pymavlink==2.4.49" \
        MAVProxy \
        pyserial \
        future \
        "pyyaml==6.0.3"

# Fail the BUILD, not the boat, if the MAVLink stack does not import.
RUN python3 -c "\
import pymavlink, serial, yaml; \
from pymavlink import mavutil; \
print('dep guard ok: pymavlink', pymavlink.__version__, '| pyserial', serial.__version__)"

# ----------------------------------------------------------------------------
# Environment sourcing: ROS, then the mounted rx26_asv workspace (guarded — it
# only exists once the mount is built).
# ----------------------------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "[ -f /root/robotx_ws/install/setup.bash ] && source /root/robotx_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/robotx_ws
CMD ["bash"]
