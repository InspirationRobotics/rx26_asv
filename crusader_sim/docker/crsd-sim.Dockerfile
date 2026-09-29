# crsd-sim — the x86 stand-in for the boat's `asv` container, for the sim rig.
#
# Runs in: WSL2 Ubuntu-22.04 (docker). tools/sitl/sim_up.sh and gz_sim_up.sh
# expect a container named `crsd-sim`, NET=host, ~/robotx_ws bind-mounted at
# /root/robotx_ws (docs/G6_task1_disruptive_handoff.md:100).
#
# The boat's Dockerfile cannot build here: its base is the ultralytics JETSON
# image (arm64, JetPack). This mirrors its ROS and Python layer only — no CUDA,
# no TensorRT, no depthai (there is no OAK-D in a sim; crusader_sim's oak_shim
# publishes the frames instead). pymavlink is pinned to the boat's version for
# the same reason the boat pins it.
#
#   docker build -t crsd-sim:humble -f crusader_sim/docker/crsd-sim.Dockerfile .
#   docker run -d --name crsd-sim --net=host --ipc=host \
#       -v ~/robotx_ws:/root/robotx_ws crsd-sim:humble sleep infinity
#
# --ipc=host IS LOAD-BEARING: the Gazebo bridge and sim shims run on the WSL
# host, the boat's nodes in here. Fast DDS sees "same machine" and moves data
# over shared memory, which fails silently across IPC namespaces — the LiDAR
# topic would list, and never deliver.
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-dev-tools \
        python3-colcon-common-extensions \
        ros-humble-cv-bridge \
        ros-humble-sensor-msgs-py \
        ros-humble-std-srvs \
        ros-humble-behaviortree-cpp \
        python3-pip python3-numpy python3-serial python3-opencv \
        build-essential cmake git iproute2 procps curl \
    && rm -rf /var/lib/apt/lists/*

# same arch-generic lib/ link the boat's Dockerfile makes, same reason
RUN L="/opt/ros/humble/lib/$(uname -m)-linux-gnu/libbehaviortree_cpp.so"; \
    [ -e /opt/ros/humble/lib/libbehaviortree_cpp.so ] || ln -s "$L" /opt/ros/humble/lib/libbehaviortree_cpp.so

RUN pip3 install --no-cache-dir "pymavlink==2.4.49" pyserial future "pyyaml==6.0.3"

# The Gazebo -> ROS bridge lives IN HERE, so every ROS message stays inside one
# container the way it does on the boat, and only gz-transport crosses to the
# WSL host (host network). Harmonic's bridge for Humble comes from OSRF.
# GZ_PARTITION must match the host's: gz-transport's default partition is
# "hostname:username", and root in here != chaser out there — the two would
# never discover each other and nothing would say so.
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-gzharmonic-bridge \
    && rm -rf /var/lib/apt/lists/*
ENV GZ_PARTITION=crusader_sim

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "[ -f /root/robotx_ws/install/setup.bash ] && source /root/robotx_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/robotx_ws
CMD ["bash"]
