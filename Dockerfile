# ============================================================================
# Crusader container image (Jetson Orin Nano, JetPack 6 / arm64).
#
# Builds the `crusader` image the rest of the repo assumes:
#   * ROS 2 Humble + build toolchain (colcon, rosdep, rosidl generators)
#   * CUDA / PyTorch / TensorRT           -> from the ultralytics base
#   * depthai (OAK-D LR) + MAVProxy        -> installed here
#   * Livox-SDK2 + livox_ros_driver2 (MID360 LiDAR, camera-LiDAR fusion)
#
# The rx26_asv package itself is NOT copied/built here — it is bind-mounted
# at /root/robotx_ws and built at runtime by tools/scripts/rebuild.sh /
# setup/install_container.sh (the container copies files at build time, so a
# mounted+runtime build is the blessed path). Only third-party code (the Livox
# driver) is baked into the image, in its own workspace.
#
# Build (on the Jetson host):
#     docker build -t crusader .
# ============================================================================
FROM ultralytics/ultralytics:latest-jetson-jetpack6

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Pin the third-party Livox sources so the image is reproducible (CLAUDE.md
# guards against "unconstrained external dependencies"). Defaults track upstream
# master; once a commit is field-tested, set these to that tag/SHA — either via
# --build-arg or by editing the defaults — so a later upstream change can't
# silently alter the image. Accepts a branch, tag, or full commit SHA.
ARG LIVOX_SDK2_REF=master
ARG LIVOX_DRIVER_REF=master

# ----------------------------------------------------------------------------
# ROS 2 Humble apt source (the ultralytics base ships no ROS repo/key)
# ----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release software-properties-common ca-certificates \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
         > /etc/apt/sources.list.d/ros2.list

# ----------------------------------------------------------------------------
# ROS 2 Humble + full build toolchain.
#   ros-base alone can't build the `interfaces` ament_cmake/rosidl package;
#   ros-dev-tools brings colcon, rosdep, and the rosidl generators. cmake +
#   build-essential are also needed for Livox-SDK2. libusb for depthai/OAK-D.
# ----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-dev-tools \
        python3-colcon-common-extensions \
        build-essential cmake git \
        libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------------------------
# Python deps: the OAK-D lib + MAVProxy, plus the pure-python bits that
# setup/install_container.sh pins (keeps the image and that script in sync).
#   depthai<3 -> v2 API used by perception (USB3 SUPER-speed check in CLAUDE.md)
# ----------------------------------------------------------------------------
RUN uv pip install --system \
        "depthai<3" \
        pymavlink \
        MAVProxy \
        pyserial \
        future \
        pyyaml \
        protobuf \
        grpcio-tools \
        pytest

# ----------------------------------------------------------------------------
# Livox-SDK2 — native SDK for the MID360 (Ethernet/UDP device)
# ----------------------------------------------------------------------------
RUN mkdir -p /opt/livox && cd /opt/livox \
    && git clone https://github.com/Livox-SDK/Livox-SDK2.git \
    && cd Livox-SDK2 && git checkout "$LIVOX_SDK2_REF" \
    && mkdir build && cd build \
    && cmake .. && make -j"$(nproc)" && make install \
    && ldconfig

# ----------------------------------------------------------------------------
# livox_ros_driver2 (ROS 2 Humble) — third-party, baked into its own workspace
# so it is NOT rebuilt on every rx26_asv edit. Publishes /livox/lidar
# (PointCloud2 / CustomMsg) + /livox/imu for the fusion node to consume.
# MID360_config.json carries the host/LiDAR IPs — TUNE for Crusader's network
# (see config/MID360_config.json header notes; the LiDAR needs its own NIC,
# not the RoboCommand RJ-45 link).
# ----------------------------------------------------------------------------
ENV LIVOX_WS=/opt/livox_ws
RUN mkdir -p $LIVOX_WS/src && cd $LIVOX_WS/src \
    && git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
    && cd livox_ros_driver2 && git checkout "$LIVOX_DRIVER_REF"
COPY config/MID360_config.json $LIVOX_WS/src/livox_ros_driver2/config/MID360_config.json
RUN source /opt/ros/humble/setup.bash \
    && cd $LIVOX_WS/src/livox_ros_driver2 \
    && ./build.sh humble

# ----------------------------------------------------------------------------
# Environment sourcing: ROS, the baked Livox ws, then the mounted rx26_asv
# workspace (guarded — it only exists once the mount is built).
# ----------------------------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "[ -f /opt/livox_ws/install/setup.bash ] && source /opt/livox_ws/install/setup.bash" >> /root/.bashrc \
    && echo "[ -f /root/robotx_ws/install/setup.bash ] && source /root/robotx_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/robotx_ws
CMD ["bash"]
