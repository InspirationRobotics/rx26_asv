# ============================================================================
# Crusader container image (Jetson Orin Nano, JetPack 6 / arm64).
#
# Builds the `asv` image the rest of the repo assumes:
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
#     docker build -t asv .
# ============================================================================
FROM ultralytics/ultralytics:latest-jetson-jetpack6

# ARG, not ENV: ENV persists into the running container, so every later `apt`
# run inside the boat's container would silently accept defaults instead of
# prompting. Build-time only is what we actually want.
ARG DEBIAN_FRONTEND=noninteractive
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
#   libpcl-dev + pcl_conversions are REQUIRED by livox_ros_driver2:
#   its CMakeLists does find_package(PCL REQUIRED), and without it the driver
#   build fails at configure time. It is a heavy dependency (pulls VTK/Boost/
#   FLANN) but there is no building the MID360 driver without it.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-dev-tools \
        python3-colcon-common-extensions \
        build-essential cmake git \
        libusb-1.0-0-dev \
        libpcl-dev \
        ros-humble-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------------------------
# Python deps. THE IMAGE IS THE ONLY PLACE RUNTIME DEPS ARE INSTALLED — nothing
# pip-installs into a running container (that state is undocumented and lost on
# `docker rm`).
#
# Everything that can move protobuf is PINNED, and for a specific reason: the
# ultralytics base ships protobuf 5.29.6, which TensorFlow 2.19 requires
# (<6.0.0dev). An unpinned `grpcio-tools` resolves to a 7.x-era protobuf and
# breaks the ML stack this boat's perception runs on. That happened on the real
# Jetson. Do not unpin these without re-running the guard below.
#   depthai<3 -> v2 API used by perception (USB3 SUPER-speed check in CLAUDE.md)
# ----------------------------------------------------------------------------
RUN uv pip install --system \
        "depthai<3" \
        "pymavlink==2.4.49" \
        MAVProxy \
        pyserial \
        future \
        "pyyaml==6.0.3" \
        "protobuf==5.29.6" \
        "grpcio==1.82.1" \
        "grpcio-tools==1.71.0" \
        pytest

# Fail the BUILD, not the boat, if a dependency resolution moved protobuf out
# from under TensorFlow/ultralytics. This import is slow (~1 min on TF) and
# worth every second of it.
RUN python3 -c "\
import google.protobuf, tensorflow, ultralytics; \
v = google.protobuf.__version__; \
assert v.startswith('5.29'), 'protobuf moved to %s — TF requires <6.0.0dev' % v; \
print('dep guard ok: protobuf', v, '| tensorflow', tensorflow.__version__, \
      '| ultralytics', ultralytics.__version__)"

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
COPY rx26_asv/config/MID360_config.json $LIVOX_WS/src/livox_ros_driver2/config/MID360_config.json
# build.sh EXITS 0 EVEN WHEN COLCON FAILS. A missing PCL made the configure step
# error out, colcon returned rc=1, build.sh swallowed it, and the image shipped
# with an empty install space — `ls /opt/livox_ws/install` looked fine while
# `ros2 pkg list` had no livox at all. Never trust build.sh's exit code; assert
# the package is actually discoverable, and dump the log if it is not.
RUN source /opt/ros/humble/setup.bash \
    && cd $LIVOX_WS/src/livox_ros_driver2 \
    && ./build.sh humble; \
    source $LIVOX_WS/install/setup.bash 2>/dev/null || true; \
    if ! ros2 pkg list 2>/dev/null | grep -qx livox_ros_driver2; then \
        echo "ERROR: livox_ros_driver2 is not on the ROS path after build.sh." >&2; \
        echo "       (build.sh exits 0 even when colcon fails — log tail below)" >&2; \
        tail -60 "$LIVOX_WS"/log/latest_build/events.log >&2 2>/dev/null || true; \
        exit 1; \
    fi; \
    echo "livox guard ok: livox_ros_driver2 built and discoverable"

# ----------------------------------------------------------------------------
# Environment sourcing: ROS, the baked Livox ws, then the mounted rx26_asv
# workspace (guarded — it only exists once the mount is built).
# ----------------------------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "[ -f /opt/livox_ws/install/setup.bash ] && source /opt/livox_ws/install/setup.bash" >> /root/.bashrc \
    && echo "[ -f /root/robotx_ws/install/setup.bash ] && source /root/robotx_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/robotx_ws
CMD ["bash"]
