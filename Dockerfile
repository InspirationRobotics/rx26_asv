# ============================================================================
# Crusader container image (Jetson Orin Nano, JetPack 6 / arm64).
#
# Builds the `asv` image the rest of the repo assumes:
#   * CUDA / PyTorch / TensorRT   -> from the ultralytics base (detection runs HERE)
#   * ROS 2 Humble + build toolchain (colcon, rosdep, rosidl generators)
#   * cv_bridge + sensor_msgs_py  -> consume Image/PointCloud2 from the sensor container
#   * MAVProxy + pymavlink + pyserial -> the autopilot and LED serial links
#
# WHAT IS DELIBERATELY ABSENT: depthai and the Livox SDK/driver. A SEPARATE
# sensor-driver container owns the OAK-D and the MID360 and publishes their raw
# data as ROS topics; this image consumes those topics. It should therefore be
# impossible to open either device from here even by accident. If you find
# yourself adding a device SDK to this file, the node you are writing belongs in
# the sensor container instead.
#
# The rx26_asv packages are NOT copied/built here — they are bind-mounted at
# /root/robotx_ws and built at runtime by tools/scripts/rebuild.sh /
# setup/install_container.sh (the container copies files at build time, so a
# mounted+runtime build is the blessed path).
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
#   ros-base alone can't build the `crusader_msgs` ament_cmake/rosidl package;
#   ros-dev-tools brings colcon, rosdep, and the rosidl generators.
#   cv_bridge + sensor_msgs_py are the seam with the sensor container: Image ->
#   numpy for the detector, PointCloud2 -> numpy for the world model. Without
#   them this image can build the packages but not consume a single frame.
# ----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-dev-tools \
        python3-colcon-common-extensions \
        ros-humble-cv-bridge \
        ros-humble-sensor-msgs-py \
        ros-humble-std-srvs \
        build-essential cmake git \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------------------------
# Python deps. THE IMAGE IS THE ONLY PLACE RUNTIME DEPS ARE INSTALLED — nothing
# pip-installs into a running container (that state is undocumented and lost on
# `docker rm`).
#
# pymavlink is PINNED: it is the wire library for the only process that talks to
# the autopilot, and an upgrade that changes message parsing is not something to
# discover on the water.
#
# Nothing here may move protobuf. The ultralytics base ships protobuf 5.29.x,
# which its TensorFlow requires (<6.0.0dev); an unpinned package that drags in a
# newer protobuf breaks the ML stack the detector runs on. That happened on the
# real Jetson, via grpcio-tools. The guard below is what catches a recurrence.
# ----------------------------------------------------------------------------
RUN uv pip install --system \
        "pymavlink==2.4.49" \
        MAVProxy \
        pyserial \
        future \
        "pyyaml==6.0.3"

# Fail the BUILD, not the boat, if the ML stack or the MAVLink stack is broken.
# The TensorFlow import is slow (~1 min) and worth every second of it.
RUN python3 -c "\
import google.protobuf, tensorflow, ultralytics, pymavlink, serial, yaml; \
from pymavlink import mavutil; \
v = google.protobuf.__version__; \
assert v.startswith('5.29'), 'protobuf moved to %s — TF requires <6.0.0dev' % v; \
print('dep guard ok: protobuf', v, '| tensorflow', tensorflow.__version__, \
      '| ultralytics', ultralytics.__version__, '| pymavlink', pymavlink.__version__)"

# Guard the absence, too: a device SDK reappearing here means perception drifted
# back into the wrong container. Cheap to check, and it fails at build time
# rather than as a mystery second client on the boat.
RUN ! python3 -c "import depthai" 2>/dev/null || \
    { echo "ERROR: depthai is in this image. The sensor container owns the OAK-D." >&2; exit 1; }

# ----------------------------------------------------------------------------
# Environment sourcing: ROS, then the mounted workspace (guarded — it only
# exists once the mount is built).
# ----------------------------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "[ -f /root/robotx_ws/install/setup.bash ] && source /root/robotx_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/robotx_ws
CMD ["bash"]
