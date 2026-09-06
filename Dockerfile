# ============================================================================
# Crusader container image (Jetson Orin Nano, JetPack 6 / arm64).
#
# Builds the `asv` image the rest of the repo assumes:
#   * CUDA / PyTorch / TensorRT   -> from the ultralytics base (detection runs HERE)
#   * ROS 2 Humble + build toolchain (colcon, rosdep, rosidl generators)
#   * depthai                     -> crusader_perception OWNS the OAK-D, in here
#   * cv_bridge + sensor_msgs_py  -> Image/PointCloud2 <-> numpy
#   * MAVProxy + pymavlink + pyserial -> the autopilot and LED serial links
#
# There are exactly TWO containers on this Jetson:
#   `asv`  (this one) — everything in this repo, INCLUDING the OAK-D nodes.
#   livox             — the MID360 driver and nothing else; publishes PointCloud2.
#
# So depthai belongs here and the Livox SDK does not. That is the reverse of the
# original plan, in which a single sensor container owned both devices and this
# image was forbidden depthai: `buoy_detector` needs the camera AND TensorRT in
# one process, because shipping 1.28 MB frames across a container boundary to
# produce a few hundred bytes of detection is the wrong trade. The absence guard
# below now protects the boundary that is actually real.
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
#   cv_bridge + sensor_msgs_py: Image -> numpy for the detector's bring-up
#   frames, PointCloud2 -> numpy for the MID360 cloud the livox container
#   publishes. Without them this image builds the packages but cannot consume a
#   single frame or point.
# ----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-dev-tools \
        python3-colcon-common-extensions \
        ros-humble-cv-bridge \
        ros-humble-sensor-msgs-py \
        ros-humble-std-srvs \
        ros-humble-behaviortree-cpp \
        build-essential cmake git \
    && rm -rf /var/lib/apt/lists/*
# behaviortree_cpp is 4.x from apt, which is what crusader_bt's XML declares
# (BTCPP_format="4"). Note it is the CORE library only: behaviortree_ros2, the
# package that provides RosActionNode, is NOT released for Humble. We do not
# need it — our leaves publish a setpoint and poll a topic rather than calling
# an action, the same shape OUXT-Polaris used at RobotX 2022.

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
# real Jetson, via grpcio-tools. The guard below is what catches a recurrence —
# and depthai is exactly the kind of package that could do it again, which is
# why it goes in HERE, above the guard, rather than being pip-installed into a
# running container where nothing would check.
# ----------------------------------------------------------------------------
RUN uv pip install --system \
        depthai \
        "pymavlink==2.4.49" \
        MAVProxy \
        pyserial \
        future \
        "pyyaml==6.0.3"

# Fail the BUILD, not the boat, if the ML stack, the MAVLink stack or the camera
# SDK is broken. The TensorFlow import is slow (~1 min) and worth every second.
RUN python3 -c "\
import google.protobuf, tensorflow, ultralytics, pymavlink, serial, yaml, depthai; \
from pymavlink import mavutil; \
v = google.protobuf.__version__; \
assert v.startswith('5.29'), 'protobuf moved to %s — TF requires <6.0.0dev' % v; \
print('dep guard ok: protobuf', v, '| tensorflow', tensorflow.__version__, \
      '| ultralytics', ultralytics.__version__, '| pymavlink', pymavlink.__version__, \
      '| depthai', depthai.__version__)"

# The depthai absence guard that used to live here is GONE: this image owns the
# OAK-D now, so asserting depthai is absent would fail the build it exists to
# protect. It is replaced by the positive import check above.
#
# The equivalent boundary that IS still real is the MID360 — the livox container
# owns it, and two processes opening one LiDAR is the same failure the old guard
# was about. No check is written for it yet ON PURPOSE: the marker would be a
# guess at the Livox SDK's installed name, and a guard that never fires because
# the name is wrong is worse than no guard (same reasoning as check_config.py's
# param-baseline parse check). Add it here once someone confirms, in the livox
# container, what the SDK actually installs — a `ros2 pkg prefix
# livox_ros_driver2` or the SDK's real library path.

# ----------------------------------------------------------------------------
# Environment sourcing: ROS, then the mounted workspace (guarded — it only
# exists once the mount is built).
# ----------------------------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "[ -f /root/robotx_ws/install/setup.bash ] && source /root/robotx_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/robotx_ws
CMD ["bash"]
