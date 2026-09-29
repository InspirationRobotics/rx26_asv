# ============================================================================
# asv_add_bt.Dockerfile — BehaviorTree.CPP on top of the image the boat RUNS.
#
# WHY NOT JUST REBUILD ../Dockerfile. The boat's `asv` container runs
# asv:socket-20260903, which is not a build of the repo's Dockerfile: its
# history has steps that file no longer has (the Livox SDK/driver build), and
# whatever else was added by hand since is not written down anywhere. A clean
# rebuild could drop any of it. This adds the ONE thing crusader_bt needs to
# that exact image, and nothing else, so the swap cannot lose anything.
#
# The repo's Dockerfile already carries the same two steps, so a future clean
# image has them too.
#
# On the Jetson host (2026-09-28):
#     cd ~/robotx_ws/src/rx26_asv/setup
#     docker build -f asv_add_bt.Dockerfile -t asv:bt-20260928 .
# then recreate the `asv` container from asv:bt-20260928 with the SAME flags
# (setup/README.md, "Recreating the asv container"), keeping the old one as
# asv_pre_bt for rollback.
# ============================================================================
ARG BASE=asv:socket-20260903
FROM ${BASE}

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# BehaviorTree.CPP 4.x from the ROS apt repo - what crusader_bt's XML declares
# (BTCPP_format="4"). The ROS apt source is already in the base image.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-behaviortree-cpp \
    && rm -rf /var/lib/apt/lists/*

# The apt package puts libbehaviortree_cpp.so under lib/<arch>-linux-gnu/,
# while ament_cmake_export_libraries looks in lib/: without this link
# find_package(behaviortree_cpp) fails, saying nothing about paths. (Same fix
# as ../Dockerfile.)
RUN ln -sf "/opt/ros/humble/lib/$(uname -m)-linux-gnu/libbehaviortree_cpp.so" \
           /opt/ros/humble/lib/libbehaviortree_cpp.so

# Fail the BUILD, not the boat, if CMake still cannot find it.
RUN test -f /opt/ros/humble/share/behaviortree_cpp/cmake/behaviortree_cppConfig.cmake \
    && test -e /opt/ros/humble/lib/libbehaviortree_cpp.so \
    && echo "behaviortree_cpp ok"
