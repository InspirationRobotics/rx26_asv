# ============================================================================
# asv_add_nav2.Dockerfile - Nav2 planner + costmap (+ STVL) on top of the image the boat RUNS.
#
# WHY A LAYER: see asv_add_bt.Dockerfile. The boat's asv container runs an image
# that is not a build of the repo's Dockerfile, so this adds the packages the
# tree's planned legs need to that exact image and nothing else.
# Spec: docs/nav2_avoidance_spec.md section 8.
#
# On the Jetson host the BASE build-arg is the image the live asv container runs
# (read it from docker inspect asv, Config.Image); the exact commands, the smoke
# test, the swap and the rollback are in setup/README.md, section "Recreating the
# asv container". Do not build this against anything but the boat's own image.
# ============================================================================
ARG BASE=asv:bt-20260928
FROM ${BASE}

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Record what the base had, so the package delta is reviewable after the fact.
RUN dpkg -l 'ros-humble-*' > /opt/crsd_pre_nav2_dpkg.txt

# BehaviorTree.CPP too, idempotently: crusader_bt is what calls the planner, and the
# boat image may still be asv:socket-20260903 (study section 4, step 5).
# tf2-ros-py is normally in ros-base already; naming it makes the image say so.
# navfn is installed only as the A/B fallback to SmacPlanner2D.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-behaviortree-cpp \
        ros-humble-nav2-planner ros-humble-nav2-smac-planner ros-humble-nav2-navfn-planner \
        ros-humble-nav2-costmap-2d ros-humble-nav2-lifecycle-manager ros-humble-nav2-msgs \
        ros-humble-nav2-util ros-humble-nav2-core ros-humble-spatio-temporal-voxel-layer \
        ros-humble-tf2-ros-py \
    && rm -rf /var/lib/apt/lists/*

# Same arch-generic lib link as asv_add_bt.Dockerfile, and ONLY when needed: an
# unconditional ln -sf would replace a real library with a link to a missing file.
RUN L=/opt/ros/humble/lib; A="$L/$(uname -m)-linux-gnu/libbehaviortree_cpp.so"; \
    if [ ! -e "$L/libbehaviortree_cpp.so" ] && [ -e "$A" ]; then ln -s "$A" "$L/libbehaviortree_cpp.so"; fi

RUN dpkg -l 'ros-humble-*' > /opt/crsd_post_nav2_dpkg.txt

# Fail the BUILD, not the boat.
RUN source /opt/ros/humble/setup.bash \
    && ros2 pkg prefix nav2_planner && ros2 pkg prefix nav2_smac_planner \
    && ros2 pkg prefix nav2_costmap_2d && ros2 pkg prefix spatio_temporal_voxel_layer \
    && ros2 pkg prefix behaviortree_cpp && python3 -c "import tf2_ros" \
    && test -f /opt/ros/humble/share/behaviortree_cpp/cmake/behaviortree_cppConfig.cmake \
    && echo "nav2 layer ok"
