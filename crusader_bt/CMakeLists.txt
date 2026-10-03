cmake_minimum_required(VERSION 3.8)
project(crusader_bt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(crusader_msgs REQUIRED)

set(DEPS rclcpp rclcpp_action std_msgs behaviortree_cpp crusader_msgs)

# NAV2 IS OPTIONAL AT BUILD TIME. NavigateTo and CircleBuoy plan through Nav2's
# planner_server when nav_mode is shadow or on (docs/nav2_avoidance_spec.md), and
# src/ros_planner_port.cpp is the only file that talks to it. The boat's `asv`
# image has no Nav2 until it is recreated (spec 8.4), and rebuild.sh builds this
# package, so a hard find_package here would break the boat's build for a feature
# it cannot run yet. Without nav2_msgs this builds with a stub that returns nullptr,
# and bt_runner_node refuses to start with nav_mode other than "off" ("built without
# nav2_msgs"). nav_mode off is the default, so the boat behaves as it always did.
#
# Build WITHOUT it on a machine that has it (the acceptance check):
#   colcon build --packages-select crusader_bt --build-base /tmp/b_nonav \
#     --install-base /tmp/i_nonav --cmake-args -DCMAKE_DISABLE_FIND_PACKAGE_nav2_msgs=TRUE
find_package(nav2_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(nav_msgs QUIET)
set(CRSD_HAVE_NAV2 FALSE)
if(nav2_msgs_FOUND AND geometry_msgs_FOUND AND nav_msgs_FOUND)
  set(CRSD_HAVE_NAV2 TRUE)
  list(APPEND DEPS nav2_msgs geometry_msgs nav_msgs)
  message(STATUS "crusader_bt: nav2_msgs found, nav_mode shadow and on are available")
else()
  message(STATUS "crusader_bt: nav2_msgs not found, built WITHOUT the Nav2 planner port "
    "(image without Nav2): bt_runner_node must run with nav_mode off")
endif()

# task3_leaves.cpp is NOT optional: leaves.cpp's registerCrusaderNodes() calls
# registerTask3Nodes(), so leaving it out is a link error, not a missing tree.
# fire_leaves.cpp likewise (registerFireNodes). ros_planner_port.cpp is always
# compiled: with Nav2 it is the planner port, without it a stub.
add_executable(bt_runner_node src/bt_runner_node.cpp src/leaves.cpp src/task3_leaves.cpp
  src/fire_leaves.cpp src/ros_planner_port.cpp)
target_include_directories(bt_runner_node PRIVATE include)
ament_target_dependencies(bt_runner_node ${DEPS})
if(CRSD_HAVE_NAV2)
  target_compile_definitions(bt_runner_node PRIVATE CRSD_HAVE_NAV2)
endif()

install(TARGETS bt_runner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# nav_math is header-only and stdlib-only ON PURPOSE, so its test needs neither
# ROS nor colcon:
#
#     g++ -std=c++17 -O2 -I include -o t test/test_nav_math.cpp && ./t
#
# It is registered here too so `colcon test` runs it, but the laptop path is the
# one that gets used — the whole point is that a sign error in the side rule is
# caught in a second, off-boat, rather than by watching the boat pass a buoy on
# the wrong side.
#
# dock_math is the same deal for Task 3 (test/test_dock_math.cpp). Off-ROS the
# whole Task 3 tree - these leaves, that XML - also runs against a simulated
# course: tools/task3_sim/ (build.py, then sim.py). offros/ is that runner and
# is deliberately NOT built here.
#
# path_math and planned_leg are the same again for obstacle avoidance: the
# geometry the planned legs and the costmap share, and the leg state machine
# against a scripted fake planner (no Nav2, no clock of its own).
if(BUILD_TESTING)
  add_executable(test_nav_math test/test_nav_math.cpp)
  target_include_directories(test_nav_math PRIVATE include)
  add_test(NAME nav_math COMMAND test_nav_math)
  add_executable(test_dock_math test/test_dock_math.cpp)
  target_include_directories(test_dock_math PRIVATE include)
  add_test(NAME dock_math COMMAND test_dock_math)
  # fire_math: the fixed-nozzle shot (range/heading hold, steadiness, bursts).
  add_executable(test_fire_math test/test_fire_math.cpp)
  target_include_directories(test_fire_math PRIVATE include)
  add_test(NAME fire_math COMMAND test_fire_math)
  add_executable(test_path_math test/test_path_math.cpp)
  target_include_directories(test_path_math PRIVATE include)
  add_test(NAME path_math COMMAND test_path_math)
  # side_fences: the Task 1 side walls, and a reference grid planner that drives the legs.
  add_executable(test_side_fences test/test_side_fences.cpp)
  target_include_directories(test_side_fences PRIVATE include)
  add_test(NAME side_fences COMMAND test_side_fences)
  # gate_waypoints: GateWaypoint moves the gate's two points off anything on the crossing.
  add_executable(test_gate_waypoints test/test_gate_waypoints.cpp)
  target_include_directories(test_gate_waypoints PRIVATE include)
  add_test(NAME gate_waypoints COMMAND test_gate_waypoints)
  add_executable(test_planned_leg test/test_planned_leg.cpp)
  target_include_directories(test_planned_leg PRIVATE include)
  add_test(NAME planned_leg COMMAND test_planned_leg)
endif()

ament_package()
