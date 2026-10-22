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
find_package(ament_index_cpp REQUIRED)

set(DEPS rclcpp rclcpp_action std_msgs behaviortree_cpp crusader_msgs ament_index_cpp)

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

# The leaves, the planner port and the mission loader (mission_loader.cpp: how a mission name
# becomes a tree, built-in or uploaded), shared by bt_runner_node and bt_tree_check as a static
# library so each file compiles once. task3_leaves.cpp is NOT optional: leaves.cpp's
# registerCrusaderNodes() calls registerTask3Nodes(), so leaving it out is a link error, not a
# missing tree. fire_leaves.cpp likewise (registerFireNodes). ros_planner_port.cpp is always
# compiled: with Nav2 it is the planner port, without it a stub.
add_library(crusader_bt_leaves STATIC src/leaves.cpp src/task3_leaves.cpp src/fire_leaves.cpp
  src/global_leaves.cpp src/ros_planner_port.cpp src/mission_loader.cpp)
target_include_directories(crusader_bt_leaves PUBLIC include)
add_executable(bt_runner_node src/bt_runner_node.cpp)
target_include_directories(bt_runner_node PRIVATE include)
# Loads each tree file the way the runner does and says OK or FAIL per file; nothing is ticked.
# Run it on a mission file before it goes near a boat: bt_tree_check behavior_trees/*.xml
add_executable(bt_tree_check src/bt_tree_check.cpp)
target_include_directories(bt_tree_check PRIVATE include)
# The whole-field Task 1 planner (global_passage.hpp, instantiated in global_leaves.cpp) is a
# grid A* run inside the tick: ~50 ms optimised, ~10x that at the default -O0 (SITL
# 2026-10-02: 300-700 ms per plan). Optimise that one file whatever the build type says.
set_source_files_properties(src/global_leaves.cpp PROPERTIES COMPILE_OPTIONS "-O2")
ament_target_dependencies(crusader_bt_leaves ${DEPS})
ament_target_dependencies(bt_runner_node ${DEPS})
ament_target_dependencies(bt_tree_check ${DEPS})
target_link_libraries(bt_runner_node crusader_bt_leaves)
target_link_libraries(bt_tree_check crusader_bt_leaves)
if(CRSD_HAVE_NAV2)
  target_compile_definitions(crusader_bt_leaves PUBLIC CRSD_HAVE_NAV2)
endif()

install(TARGETS bt_runner_node bt_tree_check DESTINATION lib/${PROJECT_NAME})
# behavior_trees/ and everything under it: the missions, and lib/ with the task subtrees they include.
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# The maths headers are stdlib-only, so their tests also build without ROS:
#     g++ -std=c++17 -O2 -I include -o t test/test_nav_math.cpp && ./t
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
  # global_passage: the whole-field Task 1 planner and its follower, closed loop.
  add_executable(test_global_passage test/test_global_passage.cpp)
  target_include_directories(test_global_passage PRIVATE include)
  add_test(NAME global_passage COMMAND test_global_passage)
  add_executable(test_planned_leg test/test_planned_leg.cpp)
  target_include_directories(test_planned_leg PRIVATE include)
  add_test(NAME planned_leg COMMAND test_planned_leg)
  # nav_params: the nav_* planner knobs as one table - declared, re-read at goal accept, checked, diffed.
  add_executable(test_nav_params test/test_nav_params.cpp)
  target_include_directories(test_nav_params PRIVATE include)
  add_test(NAME nav_params COMMAND test_nav_params)
  # run_start: what the runner recognises as RoboCommand's RunStart, and what WaitForRunStart counts.
  add_executable(test_run_start test/test_run_start.cpp)
  target_include_directories(test_run_start PRIVATE include)
  add_test(NAME run_start COMMAND test_run_start)
  # mission_nodes: WaitForRunStart, TaskAttempt and PassagePlanFresh ticked against a Context.
  add_executable(test_mission_nodes test/test_mission_nodes.cpp)
  target_include_directories(test_mission_nodes PRIVATE include)
  ament_target_dependencies(test_mission_nodes ${DEPS})
  target_link_libraries(test_mission_nodes crusader_bt_leaves)
  add_test(NAME mission_nodes COMMAND test_mission_nodes)
  # mission_loader: an uploaded mission resolves the task library by ID, and a name that is no
  # file is refused. gtest is optional here: an image without it still builds the rest.
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_mission_loader test/test_mission_loader.cpp)
    target_include_directories(test_mission_loader PRIVATE include)
    ament_target_dependencies(test_mission_loader ${DEPS})
    target_link_libraries(test_mission_loader crusader_bt_leaves)
    target_compile_definitions(test_mission_loader PRIVATE
      CRSD_TREES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees")
  else()
    message(STATUS "crusader_bt: ament_cmake_gtest not found, test_mission_loader is not built")
  endif()
  # Every built-in tree must load: an unknown node or a bad port fails the build's tests,
  # not a mission at the dock.
  file(GLOB CRSD_TREES ${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/*.xml)
  add_test(NAME builtin_trees_load COMMAND bt_tree_check ${CRSD_TREES})
endif()

ament_package()
