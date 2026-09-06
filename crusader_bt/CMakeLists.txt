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

add_executable(bt_runner_node src/bt_runner_node.cpp src/leaves.cpp)
target_include_directories(bt_runner_node PRIVATE include)
ament_target_dependencies(bt_runner_node ${DEPS})

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
if(BUILD_TESTING)
  add_executable(test_nav_math test/test_nav_math.cpp)
  target_include_directories(test_nav_math PRIVATE include)
  add_test(NAME nav_math COMMAND test_nav_math)
endif()

ament_package()
