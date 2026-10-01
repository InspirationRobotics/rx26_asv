cmake_minimum_required(VERSION 3.8)
project(crusader_nav_layers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# raster.hpp is pure (stdlib only), so its test builds whether or not Nav2 is
# here. It is registered with CMake so `colcon test` runs it; the laptop path
#   g++ -std=c++17 -O2 -I include test/test_raster.cpp
# is the one that gets used.
if(BUILD_TESTING)
  add_executable(test_raster test/test_raster.cpp)
  target_include_directories(test_raster PRIVATE include)
  add_test(NAME raster COMMAND test_raster)
endif()
install(DIRECTORY include/ DESTINATION include)

# BUILD WITHOUT NAV2. The boat's `asv` image has no Nav2 until it is recreated
# (docs/nav2_avoidance_spec.md 8.4), and rebuild.sh builds everything
# crusader_bringup depends on. A hard find_package here would break the boat's
# build for a feature it cannot run yet.
find_package(nav2_costmap_2d QUIET)
find_package(pluginlib QUIET)
find_package(rclcpp QUIET)
find_package(crusader_msgs QUIET)

if(nav2_costmap_2d_FOUND AND pluginlib_FOUND AND rclcpp_FOUND AND crusader_msgs_FOUND)
  add_library(crusader_nav_layers SHARED src/hazard_layer.cpp)
  target_include_directories(crusader_nav_layers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(crusader_nav_layers
    rclcpp nav2_costmap_2d pluginlib crusader_msgs)
  pluginlib_export_plugin_description_file(nav2_costmap_2d hazard_layer_plugin.xml)
  install(TARGETS crusader_nav_layers
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  ament_export_include_directories(include)
  ament_export_libraries(crusader_nav_layers)
  ament_export_dependencies(rclcpp nav2_costmap_2d pluginlib crusader_msgs)
else()
  message(WARNING
    "nav2_costmap_2d not found: crusader_nav_layers installs nothing (image without Nav2)")
endif()

ament_package()
