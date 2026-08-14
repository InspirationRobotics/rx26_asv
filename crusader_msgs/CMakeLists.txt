cmake_minimum_required(VERSION 3.8)
project(crusader_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# A .msg not listed here is silently NOT generated: the build stays green
# and the import fails at node start, on the boat. Add the file AND a line.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LatLonHead.msg"
  "msg/RcChannels.msg"
  "msg/FcuStatus.msg"
  DEPENDENCIES std_msgs
)

ament_package()
