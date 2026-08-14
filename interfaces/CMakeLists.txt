cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LatLonHead.msg"
  "msg/RcChannels.msg"
  "msg/FcuStatus.msg"
  DEPENDENCIES std_msgs
)

ament_package()
