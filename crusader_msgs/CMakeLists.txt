cmake_minimum_required(VERSION 3.8)
project(crusader_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# A .msg not listed here is silently NOT generated: the build stays green
# and the import fails at node start, on the boat. Add the file AND a line.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LatLonHead.msg"
  "msg/Attitude.msg"
  "msg/RcChannels.msg"
  "msg/FcuStatus.msg"
  "msg/MissionState.msg"
  "msg/Detection3D.msg"
  "msg/Detection3DArray.msg"
  "msg/Cluster3D.msg"
  "msg/Cluster3DArray.msg"
  "msg/TrackedTarget.msg"
  "msg/TrackedTargetArray.msg"
  DEPENDENCIES std_msgs
)

ament_package()
