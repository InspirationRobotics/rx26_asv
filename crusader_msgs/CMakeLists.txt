cmake_minimum_required(VERSION 3.8)
project(crusader_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)   # action generation needs the goal-id types
find_package(rosidl_default_generators REQUIRED)

# A .msg not listed here is silently NOT generated: the build stays green
# and the import fails at node start, on the boat. Add the file AND a line.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LatLonHead.msg"
  "msg/Attitude.msg"
  "msg/RcChannels.msg"
  "msg/FcuStatus.msg"
  "msg/GuidedSetpoint.msg"
  "msg/ObstacleDistance.msg"
  "msg/Detection3D.msg"
  "msg/Detection3DArray.msg"
  "msg/Cluster3D.msg"
  "msg/Cluster3DArray.msg"
  "msg/TrackedTarget.msg"
  "msg/TrackedTargetArray.msg"
  # The Task 1 passage as the UAV reports it. PassageBuoy must precede
  # PassagePlan, which contains an array of it.
  "msg/PassageBuoy.msg"
  "msg/PassagePlan.msg"
  "msg/GatePair.msg"
  # Actions. Same rule as above: a file not listed here is silently not
  # generated, and the import fails at node start, on the boat.
  "action/SafePassage.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_package()
