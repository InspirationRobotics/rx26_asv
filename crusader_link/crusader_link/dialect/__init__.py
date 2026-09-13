"""The generated RobotX MAVLink dialect, vendored.

`robotx.py` in this directory is GENERATED. Do not edit it. It is produced by
`Mesh/gen_dialect.py` from `Mesh/dialect/robotx.xml`, and this copy exists
because the boat's workspace cannot reach the `Mesh/` tree: Mesh is bench
tooling on a laptop, not a ROS package that gets installed on the Jetson.

THE RULE, which is the same one crusader_groundstation/ocs_link.py carries for
its duplicate of rx_bridge/framing.py: change the dialect in one place and you
must change it in the other IN THE SAME COMMIT. A dialect that disagrees
between the two ends of a radio link does not fail loudly -- it decodes to the
wrong message id, or to the right id with the fields shifted, and the boat acts
confidently on nonsense.

To refresh:

    python Mesh/gen_dialect.py
    tr -d '\\r' < Mesh/dialect/gen/robotx.py \\
      > Boat/rx26_asv/crusader_link/crusader_link/dialect/robotx.py

The enum that matters most here is RXL_BEACON_STATE. It agrees with
RoboCommand's BeaconState, with nav::Beacon in nav_math.hpp, and with the
constants in crusader_msgs/msg/PassageBuoy.msg -- all four start UNKNOWN=0,
OFF=1. An earlier version started at OFF=0 and put every value one below
RoboCommand's, which reads a FLASHING_RED buoy as OFF. Do not "tidy" it back.
"""
