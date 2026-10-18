"""The generated RobotX MAVLink dialect, vendored.

`robotx.py` here is GENERATED; do not edit it. `Mesh/gen_dialect.py` builds it
from `Mesh/dialect/robotx.xml`, and this copy exists because the boat's workspace
cannot reach the `Mesh/` bench tooling.

Change the dialect in one place and you must change it in the other IN THE SAME
COMMIT (same rule as crusader_groundstation/ocs_link.py's copy of
rx_bridge/framing.py): a dialect that disagrees between the two ends of a radio
link decodes to the wrong id or shifted fields, and the boat acts on nonsense.

To refresh:

    python Mesh/gen_dialect.py
    tr -d '\\r' < Mesh/dialect/gen/robotx.py \\
      > Boat/rx26_asv/crusader_link/crusader_link/dialect/robotx.py

RXL_BEACON_STATE keeps UNKNOWN=0, OFF=1, matching RoboCommand's BeaconState,
nav::Beacon in nav_math.hpp and crusader_msgs/msg/PassageBuoy.msg. An earlier
OFF=0 numbering read a FLASHING_RED buoy as OFF; do not "tidy" it back.
"""
