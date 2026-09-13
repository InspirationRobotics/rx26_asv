# One buoy of the Task 1 passage, as the UAV reported it.
#
# This is NOT a TrackedTarget. A TrackedTarget is something the boat's own
# sensors saw and is keyed by the tracker's own id; this is something the
# AIRCRAFT saw from above and is keyed by the UAV's id. Above Core tier the
# handbook says the passage is only detectable by the UAV, so this is the
# authoritative statement of what colour a buoy is showing. The two are fused in
# nav::fusePassage(), where the rule is that position may come from the tracker
# but the beacon state always comes from here.
#
# THE ID IS THE UAV'S INDEX into the arrays of the last RXL_SAFE_PASSAGE frame,
# 0-based. It is the only numbering both vehicles share, and it is what
# RXL_NEXT_BUOY_SET names when it sends a gate pair. It has nothing to do with
# TrackedTarget.id and the two must never be compared.
uint8 id

float64 latitude       # WGS84, same frame as /crsd/pose
float64 longitude

# Beacon state. These values are RoboCommand's BeaconState from rx_common.proto,
# and they are the same numbers as nav::Beacon in nav_math.hpp and
# RXL_BEACON_STATE in the MAVLink dialect. All four agree deliberately.
#
# DO NOT "tidy" OFF back to zero. An earlier version of the dialect started at
# OFF = 0, which put every value one below RoboCommand's: a FLASHING_RED buoy
# decoded as OFF and a FLASHING_GREEN as FLASHING_RED, so the boat would have
# passed red buoys on the wrong side with nothing in any log looking wrong.
uint8 BEACON_UNKNOWN=0
uint8 BEACON_OFF=1
uint8 BEACON_FLASHING_RED=2
uint8 BEACON_FLASHING_GREEN=3
uint8 BEACON_FLASHING_BLUE=4
uint8 BEACON_STEADY_BLUE=5
uint8 beacon
