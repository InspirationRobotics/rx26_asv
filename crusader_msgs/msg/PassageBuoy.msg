# One buoy as the UAV reported it: not a TrackedTarget, the boat has not seen it itself.
# id is the sender's slot, stable within one plan. beacon = RoboCommand's BeaconState.
uint8 id
float64 latitude       # WGS84, same frame as /crsd/pose
float64 longitude
uint8 BEACON_UNKNOWN=0
uint8 BEACON_OFF=1
uint8 BEACON_FLASHING_RED=2
uint8 BEACON_FLASHING_GREEN=3
uint8 BEACON_FLASHING_BLUE=4
uint8 BEACON_STEADY_BLUE=5
uint8 beacon
