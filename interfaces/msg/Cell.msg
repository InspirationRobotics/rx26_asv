# One occupied cell in the sparse occupancy grid (RX24 shape + source tag).
int32 x_coord
int32 y_coord
int16 value            # confidence, in [0, Grid.value_range]

# Source tagging (plan §3.3): objective-1 metrics and avoidance arbitration must
# distinguish perception-detected vs RoboCommand-reported obstacles.
uint8 SOURCE_PERCEPTION=0
uint8 SOURCE_COMMS=1
uint8 source
