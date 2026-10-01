# The WHOLE known-hazard set. Each message REPLACES the previous one, so a hazard
# that is gone (the UAV moved a buoy, a track expired) disappears with no
# delete message. frame_id MUST be "map"; the layer refuses anything else.
std_msgs/Header header
uint32 seq              # bumps on every publish
Hazard[] hazards
