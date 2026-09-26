# The vertical wall ahead of the boat, from the LiDAR (wall_range_node), and the
# slip fingers either side of the boat if it is in one. One message per sweep,
# valid or not.
#
# Body frame, REP-103 (x forward, y LEFT, z up), gravity-levelled. The wall is
# fitted as a vertical plane, so its perpendicular distance does not depend on
# the boat's roll or pitch: rocking moves the points, not the plane.
#
# The MID360 sits low and upside down, so what it ranges is whatever vertical
# face is below ~0.4 m ahead: on a raised dock that is the dock's edge, not the
# target panel above it. squirt_cal calibrates against THIS number, and the tree
# must use the same one; the difference to the panel then cancels out.
std_msgs/Header header       # stamp = the sweep; frame_id = base_link
bool valid                   # false = no wall found (why in the health topic)
float64 range_m              # body origin -> wall plane, perpendicular [m]
float64 angle_deg            # bearing of the wall's nearest point; 0 = square
                             # on, + = the wall faces the boat from the LEFT
                             # (turn left by this much to square up)
uint32 n_inliers
float32 rms_m                # fit residual [m]
float32 span_m               # how much wall was seen, along the wall [m]
bool has_left                # a finger found to the left
float64 left_m               # its inner edge, from the body origin, along the wall [m]
bool has_right
float64 right_m              # the right finger's inner edge, as a positive distance [m]
float64 lat_m                # (right_m - left_m) / 2: the boat's offset from the
                             # slip centre, + = LEFT; NaN unless both fingers seen
