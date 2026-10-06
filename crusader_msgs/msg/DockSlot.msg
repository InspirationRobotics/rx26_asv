# The docking slip around/ahead of the boat, from ONE LiDAR sweep
# (crusader_perception/slot_fit_core, dock_slot_node). Task 3 with the pan/tilt
# cannon: the boat docks by THIS, not by the camera - the slip's two side walls
# (the fingers' inner faces) give the centreline and the slip's axis, and the
# back wall (the deck edge between them) gives the standoff.
#
# Levelled body frame, REP-103 (x forward, y LEFT, z up), the same base_link as
# WallRange. One message per sweep, valid or not.
std_msgs/Header header       # stamp = the sweep; frame_id = base_link
bool valid                   # the slip is found: both side walls (or one and
                             # the known width) AND the back wall between them
string why                   # when not valid (or what was assumed), in words

# ---- the slip's axis and centreline (the side walls) ----
float64 angle_deg            # the slip's axis, + = it points LEFT of the bow
                             # (turn LEFT by this much to square up) - the same
                             # sign as WallRange.angle_deg
float64 lateral_m            # the slip's centreline at the BODY ORIGIN, + = LEFT
                             # of the boat (strafe LEFT this much to centre)
float64 width_m              # inner face to inner face [m] (NaN with one wall)
bool has_left                # the left side wall was fitted
bool has_right
float64 left_m               # its distance from the slip axis through the body
float64 right_m              # origin, perpendicular, as positive distances [m]
uint32 n_left                # inliers of each fit
uint32 n_right

# ---- the back wall (the deck edge at the end of the slip) ----
bool has_back
float64 back_range_m         # LiDAR origin -> back wall, along the slip axis [m]:
                             # THE STANDOFF the cannon tree holds (1.1 m docked,
                             # 1.6 m to watch)
uint32 n_back

float32 rms_m                # the side walls' fit residual [m]
