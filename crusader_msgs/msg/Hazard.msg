# One known hazard for the costmap's HazardLayer, in the `map` frame of the
# enclosing HazardArray: ENU metres from /crsd/datum, the BT's local frame.
# Drawn LETHAL at its PHYSICAL size plus keepout_m; inflation adds the 0.8 m
# hard / 2.0 m soft clearance. Nothing here is a guess: a hazard nobody can
# place is left out, not drawn somewhere plausible.
uint8 CIRCLE=0
uint8 POLYGON=1
uint8 kind

# Where it came from - for logs and the ground station; the layer ignores it.
uint8 SRC_PLAN_BUOY=0   # a passage buoy (UAV plan, position fused with the tracker)
uint8 SRC_TRACK=1       # a confirmed track no plan buoy claimed (Core: every track)
uint8 SRC_DOCK=2        # a dock finger or deck segment, from the DockBook
uint8 SRC_KEEPOUT=3     # reserved: a RoboCommand keep-out zone (Task 4); no producer yet
uint8 source
int32 id                # plan buoy id / track id / bay track id; -1 = none

float64 x               # CIRCLE centre [m]
float64 y
float64 radius_m        # CIRCLE physical radius [m]
float64[] polygon_x     # POLYGON vertices [m], convex, counter-clockwise; len == polygon_y
float64[] polygon_y
float64 keepout_m       # extra LETHAL padding beyond the surface [m]; 0 = physical size only
