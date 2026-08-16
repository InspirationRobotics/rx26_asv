# One mission target the world model is tracking, anchored to the EARTH rather
# than to the boat. A Detection3D says "something red is 12 m off the port bow
# right now"; this says "target 7 is a red buoy AT 37.8721234, -122.3041987,
# and it has been there for 41 seconds across 63 sightings".
#
# That difference is the whole point of the package: a detection dies with the
# frame it came from, a track survives the boat looking away and coming back.
#
# ---- identity ----
# `id` is assigned once, at track birth, and NEVER reused inside a run — not
# even after the track is dropped. A consumer that has stored "target 7" must
# be able to conclude that a later target 7 is the same object; recycling ids
# would silently re-point every stored reference at a different buoy.
uint32 id

# Best label so far, by MAJORITY VOTE over every camera sighting associated to
# this track, not by the most recent one. The detector flips between
# `red_buoy` and `red_pole_buoy` on alternate frames at range; the vote is what
# makes the answer stop flickering. EMPTY STRING means LiDAR-only: the boat can
# see a thing is there but nothing has named it. An empty label is a real
# obstacle, never a placeholder to be filtered out.
string label
float32 confidence          # detector confidence of the most recent sighting [0..1]

# Which sensors have contributed to this track, ever (bitmask, OR of the
# constants below). Carried because the three combinations have genuinely
# different position quality and a consumer is entitled to know which it has:
#   CAMERA only -> bearing is good, range is stereo depth (metres out at 20 m)
#   LIDAR only  -> range is good, and nothing has identified the object
#   both        -> fused: camera bearing at LiDAR range, the best available
uint8 SOURCE_CAMERA=1
uint8 SOURCE_LIDAR=2
uint8 sources

# ---- where it is ----
# WGS84, the same frame /crsd/pose reports in, so a track can be handed
# straight to a waypoint push without a conversion step.
float64 latitude
float64 longitude

# The same position in the world ENU frame the enclosing array's origin
# anchors: x = east+ [m], y = north+ [m], z = up+ [m]. Carried alongside the
# lat/lon rather than instead of it because everything that reasons about the
# target — distance gates, the display, the occupancy grid — works in metres,
# and a consumer re-deriving them would be a second copy of the projection.
#
# z is kept for the same reason Detection3D keeps it: it is the cheapest sanity
# check there is. A buoy that settles at +4 m of altitude means attitude and
# detection disagree, and dropping z hides that.
float64 x
float64 y
float64 z

# Smoothed world velocity [m/s], east+ / north+. A finite difference of the
# position estimate, EMA-smoothed — NOT a filter state, and not to be
# integrated. It exists because two of the detector's classes are
# `*_target_boat`, which move: a track that reads 1.5 m/s is a boat under way,
# and a moored buoy reading 1.5 m/s means the association gate is jumping
# between two different objects. It is as much a health signal as a state.
float64 velocity_east
float64 velocity_north

# Spread of the observations about the current estimate [m] — RMS association
# residual, EMA-smoothed. NOT a covariance and not a confidence interval: it is
# "how much do the sightings of this thing disagree with each other". Small and
# steady = a solid fix. Growing = the track is being fed sightings of more than
# one object, which is what an over-wide assoc_radius_m produces.
float64 position_stddev

# ---- where it is RELATIVE TO THE BOAT, at the instant of this update ----
# Both are snapshots against the pose in the enclosing array's header, not live
# values: by the time this is read the boat has moved. A consumer steering on
# them must re-derive from its own current pose and the lat/lon above. They are
# here because they are what a human reads first, and because re-deriving them
# for a log line is the kind of duplicated trigonometry that goes wrong once.
float64 range               # horizontal distance boat -> target [m]
float64 bearing             # ABSOLUTE bearing [deg], 0 = true north, cw+ —
                            # same convention as LatLonHead.heading, so
                            # (bearing - heading) is the relative bearing

# ---- how much to believe it ----
uint32 hits                 # sightings associated to this track since birth
float64 age                 # s since the track was created
float64 time_since_seen     # s since the last sighting was associated. Grows
                            # while the boat looks elsewhere; the track is
                            # dropped when it passes the configured timeout, so
                            # a large value here is a track about to expire.
bool confirmed              # hits >= confirm_hits. A TENTATIVE track (false)
                            # may be one frame of detector noise, so a mission
                            # should not commit to it — but it is published
                            # anyway, because deciding what is real is the
                            # consumer's call and a suppressed obstacle cannot
                            # be reconsidered.

# Axis-aligned footprint [m] from the LiDAR cluster, 0 when no LiDAR has seen
# this track. Same meaning as Cluster3D's extent: a SIZE, not corners.
float64 extent_x
float64 extent_y
float64 extent_z
