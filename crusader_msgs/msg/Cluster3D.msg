# One 3D object found by clustering a LiDAR sweep. No label and no confidence:
# a cluster is a thing that is THERE, which is all the LiDAR can say. Naming it
# is the camera's job, and fusion pairs the two.
#
# Position and extent are REP-103 BODY axes — x forward, y left, z up — in the
# frame the enclosing Cluster3DArray declares. NOT the levelled frame the
# clustering ran in: the camera is bolted to the same hull, so its detections
# are body-frame too, and fusion compares them directly. Whoever projects into
# the world applies attitude once, at that point.
float64 x
float64 y
float64 z

# Axis-aligned bounding box SIZE, not corners [m]. Carried because obstacle
# avoidance needs a footprint rather than a point, and because it is the cheapest
# way to tell a buoy from a dock: if extent disagrees with the label fusion
# attached, one of them is wrong.
float64 extent_x
float64 extent_y
float64 extent_z

# Raw returns in this cluster. A 0.3 m buoy gives roughly 37 of them at 5 m and
# 2 at 20 m before accumulation, so this doubles as a confidence proxy — a
# 6-point cluster at 20 m is real but its centroid is worth centimetres less
# than a 200-point one at 4 m.
uint32 n_points

# Horizontal range sqrt(x^2 + y^2) [m]. Not carried separately from the position
# so much as derived from it — kept so it can never disagree.
float64 range
