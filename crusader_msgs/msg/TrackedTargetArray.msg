# The world model's current picture: every mission target it is tracking,
# nearest first.
#
# PUBLISHED EVERY UPDATE, EMPTY OR NOT — the same contract Detection3DArray and
# Cluster3DArray keep, for the same reason. An empty array means "the world
# model is alive and holds no targets"; no message at all means the world model
# is dead. A consumer that cannot tell those apart will steer on a track list
# from ten seconds ago.
#
# Sorted by `range`, nearest first: the closest target is the one that matters,
# so a consumer that truncates the list keeps the near ones.
#
# header.stamp is the instant this picture is current as of — the newest sensor
# stamp that fed the update, not publish time. header.frame_id is "world": the
# local ENU frame anchored at the origin below (x east+, y north+, z up+).
std_msgs/Header header

# The lat/lon the world frame's (0, 0) sits at — the FIRST valid pose this node
# saw after startup, held for the life of the process. Every target's x/y is
# relative to it, so a consumer that wants metres must use THIS origin and not
# one of its own; crusader_common.geo.xy_to_latlon with these two numbers is
# the only sanctioned way back to lat/lon.
#
# It is carried on every message rather than latched on a separate topic
# because a track position is meaningless without it, and a consumer that
# joined the two topics and missed the latch would silently place every target
# somewhere off the coast of Africa.
float64 origin_latitude
float64 origin_longitude

# Boat state is deliberately NOT in this message — subscribe to /crsd/pose and
# /crsd/attitude for that. Folding them in would make one stream's staleness
# invisible behind another's freshness, which is the failure Attitude.msg was
# split out to avoid. What IS here is each target's range/bearing as measured
# against the pose that produced this update, which is the part that cannot be
# recovered afterwards.
TrackedTarget[] targets
