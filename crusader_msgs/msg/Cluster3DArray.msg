# Everything one accumulated LiDAR window clustered into, nearest first.
#
# PUBLISHED EVERY WINDOW, EMPTY OR NOT — the same contract Detection3DArray
# keeps. An empty array means "the LiDAR is alive and nothing passed the gates";
# no message at all means the producer is dead. A consumer that cannot tell
# those apart will steer on a cluster from ten seconds ago.
#
# Sorted by range, nearest first: the closest obstacle is the one that matters,
# and a consumer that truncates the list should keep the near ones.
#
# header.stamp is the stamp of the NEWEST sweep in the accumulation window, not
# publish time and not the oldest — it is the instant this picture is current
# as of, which is what a fusion node associates a camera frame against.
# header.frame_id is base_link (REP-103 body: x fwd, y left, z up).
std_msgs/Header header
Cluster3D[] clusters
