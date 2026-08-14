# Everything one camera frame saw, positioned in 3D.
#
# PUBLISHED EVERY FRAME, EMPTY OR NOT. An empty array means "the camera is alive
# and saw nothing"; no message at all means the producer is dead. A consumer
# that cannot tell those apart will happily steer on a detection from ten
# seconds ago — the frozen-value failure StreamCache exists to prevent.
#
# header.stamp is the camera instant (device timestamp, transport latency
# removed), NOT publish time: this is what a fusion node associates against a
# LiDAR sweep, so it has to be when the photons landed.
# header.frame_id is the frame the positions are in — camera_link by default.
std_msgs/Header header
Detection3D[] detections
