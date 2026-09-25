# ADOPTED UNCHANGED from the CV half of Task 3: github.com/ChristopherCa55/firefighting-cv
# specs/msg/DockObservation.msg, commit e1173c5d19b9a42bf00c66bf4b58ccaaa17623c6. The first line of the draft said "not in
# the repo"; now it is. Change it there first, then here, or the two drift.
#
# DRAFT for crusader_msgs (not in the repo). Everything one camera frame saw of the
# docking bays - Task 3 firefighting.
#
# PUBLISHED EVERY FRAME, EMPTY OR NOT (same rule as Detection3DArray): an empty
# `bays` means "camera alive, no bay in view"; no message means the producer is dead.
#
# header.stamp = camera instant (device timestamp), not publish time: the timing
# layer measures 1 s flashes from these stamps.
# header.frame_id = camera_link (REP-103 body axes), like oak/detections.
#
# DELIBERATELY NOT a Detection3DArray on oak/detections: bt_runner_node's
# beaconFromLabel() maps ANY label containing "red"/"green"/"blue" to a Task 1
# beacon, so dock windows published there would be treated as buoys.
std_msgs/Header header
DockBay[] bays

# Timing layer (dock_sequence_core) for the bay the node is tracking, so a
# consumer does not re-implement it. Strings, not enums, while the handbook
# patterns are still being confirmed against the rebuilt bay.
string target_pattern          # "", "steady", "flash", "code", "unresolved"
string[] target_colours        # e.g. ["red"], or ["red", "blue"] for a code
int8 target_window_index       # -1 = none yet
string last_event              # "", "hit", "hit_done", "lost" (this frame only)

# Health: the frame rate the timing layer actually saw. Below ~4 fps a 1 s / 1 s
# flash cannot be resolved (Nyquist) and target_pattern stays "unresolved".
float32 observed_fps
