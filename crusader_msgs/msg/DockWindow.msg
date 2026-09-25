# ADOPTED UNCHANGED from the CV half of Task 3: github.com/ChristopherCa55/firefighting-cv
# specs/msg/DockWindow.msg, commit e1173c5d19b9a42bf00c66bf4b58ccaaa17623c6. The first line of the draft said "not in
# the repo"; now it is. Change it there first, then here, or the two drift.
#
# DRAFT for crusader_msgs (not in the repo). One target window of one docking bay,
# as seen in ONE camera frame.
#
# IDENTITY COMES FROM GEOMETRY, NEVER FROM COLOUR. `index` is the window's slot in
# the bay design (0 = leftmost slot of the face template), assigned from where the
# window sits inside the detected face. Two frames that both say index 1 mean the
# same physical window whatever each one showed.

uint8 index                  # slot index in the bay design, 0 = leftmost
string slot                  # slot name from the template, e.g. "UL", "LR"
float32 identity_confidence  # [0..1] how well the box fits its template slot

# State of the light, decided by the white-referenced colour rule on the
# FULL-RESOLUTION frame (colour_core.window_state). UNKNOWN is a real answer:
# the rule abstained (too dim, two channels too close, no white reference).
uint8 STATE_UNKNOWN=0
uint8 STATE_OFF=1
uint8 STATE_RED=2
uint8 STATE_GREEN=3
uint8 STATE_BLUE=4
uint8 state
float32 state_confidence     # [0..1]
float32 lit_score            # max per-channel white-referenced excess (diagnostic)

float32 detector_confidence  # [0..1] window box confidence

# Box in the full-resolution RGB frame (x1, y1, x2, y2 px), like Detection3D.bbox.
uint16[4] bbox

# Aim point: the window's pixel centre pushed onto the plane fitted to the FACE's
# stereo depth (a window's own depth is a hole). REP-103 body axes in the
# enclosing header's frame (x forward, y left, z up) [m].
bool has_position
float64 x
float64 y
float64 z
