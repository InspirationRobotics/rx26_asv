# ADOPTED UNCHANGED from the CV half of Task 3: github.com/ChristopherCa55/firefighting-cv
# specs/msg/DockBay.msg, commit e1173c5d19b9a42bf00c66bf4b58ccaaa17623c6. The first line of the draft said "not in
# the repo"; now it is. Change it there first, then here, or the two drift.
#
# DRAFT for crusader_msgs (not in the repo). One docking bay face seen in one frame.

uint8 bay_index                # left to right in THIS frame (not a persistent id)
float32 detector_confidence
uint16[4] bbox                 # face box, full-res RGB px (x1, y1, x2, y2)
bool truncated                 # face cut by the image edge / hull band

# ---- docking indicator (the light that says which bay to dock in) ----
bool indicator_present
uint8 COLOUR_UNKNOWN=0
uint8 COLOUR_OFF=1
uint8 COLOUR_RED=2
uint8 COLOUR_GREEN=3
uint8 COLOUR_BLUE=4
uint8 indicator_colour         # white-referenced rule, same as the windows
float32 indicator_confidence
uint16[4] indicator_bbox

# ---- windows ----
DockWindow[] windows           # sorted by index
int8 lit_window_index          # index of the single lit window; -1 = none or
                               # more than one (ambiguous) this frame
uint8 lit_state                # DockWindow STATE_* of that window

# ---- where the bay is ----
# Face plane from stereo depth (RANSAC + least squares). has_plane false when
# depth is unavailable - e.g. closer than the stereo minimum depth (0.62 m at
# 640x400 with the default 95-px disparity search, 0.31 m with extended).
bool has_plane
float64[3] plane_normal        # unit, body axes, pointing at the camera
float64 plane_offset           # n . p + d = 0
float32 plane_rms_m
# Fallback range from the known window size (pinhole on the window HEIGHT). Always
# filled when a window is fully visible; the only range below the stereo minimum.
float32 range_from_size_m      # NaN if no fully visible window
float32 bearing_deg            # face centre, camera frame, +left (REP-103)
