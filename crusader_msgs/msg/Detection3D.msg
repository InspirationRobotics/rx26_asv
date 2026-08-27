# One detected object, positioned in 3D. The frame is declared once by the
# enclosing Detection3DArray header — every position in an array shares it.
#
# Axes are REP-103 body convention for the declared frame (camera_link):
#   x = forward+ [m]   y = left+ [m]   z = up+ [m]
# NOT the optical convention (z forward, x right, y down). The producer does
# that conversion; consumers never see optical axes.
#
# Range to the object is sqrt(x^2 + y^2 + z^2) — not carried separately, so it
# can never disagree with the position.
string label           # detector class name, e.g. "red_buoy"
float32 confidence     # [0..1]
float64 x
float64 y
float64 z

# Source bounding box in the RGB frame the detection came from: x1, y1, x2, y2
# in pixels. Kept so a consumer can re-cut the image (debug overlay, a second
# classifier) without re-running the detector, and so a bad range can be traced
# back to the box that produced it.
uint16[4] bbox
