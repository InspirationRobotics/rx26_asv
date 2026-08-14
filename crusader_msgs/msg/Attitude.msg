# Vessel attitude from the Pixhawk EKF (MAVLink ATTITUDE) — the rotation that
# turns a body-frame detection into a world-frame position. Without roll and
# pitch, a camera bearing is only correct on flat water; in chop the same buoy
# maps to a different place on every wave.
#
# Angles [rad], ArduPilot/NED body axes (x forward, y RIGHT, z DOWN):
#   roll  + = starboard side down
#   pitch + = bow up
#   yaw   + = clockwise from TRUE north (compass convention)
#
# NOTE the axes are NOT REP-103 (which Detection3D uses: x fwd, y LEFT, z UP).
# These are the autopilot's own axes, republished unconverted so the numbers on
# the wire match what Mission Planner shows during a bench check. The conversion
# lives in one place — crusader_common.geo.body_to_world_ypr — and nothing else
# should re-derive it.
#
# `yaw` is the SAME EKF yaw that LatLonHead.heading carries in degrees (this
# boat has COMPASS_USE=0 / EK3_SRC1_YAW=2, so both come from the dual-antenna
# moving-baseline GPS). They are two views of one estimate, not two estimates:
# if they disagree by more than rounding, one of the streams is stale.
# There is no "yaw unresolved" signal in this message — MAVLink ATTITUDE has no
# invalid encoding, so it reports EKF yaw unconditionally. LatLonHead.heading is
# the field that goes NaN when GPS yaw is unresolved; a consumer that must know
# whether yaw is trustworthy has to check /crsd/pose, not this topic.
std_msgs/Header header
float64 roll
float64 pitch
float64 yaw

# Body-axis angular rates [rad/s], same axes and signs as the angles above.
# Carried because they arrive in the same MAVLink frame at zero cost and answer
# a question mapping actually asks: was the boat mid-roll when this frame was
# taken? A detection ranged during a fast roll is worth less than one taken
# level, and finite-differencing the angles to find that out re-invents a number
# the autopilot already measured.
float64 rollspeed
float64 pitchspeed
float64 yawspeed
