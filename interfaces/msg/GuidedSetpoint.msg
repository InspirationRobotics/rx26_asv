# Sanctioned GUIDED-mode position setpoint. Published by task nodes (e.g.
# gate_navigator) on /crsd/guided_setpoint; telemetry_bridge forwards it to the
# autopilot as SET_POSITION_TARGET_GLOBAL_INT. This is the ONLY sanctioned
# GUIDED TX path — task nodes never open their own MAVLink connection — and it is
# gated by the SAME autonomy-drop latch as RC overrides, so a tripped latch stops
# autonomous GUIDED motion as well as RC-override motion.
std_msgs/Header header
float64 latitude
float64 longitude
float32 yaw            # deg, 0 = true north, clockwise+; NaN = leave yaw to ArduRover
