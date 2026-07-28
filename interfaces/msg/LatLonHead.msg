# Position + heading sample (RX24 shape, plus ground speed).
std_msgs/Header header
float64 latitude
float64 longitude
float64 heading        # degrees, 0 = true north, clockwise positive
# Horizontal ground speed [m/s], from GLOBAL_POSITION_INT vx/vy. Populated by
# telemetry_bridge on /crsd/pose; 0.0 on the goal/origin uses of this message,
# which carry position only. Consumers that need it (roa_apf_node's objective-2
# progress monitor) must read it from /crsd/pose, never assume it.
float64 ground_speed
