# Raw RC channel values as seen by the Pixhawk (from MAVProxy rebroadcast).
# Also used as the payload on /crsd/rc_override — the ONLY sanctioned path for
# RC override output (telemetry_bridge forwards it, subject to the autonomy-drop
# latch). Value 0 on an override channel = release that channel to the pilot.
std_msgs/Header header
uint16[18] channels    # microseconds; 0 = no signal / release
