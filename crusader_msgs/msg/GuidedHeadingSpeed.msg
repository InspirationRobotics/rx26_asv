# Hold this heading at this speed, in GUIDED, to telemetry_bridge on
# /crsd/guided_heading_speed. The station-keeping primitive for the fixed-nozzle
# shot: a boat that must stand 3.2 m off a wall, pointed at a window, needs to
# go astern as well as ahead and to choose its heading, and a position setpoint
# can do neither (GuidedSetpoint's yaw is ignored; LOITER holds only to ~2 m).
#
# The bridge sends it as MAVLink SET_ATTITUDE_TARGET (ArduRover: desired heading
# from the quaternion, desired speed = thrust * WP_SPEED), and only while the
# mode is GUIDED and the autonomy-drop latch allows it. If commands stop for
# hs_deadman_s the bridge sends one zero-speed command itself; ArduRover also
# stops on its own after 3 s without one. Keep publishing, every tick.
std_msgs/Header header
float64 heading_deg    # compass, true, clockwise from north: where the BOW points
                       # (also when going astern)
float64 speed_mps      # + ahead, - astern; 0 = stop and hold the heading.
                       # The bridge clamps it to +-hs_max_speed_mps.
