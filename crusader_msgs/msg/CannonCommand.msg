# A pan/tilt request for the water cannon, to telemetry_bridge on
# /crsd/cannon_cmd (from cannon_aim_node).
#
# The PWM is what the bridge sends (MAV_CMD_DO_SET_SERVO on its two cannon
# outputs, SERVO11 = AUX3 pan and SERVO12 = AUX4 tilt on this Pixhawk). The
# degrees are what the aim asked for, for the log: the servo calibration that
# turns one into the other (cannon_aim_core.ServoMap) is the aim node's.
#
# The bridge clamps each PWM to its own cannon_pwm_min..cannon_pwm_max and sends
# at most one command per axis per cannon_min_period_s.
std_msgs/Header header
float32 pan_deg              # + = LEFT of the bow (REP-103 yaw)
float32 tilt_deg             # + = UP from the hull's level plane
uint16 pan_pwm               # [us]; 0 = leave this axis alone
uint16 tilt_pwm              # [us]; 0 = leave this axis alone
string source                # who asked, for the log
