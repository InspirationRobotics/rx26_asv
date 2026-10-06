# The water cannon's pan/tilt servos as telemetry_bridge sees them, on
# /crsd/cannon_state (20 Hz).
#
# The PWM is what the Pixhawk says it is putting on the two outputs
# (SERVO_OUTPUT_RAW), not what anyone asked for.
std_msgs/Header header       # stamp = receipt of the SERVO_OUTPUT_RAW behind it
bool enabled                 # both cannon outputs are configured on the bridge
bool output_fresh            # SERVO_OUTPUT_RAW is arriving
uint16 pan_pwm               # [us] on the pan output; 0 = unknown
uint16 tilt_pwm              # [us] on the tilt output; 0 = unknown
uint16 pan_cmd_pwm           # [us] the last one we sent (after the clamp); 0 = none
uint16 tilt_cmd_pwm
string last_reason           # what the last command did, or why it was refused
