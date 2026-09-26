# The water pump as telemetry_bridge sees it, on /crsd/pump_state (20 Hz).
#
# `output_pwm` is what the Pixhawk says it is putting on the pump's output
# (SERVO_OUTPUT_RAW), not what anyone asked for: the pilot's switch and our
# bursts both end up there, and that is the number that runs the pump.
std_msgs/Header header       # stamp = receipt of the SERVO_OUTPUT_RAW behind it

bool enabled                 # a pump output is configured and the watchdog has
                             # not latched the path off
bool output_fresh            # SERVO_OUTPUT_RAW is arriving
uint16 output_pwm            # the pump output's PWM [us]; 0 = unknown
bool on                      # output_pwm is nearer ON than OFF
uint16 pilot_pwm             # the pilot's pump channel on RC [us]; 0 = unknown

uint8 RESULT_NONE=0          # nothing asked yet
uint8 RESULT_SENT=1          # command sent, no answer yet
uint8 RESULT_ACCEPTED=2      # COMMAND_ACK: accepted
uint8 RESULT_REJECTED=3      # COMMAND_ACK: the autopilot said no
uint8 RESULT_REFUSED=4       # the bridge refused it (see last_reason)
uint32 last_seq              # PumpCommand.seq of the last request
uint8 last_result
string last_reason           # human-readable: why refused, or what was sent
