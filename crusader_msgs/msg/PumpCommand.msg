# A request for the water pump, to telemetry_bridge on /crsd/pump_cmd.
#
# The bridge decides; this only asks. It refuses a burst unless its own checks
# pass (pump path configured, RC fresh, SB not in e-stop, the pilot's pump switch
# OFF, armed, the pump output reading OFF, not too soon after the last burst),
# and it reports what it did on /crsd/pump_state.
#
# A burst is timed by the AUTOPILOT, not by us: the bridge sends
# MAV_CMD_DO_REPEAT_SERVO with one cycle, so the output returns to its TRIM
# (= pump off) on the Pixhawk's own clock even if this node, the bridge, the
# WiFi or MAVProxy dies mid-burst. There is deliberately no "on until told
# otherwise".
std_msgs/Header header
float32 duration_s     # burst length [s]; 0 = pump OFF now (never refused)
uint32 seq             # echoed back in PumpState.last_seq
string source          # who asked, for the log (e.g. "squirt_cal")
