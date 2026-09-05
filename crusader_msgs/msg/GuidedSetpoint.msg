# Sanctioned GUIDED-mode position setpoint. Published by mission nodes on
# /crsd/guided_setpoint; telemetry_bridge forwards it to the autopilot as
# SET_POSITION_TARGET_GLOBAL_INT. This is the ONLY sanctioned GUIDED TX path —
# mission nodes never open their own MAVLink connection.
#
# GATING (changed 2026-09-05): the latch that guards this is now the FLIGHT MODE,
# not an RC channel. telemetry_bridge forwards a setpoint only while
# fcu_status.mode is in shared.autonomous_modes. Two independent things must
# therefore agree before the boat moves autonomously:
#   * we choose to send   — the mode latch here
#   * ArduPilot chooses to obey — SET_POSITION_TARGET_* is ignored outside GUIDED
# The pilot leaving GUIDED on SC stops both at once, in hardware order, with no
# software in the path. That is the autonomy drop.
#
# Restored 2026-09-05 from 8c4ffa5 (removed unverified in the v0.5 strip at
# 45cdefa). The wire format and type mask are unchanged and field-exercised —
# see POSITION_ONLY_TYPE_MASK in crusader_fcu/telemetry_bridge.py.
std_msgs/Header header
float64 latitude
float64 longitude
float32 yaw            # deg, 0 = true north, clockwise+; NaN = leave yaw to ArduRover
                       #      CURRENTLY IGNORED: the position-only type mask has
                       #      the yaw-ignore bit set, so this field is accepted and
                       #      discarded. Commanding yaw means clearing that bit —
                       #      a separate, deliberate change.
