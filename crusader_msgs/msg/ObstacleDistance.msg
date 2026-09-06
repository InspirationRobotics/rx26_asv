# A sector map of the nearest obstacle in each direction, ready to be sent to
# the autopilot as MAVLink OBSTACLE_DISTANCE. Published by proximity_bridge;
# telemetry_bridge is the only thing that puts it on the wire, same as every
# other autopilot-bound message in this repo.
#
# WHY THE AUTOPILOT AND NOT US. ArduPilot's own avoidance runs in the flight
# controller at loop rate and keeps working when the Jetson is busy, confused or
# dead. BendyRuler/Dijkstra path planning (OA_TYPE) is active in AUTO, GUIDED and
# RTL and will route AROUND an obstacle toward the commanded setpoint; simple
# avoidance (AVOID_ENABLE) is the last-ditch margin underneath. Neither needs the
# mission layer to be correct.
#
# SECTOR CONVENTION — MAV_FRAME_BODY_FRD. Index 0 is the vehicle nose, and index
# increases CLOCKWISE viewed from above. Our clusters arrive in REP-103 body
# (x forward, y LEFT), so the conversion negates the angle:
#     bearing_cw_deg = -degrees(atan2(y, x))  mod 360
# Getting that sign wrong mirrors the world left-for-right, which looks
# plausible on a symmetric test object and is catastrophic on a gate.
#
# ENCODING RULES, both of which bite:
#   * UINT16_MAX (65535) means "no reading" — a sector we cannot see.
#   * 0 means an obstacle TOUCHING the sensor. Never use 0 for "nothing here":
#     with AVOID_BEHAVE=Stop that halts the boat instantly.
# Only the forward sector is real on this hull; everything aft of the beam is the
# boat's own structure and must be sent as UINT16_MAX, not as a near return.
#
# ArduPilot consolidates these 72 sectors into 8 of 45 degrees, keeping the
# CLOSEST reading in each. So this is honest about "something is close, roughly
# there" and cannot be used to thread a gate. That is the mission layer's job.
std_msgs/Header header

uint16[72] distances        # cm. UINT16_MAX = no reading. 0 = touching.
float32 increment_deg       # angular width of one sector
float32 angle_offset_deg    # bearing of sector 0 relative to the nose
uint16 min_distance_cm      # sensor minimum; readings below are not trusted
uint16 max_distance_cm      # sensor maximum; readings above mean "clear"
