# Autopilot health at 1 Hz from telemetry_bridge. NaN / 0 / TRI_UNKNOWN when the
# stream that carries a field is stale or was never received.
std_msgs/Header header

uint8 TRI_UNKNOWN=0
uint8 TRI_NO=1
uint8 TRI_YES=2

float32 battery_v
float32 battery_pct            # 0..100
uint8 gps_fix_type             # GPS_FIX_TYPE: 0 none, 3 3D, 5 RTK float, 6 RTK fixed
uint8 gps_sats
float32 gps_hdop
uint8 ekf_ok                   # TRI_*
uint16 ekf_flags               # EKF_STATUS_REPORT.flags
uint8 prearm_ok                # TRI_*: SYS_STATUS prearm-check health bit
uint8 estop_active             # TRI_*: the e-stop channel below estop_threshold (or RC lost)
string last_statustext
uint8 last_statustext_severity # MAV_SEVERITY, 0 emergency .. 7 debug
