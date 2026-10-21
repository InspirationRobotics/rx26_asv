# crusader_behavior

The boat's own state and its one path to the autopilot.

| Node | In | Out |
|---|---|---|
| `telemetry_bridge` | MAVProxy UDP 14551; `/crsd/guided_setpoint`, `/crsd/guided_heading_speed`, `/crsd/rc_override`, `/crsd/pump_cmd`, `/crsd/set_mode`, `/crsd/force_disarm`, `crsd/obstacle_distance` | `/crsd/pose`, `/crsd/attitude`, `/crsd/fcu_status`, `/crsd/rc_channels`, `/crsd/autonomy_drop`, `/crsd/pump_state`, `/crsd/fcu_health` |
| `rc_watchdog` | `/crsd/rc_channels`, `/crsd/fcu_status` | `/crsd/force_disarm`, `/crsd/kill_active` |
| `led_status_node` | `/crsd/fcu_status`, `/crsd/rc_channels`, `/crsd/autonomy_active` | `/crsd/led_state` |
| `led_node` | `/crsd/led_state` | the LED Arduino on `/dev/crsd-led` |

Nothing else may talk to the Pixhawk. The rules behind each TX path are the ROS-free
`*_core.py` modules, tested by `python3 -m pytest crusader_behavior/test`.
