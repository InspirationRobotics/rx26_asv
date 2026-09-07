# SITL — an ArduRover 4.6.3 that ROS cannot tell from the Pixhawk

Headless. No Gazebo, no GPU, no simulated water. Runs on a laptop in WSL2.

```bash
bash tools/sitl/start_sitl.sh          # start
python3 tools/sitl/check_sitl.py       # 9 checks, ~60 s
bash tools/sitl/start_sitl.sh --stop
```

After `start_sitl.sh`, **`udp:127.0.0.1:14551` is an ArduRover 4.6.3** on the
same port map the boat uses, so `telemetry_bridge` connects to it unchanged.

## Why this exists

Until 2026-09-06 nothing in this codebase had ever sent a real
`SET_POSITION_TARGET_GLOBAL_INT`. `publish_setpoints` is false everywhere, so
`telemetry_bridge._guided_cb`'s body had never executed, and the type mask, the
frame, and the assumption that ArduRover would act on them were all unverified.

`check_sitl.py` sends the exact message `telemetry_bridge` sends and watches the
vehicle move. First run: 60 m north, arrived within 2.8 m in 16 s.

It also proves the half of the interlock we do not control: **a setpoint sent in
HOLD is ignored by the autopilot.** `telemetry_bridge` refuses to send outside
`shared.autonomous_modes`, and ArduPilot refuses to act if something does.
Neither alone is the safety story; both together are.

## What it is faithful about, and what it is not

| Faithful | Not |
|---|---|
| ArduRover **4.6.3**, the boat's exact release | `motorboat` skid hull, **not** Crusader's 4×T200 OmniX |
| Modes, arming, GUIDED semantics | Thruster mixing, lateral movement |
| The MAVLink port map, including 14551/14550 | Hydrodynamics, waves, wind |
| 5 Hz HEARTBEAT via `SET_MESSAGE_INTERVAL` | Yaw source — SITL uses its compass; the boat runs `COMPASS_USE=0` with GPS yaw |
| `OBSTACLE_DISTANCE` over MAVLink | Livox scan pattern, camera, any perception |

**Do not set `FRAME_CLASS=2`/`FRAME_TYPE=2` here.** ArduRover would mix for four
thrusters the physics model does not have and the vehicle would sit still. SITL
proves the plumbing; the hull needs water.

## The two flags that are not style

Both copied from `scripts/start_mavproxy.sh`, both learned on the boat:

- `--streamrate=-1` — without it MAVProxy stomps `SR0_*` back to 4 Hz on every
  reconnect and the ROS-side rates quietly halve.
- `--cmd="long SET_MESSAGE_INTERVAL 0 200000"` — HEARTBEAT runs on a fixed 1 Hz
  timer no `SRx_*` parameter controls. 200000 µs = 5 Hz. Runtime state, does not
  survive an autopilot reboot — same as the boat.

## Rebuilding

The checkout must be at the **Rover** tag, not Copter. ArduPilot is a monorepo
and `Copter-4.6.3`'s Rover source is not `Rover-4.6.3`:

```bash
cd ~/ardupilot && git checkout Rover-4.6.3 && ./waf configure --board sitl && ./waf rover
```

`start_sitl.sh` prints the binary's version and warns if it is not V4.6.3.
