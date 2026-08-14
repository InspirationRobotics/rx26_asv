# `crusader_behavior` — what the boat does with its own state

The **Safety** and **Indicator** boxes from the architecture diagram. Both subpackages
consume `crusader_fcu`'s republished topics; neither opens a MAVLink connection.

## `safety/` — force-disarm on RC-link loss

`rc_heartbeat_watchdog` stops the boat when the RC transmitter link dies, without anyone
pressing the physical SB switch. It **complements, never replaces**, ArduPilot's own
FS_THR/FS_GCS failsafes and the hardware e-stop relay — run all three.

All the decision logic lives in `rc_heartbeat_core.py`, which imports no ROS at all: when
to disarm, when to latch, what clears the latch, and the "bridge is down too, defer to
ArduPilot" case. The node only marshals topics. Read the core to understand the behaviour;
the node tells you nothing the core doesn't.

How link health is judged: ArduPilot reports PWM 0 on the monitored channel when the
transmitter is out of range or off, so a dropout appears as `pwm < min_valid_pwm` on
`/crsd/rc_channels`.

## `indicator/` — the LED status stack

Two nodes, deliberately split at the serial boundary:

- `pixhawk_led_status_node` — boat state → `/crsd/led_state` (1 RED / 2 YELLOW / 3 GREEN)
- `led_node` — `/crsd/led_state` → the Arduino over serial, with auto-reconnect

RED means disarmed **or** e-stopped. The e-stop keeps the vehicle ARMED (RC option 165),
so it is invisible in HEARTBEAT — the switch position has to be read from RC_CHANNELS. RC
loss reads 0, which is below the threshold, so a lost link also shows RED. That is the
fail-safe colour and it is not an accident.

`led_node` reconnects every `reconnect_s` and re-applies the last commanded colour: EMI
knocks the CH340 off the bus and it re-enumerates, and a strip that keeps showing a stale
colour after that is a strip that lies about boat state.

## Why safety and indicator share a package

They are siblings under **Behavior** in the architecture, and both are pure consumers of
boat state. If `safety/` ever grows past the watchdog — a wireless e-stop, an abort
handler — split it into its own package rather than letting it accumulate here. Nothing
in `indicator/` may ever be imported by `safety/`.

## Change impact

| You changed | Then |
|---|---|
| `safety/rc_heartbeat_core.py` | bench, props off: pull the transmitter, confirm the boat disarms and the latch behaves |
| `safety/rc_heartbeat_watchdog.py` | same bench run — the node is thin, but the topic wiring is what breaks |
| `indicator/*` or `firmware/LED.ino` | `ros2 topic pub /crsd/led_state std_msgs/msg/Int32 "{data: 2}" --once`, then flip the RC arm switch and watch the strip and the logs change together |
