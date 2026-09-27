# Gate G1 — autonomy-drop switch bench validation

**Purpose:** prove the RC autonomy-drop switch kills RC-override output on the bench,
out of WiFi range, via ELRS only — before any RC-override mechanism is allowed on the
water.

**This is a software layer above the hardware e-stop (SB switch), never a replacement.
The e-stop is tested first, separately, every session.**

## Before you start: what drives the override path now

v0.5 removed every node that published to `/crsd/rc_override`. It has one again:
`bt_runner_node` running `task3_fire_manual.xml` (the fixed-nozzle shot, MANUAL, strafing) with
`publish_setpoints` true. The **enforcement point** is `telemetry_bridge`, which since then
forwards an override only **in MANUAL**, only on **ch1/3/4** (steer, throttle, lateral — never
SB ch7, SC ch8 or the pump), clamped to ±150 µs about 1500, and **releases** the sticks after
0.5 s without one (`crusader_fcu/override_core.py`). The rows O1–O8 below test exactly that.

A hand-driven override, props off (ch1 steer, ch3 throttle, ch4 lateral; 0 = the pilot's):

```bash
ros2 topic pub -r 10 /crsd/rc_override crusader_msgs/msg/RcChannels "{channels: [1560,0,1560,1560,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"
```

Watch the latch state in another shell:

```bash
ros2 topic echo /crsd/autonomy_drop
```

## Prerequisites

- [ ] Boat on stands, **props removed** (not just disarmed).
- [ ] Radio: a free switch mapped to the drop channel (default ch7; set
      `drop_channel` param on `telemetry_bridge` to match your mapping, and record
      the chosen channel + polarity here: ch ____, high/low = drop: ____).
- [ ] `telemetry_bridge` built and launched in the container
      (`tools/scripts/rebuild.sh` first — always).
- [ ] Laptop running Mission Planner/QGC on MAVProxy's rebroadcast, RC monitor open
      (Setup → Radio Calibration shows live override effect on channel outputs).

## Test matrix (all must pass; record pass/fail + timestamp)

| # | Test | Procedure | Pass criterion |
|---|------|-----------|----------------|
| 1 | Fail-safe start | Launch nodes with the transmitter OFF, then start the override publisher | `/crsd/autonomy_drop` is `true`; no override visible in RC monitor |
| 2 | Safe enable | Turn transmitter on, switch in safe position | Override appears (neutral 1500s) within 2 s; `/crsd/autonomy_drop` goes `false` |
| 3 | Pilot drop | Flip drop switch | Override vanishes from RC monitor within one cycle (≤100 ms at 10 Hz); `telemetry_bridge` logs the trip reason |
| 4 | Latch holds | Flip switch back to safe, wait 10 s | Still dropped (no auto-clear) |
| 5 | Reset refused while unsafe | Flip switch to drop, call `/crsd/autonomy_drop_reset` | Service returns failure with "switch still in drop position" |
| 6 | Reset works | Switch to safe, call reset service | Success; override resumes |
| 7 | RC loss | Turn transmitter off mid-override | Trip within `rc_stale_timeout` (default 1 s); logs "link lost" or "stale" |
| 8 | **Range test** | Repeat tests 3 and 7 with the operator + transmitter beyond WiFi range (>150 m, WiFi disassociated on the laptop to prove no WiFi dependency) | Same behavior, observed on return via logs (`ros2 topic echo /crsd/autonomy_drop` recorded with `ros2 bag`) |
| 9 | Bypass attempt | While dropped, publish directly to `/crsd/rc_override` from a fresh shell | Nothing reaches the Pixhawk (RC monitor unchanged) — bridge-level enforcement holds |
| 10 | Disarm still works when dropped | While dropped, publish `true` to `/crsd/force_disarm` | Vehicle disarms. The force-disarm path is deliberately NOT latch-gated; if this fails, an RC-loss kill is impossible in exactly the situation that produces one |

## The GUIDED heading+speed path (the fixed-nozzle shot)

`task3_fire_test.xml` drives the boat with `/crsd/guided_heading_speed` →
`telemetry_bridge` → `SET_ATTITUDE_TARGET`. That path is latch-gated **and** mode-gated
(GUIDED only), clamps speed to `hs_max_speed_mps`, and sends a stop itself when commands go
quiet (`hs_deadman_s`) or the latch trips (`crusader_fcu/guided_hs_core.py`). Run
`tools/sitl/check_sitl_hs.py` first, so you know what the autopilot does with the message.
**Props off.** Drive it by hand:

```bash
ros2 topic pub -r 5 /crsd/guided_heading_speed crusader_msgs/msg/GuidedHeadingSpeed "{heading_deg: 0.0, speed_mps: 0.15}"
```

Watch the motor outputs on SERVO_OUTPUT_RAW (QGC MAVLink Inspector) and the bridge's log.

| # | Test | Procedure | Pass criterion |
|---|------|-----------|----------------|
| H1 | Refused outside GUIDED | SC in MANUAL, start the publisher | Outputs follow the sticks only; bridge logs `guided heading+speed DROPPED -- mode MANUAL is not GUIDED` |
| H2 | Acts in GUIDED | Arm, SC → GUIDED | Motor outputs move off neutral |
| H3 | Dead-man | Ctrl+C the publisher | Outputs back to neutral within ~0.5 s (the bridge's own stop), not 3 s |
| H4 | Pilot drop | Publisher running, flip the drop switch (ch9: SD, once mixed) | Outputs neutral at once; bridge logs the trip; the publisher's messages now DROPPED |
| H5 | Pilot mode | Publisher running, SC → MANUAL | Outputs follow the sticks at once |
| H6 | Clamp | Publish `speed_mps: 2.0` in GUIDED | Bridge warns `speed clamped to +0.40`; output no higher than at 0.4 |
| H7 | Nothing through a trip | While dropped, SC → GUIDED, publish again | Outputs stay neutral until `/crsd/autonomy_drop_reset` |

Until H1–H7 pass, `publish_setpoints` stays **false** on `bt_runner_node` for the fire tree.

## The RC override path (the MANUAL fire tree)

`task3_fire_manual.xml` takes the sticks through `/crsd/rc_override`. **Props off.** Publisher
as above; watch SERVO_OUTPUT_RAW and the bridge's log.

| # | Test | Procedure | Pass criterion |
|---|------|-----------|----------------|
| O1 | Refused outside MANUAL | SC middle (HOLD) or up (GUIDED), start the publisher | Outputs unchanged; bridge logs `rc override DROPPED -- mode HOLD is not one of MANUAL` |
| O2 | Acts in MANUAL | Arm, SC down (MANUAL) | Motor outputs move off neutral |
| O3 | Dead-man | Ctrl+C the publisher | Outputs follow the sticks again within ~0.5 s (the bridge's release), not 3 s |
| O4 | Pilot mode | Publisher running, SC → HOLD | Outputs neutral at once |
| O5 | Never SB, SC or the pump | Publish ch7 = 1000, ch8 = 1900, ch10 = 2000 as well | The boat does not e-stop, change mode or squirt |
| O6 | Clamp | Publish ch3 = 2000 | Output no further than 1650 µs of input; bridge warns `clamped` |
| O7 | SB wins | Publisher running, SB down | Motors stop |
| O8 | Drop switch | Once SD is mixed to ch9: publisher running, flip SD | Released at once; further overrides DROPPED until the reset service |
| O9 | Directions | `bt_runner_node` with the fire tree and `publish_setpoints` true, in MANUAL, the tree's log side by side | `fwd +` thrusts AHEAD, `lat +` to STARBOARD, `yaw +` turns RIGHT. If one is backwards, set `stick_reverse` for it |

Until O1–O7 and O9 pass, `publish_setpoints` stays **false** for `task3_fire_manual.xml`.

## Sign-off

- Safety lead: ____________  date: ________
- Result recorded in the field log; until signed, all RC-override mechanisms remain
  **bench-only**.

## Wiring note for any future RC-override node

Such a node must (1) have no node-local MAVLink/serial path — overrides go to
`/crsd/rc_override` and `telemetry_bridge` forwards them; (2) subscribe to
`/crsd/autonomy_drop` (latched, TRANSIENT_LOCAL) and enter its idle state while dropped,
rather than relying on the bridge to swallow its output.
