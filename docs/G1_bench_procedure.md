# Gate G1 — autonomy-drop switch bench validation

**Purpose:** prove the RC autonomy-drop switch kills RC-override output on the bench,
out of WiFi range, via ELRS only — before any RC-override mechanism (`dp_hold`, or any
Level-2-generated mechanism that overrides RC) is allowed on the water again.

**This is a software layer above the hardware e-stop (SB switch), never a replacement.
The e-stop is tested first, separately, every session.**

## Prerequisites

- [ ] Boat on stands, **props removed** (not just disarmed).
- [ ] Radio: a free switch mapped to the drop channel (default **ch9 / SE**; set
      `drop_channel` on `telemetry_bridge` to match your mapping, and record the
      chosen channel + polarity here: ch ____, high/low = drop: ____).
      **Must not be ch7** — that is arm/e-stop (`RC7_OPTION=165`) and its armed
      position trips the latch permanently. **Should be >= 9** — the bridge
      truncates overrides to 8 channels, so a channel above 8 cannot be written
      by any override mechanism in the graph.
- [ ] Record the actual detent values before test 1, by flipping the switch
      through every position while watching `ros2 topic echo /crsd/rc_channels`:
      safe = ____ us, drop = ____ us. A channel that never moves means the switch
      is not mapped, and every test below would pass vacuously against a drop
      switch that does nothing.
- [ ] `telemetry_bridge` and `rc_override_smoke` built and launched in the container
      (`tools/scripts/rebuild.sh` first — always).
- [ ] Laptop running Mission Planner/QGC on MAVProxy's rebroadcast, RC monitor open
      (Setup → Radio Calibration shows live override effect on channel outputs).

## Test matrix (all must pass; record pass/fail + timestamp)

| # | Test | Procedure | Pass criterion |
|---|------|-----------|----------------|
| 1 | Fail-safe start | Launch nodes with the transmitter OFF | `rc_override_smoke` logs BLOCKED; no override visible in RC monitor |
| 2 | Safe enable | Turn transmitter on, switch in safe position | Override appears (neutral 1500s) within 2 s |
| 3 | Pilot drop | Flip drop switch | Override vanishes from RC monitor; smoke node logs BLOCKED within one cycle (≤100 ms at 10 Hz); `telemetry_bridge` logs the trip reason |
| 4 | Latch holds | Flip switch back to safe, wait 10 s | Still BLOCKED (no auto-clear) |
| 5 | Reset refused while unsafe | Flip switch to drop, call `/crsd/autonomy_drop_reset` | Service returns failure with "switch still in drop position" |
| 6 | Reset works | Switch to safe, call reset service | Success; override resumes |
| 7 | RC loss | Turn transmitter off mid-override | Trip within `rc_stale_timeout` (1 s) **or** on the RC-receiver health verdict (see 7b); `telemetry_bridge` logs the reason. **Which path fired is data — record it.** |
| 7b | **RC loss with a receiver that holds last position** | Same as 7, with the ELRS receiver failsafe deliberately set to "Last Position". Watch `ros2 topic echo /crsd/rc_channels` — the drop channel keeps reading its held value | Latch still trips, with reason `ArduPilot reports RC receiver UNHEALTHY`. This is the 2026-08-02 case the PWM and staleness checks are both blind to; if the latch stays ACTIVE here, **stop — the interlock does not work.** Restore the receiver to "No Pulses" afterwards |
| 8 | **Range test** | Repeat tests 3 and 7 with the operator + transmitter beyond WiFi range (>150 m, WiFi disassociated on the laptop to prove no WiFi dependency) | Same behavior, observed on return via logs (`ros2 topic echo /crsd/autonomy_drop` recorded with `ros2 bag`) |
| 9 | Bypass attempt | Kill `rc_override_smoke`, publish directly to `/crsd/rc_override` while dropped (`ros2 topic pub`) | Nothing reaches the Pixhawk (RC monitor unchanged) — bridge-level enforcement holds |
| 10 | Drop stops GUIDED too | While dropped, publish a `/crsd/guided_setpoint` (`ros2 topic pub`) | No `SET_POSITION_TARGET_GLOBAL_INT` reaches the autopilot — the latch gates GUIDED motion as well as RC override |

> **If `/crsd/rc_link_health` never publishes**, `telemetry_bridge` logs a one-shot
> WARN that the autopilot does not advertise the bit. Test 7b then cannot pass on
> this airframe, and the receiver failsafe **must** be "No Pulses" — record that
> as a finding rather than signing off around it.

## Sign-off

- Safety lead: ____________  date: ________
- Result recorded in the field log; until signed, `dp_hold` and all RC-override
  mechanisms remain **sim/bench-only** (standing constraint from CLAUDE.md).

## Wiring note for dp_hold (to apply on the Jetson repo)

`dp_hold` must (1) drop any node-local MAVLink/serial path if present — overrides go
through `OverrideGuard.publish_override()` → `/crsd/rc_override` → `telemetry_bridge`;
(2) check `guard.allowed` at the top of every control cycle and enter its idle state
when false. See `rx26_asv/api/testing/rc_override_smoke.py` for the reference
pattern.
