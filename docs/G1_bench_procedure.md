# Gate G1 — autonomy-drop switch bench validation

**Purpose:** prove the RC autonomy-drop switch kills RC-override output on the bench,
out of WiFi range, via ELRS only — before any RC-override mechanism is allowed on the
water.

**This is a software layer above the hardware e-stop (SB switch), never a replacement.
The e-stop is tested first, separately, every session.**

## Before you start: there is no override publisher any more

v0.5 removed every node that published to `/crsd/rc_override`, including the
`rc_override_smoke` bench node this procedure used to drive. The **enforcement point**
survives — `telemetry_bridge` still gates the topic behind the latch — so the gate is
still testable, but you now drive it by hand with `ros2 topic pub` (below), or you write a
replacement bench node first.

A hand-driven override, props off:

```bash
ros2 topic pub -r 10 /crsd/rc_override interfaces/msg/RcChannels "{channels: [1500,1500,1500,1500,1500,1500,1500,1500,0,0,0,0,0,0,0,0,0,0]}"
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

## Sign-off

- Safety lead: ____________  date: ________
- Result recorded in the field log; until signed, all RC-override mechanisms remain
  **bench-only**.

## Wiring note for any future RC-override node

Such a node must (1) have no node-local MAVLink/serial path — overrides go to
`/crsd/rc_override` and `telemetry_bridge` forwards them; (2) subscribe to
`/crsd/autonomy_drop` (latched, TRANSIENT_LOCAL) and enter its idle state while dropped,
rather than relying on the bridge to swallow its output.
