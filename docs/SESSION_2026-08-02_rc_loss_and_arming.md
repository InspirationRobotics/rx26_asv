# Session report — 2026-08-02/03: kill-switch demo prep, arming chain, RC-loss detection

**Goal going in:** demonstrate the three kill paths (onboard switch, remote SB switch,
transmitter-battery removal) and the three LED states (RED kill / YELLOW manual /
GREEN autonomous) for a safety demo.

**Outcome:** the demo was **not completed**. The arming chain was fixed end to end and
a real defect in RC-loss detection was found and patched, but the session ended with
the RC link down for unrelated reasons. One unexplained runaway occurred. See
[Current status](#current-status) before the next session.

---

## 1. What was wrong, in the order it was found

### 1.1 `PreArm: Need Position Estimate` — mode-gated, not a general failure

Rover only requires a position estimate in modes that navigate. MANUAL arms without
one; AUTO/GUIDED/LOITER do not. This is why manual/yellow already worked while the
"auto" did not work at the start.

### 1.2 No position estimate despite 28 satellites

The yaw source had no fallback:

```
EK3_SRC1_YAW  2   (GPS moving-baseline yaw)
EK3_SRC2_YAW  0   EK3_SRC3_YAW  0
COMPASS_USE   0   (and USE2/USE3)
```

EKF3 cannot produce a position solution until it aligns yaw, and GPS yaw was the only
source. `GPS_RAW_INT.yaw = 65535` confirmed it — in MAVLink that value means
"configured to provide yaw, currently unable," as distinct from `0` = "does not
provide yaw." Fix quality was irrelevant: `fix_type: 3` with `h_acc: 1717` just means
no RTCM corrections, and the UM982 computes heading internally between its own two
antennas regardless.

**Resolution: time.** The receiver resolved heading on its own after a longer
acquisition window with open sky. A later boot logged `EKF3 IMU0 yaw aligned`,
`origin set`, `is using GPS`, and `GPS_RAW_INT.yaw` became `33257` (332.57°).

### 1.3 `Arm: Throttle (RC3) is not neutral` — a bad RC calibration from QGC

`RC1_DZ` through `RC4_DZ` were all `0` (ArduPilot's default is 30), so "neutral"
demanded the stick sit at exactly `RC3_TRIM`. Setting the deadzones to 30 did not
clear it. Neither did 100, with `chan3_raw` sampling 3 µs from trim — a combination
that should compute to zero control input.

**Resolution: recalibrating RC through Mission Planner instead of QGroundControl
fixed it completely.** QGC also produced the recurring `PreArm: RC calibrating` flag
(re-asserted ~29 s after a clean reboot, cleared by leaving QGC's Radio page).

**Use Mission Planner for RC calibration on this boat.** Something in QGC's
calibration writes a state ArduPilot rejects and MAVProxy telemetry does not reveal.

### 1.4 Runaway on arming — the check was right

Before the Mission Planner recalibration, `RC_OPTIONS` was set to `0` to clear the
neutral-throttle arming check (bit 5, `ARMING_CHECK_THROTTLE`). The vehicle armed and
**immediately ran at high speed.** No thruster command was given and no mode change
was made by the operator.

The check was reporting a real condition. Disabling it removed the last thing standing
between a bad calibration and a live boat. `RC_OPTIONS` was restored to `32`.

Contributing context, none of it confirmed as the cause:

- The vehicle was already in GUIDED before arming (`chan8_raw = 1995`, and `MODE6`
  changed from `10`/AUTO to `15`/GUIDED between the two params files) — arming in a
  navigating mode with a mission aboard and a position estimate is not inert.
- `GPS1_MB_OFS` was applying a large heading correction (§4.2).
- The LED read YELLOW, which implies the reported mode was *not* AUTO/GUIDED — but
  `/crsd/fcu_status` was flapping stale at the time (§1.5), so the LED is not a
  reliable witness for this event.

**The dataflash log has not been reviewed.** It is the only artifact that can settle
this. See §4.1.

### 1.5 `/crsd/fcu_status` flapping stale

`telemetry_bridge` logged `MAVLink stream 'fcu_status' stale (> 1.0s) — NOT
republishing; is MAVProxy still up?` continuously while `rc_watchdog` simultaneously
reported `bridge_ok=True rc_age=0.0s`.

Cause: `fcu_status` is derived solely from HEARTBEAT, ArduPilot sends HEARTBEAT at a
**fixed 1 Hz** that no `SR*` parameter raises, and `stream_timeout_s` was `1.0`. A
1.0 s staleness timeout on a 1 Hz stream flaps on jitter alone, forever. This buried
real faults in noise and left consumers frozen on cached values.

### 1.6 The real defect: RC-loss detection did not work

With the transmitter powered off completely, `/crsd/rc_channels` continued at a steady
20 Hz and ch7 held `1995` indefinitely:

```
[1486 1497 1494 1497  994 1995 1995  994  994 1498 1498 1498  875 875 875 875 0 0]
                                ^^^^ ch7, unchanged through a dead transmitter
```

Both safety consumers keyed off the assumption that ArduPilot zeroes the PWM on link
loss:

- `rc_heartbeat_watchdog` computed `monitored_valid` from `pwm >= min_valid_pwm`, so
  `_link_lost` never fired and **it never force-disarmed.**
- `pixhawk_led_status_node` computed `estop_active` from `pwm < estop_threshold`, so
  **the strip stayed YELLOW on a boat with no pilot.**

Whether a receiver zeroes its outputs on link loss is a **receiver configuration
choice** (ELRS "No Pulses" vs "Last Position"). It can never be the sole basis for a
safety interlock.

The signal that does work is ArduPilot's own `SYS_STATUS`
`MAV_SYS_STATUS_SENSOR_RC_RECEIVER` health bit. Over CRSF the receiver reports its own
failsafe state to the autopilot even while continuing to emit held channel values, so
the autopilot knows the link is dead regardless of what appears on the wire. The
boat-repo original consulted this bit; the port to this repo dropped it, and
[`rc_heartbeat_watchdog.py`](../rx26_asv/rx26_asv/api/safety/rc_heartbeat_watchdog.py)
documented the removal in a comment that also asserted the now-disproven "RC loss
reads 0" premise.

---

## 2. Code changes (commit `217cf36`, pushed to `origin/main`)

6 files, +332 / −49. Full suite green (202 passed).

### `rx26_asv/rx26_asv/api/navigation/telemetry_bridge.py`

- Parses `SYS_STATUS` and republishes the RC-receiver health bit as
  **`/crsd/rc_link_health`** (`std_msgs/Bool`).
- If the autopilot never advertises the bit, the topic is **never published** and a
  one-shot WARN fires. No fabricated `True` — absence must look like absence.
- **Split stream timeouts.** `stream_timeout_s` (1.0) now covers only the fast streams
  (pose, RC), where frozen data is most dangerous and where
  `test_pose_timeout_consumers_match_shared` pins it to `shared.pose_timeout_s`. New
  `status_timeout_s` (3.0) covers HEARTBEAT and SYS_STATUS.
- Per-stream staleness logging now reports each stream's own timeout instead of always
  quoting `_pose`'s.

### `rx26_asv/rx26_asv/api/pixhawk/pixhawk_led_status_node.py`

- Subscribes to `/crsd/rc_link_health`; a fresh unhealthy report forces RED.
- **Inputs now age out** after `input_timeout_s` (2.0). `telemetry_bridge` deliberately
  stops republishing a stale stream, and the node was treating that silence as "no
  change," freezing the last colour on the strip. A dead gateway now fails to RED.
- RED carries a reason into the log: `disarmed`, `e-stop: ch7 < 1200us`,
  `ArduPilot reports RC receiver UNHEALTHY (link lost)`, `no /crsd/fcu_status …`.

### `rx26_asv/rx26_asv/api/safety/rc_heartbeat_watchdog.py`

- Subscribes to `/crsd/rc_link_health`; `monitored_valid = pwm_ok and health_ok`.
- The health verdict is **strictly additive**: unknown (bit not advertised) and stale
  both fall back to the PWM check, so it can only add a detection, never suppress one.
- Loud throttled ERROR when PWM looks valid but the autopilot says the receiver is
  dead, naming the receiver-failsafe fix.
- `rc_health=ok|LOST|unknown` added to the 5-second status line.
- **`rc_heartbeat_core.py` is unchanged** — the pure state machine and its tests were
  not touched.

### `rx26_asv/config/crusader_params.yaml`

- `telemetry_bridge.status_timeout_s: 3.0`
- `pixhawk_led_status_node.input_timeout_s: 2.0`
- Comments corrected where they asserted "RC loss reads 0", plus a receiver-failsafe
  prerequisite note on the watchdog section.

### Tests

- `tests/test_node_wiring.py` — three guards: the bridge still reads
  `MAV_SYS_STATUS_SENSOR_RC_RECEIVER` and publishes the topic; the watchdog records the
  verdict *and* ANDs it into `monitored_valid`; the LED node stamps a receipt time in
  every input callback.
- `tests/test_config_shared.py` — `status_timeout_s` clears two HEARTBEAT periods and
  exceeds `stream_timeout_s`; the LED's `input_timeout_s` is not tighter than the
  bridge's own fast-stream timeout.

> These are source-level checks. The unit tier cannot import rclpy, so **none of the
> three nodes has been executed with these changes.** Field verification is outstanding
> (§4.1).

---

## 3. Lessons

1. **A safety check that keeps firing is data, not an obstacle.** The throttle-neutral
   check was correct for hours before it was disabled, and disabling it produced a
   runaway. If a check will not clear and the reasoning says it should, the reasoning
   is wrong.
2. **Never let a receiver's configuration be the basis of a safety interlock.** "RC loss
   reads 0" was an assumption about a setting on a separate device, written into two
   independent safety nodes as fact.
3. **Silence is not safety.** `telemetry_bridge` correctly goes quiet on a stale stream;
   both consumers cached forever and read that quiet as "unchanged." Any node holding
   safety state from a callback needs an expiry.
4. **Match staleness timeouts to actual message rates.** HEARTBEAT is a fixed 1 Hz. A
   1.0 s timeout against it produced permanent false alarms that masked real ones.
5. **Use Mission Planner for RC calibration on this boat**, not QGroundControl.
6. **Verify tool assumptions before deep theorising.** Considerable time went into a
   geometrically-compelling `GPS1_MB_OFS` theory when the actual answer was that the
   moving-baseline solution needed more acquisition time.
7. **Name param files by what they are.** `working crusader params.params` was the
   stale, regressed file (`PILOT_STEER_TYPE=0`, untuned `MOT_THST_*`); the correct one
   was `crusader ardupilot params 20260802.params`. Loading the wrong one silently
   reverted the deadzone fix mid-session.
8. **Loading a params file reverts live `param set` work.** Re-save after tuning.

---

## 4. Current status

### 4.1 Blocking

| Item | State |
|---|---|
| **Runaway not root-caused** | Pull the dataflash log (`LOG_DISARMED=0`, so it starts at arm) and read `MODE`, `RCIN.C3`, `RCOU.C1–C4`, `ATT.Yaw` vs `GPS.Yaw`. **Do not arm with thrusters powered until this is explained.** |
| **New code unverified in the field** | Rebuild in-container, then re-run all three kills. Expected: `/crsd/rc_link_health` → `data: false`, LED RED with `ArduPilot reports RC receiver UNHEALTHY`, watchdog `rc_health=LOST`, and a force-disarm that actually lands. The LED going red is the indicator, not the kill — confirm the disarm separately. |

### 4.2 Open

- **`GPS1_MB_OFS` is `(0.5, 1.0, 0)`** — a 1.118 m baseline on a 0.6 m beam, measured
  actual 0.9 m. GPS yaw resolves and the EKF aligns with these values, so it is not an
  arming blocker, but the offset direction is what rotates receiver heading into hull
  heading. **Verify before the boat drives anywhere:** point the bow at a known bearing
  and compare to reported heading. Correct to the measured fore-aft vector if it is off.
- **ch7 is triple-booked and the thresholds conflict.** `RC7_OPTION=165` (autopilot
  e-stop), `pixhawk_led_status_node.estop_threshold=1200`, and
  `telemetry_bridge.drop_channel=7 / drop_threshold=1700`. The switch position that
  clears the autopilot e-stop (ch7 = 1995) is the same position that **trips the
  autonomy-drop latch**, so RC overrides and GUIDED setpoints from the ROS graph are
  blocked whenever the boat is in its normal armed state. Harmless tonight; blocks
  `dp_hold` and `gate_navigator` entirely. Needs its own session.
- **ELRS receiver failsafe** should be set to **No Pulses**, not Last Position. The new
  code detects the link loss either way, but a receiver that keeps commanding the last
  stick position after the link dies is wrong on its own terms.
- **Commit `217cf36` message** describes one file of six. Already pushed; amending
  requires a force-push to `main`.
- **Autonomy-drop switch** (CLAUDE.md standing constraint) still does not exist. No
  RC-override mechanism may be field-tested beyond WiFi range.

### 4.3 Verified working

- EKF position estimate and GPS moving-baseline yaw (`GPS_RAW_INT.yaw = 33257`).
- Arming with `RC_OPTIONS=32` (all checks enabled) after the Mission Planner
  recalibration.
- `/crsd/rc_link_health` correctly reporting a dead transmitter — observed live as
  `rc_health=LOST` with ch7 still reading 1995, which is precisely the case that was
  undetectable before this session.
- LED RED and YELLOW states; GREEN available via GUIDED (`MODE6=15`, mode switch
  position 6) now that a position estimate exists.

### 4.4 Demo readiness

Three of four pieces are in place. Once the RC link is restored and the runaway is
explained, the sequence is: safety switch pressed → SB switch up → mode HOLD or GUIDED
→ arm → RED/YELLOW/GREEN via the switch, with auto state running in the water
