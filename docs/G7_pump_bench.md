# Gate G7: water pump bench validation

**Purpose:** prove the water pump stops every time it should, however it was
started (the pilot's switch or a software burst), **before software is allowed to
fire it**. Until this is signed off, `telemetry_bridge.pump_servo_channel` stays
`0`: no pump path, and every `/crsd/pump_cmd` burst is refused.

**Why it needs a gate:**
- The pump is on a Pixhawk output that passes the pilot's pump channel through
  (`SERVOn_FUNCTION = RCINx`). That output is not a motor.
- So **the SB e-stop (`RC7_OPTION=165`) stops the thrusters but probably not the
  pump, and disarming does not stop it either.**
- The software path's safety rests on ArduPilot behaviour that has not yet been
  seen on this boat. It was written from memory of `AP_ServoRelayEvents`:
  - `MAV_CMD_DO_REPEAT_SERVO` with one cycle returns the output to `SERVOn_TRIM`
    on the autopilot's own timer.
  - `MAV_CMD_DO_SET_SERVO` is accepted on an RCIN pass-through output, and the
    pilot's switch takes it back.
- This procedure checks all of that. Record what you actually see; where it
  differs, the code is what changes.

**Props off for all of it.** Pump unplugged for section 1; then nozzle into a
bucket.

## 0. Move the pump off channel 9

Channel 9 belongs to the autonomy-drop switch (`telemetry_bridge.drop_channel`,
trips at ≥ 1700 µs). While the pump sat on ch9, **every squirt tripped the drop
latch.**

1. **Find the pump output.** With the boat idle, dump the live params:
   ```bash
   python3 tools/scripts/dump_params.py
   ```
   It starves telemetry while it runs. Look for `SERVOn_FUNCTION = 59` (RCIN9):
   that `n` is the pump output. The committed baseline does not show it: the
   baseline is stale.
   - n = ____, MAIN or AUX: ____ (a MAIN output sits behind the IO safety switch)
2. **Transmitter (Pocket, EdgeTX mixer).**
   - Move the pump switch from CH9 to **CH10**.
   - Mix **SE → CH9** for the drop switch.
   - Check in the RC monitor: ch10 is ~1000 off / ~2000 on, and ch9 follows SE.
3. **Autopilot params.** Then reboot:

   | Param | Value | Why |
   |---|---|---|
   | `SERVOn_FUNCTION` | `60` (RCIN10) | the pump follows ch10 |
   | `SERVOn_TRIM` | the pump's **OFF** PWM | every software burst ENDS at TRIM. A TRIM of 1500 on an ESC-driven pump is half throttle, forever |
   | `RC10_DZ` | `30` | small RC noise must not cancel a burst |
   | `SR0_EXTRA1` | `30` | attitude at 30 Hz; squirt_cal's "steady" needs ≥ 10 |
   | `SR0_RC_CHAN` | `10` | RC_CHANNELS and SERVO_OUTPUT_RAW at 10 Hz |

4. **Receiver failsafe → "No Pulses", not "Last Position".** Commit 64b5dc2
   recorded ch7 holding 1995 through a full transmitter power-down; a pump on
   "Last Position" keeps running after link loss.
5. Re-export `params/working_crusader.params`.
6. In `crusader_params.yaml`, set `telemetry_bridge.pump_on_pwm` /
   `pump_off_pwm` to what you measured.
7. Run `python3 tools/scripts/check_config.py`.

## 1. By hand (pump unplugged)

Send commands from Mission Planner or QGC on the MAVProxy **14550** port (Servo
tab), or MAVProxy's `long` command. **Never open a second connection to the
Pixhawk.**

| # | Do | Expect | Saw |
|---|---|---|---|
| 1.1 | pilot switch on / off | output follows ch10 | |
| 1.2 | pump ON, then SB e-stop | **probably keeps running** (motors only). This is why the bridge watchdog exists | |
| 1.3 | pump ON, then disarm | probably keeps running | |
| 1.4 | pump ON, then transmitter off | OFF with "No Pulses"; record what really happens | |
| 1.5 | `long DO_SET_SERVO n <ON>` then `long DO_SET_SERVO n <OFF>` | on, then off | |
| 1.6 | `long DO_SET_SERVO n <ON>`, then flick the pilot switch | the pilot takes the output back | |
| 1.7 | `long DO_REPEAT_SERVO n <ON> 1 0.6` | ON ~0.3 s, then back to TRIM = OFF. Film it at 240 fps | |
| 1.8 | 1.7, with MAVProxy killed mid-burst | still ends at TRIM | |
| 1.9 | (pump plugged in, bucket) command-to-water latency and spin-up | ______ ms / ______ ms | |

## 2. Through the bridge

1. Set `telemetry_bridge.pump_servo_channel: n`, run `tools/scripts/rebuild.sh`,
   and restart the bridge. The log says `pump path: SERVOn (pilot ch10)`.
2. `ros2 topic echo /crsd/pump_state` should show `enabled: true`,
   `output_fresh: true`, `output_pwm` = OFF.
3. Burst:
   ```bash
   ros2 topic pub --once /crsd/pump_cmd crusader_msgs/msg/PumpCommand "{duration_s: 0.3, seq: 1, source: bench}"
   ```

| # | Do | Expect `last_result` / `last_reason` | Saw |
|---|---|---|---|
| 2.1 | burst, disarmed | REFUSED `disarmed` | |
| 2.2 | burst, armed (props off), or disarmed with `-p pump_allow_disarmed:=true` | ACCEPTED; ~0.3 s of water | |
| 2.3 | burst with the pilot's switch ON | REFUSED `the pilot's pump switch is ON` | |
| 2.4 | burst with SB in e-stop | REFUSED `SB e-stop engaged` | |
| 2.5 | `duration_s: 1.5` | REFUSED `outside (0, 1.00] s` | |
| 2.6 | two bursts 0.5 s apart | the second REFUSED `too soon` | |
| 2.7 | pilot switch ON, then SB e-stop | bridge log `pump watchdog: pump ON with SB e-stop engaged -- OFF sent`; pump stops within ~1 s | |
| 2.8 | **pump unplugged**: set `SERVOn_TRIM` to 1500 on purpose, then burst | watchdog latches `is SERVOn_TRIM the OFF value`; later bursts REFUSED until restart. **Put TRIM back** | |
| 2.9 | burst of 0.5 s, then kill the bridge at once | the output still returns to OFF | |
| 2.10 | `duration_s: 0` | OFF sent, never refused | |

## 3. squirt_cal on the dock

```bash
python3 tools/squirt_cal/squirt_cal.py
```

Open `http://<jetson>:8094` on the phone. Fire into the bucket, in "Fire when
steady" mode, then "Fire now". Check that each shot is logged under
`/root/robotx_ws/logs/squirt_cal/<session>/shots.jsonl`.

## Sign-off

- [ ] Pump output n = ____ passes RCIN10 through, `SERVOn_TRIM` = OFF, receiver
      failsafe "No Pulses", baseline re-exported, `check_config` PASS.
- [ ] Every row in sections 1 and 2 is as expected, or the difference is written
      down and the code changed to match.
- [ ] Team agreement that a software pump burst is outside README rule 3. It
      drives no motor, the bridge refuses it under e-stop or with the pilot's
      switch on, and the autopilot ends it.

Signed: ______________  Date: ______________

Until signed, the pump path stays off. `squirt_cal` still works in **log-only
mode**: you fire with the switch and the tool records each shot.
