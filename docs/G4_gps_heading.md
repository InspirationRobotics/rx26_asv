# G4 — GPS heading: why it was wrong, and what it is set to now

**Purpose:** record how Crusader's heading came to be wrong for weeks, what actually caused
it, and the exact configuration that fixes it — on both the autopilot and the GNSS receiver.

**Read this before changing anything about GPS yaw.** The failure mode here is silent: the
autopilot reports a confident heading the whole time it is wrong, and the obvious
parameter to reach for (`GPS1_MB_OFS`) has no effect at all while the fault is present.
Several hours were lost to experiments that could not have worked.

Settled 2026-09-08. Hardware: **Beitian BT-982G1**, a **Unicore UM982** (K922) dual-antenna
receiver, firmware `R4.10Build9984`, on ArduRover 4.6.3.

---

## 1. The symptom

Heading in QGroundControl was wrong by varying amounts — 90°, 180°, and values in between.
It appeared fine on land, went wrong on the water, changed when the boat was switched into
GUIDED, and "corrected itself" when the boat was carried back ashore. The same raw receiver
reading produced different autopilot headings minutes apart.

That pattern is the tell. It is not a constant mounting error; it is the heading being
**estimated from something other than the antennas**.

## 2. Root cause

The UM982 was configured with a fixed-baseline constraint that excluded its own measurement:

```
CONFIG HEADING FIXLENGTH
CONFIG HEADING LENGTH 112.00 3.00      -> 1.12 m ± 0.03 m
```

`UNIHEADINGA` reported the true antenna separation as **1.0716 m** on every sample. 1.074 m
is outside 1.09–1.15 m, so the constraint contradicted the observation and the solution
never resolved its integer ambiguities — it stayed `NARROW_FLOAT` indefinitely.

Someone had almost certainly tape-measured ~112 cm between the antenna housings; the true
**phase-centre** separation is 107.2 cm. A 4.6 cm error against a ±3 cm tolerance was enough
to break it completely.

ArduPilot accepts a moving-baseline yaw only from a **fixed** solution
(`libraries/AP_GPS/AP_GPS_NMEA.cpp`):

```c
if (now - _last_AGRICA_ms > 500 || ag.heading_status != 4) {
    state.have_gps_yaw = false;
    break;
}
```

So every yaw sample was discarded. With `COMPASS_USE=0` the EKF had no other yaw source and
fell back to its **EKF-GSF estimator**, which derives heading from the **direction of
travel**. On a holonomic OmniX hull that crabs sideways, direction of travel is not where
the bow points — which is exactly why it looked plausible on land and fell apart on water.

**Measured across three DataFlash logs before the fix:**

| log | duration | `GPYW` records | GPS yaw present | GPS yaw absent | `XKY0` (GSF) |
|---|---|---|---|---|---|
| `00000009` | 263 s | 0 | 0 | 1316 — **100%** | 12,143 |
| `00000008` | 585 s | 2 | 2 | 2,924 — 99.93% | 28,228 |
| `00000007` | 2093 s | 28 | 28 | 4,420 — 99.4% | 42,441 |

After the fix: **160 of 160** samples carry a real yaw, and EKF yaw tracks GPS yaw to
within 1°.

## 3. Why the `GPS1_MB_OFS` experiments were misleading

`GPS1_MB_OFS_X/Y` is only ever read inside `calculate_moving_base_yaw()`, which only runs
when the moving-baseline path succeeds. While the solution was `NARROW_FLOAT`, that function
never ran — **so every sign combination tried was inert.** The apparent 90°/180° differences
were GSF noise responding to how the boat had been moving, not to the parameter.

The lesson generalises: *before tuning a parameter, confirm the code path that reads it is
executing.*

## 4. How to diagnose this again in one pass

Three checks, cheapest first:

**`GPS_RAW_INT.yaw` has three meanings — do not test `!= 0`:**

| value | meaning |
|---|---|
| `0` | this GPS does not provide yaw (`gps_yaw_configured` false) |
| `65535` | configured to provide yaw, **currently unable** ← the broken state |
| anything else | a real heading, centidegrees |

Testing `!= 0` reads the unavailable sentinel as a valid heading. That mistake was made
during this investigation and produced a confidently wrong "100% of samples have yaw".

**In a DataFlash log:** no `GPYW` records means `calculate_moving_base_yaw()` never ran, so
`GPS1_MB_OFS` is inert. Many `XKY0` records means the GSF fallback is driving yaw. That pair
identifies the fault in a single scan.

**At the receiver:** `UNIHEADINGA` must read `NARROW_INT`. `NARROW_FLOAT` means no fix, and
ArduPilot will reject it.

## 5. Configuration as it now stands

### 5.1 Wiring

| receiver port | goes to | carries |
|---|---|---|
| **COM2** | Pixhawk GPS1 (`SERIAL3`) | `GNGGA`, `GPGGA`, `AGRICA`, `GNRMC`, `UNIHEADINGA` @ 5 Hz |
| **COM3** | Prolific PL2303 USB → Jetson `/dev/ttyUSB1` | `GPGGA`, `GPTHS` @ 2 Hz |

All three COM ports are at **230400** baud. `/dev/ttyUSB1` is the diagnostic wire — the
module answers ASCII commands there, so it can be inspected without disturbing the Pixhawk.

> **Do not read `/dev/crsd-pixhawk`.** It is MAVProxy's, and opening it steals bytes from
> the flight stack, which then sees silence rather than an error.

### 5.2 UM982 receiver configuration

Changed in this session, saved with `SAVECONFIG`:

| setting | was | **now** |
|---|---|---|
| `CONFIG HEADING LENGTH` | `112.00 3.00` | **`107.20 5.00`** |
| `CONFIG HEADING OFFSET` | `74.4 0.0` | **`55.16 0.0`** (reads back as `55.2`) |

Unchanged, for reference:

```
CONFIG HEADING FIXLENGTH            <- ArduPilot re-sends this; do not fight it
CONFIG HEADING RELIABILITY 0
CONFIG UNDULATION AUTO
CONFIG NMEAVERSION V410
CONFIG ANTENNA POWERON
CONFIG SIGNALGROUP 4 5
CONFIG RTK TIMEOUT 600
CONFIG COM1 230400 / COM2 230400 / COM3 230400
```

**`CONFIG HEADING LENGTH` is in centimetres, and it is the phase-centre separation, not the
distance between housings.** Read the value `UNIHEADINGA` reports rather than measuring with
a tape. If the antennas are ever moved, this must be re-set or heading breaks again in
exactly this silent way.

**Never simply disable `FIXLENGTH`** — ArduPilot re-sends `CONFIG HEADING FIXLENGTH` every
config period for `GPS1_TYPE=25`, so it always comes back. Correct the LENGTH instead.

### 5.3 ArduPilot parameters

| parameter | value | note |
|---|---|---|
| `GPS1_TYPE` | **25** | Unicore moving-baseline NMEA. Correct for a UM982 — it is a Unicore part, not u-blox |
| `GPS1_MB_TYPE` | 1 | `RelativeToCustomBase` |
| `GPS1_MB_OFS_X / Y / Z` | 0.94 / 0.5 / 0 | see §7 — **not verified against real geometry** |
| `GPS1_POS_X / Y / Z` | 0.47 / −0.25 / 0 | likewise unverified |
| `GPS1_RATE_MS` | 200 | 5 Hz |
| `GPS_AUTO_CONFIG` | 1 | ArduPilot configures the receiver itself — and it works |
| `GPS_SAVE_CFG` | 2 | |
| `EK3_SRC1_YAW` | 2 | GPS yaw |
| `EK3_GSF_RST_MAX` | **2** | firmware default, restored after debugging left it at 1 |
| `COMPASS_USE / 2 / 3` | 0 | compass off by design; GPS yaw is the whole heading solution |
| `AHRS_EKF_TYPE` | 3 | |
| `SERIAL3_PROTOCOL / BAUD` | 5 / 230 | GPS, 230400 |
| `SERIAL4_PROTOCOL / BAUD` | 5 / 230 | GPS, 230400 |

**`GPS1_TYPE=5` (plain NMEA) is wrong for this boat and was tried during the
investigation.** Under type 5 ArduPilot ignores `AGRICA`/`UNIHEADINGA` and looks for `HDT`
or `THS`, which COM2 does not carry — so it produces *no* yaw at all. It is strictly worse
than type 25.

## 6. Heading calibration

The receiver produces a precise heading but along the **antenna baseline**, which is not the
bow. That constant rotation was measured on 2026-09-08.

**Method.** The boat was carried on a cart with the bow held along the direction of travel
for ~1000 s of walking. Course over ground equals true heading when moving in a straight line
without crab, so `offset = COG − reported_heading`.

**Filtering.** Legs needed ≥8 m displacement and a straightness ratio
(displacement ÷ path walked) ≥ 0.90. One 459-sample leg scored **0.26** — it had wandered —
and was discarded; leaving it in would have shifted the answer by ~1.7°.

**Result, four usable legs:**

| estimate | value | spread |
|---|---|---|
| start→end bearing | **−19.24°** | 2.27 |
| instantaneous COG | −18.91° | 1.39 |
| reciprocal pair (340° / 171°) | −20.92° | |
| reciprocal pair (76° / 259°) | −17.55° | |

Two independent estimates agreeing within 0.4°, and both reciprocal pairs landing close,
means **crab was cancelled rather than absorbed into the answer**. Running both directions
is what buys that; a single leg cannot separate a sideways drift from a mounting offset.

Applied as `CONFIG HEADING OFFSET 74.4 → 55.16`. The **sign was verified by measurement**,
not assumed: reported heading was sampled before and after, and moved −19.42° as intended.

> **Do not calibrate against a phone compass.** The phone readings taken during this session
> spread over 100–131° for the same physical offset. San Diego's magnetic declination alone
> is ~11.5° E.

## 7. Still open

**The correction is split across two places.** The receiver applies 55.2° and ArduPilot adds
~152.03° derived from `GPS1_MB_OFS=(0.94, 0.5)`. The total is calibrated and correct, but
**`$GNTHS` on its own is not true heading** — it is ~152° off.

Collapsing this into one place requires knowing where the antennas actually are.
`GPS1_MB_OFS` is documented as *"X position of the base (primary) GPS antenna in body frame
from the position of the 2nd antenna"* — it is meant to describe real geometry, and the
current value has never been checked against the boat. **Measuring both antenna positions
with a tape** would let `GPS1_MB_OFS` be set honestly and the receiver offset zeroed, putting
the whole correction in one auditable place. Ten minutes of work; not blocking anything.

Unrelated but noted: `AHRS_ORIENTATION` is **30** on the boat and **0** in
`params/working_crusader.params`. That divergence is undocumented and unresolved. The
baseline was deliberately **not** re-exported after this session, because it also differs on
the battery parameters and promoting it would quietly bless those too.

## 8. Reference

- **UM982 command interface:** <https://s-taka.org/en/control-command-for-gnss-receiver-um982/>
  — covers the ASCII command form (`gngga 10`), `unlog`, `config`, `saveconfig`.

Commands used in this investigation, all on `/dev/ttyUSB1` at 230400:

| command | purpose |
|---|---|
| `VERSIONA` | firmware and model — proves the module answers on this wire |
| `UNILOGLIST` | which messages are enabled, and on which COM port |
| `CONFIG` | all current settings |
| `UNIHEADINGA 1` | enable the heading message at 1 Hz (diagnostic) |
| `UNLOG UNIHEADINGA` | remove it again |
| `CONFIG HEADING LENGTH <cm> <tol>` | the baseline constraint |
| `CONFIG HEADING OFFSET <deg> 0.0` | rotation from baseline to bow |
| `SAVECONFIG` | persist to flash |

`UNIHEADINGA` output, fields after the `;`:

```
SOL_COMPUTED, NARROW_INT, 1.0685, 315.3209, 2.0979, 0.0000, 0.3363, 0.7483, ...
     status    pos type   baseline  heading   pitch    resvd   hdg sd  pitch sd
                          (metres)  (degrees)
```

`LOG <msg> ONCE` is **not** supported by this firmware — it returns
`PARSING FAILD GRAMMAR ERROR`. Enable at a rate and `UNLOG` afterwards instead.

ArduPilot source consulted (checkout at tag `Rover-4.6.3`):

- `libraries/AP_GPS/AP_GPS_NMEA.cpp` — the `heading_status != 4` gate, the `_expect_agrica`
  flag, and the Unicore config strings ArduPilot sends
- `libraries/AP_GPS/GPS_Backend.cpp` — `calculate_moving_base_yaw()`, where `GPS1_MB_OFS` is
  applied and `GPYW` is logged
- `libraries/AP_NavEKF3/AP_NavEKF3.cpp` — `EK3_GSF_RST_MAX` default
