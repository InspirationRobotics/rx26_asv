# G5 — Obstacle avoidance: what each margin does, and what Crusader is set to

**Purpose:** record which avoidance mechanism actually runs on this boat, what the three
"margin" parameters really control, and why the values are what they are — so nobody tunes
one expecting the behaviour of another.

**The three margins are not variations on a theme.** They apply to different things, and two
of them are frequently confused. Getting this wrong produces a boat that either refuses to
enter a gate or drives through a buoy, with no error either way.

Established 2026-09-08 against ArduRover 4.6.3 (checkout tag `Rover-4.6.3`).

> **STATUS: the `OA_TYPE` change in §5 is NOT YET APPLIED.** It was blocked because the
> vehicle was armed and under way. Apply it, then power-cycle — see §6.

---

## 1. What actually runs today

```
OA_TYPE       0        <- path planning DISABLED
PRX1_TYPE     2        <- MAVLink proximity, working
AVOID_ENABLE  3        <- fence + proximity
AVOID_MARGIN  1.5
FENCE_ENABLE  0        <- no fence loaded
DISTANCE_SENSOR: 121 messages in 8 s
```

Proximity data **is** flowing. With `OA_TYPE=0`, `AVOID_ENABLE=3` gives *simple* avoidance
only: the boat **stops** in front of obstacles. Nothing routes around them.

> **`crusader_params.yaml:301-303` is stale.** It states `PRX1_TYPE=0`, "avoidance switched
> on with nothing feeding it." That was fixed at some point and the comment was not updated.
> Trust the live parameter, not the comment.

## 2. The three margins

Quoted from the firmware, not from memory:

| parameter | firmware description | applies to |
|---|---|---|
| `AVOID_MARGIN` | "stay at least this distance from **objects** while in GPS modes" | sensed obstacles |
| `FENCE_MARGIN` | "distance the autopilot should maintain from **the fence** to avoid a breach" | the geofence |
| `OA_MARGIN_MAX` | "Object Avoidance will **ignore objects more than** this many meters from vehicle" | BendyRuler's attention radius |

`OA_MARGIN_MAX` is the one most often misread. It is **not a clearance** — it is how far out
BendyRuler bothers to look. Raising it makes the planner more eager, not the boat safer.

**`AVOID_MARGIN` is measured from the vehicle origin, not the hull edge.** Real water between
hull and obstacle is `AVOID_MARGIN − (origin to bow)`.

## 3. Which planner, and why

`OA_TYPE` values in this firmware: `0:Disabled 1:BendyRuler 2:Dijkstra 3:Dijkstra with BendyRuler`

They plan against **different data**, which settles the choice:

- **BendyRuler** includes `AC_Avoidance/AP_OADatabase.h` and threads `proximity_only` through
  its search — it plans against the **live object database**, i.e. the MID360 returns.
- **Dijkstra** works on `_inclusion_polygon_pts`, `_exclusion_polygon_pts`,
  `_exclusion_circle_pts`, and returns immediately on `if (!some_fences_enabled())`. It
  avoids **fences**, not sensed obstacles.

With `FENCE_ENABLE=0` and no exclusion polygons, Dijkstra would do nothing. **BendyRuler (1)
is the only useful setting today.** Option 3 becomes worthwhile once a course boundary and
exclusion zones exist — see §4.

BendyRuler only runs in **AUTO, GUIDED and RTL**. Crusader's missions run GUIDED, so that is
satisfied — but **nothing protects the boat in MANUAL**.

## 4. The Task 1 conflict — read before trusting avoidance in a buoy field

**BendyRuler picks whichever side of an obstacle gives more margin. It has no knowledge of
the RED-to-starboard / GREEN-to-port rule.**

In the Task 1 field it will happily route the boat around the wrong side of a red buoy — a
scoring failure — while `nav::sideWaypoint()` in `crusader_bt` is simultaneously computing
the correct-side waypoint. Two controllers with different objectives on one hull.

The mitigation is to make BendyRuler a **last-resort collision guard rather than a router**,
by shrinking its attention radius so it only reacts to things genuinely about to be hit, and
leaving route choice to the behaviour tree.

**The 5 m vessel standoff cannot live in `AVOID_MARGIN`.**
`handbook/hidden-3.6-scoring-rounds.md` requires USVs to keep 5 m from stationary distractor
vessels in Semi-Finals and Finals. A blanket 5 m proximity margin would make every Task 1
gate impassable (see §5). The correct mechanism is an **exclusion circle** in the fence around
each identified vessel — which is exactly the case where `OA_TYPE=3` earns its keep: Dijkstra
handles the 5 m exclusion zones while BendyRuler keeps a small margin for buoy work. Two
standoffs for two different things.

## 5. Values, and what constrains them

| parameter | current | target | reasoning |
|---|---|---|---|
| `OA_TYPE` | 0 | **1** | only planner that uses proximity data |
| `OA_MARGIN_MAX` | 5.0 | **2.5** | at 5 m it steers around buoys the mission wants to pass close on a chosen side |
| `AVOID_MARGIN` | 1.5 | **1.5** (unchanged) | see the gate-width constraint below |
| `FENCE_MARGIN` | 1.5 | **3.0** | inert until a fence is loaded; pre-staged |
| `AVOID_ENABLE` | 3 | 3 | fence + proximity, already correct |
| `FENCE_ENABLE` | 0 | 0 for now | no course boundary polygon to load yet |

### The gate-width constraint

To pass between a red and green buoy the boat needs

```
gate gap  >  2 × AVOID_MARGIN  +  hull beam
```

or it refuses the gap and stops. At 1.5 m that needs >3 m plus beam; at 3 m it would need
>6 m plus beam and a narrow gate becomes impassable.

**`AVOID_MARGIN` was deliberately left at 1.5 rather than raised.** Two numbers needed to
set it properly are unknown:

- **Hull beam and length are not recorded.** `Resources.md` documents draft (24 cm) and
  sensor heights but no footprint. From `GPS1_POS_X = 0.47` and a 1.07 m antenna baseline the
  boat is at least ~1.2 m long, so 1.5 m of margin is realistically **under a metre of actual
  clearance**. Measure the hull and write it into `Resources.md`.
- **The handbook specifies no gate width for Task 1.** Confirmed across all 49 pages: no gate
  count, no gate width, no buoy spacing. This number can only be settled on a practice course.

Raising `AVOID_MARGIN` without those measurements would be guessing at the one value that
decides whether the boat physically fits through a gate.

### Why `FENCE_MARGIN` is worth setting even though it is inert

`handbook/5.1-team-rules-and-requirements.md` #11: a run is **terminated** if a system crosses
outside the course boundary. That is one of very few instant-failure conditions in the rules,
and cheap to guard in software. At 1.5 m a boat with `TURN_RADIUS = 0.9` — whose real turning
circle at speed is larger — would notice far too late. 3–5 m gives room to stop and turn.

`FENCE_TYPE = 6` is Circle + Polygon; polygon is the one that matters for a course boundary.
**Do not enable a fence with no polygon loaded** — it degenerates into a circle around
wherever the boat happened to arm.

## 6. Applying it — a reboot is mandatory

`Rover/system.cpp:118` calls `g2.oa.init()` **exactly once, at startup**. With `OA_TYPE=0`
that call returns before `start_thread()`, so `_thread_created` stays false and
`AP_OAPathPlanner::mission_avoidance()` bails at line 207 no matter what `OA_TYPE` is set to
afterwards.

```
1. Disarm the vehicle.
2. Set OA_TYPE=1, OA_MARGIN_MAX=2.5, FENCE_MARGIN=3.0.
3. POWER-CYCLE the boat.
4. OA_BR_TYPE and OA_BR_LOOKAHEAD now exist. Set OA_BR_LOOKAHEAD ~4.
5. Verify against a real obstacle before trusting it in a buoy field.
```

**Power-cycle, do not reboot over MAVLink.** The Pixhawk is on USB (`/dev/ttyACM*`), and a
`PREFLIGHT_REBOOT_SHUTDOWN` makes it re-enumerate; MAVProxy then holds a dead handle and logs
`no link` forever. Restarting `crsd-mavproxy` did **not** recover it on 2026-09-08 — a full
power cycle was needed. See [G4](G4_gps_heading.md).

`OA_BR_*` parameters do not exist until `OA_TYPE` selects BendyRuler — `init()` allocates the
object and loads its parameters lazily. Reading them before the reboot returns `MISSING`;
that is expected, not a fault.

## 7. Verification

Avoidance that has never been tested against a real object is not a safety feature. Before
relying on it:

- Confirm `OA_BR_TYPE` and `OA_BR_LOOKAHEAD` **exist** after the reboot — if they are still
  missing, the planner did not start.
- Drive a GUIDED leg at a stationary object and confirm the boat **routes around** it rather
  than stopping. Stopping means `OA_TYPE` did not take effect; `AVOID_ENABLE` alone only
  stops.
- Confirm it does **not** deviate around a buoy 3 m off the intended track — that is what
  `OA_MARGIN_MAX = 2.5` is meant to prevent, and it is the behaviour that would break Task 1.
- Note `CRUISE_SPEED = 0.1` at time of writing. If that is not a deliberate pool-testing
  throttle it will make every leg crawl and distort avoidance timing.

## 8. Source consulted

- `libraries/AC_Avoidance/AP_OAPathPlanner.cpp` — `OA_TYPE` values, `MARGIN_MAX` description,
  `init()` allocation, the `!_thread_created` gate at line 207
- `libraries/AC_Avoidance/AP_OABendyRuler.cpp` — uses `AP_OADatabase`, i.e. live proximity
- `libraries/AC_Avoidance/AP_OADijkstra.cpp` — fence polygons only, `some_fences_enabled()`
- `libraries/AC_Avoidance/AC_Avoid.cpp` — `AVOID_MARGIN` description and default (2.0)
- `libraries/AC_Fence/AC_Fence.cpp` — `FENCE_MARGIN` description
- `Rover/system.cpp:118` — the single `g2.oa.init()` call
