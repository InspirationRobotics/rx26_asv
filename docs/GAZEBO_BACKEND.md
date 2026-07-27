# Gazebo backend — closing the OmniX strafe-dynamics gap

## What this is for

`docker/sitl/run_sitl.sh` states the limitation that bounds every SITL result in
this repo:

> SITL's boat model ("motorboat" frame) does not model OmniX lateral thrust —
> GUIDED behavior, `AVOID_*`, `WP_*` logic are exact (same firmware), strafe
> dynamics are not. `dp_hold`-style RC-override mechanisms get logic-level
> testing only; their dynamics are bench/field territory.

Crusader is `FRAME_TYPE=2` (OmniX) — four thrusters in an X, genuinely
holonomic — and `dp_hold` is *the* lateral-hold path, because GUIDED cannot
strafe on this frame. So the one mechanism that depends most on lateral dynamics
is the one the simulator cannot evaluate. Its validation currently costs water
time, which is the scarcest resource of the season.

The `gazebo` backend keeps **everything** about the firmware identical — same
ArduRover 4.6.3, same extracted tunables, same MAVProxy topology on 14550/14551
— and swaps only the FDM. Gazebo drives a real four-thruster OmniX model, so
lateral thrust is physically simulated.

| backend | firmware | dynamics | lateral thrust | use |
|---|---|---|---|---|
| `kinematic` | none | none | n/a | CI, fast sweeps |
| `sitl` | real 4.6.3 | motorboat FDM | **no** | GUIDED/`AVOID_*`/`WP_*` logic |
| `gazebo` | real 4.6.3 | Gazebo physics | **yes** | `dp_hold`, station-keeping, docking |

Scenario JSON, evaluator, metrics and the backend contract are unchanged, so a
`gazebo` episode is directly comparable to a `sitl` one. That comparability is
the point of the backend abstraction and the reason the frame conversion is
shared verbatim (there's a test asserting it).

## Setup (crusader container, one time)

```bash
docker/sitl/install_sitl.sh      # ArduRover 4.6.3 — already pinned
docker/sitl/install_gazebo.sh    # Gazebo Harmonic + ardupilot_gazebo
```

## Run

```bash
python3 tools/sim/scenario_to_world.py --all

SCENARIO=orchestrator/scenarios/mission1_transit.json \
  docker/sitl/run_sitl_gazebo.sh

# another shell
RX26_SITL_OK=1 python3 orchestrator/run_episode.py \
    --scenario orchestrator/scenarios/mission1_transit.json \
    --backend gazebo --out /tmp/ep.json
```

`HEADLESS=1` (default) runs Gazebo server-only for CI; `HEADLESS=0` for the GUI.

## The guard that matters

**If Gazebo is not up, SITL silently falls back to its built-in motorboat FDM.**
The episode runs, the evaluator scores it, and the strafe results — the entire
reason this backend exists — are fiction. Nothing in the output says so.

So `GazeboBackend` probes the ArduPilot FDM port at construction and refuses to
start if nothing holds it. There's a `require_gazebo=False` escape hatch, but you
have to ask for it by name.

It also cross-checks the world's `<spherical_coordinates>` against
`scenario.origin` and aborts on mismatch. An origin mismatch puts the vehicle in
the right place on screen while its GPS fix is somewhere else; every
position-based metric is then wrong, and no metric reveals it.

## Worlds are build artifacts

`tools/sim/scenario_to_world.py` generates the Gazebo world **from the scenario
JSON**. Course geometry already has a single source of truth — the scenario file
feeds the kinematic backend, the SITL backend, the evaluator and the metrics
record. A hand-authored world would be a second copy, and the moment someone
nudges a buoy in the GUI, objective-1 collisions get computed against obstacles
the vehicle is not physically hitting.

Regenerate; never edit. Tests assert obstacle count, position and radius match
the scenario, and that keep-outs are *not* rendered — they're virtual
RoboCommand zones with no physical presence, and rendering them would let
perception "see" something the real course does not contain.

## Known gaps

- **Hydrodynamic coefficients in `tools/sim/models/crusader_omnix/model.sdf`
  are estimates, not measurements.** Right order of magnitude, nothing more.
  Treat `gazebo` results as *relative* comparisons between mechanisms, exactly
  as you'd treat `kinematic` results, until they're fitted from trials:
  step-thrust → drag, turning-circle → yaw drag, coast-down → added mass.
  Sway drag is deliberately set much higher than surge; getting that ratio wrong
  is precisely what would make `dp_hold` look easier in sim than on the water.
- **Control channel order must match ArduRover's OmniX motor ordering.** Verify
  against `SERVO*_FUNCTION` in `working_crusader_params.params` on first
  bring-up. If channels are permuted the boat translates when commanded to
  rotate — which looks like a controller bug and isn't.
- **Wall-clock stepping**, same as `SitlBackend`. If you raise `SIM_SPEEDUP` you
  must raise Gazebo's `real_time_factor` to match; changing one desynchronises
  the FDM link and the resulting motion is physically meaningless.
- **Only the ASV is modelled, by design.** The UUV and UAV are separate
  repositories deployed to their own devices. Cross-domain behaviour reaches
  this repo only as RoboCommand messages — already the mission planner's
  contract (`api/mission/robocomms.py`) — so no second vehicle is needed in the
  world. If you want UUV/UAV dynamics, they belong in those repos, sharing the
  scenario/RoboCommand contracts rather than this Gazebo world.

## Venue: scenarios are still at last season's origin

Every scenario carries `"origin": [32.7020, -117.2510]` — San Diego.
RobotX 2026 is Marina Bay, Singapore.

Geometry is translation-invariant, so objectives 1/2/3 don't care. **Magnetic
declination is not translation-invariant**: San Diego ≈ +11.5°, Singapore ≈
+0.2°. Anywhere true and magnetic heading are conflated — in firmware params or
in our own frame transforms — a constant ~11° is being silently absorbed today
and vanishes on arrival.

That's the failure mode where everything works all season and the boat turns the
wrong way at the venue.

```bash
python3 tools/sim/venue_check.py             # report
python3 tools/sim/venue_check.py --sweep     # same scenario, both declinations
python3 tools/sim/venue_check.py --derive 1.28165,103.85406 --write
```

Scenarios are **add-don't-mutate** (`docs/CHANGE_IMPACT_MAP.md`) — rewriting an
origin in place silently invalidates every historical result without changing
the scenario name, so old and new numbers sit in one history looking comparable
and are not. `--derive` writes `<name>_sg.json` copies instead; run both for a
while, and identical objectives across an ~11° declination change is your
evidence.

When you do switch: regenerate the worlds and update `HOME_LOC` in
`run_sitl.sh`. `run_sitl_gazebo.sh` derives it from the scenario already.

Verify the declination figures at
[NOAA NCEI](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml) for
November 2026 before trusting any of these numbers, including ours.

## Tests

```bash
python3 -m pytest orchestrator/tests/test_gazebo_backend.py -q
```

No Gazebo, SITL or pymavlink required — they cover exactly the parts that fail
silently: the guards, the backend contract, frame-conversion parity with
`SitlBackend`, and world/scenario agreement.
