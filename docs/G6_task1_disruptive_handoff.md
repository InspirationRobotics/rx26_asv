# G6 — Task 1 Disruptive tier: state of play and how to continue

Written 2026-09-10 as a handoff. Stages 0–2 are done and verified; 3–6 remain.
Read this before touching anything, then read the approved plan at
`C:\Users\chase\.claude\plans\i-want-you-to-synthetic-charm.md` (same Windows
user, so a different Claude account on this machine still sees it).

**Standing constraint from the user: do NOT `git push` until they confirm this
is tested on the boat.** Local commits are fine. As of writing, everything below
is **uncommitted working-tree state** — committing locally should probably be the
first thing the next session does, after reading the diff.

---

## What this is

Task 1 Safe Passage at **Disruptive tier**: the passage is only detectable by the
UAV, which relays it to the boat and **changes it during the transit**. Scope is
**USV only** — Ekko is greenfield (`rx26_uav` v0.1.0, nothing flown), so the boat
must be buildable and testable with no drone.

Four decisions already taken, do not relitigate:

| | |
|---|---|
| Transport | Real RXL frames over **UDP loopback** into `rxl_link_node`. Loopback → RFD900 is later a config change. |
| Beacon truth | **UAV wins.** OAK-D fuses for position; LiDAR feeds the autopilot's OA params, not the tree. |
| Passage model | **Ordered gate pairs** `(gate_seq, red_id, green_id)`. |
| Loop end | The **UAV** says when the passage is done (`255/255`). No geometric guess. |

---

## Done

### Stage 0 — the existing tree in SITL

Ran, and **found a real blocking bug** — now in memory as
[[crusader-transit-ahead-needs-exit]]. The boat approached and orbited the entry
buoy correctly, then `NextWaypoint: nothing left to steer to` and tree FAILURE at
47 s. `nav::nextWaypoint()` falls back to the **live heading** for "ahead" when
the exit is unknown; a 360° orbit ended at 207.7°, and both gate buoys 25 m north
scored `dot(rel, travel)` of −24.4 and −16.6 and were discarded as astern.

**Core tier is still broken this way and was deliberately not fixed** — the
Disruptive tree never hits it, because the plan carries the exit position. The
likely Core fix is to use the heading at mission *start*, not the live one.

### Stage 1 — gate geometry

`gateWaypoints()`, `gateFromIds()`, `findById()` in `nav_math.hpp`.
**84 checks, 0 failed** (was 57) in `crusader_bt/test/test_nav_math.cpp`.

The result worth knowing: **a gate's course is `bearingDeg(green → red) − 90`**,
a pure function of the pair with no dependence on the boat's heading. There is a
property-based check (RED scores positive against `starboardOf(u)`, GREEN against
`portOf(u)`) that a mirrored implementation cannot pass — the hand-worked compass
cases alone would not catch it.

### Stage 2 — the wire contract

* `crusader_msgs`: `PassageBuoy`, `PassagePlan`, `GatePair`. `PassagePlan.header.stamp`
  is **receipt** time — the vehicles share no time base.
* `crusader_link` package: `rxl_link_node` + ROS-free `rxl_codec.py`, with the
  23,478-line generated dialect vendored under `crusader_link/dialect/`.
* `tools/bench/bench_rxl_link.py` — stands in for the aircraft.
* Params block, `check_config` registration, and a UDP port guard
  (`rxl_endpoint` must be `udpin:`, in `1455x`, never `telemetry_bridge`'s).

**19 codec checks + 25 end-to-end, all passing. `CONFIG CHECK: PASS`.**

I narrowed a stated invariant rather than quietly breaking it:
`crusader_fcu/package.xml` said *"Nothing else may open a MAVLink connection"*;
it now says **"…to the autopilot"**, with the date and the reason.

---

## Remaining

**Stage 3 — fusion and freshness.** `fusePassage()` as a pure function in
`nav_math.hpp` (testable off-ROS). Rules in order: nearest confirmed tracker
target within `assoc_radius_m`; **beacon state ALWAYS from the plan**; position
from the tracker when associated else the plan; `id` stays the UAV's; an
unmatched tracker target is **not** a passage buoy and can never become a gate
candidate. Then `Context` fields + aging.

> **Do not skip the aging.** `ctx_->buoys` is never aged out today —
> `targets_t_` is recorded at `bt_runner_node.cpp:291` and never read, and
> `refreshFreshness()` (`:387-398`) ages only `pose_fresh` and `autonomous`. A
> dead radio must stop the boat, not leave it driving a plan it cannot confirm.

**Stage 4** — six leaves and `task1_disruptive.xml`. **Stage 5** — the virtual
UAV UI on the existing `field_gui.py`. **Stage 6** — `start_task1_sim.sh`, the
whole mission driven from a browser against SITL. That run is the definition of
done. Full detail is in the plan file.

---

## The environment, and how to not lose an hour to it

Three contexts: **Windows laptop → WSL2 `Ubuntu-22.04` → docker `crsd-sim`**.
SITL and Docker run in WSL; ROS runs only in the container, which is `NET=host`
with `~/robotx_ws` bind-mounted at `/root/robotx_ws`. This mirrors the real boat.

```bash
bash tools/sitl/sim.sh 'colcon build --packages-select crusader_link'
bash tools/sitl/sim.sh --no-sync 'ros2 topic list'
```

`sim.sh` re-establishes the whole chain every call, because **WSL2 shuts itself
down when idle** and takes dockerd and `/tmp` with it. The container then reports
`exit=255` — not OOM, not a crash, do not investigate it as one.

Bring SITL up with `bash tools/sitl/start_sitl.sh` inside WSL, then
`python3 tools/sitl/check_sitl.py` (9 checks, ~60 s, passes).

### Traps already paid for

| Trap | What it looks like |
|---|---|
| **`pkill -f <pattern>` on a command line** | The pattern is in the shell's own argv, so it kills itself: **exit 143**. Put patterns in a file. Also: `ros2 run pkg node` execs the install-path binary, so `-f 'ros2 run crusader_fcu'` matches only the wrapper — that left **three** `telemetry_bridge`es publishing `/crsd/pose`. |
| **`$VAR` in a `sim.sh` command** | Expands in the outer shell, where it is empty. `--params-file` then gets nothing. Use literal paths. |
| **WSL source is stale** | Its git HEAD lagged 7 commits while the *content* was current, so `git log` lies. `sync_to_wsl.sh` compares content across the whole tree — a file-by-file list missed `crusader_params.yaml` and crashed `target_tracker` with `KeyError: 'use_lidar'`. |
| **`set -u` before `source setup.bash`** | ROS's own setup reads `AMENT_TRACE_SETUP_FILES` unguarded and dies. Source first, then `set -u`. |
| **`g++` on Windows** | The sandbox blocks the link: exit 1, **empty stderr**. Build the off-ROS tests in the container instead — same GCC the boat uses. |
| **`--` inside an XML comment** | Illegal XML; `check_config` catches it and names the cause. |
| **The boat starts in HOLD** | `check_sitl.py` leaves it there, and `IsAutonomous` gates the whole tree, so a goal aborts in 0.01 s with outcome 4. Publish `GUIDED` to `/crsd/set_mode` first. |

Neither `bt_runner_node` nor `target_tracker` is in `core.launch.py`; start them
by hand. `publish_setpoints: true` is safe in SITL by construction — the only
vehicle listening is the simulator.

---

## Verify the handoff in about two minutes

```bash
bash tools/sitl/sim.sh 'cd src/rx26_asv && python3 tools/scripts/check_config.py 2>&1 | tail -2'
bash tools/sitl/sim.sh --no-sync 'cd src/rx26_asv && g++ -std=c++17 -O2 -I crusader_bt/include -o /tmp/t crusader_bt/test/test_nav_math.cpp && /tmp/t | tail -2'
bash tools/sitl/sim.sh --no-sync 'cd src/rx26_asv && python3 crusader_link/crusader_link/rxl_codec.py | tail -2'
```

Expect `CONFIG CHECK: PASS`, `84 checks, 0 failed`, and `PASS`. For the full link
test, start `rxl_link_node` with the params file and run
`tools/bench/bench_rxl_link.py` — 25 checks.
