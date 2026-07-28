# Phase 0–5 build log (historical)

**This is a changelog, not a procedure.** It records what landed in each phase and the
gate criteria each phase was signed off against. For setup and day-to-day workflow use
[README.md](README.md) and [docs/SETUP_GUIDE.md](docs/SETUP_GUIDE.md) instead.

Historical note: phases 0–5 were originally developed as a scaffold intended to be merged
into a separate live boat repo. That merge is complete — every node referenced below now
lives in this tree, and this repo is standalone (canonical remote:
`github.com/InspirationRobotics/rx26_asv`). The old rsync-into-another-repo procedure has
been removed; on the Jetson the repo is cloned to `~/robotx_ws/src/rx26_asv`, one package
source inside a colcon workspace.

Build after a pull, inside the `asv` container:
`cd /root/robotx_ws && colcon build --symlink-install`
(builds `interfaces`; `orchestrator/` is COLCON_IGNOREd by design).

## Gate G0 (definition of done for Phase 0)

One command runs a scripted episode headless and emits the 3-objective metric JSON:

```bash
# anywhere with python3 (no ROS needed) — kinematic backend:
python3 orchestrator/run_episode.py \
    --scenario orchestrator/scenarios/mission1_transit.json \
    --backend kinematic --seed 0 --out /tmp/ep.json

# in the container with SITL running (docker/sitl/run_sitl.sh):
RX26_SITL_OK=1 python3 orchestrator/run_episode.py \
    --scenario orchestrator/scenarios/mission1_transit.json \
    --backend sitl --mav udp:127.0.0.1:14550 --out /tmp/ep.json
```

Tests: `python3 -m pytest orchestrator/tests -q` (CI runs this on every push).

## Phase 1 additions (safety prerequisites & HAL)

New under `rx26_asv/api/` (no collisions with existing nodes):

- `common/drop_latch.py` — autonomy-drop state machine (fail-safe start, latched
  trip on switch/RC-loss/staleness, explicit-reset-only). Pure Python, fully unit-tested.
- `navigation/telemetry_bridge.py` — THE single MAVProxy-rebroadcast consumer and THE
  single RC-override sender. Publishes `/crsd/pose`, `/crsd/fcu_status`,
  `/crsd/rc_channels`, latched `/crsd/autonomy_drop`; forwards `/crsd/rc_override`
  only while the latch allows; `/crsd/autonomy_drop_reset` service.
- `common/override_guard.py` — client pattern for override-producing nodes (dp_hold
  wiring notes in `docs/G1_bench_procedure.md`).
- `navigation/frame_transform.py` — BODY→WORLD detections + latched `/crsd/world_origin`.
- `perception/oakd_guard.py` — USB3 SUPER assert (module + CLI, used by preflight).
- `testing/rc_override_smoke.py` — G1 bench node (props off).

Also: 4 new msgs in `interfaces/` (RcChannels, FcuStatus, Detection, DetectionArray),
`tools/scripts/collect_footage.py` (G2 retrain data capture), `docs/G1_bench_procedure.md`.

**Jetson wiring after merge:** add entry points to the package `setup.py`:
`telemetry_bridge`, `frame_transform`, `rc_override_smoke` → their `main()`s; then
rebuild in-container. Add `telemetry_bridge` + `frame_transform` to `core.launch.py`.
MAVProxy needs a local rebroadcast out for ROS (e.g. `--out udp:127.0.0.1:14551`) —
check `scripts/start_mavproxy.sh`. Do NOT launch `rc_override_smoke` outside the G1
bench procedure.

## Phase 2 additions (perception)

- `api/perception/depth_association.py` — bbox → median-depth → BODY-frame projection
  (the diagram's pixel→point-cloud stage); drops detections with invalid depth
  (position unknown ≠ position zero). Unit-tested.
- `api/perception/detector.py` + `config/class_map.json` — TensorRT/YOLO wrapper with
  canonical-label mapping; refuses to start on unmapped classes (fill the real model
  class list on first bench run — see the TODO in the json).
- `api/perception/pipeline_stats.py` — fps/latency budget accounting (≥15 fps,
  p95 ≤100 ms); published on `/crsd/perception_health`.
- `api/perception/perception_node.py` — single-process capture→detect→associate →
  `/crsd/detections_body`; USB SUPER assert + calibration-read intrinsics at start;
  OAK-D LR CAM_A/B/C socket assignment must be verified on first bring-up.
- `tools/training/` — `prep_dataset.py` (session-level splits: leakage guard),
  `train_buoy.py` (fine-tune + per-Jetson engine export), `eval_regression.py`
  (per-class P/R gate on held-out sessions; exits nonzero on fail).
- `tools/bench/g2_error_logger.py` + `docs/G2_bench_procedure.md` — the G2 gate:
  regression pass + ≥15 fps + p95 ≤100 ms + <0.5 m error at 10 m vs RTK truth.
  After sign-off, episode runs may pass `--perception-trusted` to `run_episode.py`;
  the evaluator default stays untrusted.

**Jetson wiring after merge:** entry point for `perception_node` in `setup.py`;
launch after `telemetry_bridge`/`frame_transform`. Extend `tools/scripts/preflight.py`
`must_exist` topics with `/crsd/pose` and `/crsd/detections_body` once nodes are in
`core.launch.py`.

## Phase 3 additions (occupancy grid + reactive avoidance)

- `api/navigation/occupancy_core.py` (+node) — world-frame sparse grid: perception
  cells decay (`v0*exp(-dt/tau)`) and prune; comms cells persist until `clear_zone`
  (All Clear); perception can never overwrite a keep-out cell. `/crsd/occupancy_grid`.
- `api/navigation/apf_core.py` + `roa_apf_node.py` — APF **advisory** (§3.2: never
  commands motors): corrected goal + speed scale on `/crsd/apf_advisory`; trap
  detection with deterministic tangential bias; moving hazards repelled from current
  AND projected position (10 m). Tuning is asserted in tests: the APF equilibrium
  distance must exceed AVOID_MARGIN.
- `api/navigation/progress_monitor.py` — preventative at-risk detector (flags BEFORE
  stall); mission planner consumes `progress_state` for escapes (Level-2 target).
- `api/navigation/fence_core.py` + telemetry_bridge wiring — `/crsd/keepouts`
  (label=zone_id, radius<=0=All Clear) → MAVLink `FENCE_CIRCLE_EXCLUSION` upload with
  **mandatory readback verify** ("a fence the autopilot doesn't echo back does not
  exist"); `/crsd/fence_state` (latched JSON). Requires FENCE_ENABLE=1 + FENCE_TYPE
  circle bit on the boat (check via param_guard, not set from code).
- **Gate G3** (`orchestrator/run_gate_g3.py`, in CI): the boat's real apf_core drives
  20 seeded kinematic episodes through a blocking-obstacle field — zero violations,
  zero stalls, all complete, plus a control run proving the scenario stresses
  avoidance. SITL repeat of G3 runs in the container before water.

**Jetson wiring after merge:** entry points for `occupancy_grid_node`, `roa_apf_node`;
gate_navigator wrapper subscribes `/crsd/apf_advisory` and blends corrected goal +
speed scale into its GUIDED setpoints (Phase 4 formalizes this as a task wrapper).

## Phase 3.5 additions (parameter management + safe exit — best-practices pass)

**`config/crusader_params.yaml` is the single source of truth for ROS-side params.**
Standard ROS 2 param-file format (usable directly with `--params-file` / launch
`parameters=[...]`), with YAML anchors tying values that must stay equal across
nodes (occupied_threshold grid↔APF; progress-monitor thresholds node↔evaluator).
Node code loads this file for its declaration defaults (`api/common/config.py`), the
episode evaluator reads the same file for objective-2 scoring, and its sha256 is
recorded as `ros_config_hash` in every episode metrics JSON. Tune between test runs
by editing the YAML (restart) or, for [DYN] params, live via `ros2 param set`.

**Every parameter now has an explicit posture** (`api/common/param_utils.py`):
- `[RO]` read_only — `ros2 param set` is rejected loudly (all telemetry_bridge
  safety params, grid geometry, engine path, monitor thresholds);
- `[DYN]` dynamic — range-validated set-callback that actually applies the value
  (APF gains, conf_threshold, decay_tau, health budgets). The pre-3.5 posture —
  declared-but-ignored — no longer exists anywhere.
APF gains + monitor thresholds are now declared params (the ROS-side Level-1
tunable surface). `declare_from_config` refuses to start a node whose config
section and spec disagree.

**Canonical safe exit** (`api/common/node_main.py`, used by all 7 entry points):
construction inside `try` with guarded `finally` (constructor failures propagate
cleanly), `ExternalShutdownException` handled (clean `ros2 launch` shutdown),
SIGTERM converted to a normal teardown path (systemd/`docker stop` now run
`destroy_node()` — thread joins, MAVLink close), `rclpy.try_shutdown()` throughout.
Also: the bridge's heartbeat wait is now interruptible and warns every 10 s
instead of blocking silently forever.

Anti-drift guards live in `tests/test_config_shared.py` — they fail the build if
the YAML anchors, ApfParams fields, monitor kwargs, or evaluator thresholds ever
diverge.

## Per-boat device configs → udev (from RoboBoat_2026 crusader.json)

Crusader's real device topology lives in the RoboBoat repo's
`config/crusader.json`, which pins devices (teensy, ball_launcher, gps, led,
oakd_lr) to physical USB paths and resolves them at runtime via
`usbLink.sh` + `deviceHelper.findFromId` string matching.
`tools/udev/gen_udev_rules.py` converts that config into native udev rules:

- **Port-chain matching** (`KERNELS=="1-2.2:1.0"`): same physical-port identity
  the team already relies on, but resolved by the kernel at plug time — and
  immune to the JetPack controller-prefix rename that differs between
  barco.json (`3610000.xhci`) and crusader.json (`bus@0/3610000.usb`).
  Tradeoff (unchanged from current practice): each device must stay on ITS
  port — label the hub ports.
- Committed artifacts, regenerate after any cabling change:
  `tools/udev/99-crusader-devpath.rules` (installed by `install_udev.sh`, which
  now installs every `99-crusader*.rules`) and `config/crusader_devices.json` —
  the same config shape with `port` rewritten to stable symlinks
  (`/dev/crsd-teensy`, `/dev/crsd-gps`, `/dev/crsd-led`, `/dev/crsd-ball-launcher`),
  so consumers open the symlink directly and `findFromId`/`usbLink.sh` become
  unnecessary at runtime.
- Preflight now recognizes the generated names. The VID-based rules in
  `99-crusader.rules` remain as fallback/permissions (OAK-D, Pixhawk for the
  RobotX fit-out); `SYMLINK+=` is additive, so the two rule files coexist.
- Note: crusader.json reflects the **RoboBoat** fit-out (Teensy + ball
  launcher). For the RobotX/ArduRover configuration, re-run the generator
  against an updated JSON that swaps the teensy entry for the Pixhawk's port
  — the tool is boat-agnostic.

## Phase 4 additions (mission planner + RoboCommand, Gate G4)

- `api/mission/task_stack.py` — TaskContext (JSON-serializable resumable state)
  + TaskStack with audit log; pop-without-push fails loudly.
- `api/mission/tasks/waypoint_mission.py` — WaypointMission (suspend/resume by
  waypoint index + reached list; gate_navigator wraps into this contract) and
  LoiterAssist (transit → dwell → on-station; dwell resets on drift-out).
- `api/mission/planner.py` — MissionPlanner: non-blocking event drain,
  AssistanceRequest → ACK_RECEIPT+ACK_INTENT → push+suspend → LoiterAssist →
  READINESS once on station → resume ONLY on matching Clearance → RESUMPTION.
  Keep-outs/moving objects ack'd + routed to the avoidance sink, never the
  stack. All logs (events/acks/transitions/resume record) in sim time for
  scoring — the evaluator scores logs, not planner claims.
- `api/mission/robocomms.py` — RoboCommandClient: listener thread → queue
  (planner never blocks on the socket), length-prefixed proto/JSON framing
  proven byte-identical against the REAL mock server in a live loopback test;
  malformed frames counted loudly, stream survives.
- `api/mission/mission_planner_node.py` + `mission_planner_node` config section
  — on-boat wiring: /crsd/active_goal out, /crsd/keepouts out (same contract as
  bridge fence + grid), mission from `mission_file` JSON. Moving-hazard topic is
  an explicit TODO (APF core already supports them).
- **Evaluator**: `score_mission4()` fills the mission4 sub-metrics from planner
  logs (ack correctness incl. ordering, immediate-ack latency, comms
  compliance, resume fidelity). objective2 is now loiter-aware: samples carry
  the commanded goal, so a commanded hold is not a stall (matches the onboard
  ProgressMonitor semantics).
- **Gate G4** (`orchestrator/run_gate_g4.py`, in CI): 10 seeds of Mission-4
  Core with the APF advisor active + a concurrent Advanced-tier keep-out:
  completion, zero violations, compliance 1.0, ordered acks ≤1 s, resume
  fidelity true, exactly one stack push. A deliberate silently-restarting
  mission is proven CAUGHT by the fidelity check (resume-state-drift test).

**Jetson wiring after merge:** entry point `mission_planner_node`; launch after
telemetry_bridge/frame_transform; set `robocommand_host/port` to the RJ-45
RoboCommand net (mock_robocommand for bench); provide `mission_file`. The
gate_navigator wrapper should consume /crsd/active_goal blended with
/crsd/apf_advisory. Compile proto/robocommand.proto in the container
(`protoc --python_out`) to switch the link from JSON fallback to protobuf.

## Phase 4.1 — cross-file integration audit fixes (planner/comms hardening)

All 8 audit findings addressed, each with a regression test:
1. **ACK_INTENT is now a commitment, not a reflex**: sent only when servicing
   begins; concurrent AssistanceRequests queue FIFO (`pending_requests`) and
   are serviced in order after resume — no promised-and-dropped requests.
   `score_mission4` reclassifies ACK_INTENT as correctness-gated (eventual),
   not latency-gated.
2. **LoiterAssist hysteresis**: dwell resets only beyond
   `loiter_radius * 1.15` — boundary pose jitter can no longer livelock
   READINESS (tested with oscillating samples).
3. **Comms dead-man's switch**: `RoboCommandClient.alive`/`health()` +
   `dead_reason` on unexpected listener death; the node polls it and logs
   "link DEAD" loudly. Planner records an "interrupt stalled Ns without
   Clearance" warning (once, surfaced by the node) — and never auto-resumes.
4. **Exception containment**: per-event dispatch is try/except'd
   (`strict=False` at runtime: count + log + continue; `strict=True` in
   tests/harness: raise), and the node's `_tick()` has a last-line guard so a
   bad event can never kill the ROS executor.
5. **Task contract documented**: `suspend()` MUST be a pure idempotent
   snapshot (planner uses it for both push and fidelity verification).
6. **Numpy-safe serialization**: `TaskContext.to_json` converts numpy
   scalars/arrays at the boundary; `WaypointMission` coerces to native floats.
7. **`resume()` validates structurally**: wp_index bounds + reached-list
   length checked at the resume boundary with clear errors.
8. **`from_json` schema-validates** with which-record/why error messages
   (it is the evaluator's resume-fidelity ground truth).
Plus: bounded event drain (`max_events_per_tick=50`) against comms storms.

## Phase 5 additions (autoresearch harness, Gate G5)

All under `orchestrator/` (COLCON_IGNOREd; raw Python threading, per the
orchestrator-vs-target boundary):

- **`llm.py`** — `AnthropicLLM` (claude-opus-4-8, adaptive thinking, structured
  outputs for Level-1 proposals, streaming for Level-2 rounds; lazy SDK import)
  and `ScriptedLLM` (deterministic double — tests/CI/G5 need no API key).
- **Level 1** (`level1/`) — `param_space.py` (bounded tunables; ArduPilot-shaped
  names checked against param_guard: PROTECTED params rejected *before*
  evaluation, TUNABLE ones flagged SITL-only), `suite.py` (fixed-suite runner
  with per-episode active-mechanism assertion + thread audit), `proposers.py`
  (heuristic + LLM; one change + hypothesis + objective), `loop.py` (keep-rule
  enforced by the evaluator, never the proposer; JSONL history feeds Level 2's
  Explore round).
- **Level 1.5** (`level1_5/strategy.py`) — freeze params proposed ≥k times with
  zero keeps (counting since last unfreeze — no instant re-freeze), unfreeze on
  failure-mode change, objective-pointed guidance. Only changes *which* params
  get attention — never proposal logic or keep rules.
- **Level 2** (`level2/`) — mechanism contract + registry (`active.json`,
  baseline_apf known-good), the 4-round Explore→Critique→Specify→Generate
  pipeline (reads real target sources + the failure trace; only WRITES
  candidates), and `validate.py`: compile-in-subprocess → contract load →
  smoke episode with mechanism assertion → thread audit (orphan = revert) →
  full suite + keep rule → promote (file first, pointer second). Revert is
  structural: the known-good mechanism is never deactivated until every stage
  passes. Fixtures `broken_import` / `thread_leaker` / `name_liar` prove each
  stage catches its failure mode.
- **Gate G5** (`run_gate_g5.py`, in CI): 60 episodes unattended
  (Level 1 + 1.5), zero orphaned threads globally and per-episode, every
  episode's mechanism assertion logged, 3 revert drills caught at their exact
  intended stages, valid-candidate mechanics clean (keep-rule correctly rejects
  no-improvement on a green suite; promotion proven in tests vs a degraded
  baseline).
- **`run_autoresearch.py`** — the LLM-driven entry point: `--proposer llm`
  `--level2-every M` for real runs (SITL episodes on the Jetson use the same
  loop; the kinematic backend is the CI-speed stand-in).

## Post-Phase-5 additions (operational stack: RoboBoat merge, LiDAR fusion, RC watchdog)

Capability ported from the Crusader boat repo / RoboBoat_2026 and rewired to this repo's
conventions (single-gateway rule, `[RO]`/`[DYN]` config, `*_core`/`*_node` split, tests in the
same commit). These are additive to the Phase 0–5 plan, not new phases:

- `api/safety/rc_heartbeat_watchdog.py` + `rc_heartbeat_core.py` — force-disarm on
  RC-transmitter link loss. Rewired from the boat repo's own-MAVLink-connection version to
  consume `telemetry_bridge` topics and route the disarm back via `/crsd/force_disarm`
  (latch-independent in the bridge). All latch/link-loss logic is in the ROS-free core,
  unit-tested in `tests/test_rc_heartbeat_core.py`. Launched in `core.launch.py`.
- `api/perception/lidar_fusion_node.py` + `lidar_fusion.py` — Livox MID360 ↔ camera
  detection-range fusion (fills the RX26 plan §3.1 LiDAR API slot). Numpy-only core,
  `tests/test_lidar_fusion.py`. **Prereqs before trusting fused ranges:** calibrated
  camera↔LiDAR extrinsic + MID360 on its own NIC/subnet (CLAUDE.md).
- `api/actuators/actuator_node.py` + `actuator_core.py` — Mission-3 effectors (delivery
  launcher + water cannon) on one Maestro serial link, exposed as ROS services. Wire-protocol
  encoding unit-tested in `tests/test_actuator_core.py`; serial/service glue is bench-tier.
- `api/ivc/ivc_node.py` + `ivc_link.py` — inter-vehicle comms over the team WiFi (Bullet AC),
  ROS-free link core with a background connection thread, `tests/test_ivc_link.py`. Separate
  from the RJ-45 RoboCommand link and the Pixhawk link.
- `Dockerfile` (the `asv` image, with `livox_ros_driver2` baked in) + `launch/` files
  (`core`, `camera`, `lidar`, `lidar_fusion`) + `config/MID360_config.json`.

New entry points added to `setup.py`: `rc_watchdog`, `actuator_node`, `ivc_node`,
`lidar_fusion_node`. All four now carry config sections asserted by
`tests/test_config_shared.py::test_all_nodes_have_config_sections`.

## What was scaffolded but is completed later on-boat

- Mission-4 Core interrupt/resume sub-metrics land with Phase 4 (above); the SITL backend is
  written but only testable in the container (Gate G0's SITL leg runs there).
- Buoy-model retraining (Gate G2) and the RC autonomy-drop switch (Gate G1) remain the two
  prerequisites gating objective-1 trust and RC-override field work, respectively.
