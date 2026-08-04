# RobotX 2026 — Crusader Software Package (v0.4)

Team Inspiration's codebase for the 2026 RobotX competition. We run a single ASV
(autonomous surface vessel), **Crusader**: a holonomic 4×T200 boat on ArduRover/Pixhawk with
a Jetson Orin Nano companion computer running ROS 2 Humble in the `asv` Docker
container. Phases are written in — see [README_PHASE0.md](README_PHASE0.md) for
the phase-by-phase delivery log.

Before developing ANY code, read [Format](#format) and the standing
[safety constraints](#safety-constraints-non-negotiable). For setup, follow
[docs/SETUP_GUIDE.md](docs/SETUP_GUIDE.md). This is the sequential read → run → update
walkthrough for each machine.

## Structure

Every top-level directory has its own README with a sequence diagram, its design rationale,
and a change-impact table. The repo-wide "edit X → re-run Y" map is
[docs/CHANGE_IMPACT_MAP.md](docs/CHANGE_IMPACT_MAP.md).

This repo is **one source dir inside a colcon workspace**, not the workspace itself.
On the Jetson it lives at `~/robotx_ws/src/rx26_asv`, alongside any other package
sources. The repo root is deliberately **not** a colcon package — that is what lets
plain `colcon build` discover both packages it ships (`rx26_asv` and `interfaces`)
instead of stopping at the first one it finds.

```
~/robotx_ws/                   # colcon WORKSPACE (not this repo; holds build/ install/ log/)
|-- missions/                  # mission JSON files containing path waypoints, converted to 
|                              #   JSON from imported .plan files
|-- models/                    # per-Jetson TensorRT engines (gitignored, copied in out-of-band)
|-- src/
     |-- rx26_asv/             # <- THIS REPO
     |    |-- rx26_asv/           # ROS 2 python package (colcon package root)
     |    |    |-- package.xml    #   package manifest + setup.py / setup.cfg
     |    |    |-- config/        #   single source of truth: ROS params YAML + devices + MID360
     |    |    |-- launch/        #   launch files (core status stack, camera, lidar, fusion)
     |    |    |-- resource/      #   registers the package with the ament index
     |    |    |-- rx26_asv/      #   the importable python module
     |    |         |-- api/common/      # shared plumbing: params, node lifecycle, drop latch
     |    |         |-- api/navigation/  # telemetry_bridge (sole MAVProxy consumer), frame
     |    |         |                    #   transform, occupancy grid, APF advisory, fence writer
     |    |         |-- api/perception/  # OAK-D -> TensorRT detect -> depth assoc; LiDAR fusion
     |    |         |-- api/mission/     # task-stack mission planner + RoboCommand comms
     |    |         |-- api/safety/      # rc_heartbeat_watchdog: force-disarm on RC-link loss
     |    |         |-- api/actuators/   # Mission-3 effectors: water cannon (Maestro)
     |    |         |-- api/ivc/         # inter-vehicle comms over modem communication
     |    |         |-- api/testing/     # bench-only nodes
     |    |-- interfaces/         # ROS 2 message package (typed contracts between nodes)
     |    |-- docs/               # setup guide, change-impact map, bench procedures (G1, G2)
     |    |-- firmware/           # Arduino sketches flashed to peripherals (LED status strip)
     |    |-- proto/              # RoboCommand protobuf schema (compiled at build, not committed)
     |    |-- scripts/            # runnable bash scripts
     |    |-- setup/              # installation scripts per machine role + git remote init
     |    |-- tests/              # unit tests for the target system
     |    |-- tools/              # udev, systemd, preflight, param_guard, rebuild, mock
     |    |                       #   RoboCommand, model training pipeline, bench instruments
     |    |-- Dockerfile          # `asv` image (ROS 2 Humble + CUDA + livox driver)
     |-- <other package sources>  # anything else in the workspace; COLCON_IGNORE what you
                                  #   are not building (see docs/SETUP_GUIDE.md §B2)
```

**Path shorthand used throughout the docs:** package-internal paths are written relative to
the package dir — `config/crusader_params.yaml` means `rx26_asv/config/crusader_params.yaml`,
`api/common/config.py` means `rx26_asv/rx26_asv/api/common/config.py`. Repo-level paths
(`tools/`, `tests/`, `docs/`, `interfaces/`, `proto/`) are written from the repo root.

## Usage

### First-time setup

| You are on… | Run | Details |
|---|---|---|
| Dev laptop (Windows) | `powershell -ExecutionPolicy Bypass -File setup\install_dev.ps1` | [setup/README.md](setup/README.md) |
| Dev laptop (Linux/macOS) | `bash setup/install_dev.sh` | ” |
| Jetson host | `sudo bash setup/install_jetson_host.sh` | udev → systemd → checks |
| Inside `asv` container | `bash /root/robotx_ws/src/rx26_asv/setup/install_container.sh` | deps → protoc → colcon → smoke |
| New/standalone clone, no git yet | `bash setup/init_git_remote.sh <remote-url>` | idempotent init + remote |

### Running the system

```bash
# from the Jetson host — THE one blessed rebuild path (builds the whole workspace):
tools/scripts/rebuild.sh
# in-container — launch a node (params ship in the package's share dir):
ros2 run rx26_asv telemetry_bridge --ros-args \
  --params-file "$(ros2 pkg prefix rx26_asv)/share/rx26_asv/config/crusader_params.yaml"
```

## Safety constraints (non-negotiable)

These come from field test lessons; tooling enforces most of them, but you
are expected to know them:

1. **One Pixhawk owner.** MAVProxy holds the serial link; everything else consumes its UDP
   rebroadcast. Code opening a second MAVLink/serial connection is auto-rejected.
2. **RC e-stop is the only safety path.** ELRS SB-switch kills motors in any mode at km
   range. WiFi/SSH/Ctrl+C is a convenience, never a safety mechanism.
3. **RC-override mechanisms are sim/bench-only until Gate G1** (the autonomy-drop switch).
   This blocks `dp_hold`-style field work, and any Level-2 mechanism like it.
4. **Steering inversion is fixed in `SERVOx_*`, never `RCx_REVERSED`** — autonomous modes
   bypass RC settings entirely. `PILOT_STEER_TYPE=3` and `COMPASS_USE=0` stay as they are.
5. **Never loosen `ARMING_*`/safety params to make testing easier.** `param_guard.py`
   hard-fails on the protected set; fix the testing friction instead.
6. **Fail loudly.** A silently-inactive avoidance mechanism is a safety issue, not a wasted
   experiment. Every injection asserts it is actually the mechanism running.

## Format

### Best practices / standards for development

1. All code is tested at the appropriate tier before pushing (unit → bench → field;
   see [tests/README.md](tests/README.md)). If pushing untested code is unavoidable (e.g.
   remote push to enable on-boat testing), mark it with a `NOTE:` comment at the top of the
   file or function.
2. Split ROS nodes into pure-logic `*_core.py` + thin `*_node.py` wrappers so the math is
   unit-testable without ROS.
3. Imports are absolute, never relative.
4. Parameters live in `config/crusader_params.yaml`, never hardcoded — every param is
   `[RO]` or `[DYN]` ([config/README.md](rx26_asv/config/README.md)); the anti-drift test fails the
   build if code and YAML disagree.
5. Every change consults [docs/CHANGE_IMPACT_MAP.md](docs/CHANGE_IMPACT_MAP.md) for its
   blast radius, and lands with its tests in the same commit.
6. Header docstring per file (purpose), docstrings per class/function (args, types,
   returns), `#` comments for the convoluted parts.

### Versioning

Current version: **0.4.0** — "phase" versioning until competition: `0.N` means Phases
0–N delivered (we are not yet at Phase-5: autoresearch/auto-kinematic config harness live). At competition freeze this
becomes `1.0.0`; afterwards, small confirmed changes in one area bump the minor, major
changes in one or more core areas bump the major.

## Dependencies

**Hardware:** Crusader — Jetson Orin Nano; Pixhawk (ArduRover 4.6.3, `FRAME_TYPE=2` OmniX);
4×T200; OAK-D LR (must enumerate USB3 `SUPER` — `oakd_guard` asserts this); RTK GPS +
dual-antenna moving-baseline heading (compass disabled by design); BNO085 IMU; ELRS RC
(e-stop path); 5.8GHz Ubiquiti + 915MHz telemetry; LED status strip (RED e-stop / YELLOW
manual / GREEN autonomous).

**Software:** Ubuntu/JetPack 6 on the Jetson; ROS 2 Humble + CUDA/TensorRT/depthai/MAVProxy
inside the `asv` container (base: `ultralytics/ultralytics:latest-jetson-jetpack6`);
plain Python ≥3.10 + numpy/pyyaml/pytest for the orchestrator anywhere. Install via
[setup/](setup/README.md).

## Glossary

| Term | Meaning here |
|---|---|
| **ASV / USV** | Autonomous/unmanned surface vessel — the boat |
| **GUIDED / MANUAL** | ArduRover modes. GUIDED = autopilot drives to pushed setpoints (cannot strafe on this frame — it turns instead); MANUAL + RC override = the only true lateral-hold path (`dp_hold`) |
| **MAVProxy rebroadcast** | MAVProxy owns the Pixhawk serial link and re-serves telemetry on UDP (14550 GCS, 14551 ROS). The only way anything else talks to the autopilot |
| **telemetry_bridge** | The single ROS consumer of that rebroadcast and single RC-override sender |
| **Autonomy-drop switch** | RC-channel-triggered software latch that kills any RC override within one control cycle, working beyond WiFi range. Gate G1 deliverable; prerequisite for all RC-override field work |
| **APF / ROA** | Artificial Potential Field / Reactive Obstacle Avoidance. Advisory only: emits corrected goal + speed scale; never commands motors (single-writer arbitration, plan §3.2) |
| **Occupancy grid** | World-anchored sparse obstacle map. Perception cells decay (`exp(-Δt/τ)`); comms keep-out cells persist until All Clear and can't be overwritten by perception |
| **Keep-out zone / moving virtual obstacle** | Mission-4 RoboCommand constraints. Static zones → MAVLink exclusion fences (hard backstop) + grid (smooth avoidance); moving objects → 10 m clearance from current AND projected position, grid/APF-only |
| **Task stack / TaskContext** | Mission-planner interrupt model: suspend the current task as JSON-serializable resumable state, run the interrupt task, resume with progress preserved — never restart |
| **RoboCommand** | Competition tasking authority; protobuf over RJ-45. Every state transition needs an ack — comms compliance is scored separately from the maneuver |
| **Keep-rule** | A change is kept only if it regresses neither collision nor local-minima metrics past safety thresholds and improves stuck-ness or completion. Enforced by the evaluator, never the proposer |
| **SITL** | ArduPilot software-in-the-loop — the real firmware in sim; primary simulator (plan §4.6) |
| **Gates G0–G6** | Phase exit criteria: G0 scripted episode → G1 autonomy-drop → G2 perception trust → G3 avoidance (20 seeds clean) → G4 interrupt/resume → G5 unattended autoresearch + revert drills → G6 on-water suite across 2 days |
| **Preflight** | `tools/scripts/preflight.py` — the do-not-arm gate run before every session |
| **Rebuild discipline** | Container copies files at build; every edit needs `tools/scripts/rebuild.sh`. "The change did nothing" = you skipped it |

## Design choices per subdirectory (summary — details in each README)

| Dir | Core design choice | Why |
|---|---|---|
| [rx26_asv/](rx26_asv/rx26_asv/README.md) | ArduRover owns actuation/estimation; nodes advise, one bridge talks MAVLink | Free EK3/failsafes/e-stop; no PWM or parallel EKF to maintain (plan §4.2) |
| [interfaces/](interfaces/README.md) | ROS 2 topics with RX24-proven message shapes | Typed drift-catching at build time vs legacy sockets (§4.3) |
| [config/](rx26_asv/config/README.md) | One YAML, `[RO]`/`[DYN]` postures, anchors tie node↔evaluator values | Config drift between scorer and boat is a silent-failure factory |
| [proto/](proto/README.md) | Protobuf at the edge, ROS inside; byte-identical mock | Competition mandates the wire format; nothing else should know it (§4.5) |
| [tools/](tools/README.md) | One blessed path per operation, shared by humans and autoresearch | No drift between "how people do it" and "how Level 2 does it" |
| [tests/](tests/README.md) | Hardware-free unit tier of a 4-tier pyramid | A change earns its next tier; never promoted on one green level |
| [setup/](setup/README.md) | One idempotent, self-verifying script per machine role | Fresh-machine bootstrap must not depend on tribal knowledge |

## Future development

- **Gate G1**: build + sign off the RC autonomy-drop switch — the long pole blocking all
  docking field work.
- **Gate G2**: retrain the buoy model on Crusader's own footage — objective-1 metrics are
  officially untrusted until then.
- Wrap the legacy `gate_navigator`/`dp_hold` under the mission planner's task contract and
  **harden `gate_navigator`** (still UNTESTED — higher priority than new mechanisms).
- Mission-planner moving-hazard topic (explicit TODO in `mission_planner_node`; APF core
  already supports projection).
- Implement and validate Phase 5 autoresearch layer for the Nav2 MPPI plugin critic tuning.
  Previously formed codebase for the autoresearch orchestrator is now at the sim/orchestrator branch.
- Phase 6 field campaign per plan §5, then Phase 7 freeze (param freeze via `param_guard`,
  runbooks, spares).
