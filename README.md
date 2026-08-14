# RobotX 2026 — Crusader Software Package (v0.5)

Team Inspiration's codebase for the 2026 RobotX competition. We run a single ASV
(autonomous surface vessel), **Crusader**: a holonomic 4×T200 boat on ArduRover/Pixhawk
with a Jetson Orin Nano companion computer running ROS 2 Humble in the `asv` Docker
container.

**What this repo is:** the boat's status, telemetry and safety layer — the part that has
actually run on the water. Four nodes, one MAVLink gateway, one LED strip, one force-disarm
watchdog.

**What this repo is not:** perception or autonomy. Camera and LiDAR run in a **separate
container** and are not this package's concern. Mission logic (gate transit, station
keeping, RoboCommand, occupancy/avoidance) was removed in v0.5 — see
[Removed in v0.5](#removed-in-v05) for what went and why.

Before developing ANY code, read [Format](#format) and the standing
[safety constraints](#safety-constraints-non-negotiable). Before your **first day**, follow
[docs/SETUP_GUIDE.md](docs/SETUP_GUIDE.md).

## Structure

This repo is **one source dir inside a colcon workspace**, not the workspace itself.
On the Jetson it lives at `~/robotx_ws/src/rx26_asv`, alongside any other package
sources. The repo root is deliberately **not** a colcon package — that is what lets
plain `colcon build` discover both packages it ships (`rx26_asv` and `interfaces`)
instead of stopping at the first one it finds.

```
~/robotx_ws/                   # colcon WORKSPACE (not this repo; holds build/ install/ log/)
|-- src/
     |-- rx26_asv/             # <- THIS REPO
     |    |-- rx26_asv/           # ROS 2 python package (colcon package root)
     |    |    |-- package.xml    #   package manifest + setup.py / setup.cfg
     |    |    |-- config/        #   single source of truth: ROS params YAML
     |    |    |-- launch/        #   core.launch.py — the whole stack
     |    |    |-- resource/      #   registers the package with the ament index
     |    |    |-- rx26_asv/      #   the importable python module
     |    |         |-- api/common/      # shared plumbing: params, node lifecycle, drop latch
     |    |         |-- api/navigation/  # telemetry_bridge — the sole MAVProxy consumer
     |    |         |-- api/safety/      # rc_heartbeat_watchdog: force-disarm on RC-link loss
     |    |         |-- api/led/         # led_node: LED state -> Arduino serial
     |    |         |-- api/pixhawk/     # pixhawk_led_status_node: boat state -> LED state
     |    |-- interfaces/         # ROS 2 message package (typed contracts between nodes)
     |    |-- docs/               # setup guide, operations manual, G1 bench procedure
     |    |-- firmware/           # Arduino sketch flashed to the LED controller
     |    |-- params/             # known-good ArduRover param baseline (param_guard diffs it)
     |    |-- scripts/            # start_mavproxy.sh — the sole Pixhawk owner
     |    |-- setup/              # installation scripts per machine role + git remote init
     |    |-- tools/              # udev, systemd, preflight, param_guard, rebuild
     |    |-- Dockerfile          # `asv` image (ROS 2 Humble + MAVProxy; no CUDA, no depthai)
     |-- <other package sources>  # anything else in the workspace; COLCON_IGNORE what you
                                  #   are not building (see docs/SETUP_GUIDE.md §B2)
```

**Path shorthand used throughout the docs:** package-internal paths are written relative to
the package dir — `config/crusader_params.yaml` means `rx26_asv/config/crusader_params.yaml`,
`api/common/config.py` means `rx26_asv/rx26_asv/api/common/config.py`. Repo-level paths
(`tools/`, `docs/`, `interfaces/`, `params/`) are written from the repo root.

## The stack

| Node | Role |
|---|---|
| `telemetry_bridge` | THE single consumer of MAVProxy's rebroadcast, and THE single MAVLink sender |
| `rc_watchdog` | force-disarm on RC-transmitter link loss |
| `pixhawk_led_status_node` | autopilot + RC state → `/crsd/led_state` |
| `led_node` | `/crsd/led_state` → LED Arduino over serial |

All four start together:

```bash
ros2 launch rx26_asv core.launch.py
```

MAVProxy owns the Pixhawk serial link and is started by systemd outside ROS — nothing
else may open `/dev/crsd-pixhawk`.

## Usage

### First-time setup

| You are on… | Run | Details |
|---|---|---|
| Dev laptop (Windows) | `powershell -ExecutionPolicy Bypass -File setup\install_dev.ps1` | [setup/README.md](setup/README.md) |
| Dev laptop (Linux/macOS) | `bash setup/install_dev.sh` | ” |
| Jetson host | `sudo bash setup/install_jetson_host.sh` | udev → systemd → checks |
| Inside `asv` container | `bash /root/robotx_ws/src/rx26_asv/setup/install_container.sh` | deps → colcon → smoke |
| New/standalone clone, no git yet | `bash setup/init_git_remote.sh <remote-url>` | idempotent init + remote |

### Running the system

```bash
# from the Jetson host — THE one blessed rebuild path (builds the whole workspace):
tools/scripts/rebuild.sh
```

## Safety constraints (non-negotiable)

These come from field test lessons; tooling enforces most of them, but you
are expected to know them:

1. **One Pixhawk owner.** MAVProxy holds the serial link; everything else consumes its UDP
   rebroadcast. `telemetry_bridge` rejects a non-udp/tcp endpoint at startup, and no other
   node may open a MAVLink connection at all.
2. **RC e-stop is the only safety path.** ELRS SB-switch kills motors in any mode at km
   range. WiFi/SSH/Ctrl+C is a convenience, never a safety mechanism.
3. **RC-override mechanisms are bench-only until Gate G1** (the autonomy-drop switch).
   The enforcement point exists in `telemetry_bridge`; nothing has passed the gate.
4. **Steering inversion is fixed in `SERVOx_*`, never `RCx_REVERSED`** — autonomous modes
   bypass RC settings entirely. `COMPASS_USE=0` stays as it is.
5. **Never loosen `ARMING_*`/safety params to make testing easier.** `param_guard.py`
   hard-fails on the protected set; fix the testing friction instead.
6. **Fail loudly.** A silently-inactive mechanism is a safety issue, not a wasted
   experiment. Silence must stay silent: a dead MAVLink stream stops being republished
   rather than being replayed with a fresh timestamp.

## Format

### Best practices / standards for development

1. **Nothing ships until it has run on the boat.** A node that compiles and has never
   moved the vehicle does not belong in `setup.py`'s entry points or `core.launch.py`.
   v0.5 exists because that rule was not applied for four phases.
2. Verification is on the bench and on the water, not in a test framework — this repo
   ships no test suite. What CI can check statically it does:
   `tools/scripts/check_config.py` guards the two config files that can ground the boat,
   and the build job proves every entry point actually imports. If pushing unverified code
   is unavoidable (e.g. remote push to enable on-boat testing), mark it with a `NOTE:`
   comment at the top of the file or function.
3. Split ROS nodes into pure-logic `*_core.py` + thin `*_node.py` wrappers, so the logic
   can be read and reasoned about without rclpy between you and it. Safety logic in
   particular belongs in the core.
4. Imports are absolute, never relative.
5. Parameters live in `config/crusader_params.yaml`, never hardcoded — every param is
   `[RO]` or `[DYN]` ([config/README.md](rx26_asv/config/README.md)); nodes declare their
   defaults *from* that file, and `tools/scripts/check_config.py` fails CI if the file and
   the shipped node list disagree.
6. Header docstring per file (purpose), docstrings per class/function (args, types,
   returns), `#` comments for the convoluted parts.

### Versioning

Current version: **0.5.0**. Phase versioning (`0.N` = phases 0–N delivered) is retired —
it counted code written, not capability proven, which is exactly the failure v0.5 corrects.
Plain semver from here: minor for a confirmed change in one area, major for core changes.
At competition freeze this becomes `1.0.0`.

## Dependencies

**Hardware:** Crusader — Jetson Orin Nano; Pixhawk (ArduRover 4.6.3, `FRAME_TYPE=2` OmniX);
4×T200; RTK GPS + dual-antenna moving-baseline heading (compass disabled by design);
ELRS RC (e-stop path); 5.8GHz Ubiquiti + 915MHz telemetry; LED status strip
(RED e-stop / YELLOW manual / GREEN autonomous). The OAK-D LR and Livox MID360 are
driven by a **separate container**.

**Software:** Ubuntu/JetPack 6 on the Jetson; ROS 2 Humble + MAVProxy/pymavlink/pyserial
inside the `asv` container (base: `arm64v8/ros:humble-ros-base`); plain Python ≥3.10 +
pyyaml anywhere. Install via [setup/](setup/README.md).

## Glossary

| Term | Meaning here |
|---|---|
| **ASV / USV** | Autonomous/unmanned surface vessel — the boat |
| **GUIDED / MANUAL** | ArduRover modes. GUIDED = autopilot drives to pushed setpoints (cannot strafe on this frame — it turns instead) |
| **MAVProxy rebroadcast** | MAVProxy owns the Pixhawk serial link and re-serves telemetry on UDP (14550 GCS + broadcast, 14551 ROS). The only way anything else talks to the autopilot |
| **telemetry_bridge** | The single ROS consumer of that rebroadcast and single MAVLink sender |
| **Autonomy-drop switch** | RC-channel-triggered software latch that kills any RC override within one control cycle, working beyond WiFi range. Gate G1 deliverable; prerequisite for all RC-override field work |
| **Preflight** | `tools/scripts/preflight.py` — the do-not-arm gate run before every session |
| **Rebuild discipline** | Every edit needs `tools/scripts/rebuild.sh`. "The change did nothing" = you skipped it |

## Removed in v0.5

The repo carried ~6,000 lines of code that had never run on the boat, which made it
impossible to tell the proven parts from the aspirational ones. Removed: the perception
stack (OAK-D + TensorRT + LiDAR fusion), the occupancy grid and APF avoidance advisory,
the mission planner and RoboCommand protobuf link, inter-vehicle comms, the Mission-3
actuators, `gate_navigator`, `dp_hold`, the keep-out fence uploader, and the model-training
pipeline. All of it is in git history at `8c4ffa5` and earlier.

The field-proven `gate_navigator` and `dp_hold` algorithms also live in
[robotx_2026](https://github.com/InspirationRobotics/robotx_2026), which completed
prequal — that is the version to port from when mission work restarts, not the
decoupled rewrite this repo deleted.

## Future development

- **Gate G1**: build + sign off the RC autonomy-drop switch — the long pole blocking all
  RC-override field work. The enforcement point is in `telemetry_bridge`; the bench
  harness that drove it was removed with the rest of the untested code and needs rewriting
  ([docs/G1_bench_procedure.md](docs/G1_bench_procedure.md)).
- Define the **topic contract with the perception container** (detections message shape,
  namespace, QoS) before any mission node returns here.
- Re-port gate transit from `robotx_2026` against that contract, one node at a time,
  each earning its place by moving the boat.
