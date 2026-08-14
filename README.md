# RobotX 2026 — Crusader Software Package (v0.5)

Team Inspiration's codebase for the 2026 RobotX competition. We run a single ASV
(autonomous surface vessel), **Crusader**: a holonomic 4×T200 boat on ArduRover/Pixhawk
with a Jetson Orin Nano companion computer running ROS 2 Humble in the `asv` Docker
container.

**What runs today:** the status, telemetry and safety layer — four nodes that have all
been on the water — plus `crusader_perception`, which owns the OAK-D and publishes
detections rather than frames. **What is scaffolded and empty:** the world model (fusion,
occupancy grid), being rebuilt after v0.5 removed the unverified version. **What is
elsewhere:** the MID360 only. A second container drives that LiDAR and publishes its point
cloud; consuming it is the unwritten half of perception.

Before developing ANY code, read [Format](#format) and the standing
[safety constraints](#safety-constraints-non-negotiable). Before your **first day**, follow
[docs/SETUP_GUIDE.md](docs/SETUP_GUIDE.md).

## Packages

Seven packages, laid out along the architecture: perception → world model → cognition →
behavior, with a shared library underneath and a bringup package on top. All seven build
and run in the `asv` container (`--packages-up-to crusader_bringup`).

| Package | Contains | State |
|---|---|---|
| [`crusader_msgs`](crusader_msgs/README.md) | Message definitions. Depends on nothing but `std_msgs`, so any image can build it cheaply | 6 msgs |
| [`crusader_common`](crusader_common/README.md) | Shared plumbing: params loader, node lifecycle, stream cache, drop latch, geodesy. No nodes | library |
| [`crusader_fcu`](crusader_fcu/README.md) | `telemetry_bridge` — the only thing that speaks MAVLink. Localization source *and* Movement actuator | **field** |
| [`crusader_perception`](crusader_perception/README.md) | Owns the OAK-D and detects in it: `oakd_publisher` → raw frames; `buoy_detector` → `oak/detections` in `camera_link`. Not launched — the two contend for the camera | 2 nodes |
| [`crusader_world_model`](crusader_world_model/README.md) | Fusion → 3D object positions; occupancy grid. Sensor-agnostic, verifiable without a camera | **empty** |
| [`crusader_behavior`](crusader_behavior/README.md) | `safety/` RC-loss force-disarm watchdog; `indicator/` LED status stack | **field** |
| [`crusader_bringup`](crusader_bringup/README.md) | Launch files + the params YAML. Ships no code; build entry point | — |

Cognition (missions, mission planner, RoboCommand interface) has no package yet — its
shape is still being decided.

Every package has its own README with its design rationale and a change-impact table.

## Structure

This repo is **a set of source dirs inside a colcon workspace**, not the workspace itself.
On the Jetson it lives at `~/robotx_ws/src/rx26_asv`, alongside any other package sources.
The repo root is deliberately **not** a colcon package — that is what lets `colcon build`
discover all seven packages instead of stopping at the first one it finds.

```
~/robotx_ws/                    # colcon WORKSPACE (not this repo; holds build/ install/ log/)
|-- src/
     |-- rx26_asv/              # <- THIS REPO
     |    |-- crusader_msgs/        # ament_cmake: msg/ + CMakeLists
     |    |-- crusader_common/      # ament_python: the shared library
     |    |-- crusader_fcu/         # ament_python: telemetry_bridge
     |    |-- crusader_perception/  # ament_python: OAK-D driver + buoy detection
     |    |-- crusader_world_model/ # ament_python: EMPTY, scaffolded
     |    |-- crusader_behavior/    # ament_python: safety/ + indicator/
     |    |-- crusader_bringup/     # ament_cmake: launch/ + config/
     |    |-- docs/                 # setup guide, operations manual, G1 bench procedure
     |    |-- firmware/             # Arduino sketch flashed to the LED controller
     |    |-- params/               # known-good ArduRover baseline (param_guard diffs it)
     |    |-- scripts/              # start_mavproxy.sh — the sole Pixhawk owner
     |    |-- setup/                # installation scripts per machine role
     |    |-- tools/                # udev, systemd, preflight, param_guard, rebuild, oak_view
     |    |-- Dockerfile            # `asv` image (ROS 2 + CUDA/TensorRT; no device SDKs)
     |-- <livox container sources, robotx_2026, ...>    # COLCON_IGNORE what you don't build
```

## The stack

| Node | Package | Role |
|---|---|---|
| `telemetry_bridge` | `crusader_fcu` | THE single consumer of MAVProxy's rebroadcast, and THE single MAVLink sender |
| `rc_watchdog` | `crusader_behavior` | force-disarm on RC-transmitter link loss |
| `pixhawk_led_status_node` | `crusader_behavior` | autopilot + RC state → `/crsd/led_state` |
| `led_node` | `crusader_behavior` | `/crsd/led_state` → LED Arduino over serial |

```bash
ros2 launch crusader_bringup core.launch.py
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

### Building

```bash
# from the Jetson host — THE one blessed rebuild path:
tools/scripts/rebuild.sh
```

It runs `colcon build --packages-up-to crusader_bringup`, which builds every package we
ship because bringup depends on all of them. **A new package must be added to
`crusader_bringup/package.xml`'s exec_depends** or it silently stops being built.

## Safety constraints (non-negotiable)

These come from field test lessons; tooling enforces most of them, but you
are expected to know them:

1. **One Pixhawk owner.** MAVProxy holds the serial link; everything else consumes its UDP
   rebroadcast. `crusader_fcu` rejects a non-udp/tcp endpoint at startup, and no other
   package may open a MAVLink connection at all.
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
   ships no test suite. What CI can check statically it does: `tools/scripts/check_config.py`
   guards the two config files that can ground the boat, and the build job proves every
   package imports and that the params file resolves from a real install space. If pushing
   unverified code is unavoidable (e.g. remote push to enable on-boat testing), mark it
   with a `NOTE:` comment at the top of the file or function.
3. Split ROS nodes into pure-logic `*_core.py` + thin `*_node.py` wrappers, so the logic
   can be read without rclpy between you and it. Safety logic in particular belongs in the
   core.
4. Imports are absolute, never relative.
5. **Respect the package boundaries.** `crusader_common` may not import a domain package.
   `indicator/` may not be imported by `safety/`. Testable code (`crusader_world_model`)
   may not depend on untestable code (`crusader_perception`). CI checks that every
   cross-package import is declared in `package.xml`.
6. Parameters live in `crusader_bringup/config/crusader_params.yaml`, never hardcoded —
   every param is `[RO]` or `[DYN]` ([config/README.md](crusader_bringup/config/README.md)).
7. Header docstring per file (purpose), docstrings per class/function (args, types,
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
(RED e-stop / YELLOW manual / GREEN autonomous). The OAK-D LR is driven from `asv` by
`crusader_perception`; the Livox MID360 is on the same Jetson but driven by the **livox
container**.

**Software:** Ubuntu/JetPack 6 on the Jetson; ROS 2 Humble + CUDA/TensorRT + depthai +
MAVProxy/pymavlink/pyserial inside the `asv` container, base
`ultralytics/ultralytics:latest-jetson-jetpack6`; no Livox SDK, by design. Plain
Python ≥3.10 + pyyaml anywhere. Install via [setup/](setup/README.md).

## Glossary

| Term | Meaning here |
|---|---|
| **ASV / USV** | Autonomous/unmanned surface vessel — the boat |
| **GUIDED / MANUAL** | ArduRover modes. GUIDED = autopilot drives to pushed setpoints (cannot strafe on this frame — it turns instead) |
| **MAVProxy rebroadcast** | MAVProxy owns the Pixhawk serial link and re-serves telemetry on UDP (14550 GCS + broadcast, 14551 ROS). The only way anything else talks to the autopilot |
| **`asv` container** | Where this whole repo runs, OAK-D included. Has ROS 2, CUDA/TensorRT, depthai, MAVProxy |
| **livox container** | The only other container. Drives the MID360 and nothing else; publishes its `PointCloud2` |
| **Autonomy-drop switch** | RC-channel-triggered software latch that kills any RC override within one control cycle, working beyond WiFi range. Gate G1 deliverable |
| **Preflight** | `tools/scripts/preflight.py` — the do-not-arm gate run before every session |
| **Rebuild discipline** | Every edit needs `tools/scripts/rebuild.sh`. "The change did nothing" = you skipped it |

## Removed in v0.5

The repo carried ~6,000 lines that had never run on the boat, which made it impossible to
tell the proven parts from the aspirational ones. Removed: the perception pipeline, LiDAR
fusion, the occupancy grid and APF avoidance advisory, the mission planner and RoboCommand
protobuf link, inter-vehicle comms, the Mission-3 actuators, `gate_navigator`, `dp_hold`,
and the keep-out fence uploader. All of it is in git history at `8c4ffa5`.

Perception and world-model work is being rebuilt in their own packages. The field-proven
`gate_navigator` and `dp_hold` algorithms live in
[robotx_2026](https://github.com/InspirationRobotics/robotx_2026), which completed
prequal — that is the version to port from when mission work restarts.

## Future development

- **Topic contract with the livox container** — name, type, QoS, frame id for the MID360
  cloud. Blocks the LiDAR half of `crusader_perception`.
- **Mission-element mapping** — `/crsd/pose` + `/crsd/attitude` + `oak/detections` through
  `geo.body_to_world_ypr` into a world-frame object list. The inputs all exist now.
- **Gate G1**: build + sign off the RC autonomy-drop switch. The enforcement point exists;
  the bench harness needs rewriting ([docs/G1_bench_procedure.md](docs/G1_bench_procedure.md)).
- Rebuild fusion and the occupancy grid in `crusader_world_model`, verified off-boat before
  they ever see a camera.
- Decide the shape of Cognition, then give it a package.
