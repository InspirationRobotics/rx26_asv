# CRUSADER USV — OPERATIONS MANUAL (RobotX 2026)

**Updated 2026-07-28.** Field/day-of manual for the `rx26_asv` stack.

For first-time installation on a machine, use [SETUP_GUIDE.md](SETUP_GUIDE.md).
For "if I change X, what do I re-run?", use [CHANGE_IMPACT_MAP.md](CHANGE_IMPACT_MAP.md).
This document assumes the boat is already installed and you are operating it.

> **This supersedes the `robotx_2026` documentation.** Package, container, repo
> and startup procedure all changed. The single biggest difference: **MAVProxy
> and the container start automatically via systemd** — they are no longer
> Terminal 1 / Terminal 2 commands you type.

---

## 1. The big picture

- The boat's computer is a **Jetson Orin Nano**, reachable over the boat's WiFi.
- All code lives on the Jetson at **`~/robotx_ws/src/rx26_asv`**, version-controlled with git.
  `~/robotx_ws` is a **colcon workspace**, not the repo — the repo is one source
  directory inside `src/`, alongside any others.
- Code **runs** inside a Docker container named **`asv`** (ROS 2 Humble); code is
  **edited** outside the container. Both see the same files via a bind mount.
- The **Pixhawk is owned by exactly one program, MAVProxy**, which rebroadcasts
  over UDP to everything else (our nodes, Mission Planner/QGC on your laptop).
  Nothing else may open the serial device. Ever.
- The **LED strip** shows boat status: **RED** = e-stopped, **YELLOW** = armed/manual,
  **GREEN** = autonomous.

### Boot chain (automatic, since 2026-07-28)

```
power on
  └─ crsd-mavproxy.service   (host) ── takes /dev/crsd-pixhawk, rebroadcasts UDP
       └─ crsd-container.service (host) ── docker start -a asv
```

Both are `systemctl enable`d, so a power cycle brings them up in order. The
container unit is ordered `After=crsd-mavproxy.service` so no node ever races
MAVProxy for the serial device.

Check the chain:

```bash
systemctl status crsd-mavproxy crsd-container --no-pager
```

Healthy looks like `active (running)` on both, and `NRestarts=0`:

```bash
systemctl show crsd-mavproxy -p NRestarts -p ActiveState
```

> `NRestarts` is cumulative and does **not** reset on `systemctl restart`. If you
> are checking after a known crash loop, run `sudo systemctl reset-failed
> crsd-mavproxy` first or the number is meaningless. A few seconds of
> `active` is not proof — with `RestartSec=3` you can easily sample between
> restarts. Wait 30 s.

---

## 2. Connect to the Jetson

1. Connect your laptop to the boat's WiFi (`TeamInspirationField_2.0` or `MESAFSD`).
   **Credentials are not stored in this repo** — they're in the team drive. Do not
   commit them here; a private repo is not a secret store, and the history is forever.
2. SSH in. Use `-A` so git operations on the Jetson use *your* key (see §10):

```bash
ssh -A crusader@crusader-asv.local
```

If mDNS doesn't resolve, use the IP directly:

```bash
ssh -A crusader@<JETSON_IP>
```

You're on the Jetson when the prompt reads `crusader@crusader-asv`.

### If WiFi won't work — USB fallback

1. Plug USB-C from the Jetson to your laptop.
2. `ssh crusader@192.168.55.1`
3. Find the WiFi address: `ip addr` (look for the `wl*` interface, e.g. `wlP1p1s0`).
4. Unplug USB, then SSH to that address over WiFi.

### Known field fix — QGC won't connect at the lake

If MAVProxy is running but QGroundControl/Mission Planner never sees the boat,
the wired interface may have no address in the laptop's subnet:

```bash
sudo ip addr add 192.168.100.109/24 dev enP8p1s0
```

Confirm the interface name with `ip addr` first — it is not the same on every
Jetson. This is not persistent across reboot.

---

## 3. Find your laptop's IP (for telemetry)

MAVProxy forwards telemetry to your laptop. On Windows:

```
ipconfig
```

Use the `Wireless LAN adapter Wi-Fi` → `IPv4 Address`. That's `<LAPTOP_IP>` below.
It changes per laptop and often per day.

Set it once so the service picks it up on every start, instead of editing the unit:

```bash
echo 'LAPTOP_IP=<LAPTOP_IP>' | sudo tee /etc/default/crusader && sudo systemctl restart crsd-mavproxy
```

The unit reads that file via `EnvironmentFile=-/etc/default/crusader`. If it's
absent, `start_mavproxy.sh` falls back to its built-in default.

---

## 4. Docker: in and out

Is it running? Look for `asv`:

```bash
docker ps
```

systemd should have started it. If not:

```bash
sudo systemctl start crsd-container
```

Enter it (prompt becomes `root@crusader-asv`):

```bash
docker exec -it asv bash
```

Leave it — the container keeps running, which is correct and safe:

```bash
exit
```

**Rule of thumb:** edit code and use git **outside** the container; build and run
**inside** it. Two or three SSH windows is normal, each entering separately.

---

## 5. Running the stack

MAVProxy and the container are already up (§1). What you start by hand are nodes.

### The core status stack

Inside the container:

```bash
cd /root/robotx_ws && source install/setup.bash && ros2 launch rx26_asv core.launch.py
```

That starts four nodes:

| Node | Role |
|---|---|
| `telemetry_bridge` | THE single consumer of MAVProxy's rebroadcast, and THE single RC-override sender |
| `led_node` | `/crsd/led_state` → LED serial |
| `pixhawk_led_status_node` | Pixhawk state → LED state |
| `rc_watchdog` | force-disarm on RC-link loss |

**Verify:** flip the RC arm switch — the LED strip and the launch logs should
change together.

### Perception (camera)

```bash
ros2 launch rx26_asv camera.launch.py
```

### LiDAR and fusion

```bash
ros2 launch rx26_asv lidar.launch.py
```

```bash
ros2 launch rx26_asv lidar_fusion.launch.py
```

`lidar_fusion.launch.py` runs the driver **and** the fusion node together.
See §14 for prerequisites — these will not produce useful ranges until the
MID360 is on its own NIC and the extrinsic is calibrated.

### Laptop side

Mission Planner: connection dropdown (top right) → **UDP**, port **14550**, Connect.
QGroundControl connects by itself.

---

## 6. RC controller reference

| Switch | Channel | Positions |
|---|---|---|
| **SA** | 5 | pressed in = Pixhawk control, released = Teensy control. **Keep pressed in** — the Teensy is not in use. |
| **SB** | 7 | down = **e-stop** (RED, ~994) · middle = released (~1498) · up = **arm** (YELLOW + arming tune, ~1995). `RCx_OPTION=165`. |
| **SC** | 8 | down = Manual · middle = Hold (boat actively stops) · up = **Guided** (GREEN — needs a mission loaded and GPS lock, else the mode change is rejected). |
| **SE** | 9 | **PROPOSED, NOT BUILT** — autonomy-drop for RC-override nodes. See §18. |

**SB down is the real e-stop.** It kills motors instantly in any mode,
independent of any node or software, at full RC range.

---

## 7. Useful individual commands

Inside the container, after `source install/setup.bash` in `/root/robotx_ws`.

Run one node instead of the launch file:

```bash
ros2 run rx26_asv led_node
```

List every executable the package ships (16 of them):

```bash
ros2 pkg executables rx26_asv
```

Set the LED colour manually (0=off 1=red 2=yellow 3=green) — works with only
`led_node` running:

```bash
ros2 topic pub /crsd/led_state std_msgs/msg/Int32 "{data: 2}" --once
```

See what's publishing:

```bash
ros2 topic list
```

Camera check without ROS — then open `http://<JETSON_IP>:8080`, Ctrl+C to stop:

```bash
python3 /root/robotx_ws/src/rx26_asv/tools/oak_view.py
```

> One camera user at a time. Stop `perception_node` before running `oak_view.py`.
> Unlike the old `oak_buoy_view.py`, detections now come from the ROS pipeline
> (`perception_node` → `/crsd/detections_body`), not from this tool.

Confirm MAVLink actually reaches container-side code — this is the single best
end-to-end health check:

```bash
docker exec asv bash -lc 'timeout 15 python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection(\"udpin:127.0.0.1:14551\")
print(\"heartbeat: system\", m.target_system) if m.wait_heartbeat(timeout=10) else print(\"NO HEARTBEAT\")
"'
```

> This **binds** port 14551, which is `telemetry_bridge`'s port. Only run it while
> the bridge is stopped, or they'll fight over it.

---

## 8. After editing code

The container **copies files at build time — it does not symlink**. Every edit
needs a rebuild before it takes effect. From the **host**:

```bash
bash tools/scripts/rebuild.sh
```

That is the one blessed path: it builds both packages inside the container and
runs an import smoke test. Equivalent by hand, inside the container:

```bash
cd /root/robotx_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select interfaces rx26_asv && source install/setup.bash
```

> **`--packages-select` is deliberate.** The workspace may hold other package
> sources (e.g. `robotx_2026`) whose build state is not ours to change and whose
> build failure must not block ours. A bare `colcon build` drags them in.

**If a change "did nothing", you almost certainly skipped the rebuild.**

---

## 9. Troubleshooting quick hits

- **Only ONE program may open a serial device.** If something can't reach the
  Pixhawk, check there isn't a second MAVProxy — or Mission Planner on USB —
  fighting for it. `sudo systemctl status crsd-mavproxy` first.
- **Stable device names:** `ls -l /dev/crsd-*` and `ls -l /dev/serial/by-id/`
- **Watch USB events live** (unplug/replug to identify something): `sudo dmesg -w`
- **OAK-D showing as "Luxonis Bootloader" in `lsusb` is NORMAL** when idle.
- **LED frozen?** `led_node` auto-reconnects within ~5 s of a serial drop. Check
  the logs; repeated reconnects suggest electrical interference (`dmesg` for EMI).
- **Pixhawk dead in Mission Planner?** Restart MAVProxy first — everything
  depends on it: `sudo systemctl restart crsd-mavproxy`
- **MAVProxy restarting forever?** Check `journalctl -u crsd-mavproxy -n 40
  --no-pager --output=cat`. A log ending in `MAV> Unloading module …` is the
  interactive console reading EOF — it means `--daemon` is missing from
  `start_mavproxy.sh`. That is a regression; the flag is required under systemd.
- **`Permission denied` running a script?** Check the executable bit survived:
  `git ls-files -s '*.sh'` should show `100755`, not `100644`. CI enforces this.
- **Drives fine in MANUAL but wrong in auto?** See §12 — do not "fix" it in RC settings.

---

## 10. GitHub

Repo: **`git@github.com:InspirationRobotics/rx26_asv.git`** (private).
The live copy is `~/robotx_ws/src/rx26_asv` on the Jetson.

**No credential is stored on the boat.** SSH in with `-A` and the Jetson uses
your forwarded key, so pushes are attributed to you and nothing has to be
revoked if the Jetson is lost.

Pull the latest (on the **host**, not in the container):

```bash
cd ~/robotx_ws/src/rx26_asv && git pull
```

If new nodes/files landed, rebuild (§8). If udev rules or systemd units changed,
re-run the host installer:

```bash
sudo bash setup/install_jetson_host.sh
```

After a successful day, from the host:

```bash
cd ~/robotx_ws/src/rx26_asv && git add -A && git commit -m "short description" && git push
```

---

## 11. Workspace tree

```
~/robotx_ws/                        # colcon WORKSPACE — holds build/ install/ log/
├── models/                         # per-Jetson TensorRT engines (gitignored, copied in)
└── src/
    ├── rx26_asv/                   # <- THIS REPO
    │   ├── rx26_asv/               # the ROS 2 package (colcon package root)
    │   │   ├── package.xml  setup.py  setup.cfg
    │   │   ├── config/             # crusader_params.yaml, devices, MID360_config.json
    │   │   ├── launch/             # core, camera, lidar, lidar_fusion
    │   │   ├── resource/
    │   │   └── rx26_asv/api/       # the importable python module
    │   │       ├── common/         # params, node lifecycle, autonomy-drop latch
    │   │       ├── navigation/     # telemetry_bridge, frame_transform, occupancy,
    │   │       │                   #   APF advisory, progress monitor, fence writer,
    │   │       │                   #   gate_navigator, dp_hold
    │   │       ├── perception/     # OAK-D -> TensorRT -> depth assoc; LiDAR fusion
    │   │       ├── mission/        # task-stack mission planner + RoboCommand comms
    │   │       ├── safety/         # rc_heartbeat_watchdog
    │   │       ├── actuators/      # Mission-3 effectors
    │   │       ├── ivc/            # inter-vehicle comms
    │   │       └── testing/        # bench-only nodes
    │   ├── interfaces/             # ROS 2 message package
    │   ├── docs/  proto/  firmware/  tests/
    │   ├── scripts/start_mavproxy.sh
    │   ├── setup/                  # install_jetson_host.sh, install_container.sh
    │   ├── tools/                  # udev, systemd, preflight, rebuild, oak_view, training
    │   └── Dockerfile              # builds the `asv` image
    └── robotx_2026/                # legacy boat repo — NOT built by us (see §19)
```

The repo root is deliberately **not** a colcon package. If it were, colcon would
stop descending and never find `interfaces/`. Do not add a `package.xml` at the
repo root.

---

## 12. Thruster configuration — the steering-inversion lesson (2026-07-10)

Rotation was inverted in manual, and it was "fixed" by reversing the RC axis
(`RC1_REVERSED=1`). Manual felt perfect — but **autonomous modes send steering
straight to the motors and never pass through RC settings.** The motors stayed
inverted for the autopilot: the boat spun in circles, parked facing away from
waypoints, and never throttled forward.

**The rule: every axis must be correct in manual with all `RCx_REVERSED=0`. If an
axis is backwards, fix it in the motor config (`SERVOx_FUNCTION` /
`SERVOx_REVERSED`), never in RC settings.**

Current correct config — do not change without reading the above:

- `FRAME_TYPE=2` (OmniX)
- `SERVO1_FUNCTION=36`, `SERVO2=35`, `SERVO3=33`, `SERVO4=34`
- `SERVO1_REVERSED=1`, `SERVO4_REVERSED=1` (rear thrusters mounted opposite — physical, correct)
- All `RCx_REVERSED=0`
- `PILOT_STEER_TYPE=3` (found 2026-07-16). ArduRover flips steering when reversing;
  our autonomous steering assumes it doesn't. Value 3 ("unchanged when backing up")
  stops the boat spinning out during autonomous reverse.

Known-good params: `working_crusader_params.params` in the drive.

---

## 13. Camera + buoy detection

The model runs **on the Jetson GPU via TensorRT**, not on the camera. The OAK-D LR
streams RGB + stereo depth; the Jetson infers and samples depth per detection —
~30 fps, versus ~5 fps on-camera.

`perception_node` runs the whole pipeline in one process (capture → detect →
depth-associate) and publishes only the small `DetectionArray` on
`/crsd/detections_body` (BODY frame: x = starboard+, y = forward+). Shipping
1080p frames over DDS would blow the latency budget.

Model files live at the **workspace** level, `~/robotx_ws/models/` — gitignored
and copied in out-of-band:

| File | What |
|---|---|
| `buoy_v16.pt` | source weights (MHSeals V16, YOLOv11, 13 classes) |
| `buoy_v16.engine` | TensorRT engine, **compiled for this Jetson** |

Regenerate the engine if the Jetson or JetPack changes:

```bash
yolo export model=buoy_v16.pt format=engine half=True imgsz=352,640 device=0
```

**Camera must be USB 3.** `oakd_guard` asserts this at startup and refuses to run
on USB 2. Check manually:

```bash
python3 -c "import depthai as dai; print(dai.Device().getUsbSpeed())"
```

Must print `SUPER`. `HIGH` = USB 2 = wrong cable or port.

> **Detection quality on our buoys is UNVERIFIED.** The model was trained on
> another team's buoys. Retraining on our own footage is a prerequisite for
> trusting any collision-avoidance metric — see [G2_bench_procedure.md](G2_bench_procedure.md).
> Use `tools/scripts/collect_footage.py` to capture training data on water days.

---

## 14. LiDAR (Livox MID360)

The MID360 fills the LiDAR slot for camera–LiDAR fusion. `livox_ros_driver2` is
**baked into the `asv` image** in its own workspace (`/opt/livox_ws`) and owns the
device the way MAVProxy owns the Pixhawk — no other node opens it.

`lidar_fusion_node` subscribes to `/livox/lidar` **and** `/crsd/detections_body`,
and refines each detection's **range** from LiDAR returns in its bearing sector:
the camera keeps the reliable bearing, the LiDAR supplies the accurate range.
Output on `/crsd/detections_fused`.

**Safety invariant:** a detection with no LiDAR support is **passed through with
its camera range, never dropped**. Losing an obstacle is worse than carrying a
coarse range. A stale or absent cloud degrades to camera-only plus a logged WARN.

Verify the driver is present in the container:

```bash
docker exec asv bash -lc 'source /opt/ros/humble/setup.bash && source /opt/livox_ws/install/setup.bash && ros2 pkg list | grep livox'
```

Should print `livox_ros_driver2`. If it prints nothing, the image was built
without PCL — rebuild it; the Dockerfile now fails the build in that case.

**Two prerequisites before fused ranges are trustworthy:**

1. **Extrinsic calibration.** The `lidar_*` params in `crusader_params.yaml` and
   `extrinsic_parameter` in `MID360_config.json` are placeholder zeros. Keep the
   *driver* extrinsic at identity and treat `crusader_params.yaml` as the single
   source of truth, so points are not transformed twice.
2. **Its own NIC/subnet.** The MID360 is an Ethernet/UDP device and must not share
   the RoboCommand RJ-45 link.

---

## 15. GPS heading (dual-antenna, no compass)

Heading comes from **two GPS antennas (moving-baseline yaw)**, not the compass.
The compass is **disabled** (`COMPASS_USE=0`) — intentional, and it permanently
fixed the pool "mag field" arming errors. **Do not re-enable it.**

Key params (the saved param file in the drive is the source of truth):

- `GPS1_TYPE=25`, `GPS1_MB_TYPE=1`
- `GPS1_MB_OFS_X/_Y` = antenna offset (Master relative to Slave)
- `GPS1_COM_PORT=1` — the UM982 serial port; wrong value = heading never appears
- `EK3_SRC1_YAW=2`, `AHRS_EKF_TYPE=3`, `COMPASS_USE/USE2/USE3=0`

If heading is wrong: it needs **open sky and 2–3 minutes** to resolve. Verify by
rotating the boat by hand and watching heading track. If heading **never** appears,
suspect `GPS1_COM_PORT` before anything else.

---

## 16. The container

Image `asv`, built **from this repo's Dockerfile** — not a hand-made container.
Base is `ultralytics/ultralytics:latest-jetson-jetpack6` (CUDA + PyTorch +
TensorRT), plus ROS 2 Humble, MAVProxy, depthai, and Livox-SDK2 +
`livox_ros_driver2`.

Rebuild it on the Jetson host:

```bash
cd ~/robotx_ws/src/rx26_asv && docker build -t asv .
```

**Build to a throwaway tag first** if the current container is working, verify,
then retag and recreate:

```bash
docker build -t asv:next . && docker tag asv:next asv:latest && docker rm -f asv && docker create -it --name asv --network host --privileged --runtime nvidia -v /dev:/dev -v /home/crusader/robotx_ws:/root/robotx_ws asv && docker start asv
```

Those create flags are not optional:

| Flag | Why |
|---|---|
| `--network host` | MID360 UDP, RoboCommand RJ-45, MAVProxy loopback rebroadcast, DDS multicast |
| `--privileged` + `-v /dev:/dev` | OAK-D USB and serial devices |
| `--runtime nvidia` | GPU for TensorRT |
| `-it` | keeps `CMD ["bash"]` alive so `docker start -a` works |

**The image is the only place runtime dependencies are installed.** Never
`pip install` into a running container: that state is undocumented and lost on
`docker rm`. An unpinned `pip install protobuf grpcio-tools` once resolved
protobuf to 7.x and broke TensorFlow — and with it the whole perception stack.
The Dockerfile pins those and asserts the ML stack still imports at build time.

Jetson power mode should be `MAXN_SUPER` (`sudo nvpmodel -m 2`) for full GPU speed.

---

## 17. Position + yaw hold (`dp_hold`) — docking, Mission 3

`dp_hold` locks onto a visual target (buoy/dock) and holds a set yaw and
distance/side offset using **RC override in MANUAL mode**. Strafe, throttle and
rotate are all driven by the node. LED stays GREEN.

**Why MANUAL and not GUIDED:** ArduRover's GUIDED cannot strafe on this frame —
it turns instead of translating laterally. RC override in MANUAL is currently the
only way to get true lateral hold on this omni boat.

Status: working, gains being tuned. `MOT_THST_ASYM ≈ 1.5`, `MOT_THST_EXPO ≈ 0.65`
in progress. Reverse-spin was fixed by `PILOT_STEER_TYPE=3` (§12).

**Do not run beyond WiFi range.** See §18.

---

## 18. Safety and manual takeover

Two radio links, very different range:

| Link | Range | Use |
|---|---|---|
| WiFi / SSH | ~100 m | convenience only. **Ctrl+C is NOT a safety tool.** |
| ELRS RC | km | **all safety actions** |

**Emergency:** **SB down = e-stop.** Kills motors instantly in any mode,
independent of any node, at full RC range.

### Known gap — RC-override nodes win against your sticks

While `dp_hold` (or any RC-override node) runs, it overrides your sticks.
Flipping SC to Manual does **not** return control — the override keeps winning.
To take back control you must Ctrl+C the node (needs WiFi) **or** e-stop.

**Until the autonomy-drop switch exists, do not run `dp_hold` — or any
RC-override mechanism — beyond WiFi range.** This is a standing constraint, not
a field-ops preference.

The planned fix is an RC-based, software-latched autonomy-drop switch read via a
Pixhawk RC channel, so it works out of WiFi range. `api/common/drop_latch.py` and
`telemetry_bridge`'s latched `/crsd/autonomy_drop` implement the software side;
the switch assignment and bench sign-off are Gate G1
([G1_bench_procedure.md](G1_bench_procedure.md)). **RC7 already carries
arm/e-stop** (`RCx_OPTION=165`, 994/1498/1995), which is most of what the latch
needs to read — confirm what option 165 maps to in Rover 4.6.3 before building on it.

---

## 19. `gate_navigator` — Mission 1 transit

GUIDED-mode gate navigation: drives QGC-uploaded waypoints and corrects toward
buoy-gate midpoints. **Status: UNTESTED on water.**

Order of operations:

1. Boot chain up (§1) — MAVProxy and container are automatic.
2. Upload initial waypoints from QGC / Mission Planner.
3. Core stack: `ros2 launch rx26_asv core.launch.py`
4. Perception: `ros2 launch rx26_asv camera.launch.py`
5. Confirm detections before anything moves.
6. `ros2 run rx26_asv gate_navigator`

The entry/exit circling maneuvers are a real local-minima risk given GUIDED's
turn-instead-of-strafe behaviour on this frame.

---

## 20. Sharing the workspace with `robotx_2026`

The legacy boat repo lives at `~/robotx_ws/src/robotx_2026`. It is **not built by
us** — `rebuild.sh` and `install_container.sh` use
`--packages-select interfaces rx26_asv`.

**Never launch both stacks.** `robotx_2026` ships its own `gate_navigator`,
`dp_hold`, `led_node` and `rc_watchdog`, each opening its own
`udpin:127.0.0.1:14551`, and its watchdog sends `MAV_CMD_COMPONENT_ARM_DISARM`.
Running both gives you two processes competing for MAVProxy's rebroadcast port
and two independent disarm authorities. Package names don't collide, so builds
are fine — the hazard is entirely at runtime.

---

## 21. Preflight

```bash
docker exec -it asv python3 /root/robotx_ws/src/rx26_asv/tools/scripts/preflight.py
```

**Exit nonzero = do not arm.** Manual items it can't check: GPS yaw resolved
(open sky, 2–3 min)? ELRS e-stop range-tested today? Autonomy-drop verified if
any RC-override task is planned?

> **Known bug:** run inside the container, the `asv container` check fails with
> `No such file or directory: 'docker'`, and the ROS-topics and TensorRT-engine
> checks then SKIP behind it. Those two SKIPs are noise, not information. Fix
> pending; until then verify ROS topics and the engine file by hand.

---

## 22. Known gaps and stale artifacts

Honest list — these are known-wrong, not merely untested:

- **`preflight.py` host/container detection** (§21). The arm gate can't currently
  check ROS topics or the TensorRT engine the way the docs say to run it.
- **`crusader_devices.json`** still describes a ball launcher, a Teensy and a
  Jetson-side GPS that are **not aboard**, with port chains that point at the
  Pixhawk. Documentation-only — no node loads it — but wrong.
- **GPS serial placeholders** in `99-crusader.rules` (`TODO_GPS1_SERIAL`) refer to
  u-blox units that aren't attached. The installer warns about them on every run.
- **Buoy model not retrained** on our buoys (§13). This bottlenecks every
  collision-avoidance number.
- **MID360 not yet on its own NIC**, extrinsic not calibrated (§14).
- **Autonomy-drop switch not built** (§18) — blocks `dp_hold` field testing.

### Confirmed hardware (2026-07-28)

| Device | Identity | udev |
|---|---|---|
| Pixhawk | ArduPilot Pixhawk1, `1209:5741` | `/dev/crsd-pixhawk` |
| LED Arduino | CH340 `1a86:7523`, **no serial number** | `/dev/crsd-led` |
| OAK-D LR | MX ID `194430101110C82F00` | (not a tty) |

A Teensy (`16c0:0483`) and a Prolific PL2303 (`067b:23a3`) also enumerate but are
**not on the official hardware list** — treat as unused until someone traces the
cable. They get no symlink deliberately: a wrong symlink is worse than none.

> **Do not add a second CH340.** That chip has no serial number, so two of them
> cannot be told apart by serial and the LED symlink would become ambiguous.
