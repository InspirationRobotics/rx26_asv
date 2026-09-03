# CRUSADER USV — OPERATIONS MANUAL (RobotX 2026)

**Updated 2026-08-13 (v0.5).** Field manual for the Crusader stack (repo `rx26_asv`).

For first-time installation on a machine, use [SETUP_GUIDE.md](SETUP_GUIDE.md).
This document assumes the boat is already installed and you are operating it.

> **This supersedes the `robotx_2026` documentation.** Package, container, repo
> and startup procedure all changed. The single biggest difference: **MAVProxy
> and the container start automatically via systemd** — they are no longer
> Terminal 1 / Terminal 2 commands you type.
>
> **v0.5 removed everything that had never run on the boat** — perception,
> avoidance, mission planning, station keeping, gate transit — and split what
> remained into seven `crusader_*` packages. What is documented here is what
> exists. The OAK-D is driven from `asv` by `crusader_perception`, which also
> detects in it; only the MID360 lives in a second container (§13).

---

## 1. The big picture

- The boat's computer is a **Jetson Orin Nano**, reachable over the boat's WiFi.
- All code lives on the Jetson at **`~/robotx_ws/src/rx26_asv`**, version-controlled with git.
  `~/robotx_ws` is a **colcon workspace**, not the repo — the repo is one source
  directory inside `src/`, alongside any others.
- Code **runs** inside a Docker container named **`asv`** (ROS 2 Humble + CUDA);
  code is **edited** outside the container. Both see the same files via a bind mount.
- A **second container drives the MID360 LiDAR** and nothing else, publishing its
  point cloud as a ROS topic. The OAK-D is **not** in it — `crusader_perception`
  owns that camera from inside `asv`, and detects in it there (§13).
- The **Pixhawk is owned by exactly one program, MAVProxy**, which rebroadcasts
  over UDP to everything else (our nodes, Mission Planner/QGC on your laptop).
  Nothing else may open the serial device.
- The **LED strip** shows boat status: **RED** = e-stopped, **YELLOW** = manual,
  **GREEN** = autonomous.

### Boot chain (automatic, since 2026-07-28)

```
power on
  ├─ crsd-mavproxy.service   (host) ── takes /dev/crsd-pixhawk, rebroadcasts UDP
  │    └─ crsd-container.service (host) ── docker start -a asv
  └─ crsd-livox.service      (host) ── MID360 driver in the livox container
```

All three are `systemctl enable`d, so a power cycle brings them up. The `asv`
container is ordered `After=crsd-mavproxy.service` so no node ever races MAVProxy
for the serial device.

**The LiDAR branch is deliberately independent.** It shares no device with the
Pixhawk, so ordering it behind MAVProxy would only make a LiDAR problem look like
an autopilot problem. `lidar_cluster_node` retries until the topic appears, so
nothing downstream needs to win a start-order race.

Check the chain:

```bash
systemctl status crsd-mavproxy crsd-container crsd-livox --no-pager
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
   commit them here.
2. SSH in. Use `-A` so git operations on the Jetson use *your* key (see §10):

```bash
ssh -A crusader@crusader-asv.local
```

If mDNS doesn't resolve, use the IP directly:

```bash
ssh -A crusader@<JETSON_IP>
# latest:
ssh -A crusader@192.168.100.109
```

You're on the Jetson when the prompt reads `crusader@crusader-asv`.

### If WiFi won't work — USB fallback

1. Plug USB-C from the Jetson to your laptop.
2. `ssh crusader@192.168.55.1`
3. Find the WiFi address: `ip addr` (look for the `wl*` interface, e.g. `wlP1p1s0`).
4. Unplug USB, then SSH to that address over WiFi.

### Known field fix — QGC won't connect at the lake

If MAVProxy is running but QGroundControl/Mission Planner never sees the boat,
**do not start by adding addresses.** As of 2026-09-03 the wired interface
already carries two static addresses on purpose (`192.168.1.5` for the MID360,
`192.168.8.109` for the Bullet bridge — see §2 and `Resources.md`), and an
`ip addr add` of a *WiFi-subnet* address onto the wired NIC stacks a third one
onto a config that is already correct, does not survive reboot, and gives you a
host route that quietly competes with the real one.

Work down this list instead:

1. `ip -brief addr` — what does the boat actually have, on which interface?
2. Is your laptop on the same subnet as any of them? Broadcast does not cross
   subnets, and the Bullet bridge is its own.
3. If not, add your laptop as an explicit unicast target — that is exactly what
   `GCS_IPS` is for, and it is additive (below).
4. Only if the wired *link itself* is misconfigured, fix it in the
   NetworkManager profile (`nmcli con mod "Wired connection 1" ...`) so the fix
   survives a reboot. There is no netplan on this machine.

---

## 3.1 HEARTBEAT rate — why it is not a parameter

`/crsd/fcu_status` (mode + armed) is fed by **HEARTBEAT alone**, and it has to be
fresh enough to feed the RoboCommand link.

The handbook (`Boat/handbook/3.4-communications-protocol.md` §3.4.11) says *"The
OCS publishes a separate `RxReport` heartbeat on behalf of each active vehicle at
2Hz"*, carrying robot state, position, speed/heading, roll/pitch and current
task. **The 2 Hz emitter is our own OCS, not the boat** — the boat's job is to
keep it supplied. A 1 Hz source cannot honestly feed a 2 Hz report, which is why
the source is set comfortably above it rather than at it.

**ArduPilot sends HEARTBEAT on a fixed 1 Hz timer belonging to no `SRx_*` stream
group** — there is no `SR0_HEARTBEAT` to raise, and `--streamrate` does not touch
it. The only lever is `MAV_CMD_SET_MESSAGE_INTERVAL`. Confirmed on Crusader
2026-09-03: ArduRover 4.6.3 answers `MAV_RESULT_ACCEPTED` and the measured rate
goes **1.00 → 5.33 Hz**.

`start_mavproxy.sh` requests it on every start:

```bash
--cmd="long SET_MESSAGE_INTERVAL 0 200000"     # 200000 us = 5 Hz
```

Override with `CRSD_HEARTBEAT_US` in `/etc/default/crusader`.

> **Interval `-1` means DISABLE, not "restore the default".** Sending it switches
> HEARTBEAT off completely, which takes `/crsd/fcu_status`, the LED status stack
> and the RC watchdog's `bridge_ok` with it — the boat goes dark in a way that
> looks like a dead link. Use `0` for the vehicle default. Recovery is one more
> SET_MESSAGE_INTERVAL at the rate you wanted.

**SET_MESSAGE_INTERVAL is runtime state, not EEPROM.** The `--cmd` covers a host
reboot and a `systemctl restart`. It does **not** cover the autopilot rebooting
on its own — after that the rate silently falls back to 1 Hz, the source period
again equals `telemetry_bridge`'s 1.0 s staleness threshold, and the log fills
with `stream 'fcu_status' stale`. That storm is the signal, not the disease:

```bash
sudo systemctl restart crsd-mavproxy
```

Check the live rate any time with `tools/scripts/preflight.py`, or directly:

```bash
python3 -c "
from pymavlink import mavutil, mavlink
import time
m=mavutil.mavlink_connection('udpin:127.0.0.1:14550'); m.wait_heartbeat()
t0=time.time(); n=0
while time.time()-t0 < 10:
    if m.recv_match(type='HEARTBEAT', blocking=True, timeout=3): n+=1
print(f'{n/10:.2f} Hz')"
```

## 3. Telemetry to your laptop

MAVProxy **broadcasts** telemetry to the whole field subnet, so nothing has to be
typed in per-laptop or per-day: Mission Planner and QGroundControl both listen on
14550 for traffic from any sender.

You only need to touch this if the field network hands out a different subnet than
`192.168.100.0/24`. Set the subnet's BROADCAST address (host bits all 1 — `.255`
for a /24), not a laptop's own address:

```bash
echo 'CRSD_BCAST_ADDR=192.168.100.255' | sudo tee /etc/default/crusader && sudo systemctl restart crsd-mavproxy
```

The unit reads that file via `EnvironmentFile=-/etc/default/crusader`. If it's
absent, `start_mavproxy.sh` falls back to its built-in default.

> This variable used to be called `LAPTOP_IP`, from when the `--out` was unicast.
> Putting one laptop's address in it now sends "broadcast" traffic to that single
> host, and everyone else on the field network sees nothing.
>
> Broadcast is subnet-scoped, not laptop-scoped: if two teams share a field
> network, every laptop sees every broadcasting boat. Pick the right vehicle in
> the GCS connection list.

### When a laptop still sees nothing — `GCS_IPS`

A subnet broadcast only reaches hosts **on that subnet**, and only if nothing in
between drops it. An AP with client isolation, a laptop on the far side of the
Bullet bridge, or a machine on the wired `192.168.1.0/24` all see nothing while
the boat looks perfectly healthy from the Jetson. Add explicit unicast targets —
space-separated, quoted, in the same file:

```bash
echo 'GCS_IPS="192.168.100.50 192.168.1.20"' | sudo tee -a /etc/default/crusader && sudo systemctl restart crsd-mavproxy
```

`start_mavproxy.sh` adds one `--out` per address. This is **additive**: broadcast
stays on, so a laptop that already worked keeps working whether or not it is
listed. Confirm on the *running process* — the `--out` flags are built inside the
script, so `systemctl show ... -p ExecStart` will not show them:

```bash
pgrep -af mavproxy | tr ' ' '\n' | grep -- --out
```

> Note `tee -a` — the earlier command uses plain `tee`, which would replace the
> file and drop `CRSD_BCAST_ADDR`.

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
cd /root/robotx_ws && source install/setup.bash && ros2 launch crusader_bringup core.launch.py
```

That starts four nodes:

| Node | Package | Role |
|---|---|---|
| `telemetry_bridge` | `crusader_fcu` | THE single consumer of MAVProxy's rebroadcast, and THE single MAVLink sender |
| `pixhawk_led_status_node` | `crusader_behavior` | Pixhawk state → LED state |
| `led_node` | `crusader_behavior` | `/crsd/led_state` → LED serial |
| `rc_watchdog` | `crusader_behavior` | force-disarm on RC-link loss |

**Verify:** flip the RC arm switch — the LED strip and the launch logs should
change together.

### Camera

`crusader_perception` owns the OAK-D from inside `asv` (§13). Two nodes, and only
one may run — the device admits a single client:

```bash
ros2 run crusader_perception buoy_detector    # detections; what the stack wants
ros2 run crusader_perception oakd_publisher   # raw frames; for a human, ~38 MB/s
```

`buoy_detector` serves its own annotated view at `http://<JETSON_IP>:8080` — boxes,
ranges, and where depth was sampled. That is the one to use when checking the
camera. `tools/oak_view.py` is the alternative for a *raw* topic:

```bash
python3 /root/robotx_ws/src/rx26_asv/tools/oak_view.py --topic /oak/rgb
```

It subscribes rather than opening the device, so it cannot take the camera away
from anything and any number can run at once. It cannot share port 8080 with
`buoy_detector`'s view, and they are never both useful at once.

> Neither perception node is in `core.launch.py` — they contend for the one
> camera, so which runs is an operator choice per session. World-model nodes are
> not there either: that package is still scaffolded and empty (§13).

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
| **SE** | 9 | **PROPOSED, NOT BUILT** — autonomy-drop for RC-override nodes. See §17. |

**SB down is the real e-stop.** It kills motors instantly in any mode,
independent of any node or software, at full RC range.

---

## 7. Useful individual commands

Inside the container, after `source install/setup.bash` in `/root/robotx_ws`.

Run one node instead of the launch file (note the package name):

```bash
ros2 run crusader_behavior led_node
```

List every executable we ship:

```bash
ros2 pkg executables crusader_fcu crusader_behavior
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

Watch the camera from a laptop browser — then open `http://<JETSON_IP>:8080`,
Ctrl+C to stop:

```bash
python3 /root/robotx_ws/src/rx26_asv/tools/oak_view.py --topic /oak/rgb
```

> This subscribes to a topic and re-serves it; it never opens the device, so it
> cannot take the camera away from anything. Run as many as you like. Against
> `oakd_publisher` you must pass `--topic /oak/rgb` — the built-in default is a
> `/compressed` name that our node does not publish.

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
cd /root/robotx_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to crusader_bringup && source install/setup.bash
```

> **`--packages-up-to crusader_bringup` is deliberate.** The workspace holds other
> package sources (`robotx_2026`, the livox container's) whose build state is not
> ours to change and whose build failure must not block ours. bringup depends on
> every package we ship, so "up-to" is exactly our stack — and it stays correct
> when a package is added, which a `--packages-select` list does not.

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
└── src/
    ├── rx26_asv/                   # <- THIS REPO (seven packages, not one)
    │   ├── crusader_msgs/          # ament_cmake — msg definitions (6)
    │   ├── crusader_common/        # shared lib: params, lifecycle, latch, geo
    │   ├── crusader_fcu/           # telemetry_bridge — the only MAVLink talker
    │   ├── crusader_perception/    # OAK-D driver + buoy detection
    │   ├── crusader_world_model/   # EMPTY — fusion + occupancy grid
    │   ├── crusader_behavior/      # safety/ watchdog + indicator/ LED stack
    │   ├── crusader_bringup/       # ament_cmake — launch/ + config/; build entry point
    │   ├── docs/  firmware/  params/
    │   ├── scripts/start_mavproxy.sh
    │   ├── setup/                  # install_jetson_host.sh, install_container.sh
    │   ├── tools/                  # udev, systemd, preflight, param_guard, rebuild, oak_view
    │   └── Dockerfile              # builds the `asv` image
    ├── robotx_2026/                # legacy boat repo — NOT built by us (see §19)
    └── <livox container sources>   # MID360 driver only
```

Build with `colcon build --packages-up-to crusader_bringup` — bringup depends on
every package we ship, so that one target is the whole stack, and it stays correct
when a package is added. **A new package must be listed in
`crusader_bringup/package.xml`** or it silently stops being built.

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

Known-good params: **`params/working_crusader.params`**, committed in this repo
and diffed against the live vehicle by `preflight.py` before every arm (§18).
Re-export it from QGC after any deliberate param change.

> **Resolved 2026-09-03.** The live board reads `PILOT_STEER_TYPE=3`, agreeing
> with this section and with `param_guard`'s PROTECTED comment. The committed
> baseline was the stale one and has been corrected to 3; evidence is
> `QGC params/2026-09-03_crusader_ardurover463.params`.
>
> While it disagreed, `param_guard` returned **FAIL on a correct boat** — every
> run, for weeks. That is worse than no gate: a check that always fails is a
> check nobody reads. If preflight ever starts failing on something you believe
> is right, suspect the baseline before you suspect the boat, and settle it by
> dumping params rather than by editing either file to match the other.

---

## 13. The two containers

There are exactly two, and the boundary is **one device**:

| | `asv` (this repo) | livox container |
|---|---|---|
| Owns | Pixhawk, LED Arduino, **OAK-D LR** | Livox MID360, and nothing else |
| Has | ROS 2, CUDA/TensorRT, **depthai**, cv_bridge, MAVProxy | the Livox SDK/driver |
| Publishes | `oak/detections`, boat state, world model | `PointCloud2` |
| Does NOT | open the LiDAR | detect, fuse, or map |

This is not the split the repo originally described. The old plan had a single sensor
container owning *both* devices and publishing raw frames, with detection running in `asv`
against those topics — and the `asv` Dockerfile asserted depthai was absent to enforce it.
**`buoy_detector` made that untenable**: shipping 1.28 MB frames (~38 MB/s at 30fps) across
a container boundary to produce a few hundred bytes of `Detection3DArray` is the wrong
trade, so the camera and the inference belong in one process. That process needs depthai
*and* TensorRT, and `asv` is the only image with the CUDA stack — so depthai moved in and
the guard came out. The package formerly called `crusader_sensors` is now
`crusader_perception` for the same reason: it does both jobs.

- `crusader_perception` — owns the OAK-D, publishes detections with ranges in
  `camera_link`. Runs in `asv`. Its LiDAR half is unwritten.
- `crusader_world_model` — sensor-agnostic: fusion into 3D object positions, occupancy
  grid. Still **empty and scaffolded**.

The pre-v0.5 LiDAR fusion was removed unverified. It is recoverable from git at `8c4ffa5`
and worth reading before rewriting — see each package's README.

### Battery voltage and the low-voltage shutdown

**Crusader runs a 4S LiPo.** Voltage sensing was dead until 2026-09-03 and the
way it failed is worth knowing, because it looks like working config:
`BATT_MONITOR=3` (Analog V+I) was set, but the autopilot had never been rebooted
since. **ArduPilot only creates the analog backend params on a reboot after
`BATT_MONITOR` changes** — so `BATT_VOLT_PIN`, `BATT_VOLT_MULT`, `BATT_CURR_PIN`
and `BATT_AMP_PERVLT` did not exist at all, and `SYS_STATUS.voltage_battery`
read 0 mV while the parameter list looked correctly configured.

After the reboot the backend appeared (`BATT_VOLT_PIN=2`, `BATT_CURR_PIN=3`) and
the divider was calibrated against a multimeter to **`BATT_VOLT_MULT=14.5964`**.
It now reads 16.39 V on a charged pack, ≈4.10 V/cell.

> **Current is still not sensed.** `current_battery` reports −1 and
> `BATT_CAPACITY` is a stock 3300 mAh that describes nothing aboard. Do not
> build anything on mAh consumed.

`crsd-battwatch` ([`tools/scripts/batt_watchdog.py`](../tools/scripts/batt_watchdog.py))
reads MAVProxy's **14552** `--out` and asks `crsd-power` to poweroff when the
voltage stays low. It is unprivileged and goes through that socket rather than
being a second root program.

| Variable | Default | Notes |
|---|---|---|
| `CRSD_BATT_SHUTDOWN_V` | `13.2` | 4S LiPo at 3.30 V/cell. Poweroff below this |
| `CRSD_BATT_WARN_V` | `14.0` | 4S LiPo at 3.50 V/cell. Logs only, never acts |
| `CRSD_BATT_HOLD_S` | `30` | Must stay low continuously this long. Four T200s sag the pack hard on a step input; a momentary dip must not power the boat off |
| `CRSD_BATT_STALE_S` | `10` | No sample for this long clears the countdown |
| `CRSD_BATT_ENDPOINT` | `udp:127.0.0.1:14552` | Its own port. 14551 is telemetry_bridge's, 14550 stays free for tooling |

**What it is for.** The boat sitting powered-up on the cart with nobody
watching — that is how packs die here. It is not a mission-time energy policy:
the Jetson is ~15 W against four T200s pulling hundreds, so powering it off does
not meaningfully extend a run.

**Before trusting a threshold on a new pack**, watch it without letting it act.
**Stop the service first** — two readers on one UDP port split the datagrams
between them, so a dry-run beside the live service blinds the live one. Measured
2026-09-03: the service logged `no usable voltage (last good reading: 10s ago);
not acting` for the whole 40 s the second process was up. It failed safe, but
the boat had no low-voltage protection for those 40 seconds. The script now
refuses to start in `--dry-run` when the port is already held.

```bash
sudo systemctl stop crsd-battwatch
tools/scripts/batt_watchdog.py --dry-run
sudo systemctl start crsd-battwatch
```

**Autopilot-side battery failsafe is deliberately still off** — `BATT_LOW_VOLT`,
`BATT_CRT_VOLT`, `BATT_FS_LOW_ACT` and `BATT_FS_CRT_ACT` are all 0. Those change
what the *vehicle* does on the water (Rover can RTL or Hold on them); the Jetson
watchdog does not. Enabling them is a separate, deliberate decision.

### Starting the MID360

`crsd-livox.service` runs it at boot via
[`scripts/start_livox.sh`](../scripts/start_livox.sh). By hand:

```bash
sudo systemctl restart crsd-livox && journalctl -u crsd-livox -f
```

Settings live in `/etc/default/crusader` beside `CRSD_BCAST_ADDR`, so none of
this needs a unit edit:

| Variable | Default | Notes |
|---|---|---|
| `CRSD_LIVOX_CONTAINER` | `crusader_legacy` | The **name**, never the 12-hex ID — an ID changes every time the container is recreated |
| `CRSD_LIVOX_LAUNCH` | `ros2 launch livox_ros_driver2 rviz_MID360_launch.py` | See the rviz trap below |
| `CRSD_LIVOX_HOST_IP` | `192.168.1.5` | The address the driver binds, from `MID360_config.json` |
| `CRSD_LIVOX_SETUP` | `/opt/ros/humble/setup.bash /root/livox_ws/install/setup.bash …` | ROS setup files to source **inside** the container, base distro first. Missing entries are skipped, and the wrapper autodiscovers if none of them resolve `livox_ros_driver2` |

> **`ros2: command not found` — the first thing that goes wrong.** `docker exec
> bash -lc` is not enough. A login shell reads `/etc/profile` and
> `~/.bash_profile`; ROS setup conventionally lands in `~/.bashrc`, which a
> *non-interactive* login shell never reads. So `ros2` works perfectly when you
> `docker exec -it … bash` by hand and is missing under systemd.
>
> `start_livox.sh` sources `CRSD_LIVOX_SETUP` explicitly and, if `ros2` is still
> absent, prints which paths were found or missing, what is under `/opt/ros`, and
> any overlay `setup.bash` it can find. Point `CRSD_LIVOX_SETUP` at the right
> ones. To see the container's actual layout:
>
> ```bash
> docker exec crusader_legacy bash -c 'ls -d /opt/ros/*/; find /opt /root -maxdepth 4 -name setup.bash -path "*install*"'
> ```

> **The rviz trap — but measure before you believe it here.**
> `rviz_MID360_launch.py` starts rviz2 alongside the driver. Where the stock
> launch registers an `OnProcessExit` handler that **shuts down the whole launch
> when rviz exits**, a headless boot kills rviz2 instantly and takes the driver
> with it; systemd restarts, it dies again, and the journal reads like an orderly
> shutdown rather than an error — the LiDAR is simply never there.
>
> **On this boat, as of 2026-09-03, that does NOT happen.** rviz2 died at boot as
> expected (`[ERROR] [rviz2-2]: process has died … exit code -6`) and the driver
> stayed up: `livox_ros_driver2_node` still running hours later, `/livox/lidar`
> still at 10.0 Hz, `/livox/imu` at 200.0 Hz. Whatever launch file
> `crusader_legacy` actually carries does not register that handler.
>
> So the ERROR line in `journalctl -u crsd-livox` is **not** evidence the LiDAR is
> down. Check the topic rate from inside `asv` before chasing it:
> `ros2 topic hz /livox/lidar`. The headless copy below is still worth doing — it
> removes a wasted process and a permanently alarming log line — but it is
> housekeeping here, not a repair.
>
> It is still the default because it is the launch that publishes **PointCloud2**
> (`xfer_format: 0`); `msg_MID360_launch.py` publishes `CustomMsg`, which ROS 2
> will not connect to a PointCloud2 subscriber at all. Fix it once, inside the
> livox container:
>
> ```bash
> cd $(ros2 pkg prefix livox_ros_driver2)/share/livox_ros_driver2/launch_ROS2
> cp rviz_MID360_launch.py MID360_headless_launch.py
> # then delete the rviz2 Node and the OnProcessExit/Shutdown handler
> ```
>
> and point `CRSD_LIVOX_LAUNCH` at it. `start_livox.sh` prints this whenever the
> configured launch name contains "rviz".

The wrapper also **waits (bounded) for `CRSD_LIVOX_HOST_IP` to appear** before
starting the driver. `network-online.target` does not mean "this interface has
this address": at boot the wired link often comes up seconds late, and the driver
then fails to bind with an error that reads like a LiDAR fault rather than a
timing one.

`ExecStop` kills only the driver process, not the container — `crusader_legacy`
may hold other things, and stopping the LiDAR must not take them down.

**The topic contract with the livox container**, now settled: `/livox/lidar`,
`sensor_msgs/PointCloud2`, `BEST_EFFORT`. A QoS mismatch is silent — a
BEST_EFFORT publisher and a RELIABLE subscriber match nothing, `ros2 topic list`
looks perfect, and no data flows. A **type** mismatch is equally silent and
easier to hit: see §13's `xfer_format` note and `tools/lidar_view.py`.

### Watching the camera from a laptop

```bash
python3 /root/robotx_ws/src/rx26_asv/tools/oak_view.py --topic /oak/rgb
```

Then open `http://<JETSON_IP>:8080`. It subscribes to the camera topic and re-serves it as
MJPEG; it never opens the device, so it cannot take the camera away from anything, and any
number can run at once. Pass `--topic` to point it at `/oak/rgb` — it defaults to a
`/compressed` topic, which `oakd_publisher` does not publish.

### Host-level bits that stay with us

The OAK-D permission rule and the usbfs memory bump live in `tools/udev/` even though the
camera isn't ours: udev rules and kernel parameters are **host** state and cannot live
inside the other container's image. They grant access only. Move them when that container
grows its own host installer.

---

## 14. GPS heading (dual-antenna, no compass)

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

## 15. The container

Image `asv`, built **from this repo's Dockerfile** — not a hand-made container.
Base is `ultralytics/ultralytics:latest-jetson-jetpack6` (CUDA + PyTorch + TensorRT),
plus ROS 2 Humble, `cv_bridge`/`sensor_msgs_py`, and MAVProxy/pymavlink/pyserial.

**depthai yes, Livox SDK no** — this image owns the OAK-D and the livox container owns the
MID360 (§13). The build imports depthai as a positive check; the depthai *absence* guard
that used to be here was written for the old split and would now fail the build it was
meant to protect.

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
| `--network host` | MAVProxy loopback rebroadcast, DDS multicast, and the MID360 cloud from the livox container |
| `--privileged` + `-v /dev:/dev` | the Pixhawk and LED serial devices |
| `--runtime nvidia` | GPU for TensorRT — detection runs in this container |
| `-it` | keeps `CMD ["bash"]` alive so `docker start -a` works |

**The image is the only place runtime dependencies are installed.** Never
`pip install` into a running container: that state is undocumented and lost on
`docker rm`. An unpinned install once resolved protobuf past what TensorFlow accepts and
broke the whole ML stack; the Dockerfile pins those and asserts the stack still imports at
build time.

Jetson power mode should be `MAXN_SUPER` (`sudo nvpmodel -m 2`) for full GPU speed.

---

## 16. Station keeping and gate transit — removed in v0.5

`dp_hold` (RC-override lateral hold for docking) and `gate_navigator` (GUIDED
buoy-gate transit) are **no longer in this repo**. This repo's versions were
decoupled rewrites that had never run; `dp_hold` was also blocked from field work
by the missing autonomy-drop switch (§17), and `gate_navigator` was untested on
water.

The field-proven versions — the ones that completed prequal — live in
[robotx_2026](https://github.com/InspirationRobotics/robotx_2026). That is what
to port from when mission work restarts, against `oak/detections`.

The ArduRover-side lesson from `dp_hold` is worth keeping regardless: **GUIDED
cannot strafe on this frame** — it turns instead of translating laterally. RC
override in MANUAL is the only way to get true lateral hold on this omni boat,
which is exactly why the autonomy-drop switch gates that whole class of work.

---

## 17. Safety and manual takeover

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

## 18. Preflight

Run it on the Jetson **host**, not in the container:

```bash
python3 ~/robotx_ws/src/rx26_asv/tools/scripts/preflight.py
```

**Exit nonzero = do not arm.** It checks the udev symlinks, disk space, that
MAVProxy is alive, that the `asv` container is up, that `core.launch.py`'s topics
are actually publishing, and diffs the live ArduRover params against the
committed baseline (`params/working_crusader.params`).

**A SKIP is not a PASS.** The script says how many checks skipped and why; decide
each one before arming. Manual items it cannot check: GPS yaw resolved (open sky,
2–3 min)? ELRS e-stop range-tested today? LED showing the state you expect?

---

## 19. Sharing the workspace with `robotx_2026`

The legacy boat repo lives at `~/robotx_ws/src/robotx_2026`. It is **not built by
us** — `rebuild.sh` and `install_container.sh` use
`--packages-up-to crusader_bringup`, which reaches only our seven packages.

**Never launch both stacks.** `robotx_2026` ships its own `led_node`,
`pixhawk_led_node`, `gate_navigator` and `dp_hold`, each opening its own
`udpin:127.0.0.1:1455x` connection to MAVProxy's rebroadcast. Running both gives
you two processes competing for the same rebroadcast ports and two independent
authorities driving the boat. Package names don't collide, so builds are fine —
the hazard is entirely at runtime.

It is kept on the boat on purpose: it holds the prequal-proven `gate_navigator`
and `dp_hold`, which is what mission work will be ported from (§16).

---

## 20. Known gaps

Honest list — these are known-wrong or known-missing, not merely untested:

- **Autonomy-drop switch not built** (§17). It blocks every RC-override
  mechanism, which includes any future station-keeping work. The enforcement
  point exists in `telemetry_bridge`; the bench harness that drove it was removed
  with the rest of the untested code, so [G1](G1_bench_procedure.md) needs a
  replacement publisher written first.
- ~~`PILOT_STEER_TYPE` disagrees between the docs and the boat.~~ **Closed
  2026-09-03**: the live board reads 3, the baseline was stale and is now 3 too.
  See §12.
- **`MOT_THST_ASYM` / `MOT_THST_EXPO` were mid-tuning** when §16's work stopped.
  Confirmed 2026-09-03: **the boat is running the in-progress pair**, `1.5` and
  `0.65`, while the baseline still has `1.0` and `0.0`. Tunable, not protected —
  but that is which one you are running, and the baseline is not it.
- **The livox topic contract is half-settled.** Measured on the boat 2026-09-03:
  `/livox/lidar` is `sensor_msgs/PointCloud2` at **10.0 Hz**, ~20k points per
  message, RELIABLE/VOLATILE, frame `livox_frame`; `/livox/imu` is
  `sensor_msgs/Imu` at **200.0 Hz**, same frame. Both are visible from inside
  `asv` — the containers share host network and the default `ROS_DOMAIN_ID`, so
  no bridge is needed. What is still open is not the plumbing but the **frame
  convention**: `livox_frame` is the raw upside-down sensor frame (+y starboard,
  +z down), and nothing downstream has yet agreed where the negation to REP-103
  body happens. The camera side is settled: `oak/detections`,
  `Detection3DArray`, `camera_link`.
- ~~`fcu_status` trips its own staleness rule at idle.~~ **Closed 2026-09-03 by
  raising HEARTBEAT to 5 Hz** (§3.1). `stream_timeout_s` stays at 1.0 s
  deliberately: with a 0.2 s source period there is 5× margin, so that error now
  means the rate request was lost rather than firing on jitter. Left as a note
  because the reasoning matters — the threshold used to equal the source period
  exactly (HEARTBEAT mean 1.000 s, max 1.028 s, 16 of 24 intervals over 1.0 s),
  and an error that fires constantly on a healthy vehicle teaches everyone to
  scroll past the one line that matters.
- **The OAK-D LR is off the USB bus** (2026-09-03) and cannot be recovered in
  software. `dmesg` records it enumerating on `usb 1-2.2` at t=1350 s and
  disconnecting at t=1450 s, never returning; `lsusb` now shows no `03e7` device
  and that port is empty. A bootloader reload needs the device to enumerate
  first, so there is nothing to flash. This hub exposes no per-port power
  control (`/sys/bus/usb/devices/1-2/port2/disable` does not exist) and
  resetting the parent hub would drop the Pixhawk with it. **Reseat the cable
  and check its power.** Worth noting it was on a USB 2.0 path (480 Mb/s) — the
  SuperSpeed bus has two hubs and no devices, so the LR was never on USB 3.
- **The `asv` container lacks the power-socket bind mount**, so the ground
  station's System tab reports "power helper unreachable" and cannot shut the
  Jetson down. Everything else on the page works. Fixing it requires recreating
  the container, which destroys anything living only inside it — left alone
  deliberately. `crsd-battwatch` is unaffected; it runs on the host.

### Confirmed hardware (2026-07-28)

| Device | Identity | udev |
|---|---|---|
| Pixhawk | ArduPilot Pixhawk1, `1209:5741` | `/dev/crsd-pixhawk` |
| LED Arduino | CH340 `1a86:7523`, **no serial number** | `/dev/crsd-led` |
| OAK-D LR | MX ID `194430101110C82F00` | (not a tty; opened by `crusader_perception`) |

A Teensy (`16c0:0483`) and a Prolific PL2303 (`067b:23a3`) also enumerate but are
**not on the official hardware list** — treat as unused until someone traces the
cable. They get no symlink deliberately: a wrong symlink is worse than none.

> **Do not add a second CH340.** That chip has no serial number, so two of them
> cannot be told apart by serial and the LED symlink would become ambiguous.
