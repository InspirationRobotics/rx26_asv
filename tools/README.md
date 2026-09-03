# `tools/` — one blessed path per operation

Everything here exists so that "how a person does X" has exactly one answer. A second
way to do the same operation is how two machines end up configured differently.

| Dir | Contents | Design choice |
|---|---|---|
| `udev/` | `99-crusader.rules` (VID/PID → stable `/dev/crsd-*` symlinks + permissions), `install_udev.sh` | Device identity resolved by the kernel at plug time, so nothing chases `ttyACM` numbering. Matching is by VID/PID, not USB port chain: it survives recabling, and the two devices aboard are distinguishable. **Only hardware actually on the boat gets a rule** — a symlink for absent hardware makes a config look satisfied while pointing at whatever else enumerated. |
| `systemd/` | `crsd-mavproxy.service`, `crsd-container.service`, `crsd-ros.service`, `crsd-livox.service`, `crsd-power.service`, `crsd-battwatch.service` (templates; `__PLACEHOLDERS__` expanded by `setup/install_jetson_host.sh`) | The boot chain is ordered so MAVProxy owns the Pixhawk before any node starts. A power cycle brings the boat up with no typed commands. The LiDAR branch is deliberately **not** ordered behind MAVProxy — they share no device, and coupling them would make a LiDAR fault look like an autopilot fault. `crsd-power` is ordered behind nothing at all: it is what shuts the machine down cleanly, which is exactly when a fault elsewhere must not have taken it away. `crsd-battwatch` only `Wants=` its two dependencies, so a missing MAVProxy makes it come up and *report* it has no voltage rather than refusing to start — a watchdog nobody notices is missing is the failure mode being avoided. |
| `scripts/` | `check_config.py` (static config guards), `param_guard.py` (PROTECTED vs TUNABLE param diff), `preflight.py` (do-not-arm gate), `dump_params.py` (full param pull in QGC format), `rebuild.sh` (the one blessed rebuild), `crsd_power_helper.py` (the host's shutdown/reboot socket), `batt_watchdog.py` (low-voltage poweroff) | Fail loudly and early: a check that cannot fail is worse than no check. The power helper is the exception to "scripts are unprivileged": it runs as root because a container cannot power off its host, and it is deliberately tiny — one socket, two verbs, and **nothing a client sends ever reaches a command**. `batt_watchdog.py` stays unprivileged by going *through* that socket rather than becoming a second root program. |
| `oak_view.py` | Subscribes to a raw camera topic and re-serves it as MJPEG to a laptop browser (:8080). `buoy_detector`'s own annotated view is usually the better tool | A viewer, not a driver. It never opens the OAK-D, so it cannot take the camera away from perception, and any number can run at once. Runs anywhere with ROS on the path, `asv` included. |
| `lidar_view.py` | MID360 cloud as Plan / Elevation / Both **tabs** in a browser (:8081). `--frame raw\|body` is the LiDAR orientation check — see [docs/G2_lidar_orientation.md](../docs/G2_lidar_orientation.md) | The extrinsic is read from `crusader_params.yaml` when it is built, so the bench view and the clustering node cannot disagree. A check performed against different numbers than the node uses would be worse than no check. Its two filters (`--r-min` clear sphere, `--fov` forward sector) are **drawn on the panel**, so what was excluded is visible rather than implied. |
| `mjpeg_server.py` | Shared HTTP/MJPEG plumbing behind both viewers: framebuffer, tab bar, client counting | Extracted rather than copied: two copies of the dead-stream and browser-closed handling drift, and only one of them gets the fix. Buffers know whether anyone is watching, so a producer can skip rendering a view nobody has open. |

Both viewers render on the Jetson and serve JPEG, so the laptop needs a browser and nothing
else — no ROS on Windows, no rviz2, no X forwarding. They use **different default ports on
purpose**: `buoy_detector`'s built-in view already claims 8080, and two servers cannot bind
one port.

`crusader_groundstation`'s `ground_station` is the fourth thing on that list and takes
**8090**. It is a node rather than a tool here because it is an operational display, not a
bench check — and it sends JSON rather than JPEG, because unlike a camera frame a map is
small enough to be data. It EMBEDS the two viewers below rather than reimplementing them,
which is why it has to avoid their ports rather than share them. Same rule at the laptop
end: a browser and nothing else.

| Port | Served by | Shows |
|---|---|---|
| 8080 | `buoy_detector` (`stream_enable`) *or* `tools/oak_view.py` | annotated camera frames — one at a time; they cannot both bind it |
| 8081 | `tools/lidar_view.py` | MID360 cloud, plan / elevation |
| 8090 | `crusader_groundstation`'s `ground_station` | everything: nodes, telemetry, both viewers, the map, host power |

## The param baseline

`params/working_crusader.params` is the known-good ArduRover config, exported from
QGroundControl. `param_guard.py` diffs the live vehicle against it and hard-fails on any
PROTECTED parameter that differs; `preflight.py` runs that diff before every arm.

**Re-export it after any deliberate, verified param change on the boat.** A stale baseline
turns the gate into noise, and noise is how a real drift gets waved through.

Parameter dumps come in three shapes and they disagree about where the name sits —
Mission Planner writes `NAME,VALUE`, QGroundControl writes
`<vehicle-id>⇥<component-id>⇥NAME⇥VALUE⇥<type>`. `load_param_file` finds the name rather
than assuming a column, and `check_config.py` asserts the committed baseline still parses
into real parameter names.

## Change impact

| You changed | Re-run |
|---|---|
| `udev/99-crusader.rules` | `sudo bash tools/udev/install_udev.sh`, replug, check `ls -l /dev/crsd-*`; affects every device open on the boat |
| `systemd/*.service` | `sudo bash setup/install_jetson_host.sh`, then `systemctl daemon-reload` + reboot to prove the boot chain |
| the `asv` container's mounts | they are fixed at `docker run`, so the container must be RECREATED — `install_jetson_host.sh` checks and prints the line, but will not do it for you |
| `scripts/start_livox.sh` or the livox launch | `systemctl restart crsd-livox`, then `ros2 topic hz /livox/lidar` from inside `asv` — a unit that starts cleanly and publishes nothing is the failure mode here |
| `scripts/param_guard.py` PROTECTED list | `python3 tools/scripts/check_config.py`; a change here needs safety review — it is the list of things nobody may quietly retune |
| `params/working_crusader.params` | `python3 tools/scripts/check_config.py`, then a preflight run against the live boat |
| `scripts/preflight.py` | run it on the Jetson host and confirm no check silently SKIPs |
| `mjpeg_server.py` | both viewers **and** the ground station's camera/LiDAR tabs, which embed them — start each and confirm a browser still gets frames and a closed tab does not pin shutdown |
| `scripts/crsd_power_helper.py` | `sudo bash setup/install_jetson_host.sh`, `systemctl status crsd-power`, then confirm a bad verb is REFUSED before testing a good one. Needs safety review: it is the only root-privileged thing the ground station can reach |
| `scripts/batt_watchdog.py` or any `CRSD_BATT_*` threshold | `tools/scripts/batt_watchdog.py --dry-run` against the real pack FIRST — it logs what it would do and never contacts the power helper. Only then `sudo systemctl restart crsd-battwatch`. Needs safety review: it can power the boat off, and a threshold set for the wrong pack does that on a healthy battery |
| `scripts/start_mavproxy.sh` `--out` list | `sudo systemctl restart crsd-mavproxy`, then `ps -eo cmd \| grep [m]avproxy` — the flags are built inside the script, so `systemctl show -p ExecStart` will not show them. Every consumer needs its own port: a `udpin` bind steals datagrams and the loser sees silence, not an error |
| the LiDAR extrinsic in `crusader_params.yaml` | re-run [G2](../docs/G2_lidar_orientation.md); every map position derives from those signs |
