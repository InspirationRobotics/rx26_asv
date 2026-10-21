# `tools/` — one blessed path per operation

Everything here exists so that "how a person does X" has exactly one answer. A second
way to do the same operation is how two machines end up configured differently.

| Dir | Contents | Design choice |
|---|---|---|
| `udev/` | `99-crusader.rules` (VID/PID → stable `/dev/crsd-*` symlinks + permissions), `install_udev.sh` | Device identity resolved by the kernel at plug time, so nothing chases `ttyACM` numbering. Matching is by VID/PID, not USB port chain: it survives recabling, and the two devices aboard are distinguishable. **Only hardware actually on the boat gets a rule** — a symlink for absent hardware makes a config look satisfied while pointing at whatever else enumerated. |
| `systemd/` | `crsd-mavproxy.service`, `crsd-container.service`, `crsd-ros.service`, `crsd-livox.service`, `crsd-power.service` (plus `crsd-battwatch.service`, kept but **not installed** since 2026-09-05) (templates; `__PLACEHOLDERS__` expanded by `setup/install_jetson_host.sh`) | The boot chain is ordered so MAVProxy owns the Pixhawk before any node starts. A power cycle brings the boat up with no typed commands. The LiDAR branch is deliberately **not** ordered behind MAVProxy — they share no device, and coupling them would make a LiDAR fault look like an autopilot fault. `crsd-power` is ordered behind nothing at all: it is what shuts the machine down cleanly, which is exactly when a fault elsewhere must not have taken it away. `crsd-battwatch` is no longer enabled: it powered the Jetson off on a threshold derived from a disputed `BATT_VOLT_MULT`, which is a coin flip rather than a safety feature. Nothing protects the battery automatically now. |
| `scripts/` | `check_config.py` (static config guards), `check_entry_points.py` (every console script imports), `param_guard.py` (PROTECTED vs TUNABLE param diff), `preflight.py` (do-not-arm gate), `dump_params.py` (full param pull in QGC format), `rebuild.sh` (the one blessed rebuild), `crsd_power_helper.py` (the host's shutdown/reboot socket), `batt_watchdog.py` (voltage **monitor** — its automatic poweroff was removed 2026-09-05; run it with `--dry-run`) | Fail loudly and early: a check that cannot fail is worse than no check. The power helper is the exception to "scripts are unprivileged": it runs as root because a container cannot power off its host, and it is deliberately tiny — one socket, two verbs, and **nothing a client sends ever reaches a command**. `batt_watchdog.py` stayed unprivileged by going *through* that socket rather than becoming a second root program; it is now run by hand as a monitor, not as a service. |
| `oak_record.py` | CV training data straight off an OAK-D LR (all three cameras, synced, sampled down from the deployment frame rate) | A bench tool that owns the camera while it runs: stop `camera_node` first |
| `sitl/` | A headless ArduRover 4.6.3 that ROS cannot tell from the Pixhawk, and its checks. See its README | No Gazebo: the autopilot only, for telemetry_bridge and the GUIDED paths |
| `squirt_cal/` | Task 3's fixed-nozzle calibration: fire a short burst, tell it (on a phone) whether the boat was too far forward/back/left/right, and it brackets the range that hits each window. `--fake` runs it against a simulated boat with no ROS. See its README | A bench tool, not a launch-file node: it has no entry point, and it cannot hold the pump on — it only asks `telemetry_bridge` for bursts the autopilot itself ends. Logs every shot against `/crsd/wall_range`, the number the tree will fire by |

The camera and LiDAR pictures are the ground station's Sensors tab: `gcs_bridge` sends them
over its one TCP link (14559) only while the tab is open, and the laptop's `gcs_app` serves
the page on http://localhost:8090. The only other web page on the boat is squirt_cal's
(8094, for a phone).

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
| `scripts/crsd_power_helper.py` | `sudo bash setup/install_jetson_host.sh`, `systemctl status crsd-power`, then confirm a bad verb is REFUSED before testing a good one. Needs safety review: it is the only root-privileged thing the ground station can reach |
| `scripts/batt_watchdog.py` or any `CRSD_BATT_*` threshold, or RE-ENABLING `crsd-battwatch` | `python3 tools/scripts/batt_watchdog.py --dry-run` against the real pack FIRST — it logs what it would do and never contacts the power helper. Re-enabling the service additionally needs `BATT_VOLT_MULT` and the cell count settled (both are open), because a threshold set for the wrong pack powers the boat off on a healthy battery. Needs safety review |
| `scripts/start_mavproxy.sh` `--out` list | `sudo systemctl restart crsd-mavproxy`, then `ps -eo cmd \| grep [m]avproxy` — the flags are built inside the script, so `systemctl show -p ExecStart` will not show them. Every consumer needs its own port: a `udpin` bind steals datagrams and the loser sees silence, not an error |
| the LiDAR extrinsic in `crusader_params.yaml` | re-run [G2](../docs/G2_lidar_orientation.md); every map position derives from those signs |
