# `tools/` — one blessed path per operation

Everything here exists so that "how a person does X" has exactly one answer. A second
way to do the same operation is how two machines end up configured differently.

| Dir | Contents | Design choice |
|---|---|---|
| `udev/` | `99-crusader.rules` (VID/PID → stable `/dev/crsd-*` symlinks + permissions), `install_udev.sh` | Device identity resolved by the kernel at plug time, so nothing chases `ttyACM` numbering. Matching is by VID/PID, not USB port chain: it survives recabling, and the two devices aboard are distinguishable. **Only hardware actually on the boat gets a rule** — a symlink for absent hardware makes a config look satisfied while pointing at whatever else enumerated. |
| `systemd/` | `crsd-mavproxy.service`, `crsd-container.service` (templates; `__PLACEHOLDERS__` expanded by `setup/install_jetson_host.sh`) | The boot chain is ordered so MAVProxy owns the Pixhawk before any node starts. A power cycle brings the boat up with no typed commands. |
| `scripts/` | `check_config.py` (static config guards), `param_guard.py` (PROTECTED vs TUNABLE param diff), `preflight.py` (do-not-arm gate), `rebuild.sh` (the one blessed rebuild) | Fail loudly and early: a check that cannot fail is worse than no check. |
| `oak_view.py` | Subscribes to the camera container's ROS topic and re-serves it as MJPEG to a laptop browser | A viewer, not a driver. It never opens the OAK-D, so it cannot take the camera away from perception, and any number can run at once. Runs anywhere with ROS on the path, `asv` included. |

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
| `scripts/param_guard.py` PROTECTED list | `python3 tools/scripts/check_config.py`; a change here needs safety review — it is the list of things nobody may quietly retune |
| `params/working_crusader.params` | `python3 tools/scripts/check_config.py`, then a preflight run against the live boat |
| `scripts/preflight.py` | run it on the Jetson host and confirm no check silently SKIPs |
