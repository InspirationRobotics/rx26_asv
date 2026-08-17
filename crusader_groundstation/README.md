# `crusader_groundstation` — the whole boat in one browser tab

```bash
ros2 run crusader_groundstation ground_station
# then, on the laptop:  http://<JETSON_IP>:8090
```

**State: one node, not proven on the boat.** It starts and stops real nodes and can power
the Jetson off, and none of that has been exercised on the water. It carries a `NOTE:`
header and is not in `core.launch.py`.

| Tab | Shows | Can do |
|---|---|---|
| Nodes | every node in the registry, running or not, from the ROS graph | start anything, stop what it started, run a profile |
| Telemetry | lat/lon, speed, heading, roll/pitch/yaw, mode, armed | — |
| Camera | `buoy_detector`'s annotated view or `oak_view`, on :8080 | start whichever is missing |
| LiDAR | `lidar_view`'s plan/elevation, on :8081 | start it |
| Map | vessel, wake, and `crsd/world_targets` on a north-up plan | pan, zoom, follow, clear trail |
| Record | sessions on disk, sizes, live capture state | start/stop a session, download a `.tar.gz`, delete |
| Logs | every node's `/rosout` output, filterable by level and node | clear the buffer |
| System | CPU, temperature, memory, disk, uptime | shut down / reboot the host |

## Why this is its own package

It is the one package allowed to **know about** every other package — it names their nodes
in order to launch them — while **importing** none of them. It depends on `rclpy`,
`crusader_msgs` and `crusader_common` and nothing else, which is what keeps it off the
dependency graph of the code it starts.

`map_server` used to live in `crusader_world_model` and is now tab 5. A display of world
state belongs with the operator's other controls rather than inside the package that
computes it, and moving it puts that package back to pure geometry with no HTTP server
bolted on. It is not `crusader_bringup` either: that package ships no code by charter.

## The two rules it holds

**Protected nodes cannot be stopped from here.** The repo's second standing safety
constraint says the RC e-stop is the only safety path and WiFi is never a safety mechanism.
A web page with a stop button next to `rc_watchdog` inverts that: anyone who can reach the
Jetson could switch off the RC-loss force-disarm, from a laptop, silently. So the core stack
shows status and can be *started* — that can only move the boat toward safe — and nothing
served over HTTP can take it down.

**Power is locked while armed**, then requires the hostname typed. An *unknown* armed state
locks it too: a missing `FcuStatus` usually means the bridge is down, which is not evidence
the boat is safe to reboot.

Both are re-checked **server-side on every request**, not just rendered as disabled controls.
Anyone can edit JavaScript in a browser or `curl` the endpoint, so a rule enforced only in
the page is decoration. Verified by calling both endpoints directly, bypassing the UI:

```
POST /node/stop {"name":"rc_heartbeat_watchdog"}  ->  refused, with the reason
POST /power {"verb":"reboot","confirm":"crusader-asv"}  ->  refused: vehicle is ARMED
```

## Shutdown needs a host-side helper

**A container cannot power off the Jetson.** It shares the host's kernel but has no init to
ask, so `shutdown` inside `asv` either fails or does something worse than nothing. The usual
workaround — `--privileged` with the host PID namespace — buys one button by giving every
process in that container root on the Jetson, permanently, including the inference stack and
the MAVLink bridge.

So there is a small helper on the **host** instead:

```
tools/scripts/crsd_power_helper.py    root, systemd, outside every container
tools/systemd/crsd-power.service      the unit
```

It listens on a Unix socket, accepts the words `shutdown` and `reboot`, and refuses
everything else. **Nothing a client sends is ever interpolated into a command** — the verb
selects one of two fixed argv lists compiled into the file. Read that module before changing
it: the moment a client-supplied string reaches a subprocess, it stops being a helper and
becomes remote root.

The socket must be bind-mounted into `asv`. If it is missing, the page says so and disables
the buttons with that reason attached, rather than failing silently — someone will otherwise
press it twice, conclude the boat is wedged, and go pull the battery.

`allow_power: false` in the params turns the whole tab off.

## Presence is global, control is local

A node started by `core.launch.py`, by systemd, or by hand in another terminal **shows as
running** — from the ROS graph *and* from `/proc` (below), because nobody wants a dashboard
reporting the telemetry bridge is down just because it did not personally start it.

Stopping used to be limited to processes this server spawned, on the grounds that killing
anything else meant guessing a PID from a node name. `/proc` removed the guess, so that
limit is gone — but the **signalling differs**, and that distinction now carries the weight
the old restriction did. See the `/proc` section below.

For our own children: SIGTERM to the process **group**, then SIGKILL after 5 s. The group, because
`ros2 run` execs the node as a child and signalling only the parent leaves the node alive
and orphaned — still holding the camera. SIGTERM first, because every node here routes it
through `crusader_common.node_main` into a clean `destroy_node()`; going straight to SIGKILL
would leave `telemetry_bridge`'s RC overrides latched in the autopilot.

**Closing the dashboard does not stop the boat.** Children are spawned in their own session,
and `destroy_node` shuts down only the web server. A ground station that killed the stack on
exit is one nobody would dare restart mid-session.

## The camera pair contends for one device

The OAK-D admits one client, so `buoy_detector` and `oakd_publisher` cannot both run — the
second to start fails with a depthai error that does not say "the camera is busy". Nodes
that contend share an `exclusive` tag, and starting one offers to stop the incumbent instead
of letting the operator discover the conflict from a traceback. **Profiles never resolve a
conflict on their own**: silently stopping the camera node someone deliberately chose is the
kind of helpfulness that loses a run.

## Discovery: the graph *and* `/proc`

Borrowed from the team's UUV ground station (`robotx_graey_2026`), which scans `/proc` for
its own executables rather than asking ROS — instant, no `ros2 daemon`, and it sees whatever
started the process. We run **both**, because they see different worlds:

- the **ROS graph** sees nodes across the DDS domain, including other containers. The livox
  driver is a node we can observe and never a process we can find.
- **`/proc`** sees things that are not ROS nodes at all. `tools/oak_view.py` and
  `tools/lidar_view.py` are plain scripts; before this, one started from a terminal showed
  as stopped and the page offered a Start button that would have collided on its port.

Matching is on whole path components, copied from their implementation — `led_node` must not
match `pixhawk_led_status_node`. A substring test looks right, passes every casual check, and
then reports the wrong node as running on the day it matters.

Having real PIDs also removed the reason we could not stop foreign processes. **But a
foreign process is signalled by PID, never by process group**: our own children get their own
session, while a node started by `core.launch.py` shares its group with the entire launch —
signalling that group to stop one node would take down the whole core stack.

## Logs: `/rosout`, not journalctl

The clearest gap when comparing against Graey's GUI: once systemd or a launch file owns the
nodes, their output goes somewhere the operator is not, and a dashboard you must leave for a
terminal is not doing its job.

Their answer is a journalctl tab. Ours is `/rosout`, because we are **inside a container** —
`journalctl -u crsd-container` from in here reads nothing, and mounting `/var/log/journal`
would be a privilege grant made to read a log file. `/rosout` carries every rclpy node's
logger output tagged with node and severity, crosses the DDS domain so it sees other
containers too, and needs no privilege at all.

What it misses: output written straight to stdout rather than through the ROS logger, and
anything printed before a node finished constructing — which is exactly when a bad parameter
kills it. That half is covered by the per-process tail for children we started. The remaining
blind spot is named in the tab rather than papered over.

The tab reads **incrementally**: the page sends the newest sequence number it holds and gets
only what is new. Resending the whole ring at the poll rate would cost more than every other
tab combined.

## Recording

Three streams into one timestamped directory under `record_dir`:

```
telemetry.jsonl   pose, attitude, autopilot state and every tracked target
camera/*.jpg      frames pulled from whichever camera viewer is running
lidar/*.jpg       the same, from the LiDAR viewer
```

**Frames are pulled from the viewers, not captured again.** `buoy_detector` and `lidar_view`
already encode JPEG for the browser, and `mjpeg_server` already counts clients. Subscribing
to the image topics and encoding our own would double the encode cost on a Jetson already
running inference, and would produce frames that are not what the operator was looking at.
It also means recording only works while a viewer is up — which is honest: there is nothing
to record from a camera nobody turned on.

**Recording happens on the boat; the download is separate and explicit.** Writing frames
across the WiFi link would put ~18 Mbps on the air for the whole run and lose the recording
whenever the link dropped — which is when you most want to know what the boat saw. Local
disk is the only medium that survives the link going away.

`telemetry.jsonl` is line-delimited so a run that ends in a crash still parses up to the last
complete line, and it is flushed every sample: a session that ends with the boat losing power
is exactly the session a recording exists for. Staleness is recorded as a **field**
(`"pose_ok": false`), never as a reason to skip a sample — a gap in the file is
indistinguishable from a dead recorder.

### Recordings must outlive the container

`record_dir` defaults to `/root/robotx_ws/logs/sessions`, which on this fleet is inside the
**already bind-mounted workspace** — the host's `~/robotx_ws`. That is why `git pull` on the
host followed by `rebuild.sh` in the container works, and it is why recordings land on the
Jetson's real filesystem and survive `docker rm`.

The node does not assume it. At startup it reads `/proc/self/mountinfo`, finds the mount
`record_dir` actually sits on, and says which:

```
recordings persist: /root/robotx_ws/logs/sessions is on /root/robotx_ws
  (from /dev/nvme0n1p2) — they survive the container being recreated
```

and if not, loudly:

```
RECORDINGS WILL NOT SURVIVE: ... is on the container's own filesystem, so
  `docker rm` discards every session.
```

The Record tab shows the same thing. Hardcoding "anything under `/root/robotx_ws` is a bind
mount" would be true here today and silently wrong on the first machine set up differently —
wrong in the direction that loses data.

**Mounts cannot be added by the systemd unit.** `crsd-container.service` runs `docker start`,
and mounts are fixed when a container is *created*. `setup/install_jetson_host.sh` therefore
only **checks** them and prints the exact `docker run` line if either is missing; recreating
the container is your call, because `docker rm` throws away anything living only inside it.
Two mounts matter:

```
-v ~/robotx_ws:/root/robotx_ws              recordings persist (and the workspace is shared)
-v /run/crsd-power.sock:/run/crsd-power.sock  the System tab can power the host down
```

**The disk guard is not optional.** Frames fill a Jetson faster than anyone expects, and a
full root filesystem takes down the whole stack, not just the recording. Recording refuses to
start below `record_min_free_gb` and stops itself, loudly, if it crosses it mid-run.

Session names are generated, and validated twice on the way back in — a name regex *and* a
resolved-path containment check — because this is user input that becomes a filesystem path.
Verified that `../../etc`, `..`, `a/b` and over-long names are all refused.

## Bandwidth

`/state` at the 5 Hz default is roughly **0.5 Mbps** per browser with a saturated trail —
the measurements are in [crusader_world_model](../crusader_world_model/README.md), and the
payload is the same shape plus the node list.

The camera tab is a different order of magnitude: an open MJPEG stream is **10–25 Mbps**.
`mjpeg_server` counts clients and skips rendering when nobody is attached, so the tab costs
nothing until opened — but the page drops the `<img>` when you switch away rather than
merely hiding it, because a hidden `<img>` keeps its connection open and would keep the
Jetson encoding for a tab nobody is looking at.

## Ports

| Port | Served by |
|---|---|
| 8080 | `buoy_detector`'s annotated view, or `tools/oak_view.py` |
| 8081 | `tools/lidar_view.py` |
| 8090 | this |

`check_config.py` pins them distinct — two servers cannot bind one socket, and the loser
dies with an address-in-use that reads like a crash.

## Change impact

| You changed | Re-run |
|---|---|
| `node_registry.py` | it is pure — exercise it directly, then confirm the Nodes tab still groups correctly |
| a protection or exclusion rule | call the endpoint with `curl`, not the button; the page is not where the rule lives |
| `gcs_page.py` | open every tab, and check the **stale** paths: pull `/crsd/pose` and confirm the banner fires |
| `crsd_power_helper.py` | `sudo bash setup/install_jetson_host.sh`, `systemctl status crsd-power`, then confirm a bad verb is refused before testing a good one |
| ports in `crusader_params.yaml` | `python3 tools/scripts/check_config.py` |
| `system_info.py` | it must return `None`, never raise — one reader that throws blanks every tab, which is how the `os.statvfs` case was found |
| `recorder.py` | record a short session against a running viewer and check the frame COUNT, not just that files appeared — `HTTPResponse.read(n)` blocks for a full buffer and silently cost 9 frames in 10 |
| `proc_scan.py` | it is pure — check a name that is a prefix of another still does not match |
