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
| Camera | whether `buoy_detector`/`oak_view` is up on :8080 | **show/stop the stream** (off by default), start whichever is missing |
| LiDAR | whether `lidar_view` is up on :8081 | **show/stop the stream**, start it |
| Map | vessel, wake, `crsd/world_targets`, and optionally raw clusters and the PRX1 sectors | pan, zoom, follow, **bow-up**, **layer toggles**, clear trail |
| Tuning | any running node's parameters, with its own descriptions and ranges | set a dynamic value live; revert one to the YAML |
| Record | sessions on disk, sizes, live capture state, **live bag growth rate** | tick topics, start/stop a session **with a rosbag**, download a `.tar.gz`, delete |
| Logs | every node's `/rosout` output, filterable by level and node | clear the buffer |
| System | CPU, temperature, memory, disk, uptime | shut down / reboot the host |

## Day mode

The **&#9728; day** button in the top-right swaps to a high-contrast palette and
remembers the choice per browser. It is not "the light theme" — it is a *sunlight*
theme, for a laptop on a dock in Singapore at midday, which is a different design
problem. Text goes to near-black on near-white, borders are dark enough to survive
glare, and the status colours are **darkened rather than lightened**: the night
palette's `#5fbf6a` green is pleasant on `#111` and invisible on white in sun, which
would make the one thing worth seeing at a glance — is it OK or not — the first thing
to disappear.

Every colour on the page is a CSS variable, including button faces and field
backgrounds, because the map is a `<canvas>` and cannot inherit CSS: it reads the same
variables back out of the computed style at theme-change time. A hardcoded hex anywhere
is therefore a bug that shows up as one element staying dark in daylight. The attribute
is set by a tiny inline script in `<head>`, before the body paints, so there is no frame
of the dark theme on load — one frame of an unreadable screen is exactly what this mode
exists to prevent.

## The viewer streams are opt-in

Tearing the iframe down on tab switch was already here and was not enough. The expensive
case is the operator who *wants* the Camera tab open — to see whether the detector is
up, to reach the Start buttons — and gets a live MJPEG stream with it. That stream is
640&times;400 JPEG at quality 60, 30 fps: **order 1 MB/s, against roughly 0.5 Mbps for
everything else this page sends per client.** Opening the tab cost more than the whole
rest of the dashboard by more than an order of magnitude.

So the tab shows the viewer's *status* for free and streams only when asked, and stops
again when the **browser tab goes to the background** — a dashboard on a second monitor
or behind a chart window was streaming the whole time. Backgrounding suspends it without
clearing the choice, so coming back does not need another click. `mjpeg_server` counts
clients and skips encoding when nobody is attached, so an un-started stream costs the
Jetson nothing either: the saving is on both ends of the link.

## The map's three layers, and why they are not the same thing

| Layer | Topic | What it is |
|---|---|---|
| targets | `crsd/world_targets` | what the tracker **believes**, after fusion, association and decay |
| clusters | `crsd/lidar_clusters` | what the LiDAR actually **returned** this window, before any of that |
| PRX1 | `crsd/obstacle_distance` | what the autopilot was **told** — the same 72 sectors `telemetry_bridge` forwards as MAVLink `OBSTACLE_DISTANCE` |

Laying the three over one another turns "avoidance is behaving oddly" into a question
with an answer:

- PRX1 here matches QGC's proximity view but the clusters under it do not → the bug is
  in `proximity_bridge`'s sector maths.
- PRX1 here and QGC disagree → the fault is between this ROS graph and the flight
  controller.
- Clusters agree with both but the targets are elsewhere → it is the tracker.

`UINT16_MAX` sectors are **not drawn**, ever. `ObstacleDistance.msg` is emphatic that
"not seen" and "seen and clear" are different facts, and a ring of max-range arcs would
render the whole unseen aft sector as a wall.

**Bow-up** rotates the map so the bow points up, which is the frame PRX1 is drawn in, so
the two can be read side by side without arithmetic. It is implemented by rotating the
*coordinate mapping*, not the canvas — a canvas rotation carries the text with it, and a
bow-up map whose labels are sideways is unreadable at exactly the moment you are using it
to read a range off an obstacle. The north arrow turns with the view, because in bow-up it
is the only thing left telling you which way north is.

Both extra layers are **off by default and ride in the query string** (`/state?layers=…`)
rather than a server-side preference: with a shoreline in view the cluster layer is a few
KB per poll, and two laptops can have the page open with different layers on. Clusters
carry both their body-frame and world-frame positions, computed server-side through
`geo.body_to_world_ypr`, because that is the repo's one implementation of that transform
and a JavaScript copy would be a second one that drifts.

## Recording: telemetry, frames, and a real rosbag

A session is one directory holding three things, and they are not redundant:

- `telemetry.jsonl` — the one that still parses after a power cut mid-line.
- `camera/`, `lidar/` — MJPEG frames pulled from whichever viewer is up. **The bag cannot
  replace these**: `buoy_detector` publishes no image topic unless `publish_frames` is on,
  and that is off because it costs 38 MB/s on the DDS bus. These are the only record of
  what the operator was actually looking at.
- `bag/` — a real rosbag2 of whichever topics were ticked. The replayable one, and the only
  one that can carry the point cloud.

The topic list is **the live ROS graph**, not a curated set: a recording is worth making
because something unexpected happened, and the curated list is the judgement that turns
out to be wrong on the day. Presets tick everything except `PointCloud2`/`Image` (matched
on **type**, never on topic name, so a renamed or second sensor is still caught).

`ros2 bag record` runs as a **separate process**. `rosbag2_py` is available and would be
fewer moving parts, but it would put `/livox/lidar` — a few MB a second — through this
node's single-threaded executor, and the dashboard would stall exactly when a recording is
running, which is when nobody can afford to restart it.

**It is stopped with SIGINT, not SIGTERM**, and that is why it is not a second caller of
`process_manager`. rosbag2 finalises the sqlite database and writes `metadata.yaml` in its
SIGINT handler; a bag without `metadata.yaml` will not open, and `ros2 bag info` and
`ros2 bag play` both refuse it. The recording looks fine right up until somebody needs it.
If it has to be escalated to SIGKILL the tab says so, in those words.

**The size problem is not solved by a warning.** With 38 GB free and `/livox/lidar` ticked,
the disk fills in under three hours. So the tab does not estimate — it measures the bag
directory as it grows and shows MB/s and hours-remaining at the current rate, and shows a
blank until there are two samples far enough apart to divide. A reassuring number computed
from no data is the thing this repo keeps designing out.

## The Tuning tab

Every knob that matters is discovered on the water, and the alternative to this tab is an
SSH session on the same laptop that is already showing the map.

**It knows nothing about any parameter.** Rows are built from the target node's own
`ParameterDescriptor`s, which already carry the description, the numeric range and the
`read_only` posture because `crusader_common.param_utils.declare` puts them there. A knob
appears in the list because a node declares it, never because this package was edited to
match — which is also why the node selector is the live ROS graph rather than the registry:
anything running can be tuned, including something started by hand in another terminal.

**Read-only parameters are listed, greyed, with the reason.** A knob you cannot turn and a
knob you cannot see are different problems, and the second one sends someone looking for it
in the wrong file.

**It validates nothing.** The page's `min`/`max` are hints from the descriptor; the refusal
comes from the node's own set-callback, in the node's own words — `assoc_radius_m=99
outside [0.2, 20.0]` is `check_range`'s message, not this package's. Two validators
disagreeing is worse than one, and a second copy of the bounds here is a copy that goes
stale.

**Nothing persists.** A set lands in the running node and dies with it. `crusader_params.yaml`
stays the source of truth, and a browser POST has no business rewriting a file whose
comments are most of its value. So the tab shows the live value against the YAML default and
marks the drift — that marker *is* the product of a tuning session: it is the list of lines
to write back into the file before the next run. Every set is also logged to `/rosout`, so a
value changed from a browser and nowhere else is still findable afterwards.

Costs nothing when unopened: values are fetched by `POST /params/list` on demand, never in
`/state`, so the poll budget below is unchanged, and the service clients are created on
first use.

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

## A viewer tab waits for the PORT, not the process

`buoy_detector` appears in the process table within a second of being started, then spends
another ten to thirty loading a TensorRT engine and opening the OAK-D before its stream
server binds. Three states, not two:

| Server says | Tab shows |
|---|---|
| not running | "Camera viewer not running" + Start buttons |
| running, port closed | "buoy_detector is starting…" |
| running, port open | the live view |

The middle one exists because without it the page pointed an iframe at a socket nothing was
listening on yet, got connection-refused, and — since it will not reload a stream it thinks
is already correct — **stayed on that error page permanently**, while opening the same URL
by hand worked fine because that load happened after the port came up.

So the server withholds `source` until a TCP connect to the port actually succeeds, probed
once per graph tick rather than per browser poll. The recorder uses the same signal: it will
not try to pull frames from a viewer that is not serving yet.

The rebuild guard keys on the whole rendered **state**, not just the URL. Two of the three
states have no URL, so keying on the URL alone left the tab showing Start buttons after the
node had already started.

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
