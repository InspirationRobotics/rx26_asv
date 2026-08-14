# `crusader_msgs` — ROS 2 message package

The typed contracts between nodes. Three messages, all produced by `telemetry_bridge`
from MAVProxy's rebroadcast and consumed by everything else.

## Messages

| Message | Producer → consumer | Notes |
|---|---|---|
| `LatLonHead` | `telemetry_bridge` → any consumer | lat/lon/heading + `ground_speed`, taken from `GLOBAL_POSITION_INT`'s own vx/vy so nobody has to finite-difference position. `heading` is NaN when GPS yaw is unresolved — that is a value to check, not to smooth over |
| `FcuStatus` | `telemetry_bridge` → LED status, watchdog | mode string, armed flag, system status, from `HEARTBEAT` |
| `RcChannels` | `telemetry_bridge` → LED status, watchdog; and any override publisher → `telemetry_bridge` | 18 raw PWM values. Also the TX direction: an override publisher fills channels 1–8 |

Every message carries a `std_msgs/Header`. **The stamp is the time the MAVLink frame was
RECEIVED**, not the time it was republished — a consumer judging freshness needs the age
the data actually has.

## Design choice

Typed ROS messages, drift caught at build time, rather than the legacy socket-and-dict
approach. The cost is that a producer/consumer pair must be rebuilt together; `colcon
build --packages-up-to crusader_bringup` rebuilds everything downstream of a message
change, which is why the blessed rebuild path uses that form.

## Adding a message

Add the `.msg` file **and** the entry in `CMakeLists.txt` — a file that is not listed is
silently not generated, and the import failure shows up at node start on the boat rather
than at build time.

Messages for capabilities this repo does not have are not kept "for later": the v0.5 strip
removed seven of them (detections, occupancy grid, APF advisory, guided setpoint) along
with the nodes that used them; they are in git at `8c4ffa5`. Re-add one when the node that
fills it is landing.

This package is also the **contract with the sensor container**: it publishes against
these definitions, so keep the dependency list at `std_msgs` and nothing more — anything
heavier makes it expensive for another image to build.

## Change impact

| You changed | Re-run |
|---|---|
| any `.msg` field | full `colcon build` of both packages (`tools/scripts/rebuild.sh`), then restart every node — a mismatched message is a silent deserialization failure |
| `LatLonHead.ground_speed` | `telemetry_bridge` is its only producer — check it still populates the field |
