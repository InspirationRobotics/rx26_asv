# `crusader_msgs` — ROS 2 message package

The typed contracts between nodes. Four telemetry messages produced by
`telemetry_bridge` from MAVProxy's rebroadcast, plus the perception pair that
`crusader_perception/buoy_detector` fills.

## Messages

| Message | Producer → consumer | Notes |
|---|---|---|
| `LatLonHead` | `telemetry_bridge` → any consumer | lat/lon/heading + `ground_speed`, taken from `GLOBAL_POSITION_INT`'s own vx/vy so nobody has to finite-difference position. `heading` is NaN when GPS yaw is unresolved — that is a value to check, not to smooth over |
| `Attitude` | `telemetry_bridge` → mapping/fusion | roll/pitch/yaw + body rates [rad, rad/s] from `ATTITUDE`, in the **autopilot's** NED axes, not REP-103. Its own topic because `ATTITUDE` and `GLOBAL_POSITION_INT` are separate MAVLink streams that go stale separately. `yaw` is the same EKF estimate `LatLonHead.heading` carries in degrees |
| `FcuStatus` | `telemetry_bridge` → LED status, watchdog | mode string, armed flag, system status, from `HEARTBEAT` |
| `RcChannels` | `telemetry_bridge` → LED status, watchdog; and any override publisher → `telemetry_bridge` | 18 raw PWM values. Also the TX direction: an override publisher fills channels 1–8 |
| `Detection3D` | inside `Detection3DArray` | one object: label, confidence, position, source bbox. Position is REP-103 body axes (x forward, y left, z up) in the frame the array declares — never optical axes |
| `Detection3DArray` | `buoy_detector` → fusion, world model | everything one camera frame saw. **Published every frame, empty or not**: empty means "alive, saw nothing", silence means the producer died |
| `Cluster3D` | inside `Cluster3DArray` | one LiDAR object: centroid, AABB extent, point count, range. **No label and no confidence** — a cluster is a thing that is *there*, which is all the LiDAR can say; naming it is the camera's job |
| `Cluster3DArray` | `lidar_cluster_node` → fusion, world model | one accumulated window, **nearest first** so a consumer that truncates keeps the near ones. Same every-window-empty-or-not contract as `Detection3DArray` |

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

Keep the dependency list at `std_msgs` and nothing more. Nothing outside `asv` builds this
today — the livox container publishes a stock `PointCloud2` — but the moment one needs to,
a heavier dependency list is what makes that expensive, and this package is small enough
that staying cheap costs nothing.

## Change impact

| You changed | Re-run |
|---|---|
| any `.msg` field | full `colcon build` of both packages (`tools/scripts/rebuild.sh`), then restart every node — a mismatched message is a silent deserialization failure |
| `LatLonHead.ground_speed` | `telemetry_bridge` is its only producer — check it still populates the field |
| `Attitude` axes or units | `crusader_common/geo.py:body_to_world_ypr` is the only thing that interprets them; re-check its FRD conversion, then anything mapping detections |
