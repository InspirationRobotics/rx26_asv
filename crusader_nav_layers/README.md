# crusader_nav_layers

`crusader_nav_layers::HazardLayer`: a Nav2 `costmap_2d` layer that draws the BT's **known**
hazards LETHAL. Spec: `docs/nav2_avoidance_spec.md` section 3.3.

bt_runner publishes the *whole* set on `/crsd/nav/hazards` (`crusader_msgs/HazardArray`,
reliable, transient_local, depth 1, 2 Hz). Each message replaces the previous one, so a hazard
that goes away disappears with no delete message. Circles and convex polygons are drawn at their
**physical** size plus `keepout_m`; the inflation layer adds the 0.8 m hard / 2.0 m soft band.

## Behaviour

* **Stateless.** It keeps the last message and redraws whatever falls inside the bounds each
  cycle. The footprint of a replaced set is re-dirtied so its cells are cleared.
* **Current only while fed.** Not current before the first message, or when the last one is
  older than `max_age_s` (5.0, receipt time on the node clock): a dead bt_runner makes the
  costmap non-current and plans hang instead of planning around hazards that may have moved.
* **Frame must be the costmap's global frame** (`map`). Anything else is refused with an ERROR
  (throttled) and the layer goes non-current.
* **Malformed hazards are left out, loudly** (non-finite numbers, negative radius or keepout,
  fewer than 3 polygon vertices, ragged polygon arrays): "a hazard nobody can place is not drawn
  somewhere plausible".
* `reset()` (which `clear_entirely` calls on every layer) keeps the set and redraws it.
  `isClearable()` is false.
* **Conservative raster** (`include/crusader_nav_layers/raster.hpp`, pure): a cell is marked when
  its centre is within `r + 0.7072 * resolution` of the surface, or when it contains the circle
  centre. It never undershoots; it can overshoot by up to 0.07 m at 0.1 m cells, which is the
  spec's "0.80 m, -0.07 / +0.07 m".

Parameters (under the layer name in `nav2_params.yaml`): `enabled` (true), `topic`
(`/crsd/nav/hazards`, must equal `bt_runner_node.nav_hazards_topic`; `check_config` pins it),
`max_age_s` (5.0).

## Building without Nav2

`CMakeLists.txt` looks for `nav2_costmap_2d`, `pluginlib`, `rclcpp` and `crusader_msgs` with
`QUIET`. If they are missing it prints a CMake warning and installs nothing, so the boat's
`rebuild.sh` still succeeds in an image without Nav2. `test_raster` always builds.

## Test

    g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include -o /tmp/t test/test_raster.cpp && /tmp/t
