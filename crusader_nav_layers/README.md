# crusader_nav_layers

`crusader_nav_layers::HazardLayer`, a Nav2 costmap layer that draws the BT's known hazards
LETHAL (`docs/nav2_avoidance_spec.md` 3.3). `bt_runner_node` publishes the whole set on
`/crsd/nav/hazards` (HazardArray, transient_local, 2 Hz); each message replaces the last, so a
hazard that is gone disappears with no delete. Shapes are drawn at their physical size plus
`keepout_m`; the inflation layer adds the soft band. A set older than `max_age_s` makes the
costmap non-current, so a dead runner holds every planned leg.

Builds without Nav2 (the plugin is skipped, so the boat's image still builds); the rasteriser
(`raster.hpp`) is header-only and `test_raster` always builds.
