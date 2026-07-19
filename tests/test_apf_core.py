import math

from robotx_2026.api.navigation.apf_core import (
    ApfParams, MovingHazard, ObstaclePoint, compute)


def test_clear_path_heads_to_goal_full_speed():
    a = compute(0, 0, 0, 50)
    assert a.repulsion == 0 and a.speed_scale == 1.0 and not a.potential_trap
    assert abs(a.goal_x) < 1e-9 and 0 < a.goal_y <= 50


def test_side_obstacle_pushes_away():
    # obstacle to starboard (east) of the path -> corrected goal biased west
    a = compute(0, 0, 0, 50, obstacles=[ObstaclePoint(1.5, 5, 0.3)])
    assert a.repulsion > 0
    assert a.goal_x < 0


def test_head_on_obstacle_triggers_trap_and_tangent():
    # dead ahead: attraction north, repulsion south -> trap; tangent must
    # produce a lateral corrected goal, not a zero vector
    a = compute(0, 0, 0, 50, obstacles=[ObstaclePoint(0, 3, 0.3)])
    assert a.potential_trap
    assert abs(a.goal_x) > 0.5           # deterministic lateral break


def test_speed_scales_down_near_obstacles():
    far = compute(0, 0, 0, 50, obstacles=[ObstaclePoint(0, 30, 0.3)])
    near = compute(0, 0, 0, 50, obstacles=[ObstaclePoint(1.2, 2, 0.3)])
    assert far.speed_scale == 1.0
    assert near.speed_scale < 1.0
    assert near.speed_scale >= ApfParams().min_speed_scale


def test_moving_hazard_repels_from_projection_too():
    # hazard currently far east, moving west fast: its PROJECTED position lands
    # near the path ahead -> repulsion even though current position is far
    m = MovingHazard(x=25, y=20, vx=-4.0, vy=0.0)      # projects to x=5 at 5 s
    a = compute(0, 15, 0, 50, moving=[m])
    assert a.repulsion > 0
    assert "moving" in a.sources
    assert a.goal_x < 0                  # pushed away from the incoming side


def test_sources_reported():
    a = compute(0, 0, 0, 50, obstacles=[
        ObstaclePoint(1.5, 5, 0.3, source="perception"),
        ObstaclePoint(-1.5, 5, 0.3, source="comms")])
    assert a.sources == ["comms", "perception"]


def test_at_goal_no_advisory_motion():
    a = compute(10, 10, 10, 10, obstacles=[ObstaclePoint(11, 11, 0.3)])
    assert (a.goal_x, a.goal_y) == (10, 10)


def test_equilibrium_distance_exceeds_avoid_margin():
    # static analysis of the tuning: the passing distance where repulsion
    # balances attraction must be beyond AVOID_MARGIN (2.0 m) so the advisory
    # keeps the boat outside the violation band without AVOID_* intervening
    p = ApfParams()
    # k_rep*(1/d - 1/influence) = k_att  ->  d = 1/(k_att/k_rep + 1/influence)
    d_eq = 1.0 / (p.k_att / p.k_rep + 1.0 / p.influence_m)
    assert d_eq > 2.0, f"equilibrium {d_eq:.2f} m inside AVOID_MARGIN"
