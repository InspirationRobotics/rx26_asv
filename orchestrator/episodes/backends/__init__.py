"""Episode backends.

Contract (duck-typed, both backends implement it):
    reset(scenario, seed) -> None       # place vehicle at first waypoint's approach
    set_target(x, y) -> None            # commanded goal in scenario world frame
    step(dt) -> None                    # advance sim time (SITL: sleep-real-time)
    state() -> (x, y, heading_rad, speed_mps)
    shutdown() -> None                  # deterministic teardown, idempotent
"""
