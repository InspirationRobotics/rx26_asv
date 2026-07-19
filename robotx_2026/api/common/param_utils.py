"""Parameter declaration/validation helpers (Phase 3.5).

Every parameter gets an explicit posture:
  * read_only=True  -> `ros2 param set` is REJECTED by rclpy with an error.
    For safety/structural params: the change path is config YAML + node restart.
  * read_only=False -> the node MUST install a set-callback (make_set_callback)
    so runtime sets are range-validated and actually APPLIED. A declared-but-
    ignored parameter (the pre-3.5 state) is the worst posture: `param set`
    succeeds silently and changes nothing.

check_range() is pure and unit-tested without rclpy; the rcl_interfaces imports
are function-local so this module imports anywhere.
"""


def check_range(name, value, ranges):
    """ranges: {param_name: (lo, hi)} inclusive. Returns error str or None.
    Pure function — shared by the ROS callback and local tests."""
    if name not in ranges:
        return None
    lo, hi = ranges[name]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return f"{name}: expected numeric, got {type(value).__name__}"
    if not (lo <= value <= hi):
        return f"{name}={value} outside [{lo}, {hi}]"
    return None


def declare(node, name, default, *, read_only=False, lo=None, hi=None,
            description=""):
    """Declare one parameter with a full descriptor. Returns the resolved value
    (YAML/launch override wins over `default`)."""
    from rcl_interfaces.msg import (FloatingPointRange, IntegerRange,
                                    ParameterDescriptor)
    d = ParameterDescriptor(description=description, read_only=read_only)
    if lo is not None and hi is not None:
        if isinstance(default, float):
            d.floating_point_range = [FloatingPointRange(
                from_value=float(lo), to_value=float(hi), step=0.0)]
        elif isinstance(default, int) and not isinstance(default, bool):
            d.integer_range = [IntegerRange(
                from_value=int(lo), to_value=int(hi), step=0)]
    node.declare_parameter(name, default, d)
    return node.get_parameter(name).value


def declare_from_config(node, defaults, spec):
    """Declare a node's parameters from the shared config defaults.

    defaults: {name: value} (api.common.config.node_params output)
    spec: {name: dict(read_only=..., lo=..., hi=..., description=...)}
          — every name in defaults MUST appear in spec (posture is mandatory).
    Returns {name: resolved_value}.
    """
    missing = set(defaults) - set(spec)
    if missing:
        raise ValueError(f"no declared posture for params: {sorted(missing)}")
    return {name: declare(node, name, defaults[name], **spec[name])
            for name in defaults}


def make_set_callback(node, ranges, apply_fn):
    """Build the on-set-parameters callback for a node's DYNAMIC params.

    ranges: {name: (lo, hi)} for validation (read_only params never reach this).
    apply_fn: called with {name: new_value} AFTER validation; must actually
    apply the values (mutate the core object, etc.). Register with:
        node.add_on_set_parameters_callback(make_set_callback(...))
    """
    from rcl_interfaces.msg import SetParametersResult

    def _cb(params):
        changes = {}
        for p in params:
            err = check_range(p.name, p.value, ranges)
            if err:
                node.get_logger().error(f"param set rejected: {err}")
                return SetParametersResult(successful=False, reason=err)
            changes[p.name] = p.value
        try:
            apply_fn(changes)
        except Exception as e:                 # apply must never half-succeed
            node.get_logger().error(f"param apply failed: {e}")
            return SetParametersResult(successful=False, reason=str(e))
        node.get_logger().info(f"params updated: {sorted(changes)}")
        return SetParametersResult(successful=True)
    return _cb
