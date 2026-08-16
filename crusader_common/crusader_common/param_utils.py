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
    try:
        node.declare_parameter(name, default, d)
    except Exception as e:
        # rcl reports "out of range Min: 1.0, Max: 3600.0, value: 0.0" and
        # stops there. That is true and nearly useless: the VALUE came from the
        # YAML and the RANGE came from this node's code, so the two disagreeing
        # is not a bad parameter, it is two halves of the build that are not
        # the same age. Same treatment config.py gives a missing params file.
        raise ValueError(range_conflict_message(name, default, lo, hi, e)) from e
    return node.get_parameter(name).value


def range_conflict_message(name, value, lo, hi, error) -> str:
    """Explain a declaration failure in terms of what a person can act on.

    Pure and importable without ROS so the wording can be checked off-boat —
    the whole point is that this text is read exactly once, on a boat, by
    someone who wants to know what to type next.
    """
    lines = [f"could not declare parameter {name!r} = {value!r}", f"  {error}"]
    if lo is not None and hi is not None and check_range(name, value,
                                                         {name: (lo, hi)}):
        lines += [
            f"  This node's code declares {name} valid over [{lo}, {hi}], and "
            f"the params file supplied {value!r}. Those come from two "
            "different files:",
            "    value:  crusader_bringup/config/crusader_params.yaml",
            "    range:  this node's PARAM_SPEC",
            "  They can only disagree if one is newer than the other — almost "
            "always a STALE INSTALL SPACE, where the YAML was picked up and "
            "the node code was not (or the reverse).",
            "  Fix: tools/scripts/rebuild.sh, then run the node again.",
            "  If a rebuild does not clear it, the YAML value really is out of "
            "range: change the value, or widen the range in PARAM_SPEC.",
        ]
        try:
            from crusader_common import config as _config
            lines.append(f"  params file in use: {_config.DEFAULT_CONFIG_PATH}")
        except Exception:
            pass
    return "\n".join(lines)


def declare_from_config(node, defaults, spec):
    """Declare a node's parameters from the shared config defaults.

    defaults: {name: value} (crusader_common.config.node_params output)
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
