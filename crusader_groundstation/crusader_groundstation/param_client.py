"""param_client — read and write another node's parameters, from the page.

The Tuning tab exists because the knobs that matter are discovered on the water;
this is `ros2 param set` made from the boat's own dashboard, with the range and
description in front of you.

NOTHING HERE KNOWS WHAT ANY PARAMETER IS. It asks the node. Every Crusader node
declares parameters through crusader_common.param_utils.declare, which puts the
description, read_only posture and numeric range into the ParameterDescriptor,
so a second table here would go stale. A parameter appears in the GUI because
the node declares it, or not at all.

READ-ONLY PARAMETERS ARE LISTED, NOT HIDDEN, greyed out with the posture said
out loud: "cannot turn it from here" is a design decision with a reason (the
camera extrinsic invalidates every target on the map), while "cannot see it"
reads as the parameter not existing.

WHAT IT CANNOT DO: persist anything. A set lands in the running node and dies
with it, because crusader_params.yaml is the commented source of truth that a
browser POST must not rewrite. The page shows the live value against the YAML
default and marks the drift.

THREADING. Called from the HTTP thread while the node spins on the main one.
Every call is `call_async` plus a wait on the future's done-callback, never
spin_until_future_complete (a second executor spinning the same node wedges a
ROS web bridge).
"""
import threading

from crusader_common import param_utils

# The four services every rclpy/rclcpp node exposes under its own name.
_LIST = "list_parameters"
_DESCRIBE = "describe_parameters"
_GET = "get_parameters"
_SET = "set_parameters"

# rcl_interfaces/msg/ParameterType, by value: named here so the pure half of this
# module reads and tests off-boat.
TYPE_NAMES = {
    0: "not_set", 1: "bool", 2: "integer", 3: "double", 4: "string",
    5: "byte_array", 6: "bool_array", 7: "integer_array", 8: "double_array",
    9: "string_array",
}
# The types the page renders as an editable control. The rest are read-only: an
# array editor is different UI, and guessing a list separator makes one label
# called "red_buoy green_buoy".
SCALAR_TYPES = (1, 2, 3, 4)

# Node names never worth offering: the ros2 CLI's daemon and per-invocation nodes
# would flicker in and out of the selector.
_HIDDEN_PREFIXES = ("_", "/_", "/ros2cli", "/launch_ros")


def visible_nodes(names):
    """Filter a ROS graph node list down to what is worth tuning.

    Args:
      names: iterable of fully-qualified node names ("/target_tracker").

    Returns:
      sorted list of names, CLI plumbing removed.
    """
    return sorted(n for n in names
                  if not any(n.startswith(p) for p in _HIDDEN_PREFIXES)
                  and not n.lstrip("/").startswith("_"))


def range_of(descriptor):
    """(lo, hi) from a ParameterDescriptor, or (None, None).

    floating_point_range and integer_range are empty arrays when unset, so both
    are checked: param_utils picks by the DEFAULT's Python type, and an int-valued
    knob declared from a float default is real.
    """
    for field in ("floating_point_range", "integer_range"):
        rng = getattr(descriptor, field, None)
        if rng:
            return rng[0].from_value, rng[0].to_value
    return None, None


def value_to_python(pv):
    """rcl_interfaces/msg/ParameterValue -> a JSON-serialisable Python value."""
    t = pv.type
    if t == 1:
        return bool(pv.bool_value)
    if t == 2:
        return int(pv.integer_value)
    if t == 3:
        return float(pv.double_value)
    if t == 4:
        return str(pv.string_value)
    if t == 5:
        return list(pv.byte_array_value)
    if t == 6:
        return [bool(v) for v in pv.bool_array_value]
    if t == 7:
        return [int(v) for v in pv.integer_array_value]
    if t == 8:
        return [float(v) for v in pv.double_array_value]
    if t == 9:
        return list(pv.string_array_value)
    return None                      # PARAMETER_NOT_SET: genuinely no value


def python_to_value(value, type_id):
    """A Python value -> ParameterValue of the parameter's DECLARED type.

    The type comes from the descriptor, never the incoming value: JSON has one
    number type, so a browser sending 5 for a double arrives as an int, would be
    built as PARAMETER_INTEGER, and rcl would reject it as a type change.

    Raises:
      ValueError: the value cannot be expressed in the declared type, or the
        type is one this module will not build.
    """
    from rcl_interfaces.msg import ParameterValue

    pv = ParameterValue(type=type_id)
    if type_id == 1:
        if not isinstance(value, bool):
            raise ValueError(f"expected true/false, got {value!r}")
        pv.bool_value = value
    elif type_id == 2:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"expected a whole number, got {value!r}")
        if float(value) != int(value):
            raise ValueError(f"{value} is not a whole number")
        pv.integer_value = int(value)
    elif type_id == 3:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"expected a number, got {value!r}")
        pv.double_value = float(value)
    elif type_id == 4:
        if not isinstance(value, str):
            raise ValueError(f"expected text, got {value!r}")
        pv.string_value = value
    else:
        raise ValueError(
            f"{TYPE_NAMES.get(type_id, type_id)} parameters cannot be set from "
            "the page — use ros2 param set")
    return pv


class ParamBridge:
    """One node's-worth of parameter service clients, for every node.

    Args:
      node: the ground station's own rclpy Node — clients are created on it.
      timeout_s: how long any one service call may take before it is reported
        as unreachable. Short on purpose: a browser is waiting on it.
    """

    def __init__(self, node, timeout_s=3.0):
        self._node = node
        self._timeout = timeout_s
        self._clients = {}                    # (node_name, service) -> Client
        self._lock = threading.Lock()

    # ---------- plumbing ----------

    def _client(self, node_name, service, srv_type):
        """Cached client for one node's one service, created on first use.

        Lazy because a client per service per node is four graph entities each
        and most would never be called. The lock is because two browser tabs can
        open the same node's panel at once, and two threads creating the same
        client would orphan one.
        """
        key = (node_name, service)
        with self._lock:
            client = self._clients.get(key)
            if client is None:
                client = self._node.create_client(
                    srv_type, f"{node_name.rstrip('/')}/{service}")
                self._clients[key] = client
            return client

    def _call(self, node_name, service, srv_type, request):
        """One service round trip, waited on from a NON-executor thread.

        Raises:
          TimeoutError: the service never appeared, or never answered.
        """
        client = self._client(node_name, service, srv_type)
        if not client.service_is_ready() and not client.wait_for_service(
                timeout_sec=self._timeout):
            raise TimeoutError(
                f"{node_name} does not answer {service} — it is in the ROS "
                "graph but its parameter services are not up (or it is a node "
                "that declares none)")
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(self._timeout):
            future.cancel()
            raise TimeoutError(
                f"{node_name} accepted {service} but did not answer within "
                f"{self._timeout:.0f}s")
        return future.result()

    # ---------- the two operations the page needs ----------

    def list(self, node_name, defaults=None):
        """Every parameter of one node, described and valued.

        Args:
          node_name: fully-qualified ("/target_tracker").
          defaults: {name: value} from crusader_params.yaml, or None. Used only
            to mark drift; a parameter missing from it gets default None, which
            the page renders as "not in the YAML" rather than as agreement.

        Returns:
          list[dict] ready for JSON; every field comes from the node, not this
          package.

        Three calls, not one: names, then descriptors, then values (the latter two
        batched over all names), so thirty parameters still cost three round trips.
        """
        from rcl_interfaces.srv import (DescribeParameters, GetParameters,
                                        ListParameters)

        listed = self._call(node_name, _LIST, ListParameters,
                            ListParameters.Request())
        names = sorted(listed.result.names)
        if not names:
            return []

        described = self._call(node_name, _DESCRIBE, DescribeParameters,
                               DescribeParameters.Request(names=names))
        got = self._call(node_name, _GET, GetParameters,
                         GetParameters.Request(names=names))

        defaults = defaults or {}
        out = []
        for name, desc, pv in zip(names, described.descriptors, got.values):
            lo, hi = range_of(desc)
            editable = bool(not desc.read_only and desc.type in SCALAR_TYPES)
            out.append({
                "name": name,
                "type": TYPE_NAMES.get(desc.type, str(desc.type)),
                "value": value_to_python(pv),
                "default": defaults.get(name),
                "in_yaml": name in defaults,
                "read_only": bool(desc.read_only),
                "editable": editable,
                "reason": _not_editable_reason(desc),
                "description": desc.description,
                "lo": lo, "hi": hi,
                # Straight off the descriptor, so a new mode shows in the
                # dropdown the moment the node runs.
                "choices": list(param_utils.choices_of(desc)),
            })
        return out

    def set(self, node_name, values):
        """Apply {name: value} to a running node. Returns [{name, ok, reason}].

        Types come from a fresh describe_parameters, not the page: a node
        restarted since the last poll may have a different posture.

        The node's own on-set callback still validates the range and may refuse;
        its reason comes back. The page's inputs are hints and neither is THIS
        the authority.
        """
        from rcl_interfaces.msg import Parameter
        from rcl_interfaces.srv import DescribeParameters, SetParameters

        names = list(values)
        if not names:
            return []
        described = self._call(node_name, _DESCRIBE, DescribeParameters,
                               DescribeParameters.Request(names=names))
        types = {d.name: d.type for d in described.descriptors}

        params, results = [], []
        for name in names:
            if name not in types or types[name] == 0:
                results.append({"name": name, "ok": False,
                                "reason": f"{node_name} has no parameter "
                                          f"{name!r}"})
                continue
            try:
                params.append(Parameter(
                    name=name,
                    value=python_to_value(values[name], types[name])))
            except ValueError as exc:
                results.append({"name": name, "ok": False, "reason": str(exc)})

        if params:
            applied = self._call(node_name, _SET, SetParameters,
                                 SetParameters.Request(parameters=params))
            for param, result in zip(params, applied.results):
                results.append({
                    "name": param.name, "ok": bool(result.successful),
                    "reason": result.reason or ""})
        return results


def _not_editable_reason(desc):
    """Why the page will not offer a control for this parameter, or "".

    Phrased as what to do instead: naming the file that owns the value does not
    read as a dashboard bug.
    """
    if desc.read_only:
        return ("[RO] structural — change it in crusader_params.yaml and "
                "restart the node")
    if desc.type not in SCALAR_TYPES:
        return (f"{TYPE_NAMES.get(desc.type, desc.type)} values are not "
                "editable here — use ros2 param set")
    return ""
