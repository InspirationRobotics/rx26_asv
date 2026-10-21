"""run_node: the entry point every Crusader Python node uses; declare_params and
on_param_set: its parameters.

run_node turns SIGTERM (systemd, docker stop, process_manager) into a normal exit so
destroy_node() runs on every exit path, and keeps exit code 143 for supervisors.
"""
import signal
import sys

import rclpy
from rclpy.executors import ExternalShutdownException


class _SigTerm(SystemExit):
    pass


def _raise_sigterm(signum, frame):
    raise _SigTerm(143)


def declare_params(node, defaults, dynamic=(), ranges=None):
    """Declare every {name: default} on `node` and return {name: value}.

    Read-only unless the name is in `dynamic`, so a `ros2 param set` on a value the
    node only reads at startup is refused instead of silently doing nothing. `ranges`
    {name: (lo, hi)} goes in the descriptor: ROS then refuses a value outside it from
    the YAML at startup and from every later set.
    """
    from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
    ranges = ranges or {}
    for name, default in defaults.items():
        d = ParameterDescriptor(read_only=name not in dynamic)
        if name in ranges:
            lo, hi = ranges[name]
            if isinstance(default, int) and not isinstance(default, bool):
                d.integer_range = [IntegerRange(from_value=int(lo), to_value=int(hi), step=0)]
            else:
                d.floating_point_range = [FloatingPointRange(from_value=float(lo),
                                                             to_value=float(hi), step=0.0)]
        node.declare_parameter(name, default, d)
    return {name: node.get_parameter(name).value for name in defaults}


def on_param_set(node, apply):
    """Call apply({name: value}) for each `ros2 param set` that passed its type and range
    checks. An exception from apply refuses the set, and its text is the reason given."""
    from rcl_interfaces.msg import SetParametersResult

    def callback(params):
        try:
            apply({p.name: p.value for p in params})
        except Exception as e:  # noqa: BLE001
            return SetParametersResult(successful=False, reason=str(e) or type(e).__name__)
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(callback)


def run_node(node_factory, args=None, executor_factory=None):
    """Construct the node inside rclpy and spin it until shutdown.

    executor_factory: zero-arg callable returning an Executor (built after rclpy.init()),
    or None for the default single-threaded spin.
    """
    signal.signal(signal.SIGTERM, _raise_sigterm)
    rclpy.init(args=args)
    node = None
    executor = None
    exit_code = 0
    try:
        node = node_factory()
        if executor_factory is None:
            rclpy.spin(node)
        else:
            executor = executor_factory()
            executor.add_node(node)
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _SigTerm as e:
        exit_code = e.code
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    if exit_code:
        sys.exit(exit_code)
