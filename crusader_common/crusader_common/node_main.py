"""run_node: the entry point every Crusader Python node uses; declare_params: its parameters.

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


def declare_params(node, defaults, dynamic=()):
    """Declare every {name: default} on `node` and return {name: value}.

    Read-only unless the name is in `dynamic`, so a `ros2 param set` on a value the
    node only reads at startup is refused instead of silently doing nothing.
    """
    from rcl_interfaces.msg import ParameterDescriptor
    for name, default in defaults.items():
        node.declare_parameter(name, default,
                               ParameterDescriptor(read_only=name not in dynamic))
    return {name: node.get_parameter(name).value for name in defaults}


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
