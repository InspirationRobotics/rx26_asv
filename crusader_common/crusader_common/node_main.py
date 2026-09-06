"""run_node — the one canonical entry point for every Crusader node (Phase 3.5).

Fixes three defects of the naive init/spin/shutdown idiom:

  1. Constructor failure: our nodes fail loudly by design (bad endpoint, USB2,
     unmapped classes). Construction happens INSIDE the try and the finally
     guards `node is not None`, so the original exception propagates cleanly
     instead of being masked by UnboundLocalError, and rclpy still shuts down.
  2. `ros2 launch` shutdown surfaces as ExternalShutdownException, not
     KeyboardInterrupt — both are caught; `try_shutdown()` is idempotent where
     `shutdown()` raises if the context is already down.
  3. SIGTERM (systemd units, `docker stop`) is converted to a normal exception
     so destroy_node() runs — thread joins, MAVLink close, override release
     bookkeeping — honoring the deterministic-teardown rule on ALL exit paths,
     not just Ctrl+C. Exit code 143 (128+15) is preserved for supervisors.

`executor` exists for action servers. An rclpy action server whose
execute_callback blocks CANNOT service the cancel request it is waiting on: the
single-threaded executor is busy running the callback, so the cancel sits in the
queue until the mission finishes on its own — which is precisely never, from the
operator's point of view. Such a node passes `MultiThreadedExecutor()` here and
puts its server in a ReentrantCallbackGroup. Every other node keeps the default
single-threaded spin and is unaffected.
"""
import signal
import sys

import rclpy
from rclpy.executors import ExternalShutdownException


class _SigTerm(SystemExit):
    pass


def _raise_sigterm(signum, frame):
    raise _SigTerm(143)


def run_node(node_factory, args=None, executor=None):
    """node_factory: zero-arg callable returning the Node (construct INSIDE).

    executor: an rclpy Executor, or None for the default single-threaded spin.
    Shut down here on every exit path, so a MultiThreadedExecutor's worker
    threads are joined before destroy_node() rather than after it.
    """
    signal.signal(signal.SIGTERM, _raise_sigterm)
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = node_factory()
        if executor is None:
            rclpy.spin(node)
        else:
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
