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
"""
import signal
import sys

import rclpy
from rclpy.executors import ExternalShutdownException


class _SigTerm(SystemExit):
    pass


def _raise_sigterm(signum, frame):
    raise _SigTerm(143)


def run_node(node_factory, args=None):
    """node_factory: zero-arg callable returning the Node (construct INSIDE)."""
    signal.signal(signal.SIGTERM, _raise_sigterm)
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = node_factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _SigTerm as e:
        exit_code = e.code
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    if exit_code:
        sys.exit(exit_code)
