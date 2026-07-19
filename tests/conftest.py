import sys
from pathlib import Path

# repo root on sys.path -> `import robotx_2026.api...` resolves the package dir.
# Only rclpy-free modules (api.common.*) are imported by these tests.
sys.path.insert(0, str(Path(__file__).parent.parent))
