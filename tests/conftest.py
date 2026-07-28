import sys
from pathlib import Path

# The rx26_asv PACKAGE dir (<repo>/rx26_asv, which holds setup.py) goes on
# sys.path -> `import rx26_asv.api...` resolves the module dir nested inside it.
# Only rclpy-free modules (api.common.*) are imported by these tests.
sys.path.insert(0, str(Path(__file__).parent.parent / "rx26_asv"))
