import sys
from pathlib import Path

# make `episodes` / `evaluator` importable without packaging the orchestrator
sys.path.insert(0, str(Path(__file__).parent.parent))
