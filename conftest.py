# pytest from the repo root imports every package from its source dir, so tests import
# crusader_x.module the same way they do after `colcon build`.
import pathlib
import sys

for pkg in sorted(pathlib.Path(__file__).parent.glob("crusader_*/")):
    if (pkg / pkg.name / "__init__.py").exists() and str(pkg) not in sys.path:
        sys.path.insert(0, str(pkg))
