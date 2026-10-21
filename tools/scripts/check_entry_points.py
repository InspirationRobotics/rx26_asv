#!/usr/bin/env python3
"""check_entry_points: every console script of the boat's packages imports.

A console_script pointing at a moved module builds clean and dies at `ros2 run`, on the boat.
Run with the built workspace sourced (CI, setup/install_container.sh):

    python3 tools/scripts/check_entry_points.py
"""
import ast
import importlib
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]
LAPTOP_ONLY = {"crusader_sim"}


def entry_points(setup):
    """(name, 'module:function') for each console script of a setup.py, read without running it."""
    for node in ast.walk(ast.parse(setup.read_text())):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if getattr(key, "value", None) == "console_scripts":
                    for entry in ast.literal_eval(value):
                        name, target = (s.strip() for s in entry.split("="))
                        yield name, target


def main():
    bad = 0
    for setup in sorted(REPO.glob("crusader_*/setup.py")):
        if setup.parent.name in LAPTOP_ONLY:
            continue
        for name, target in entry_points(setup):
            module, function = target.split(":")
            try:
                getattr(importlib.import_module(module), function)
                print(f"[ ok ] {name}")
            except Exception as e:  # noqa: BLE001
                print(f"[FAIL] {name} -> {target}: {type(e).__name__}: {e}")
                bad += 1
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
