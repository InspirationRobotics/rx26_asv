#!/usr/bin/env python3
"""Extract the SITL-safe tunable subset of Crusader's params.

Reads the known-good boat param file, keeps only params on param_guard's TUNABLE
list (plus PILOT_STEER_TYPE, which is behavioral, not hardware), and prints them
in `NAME VALUE` .parm format for sim_vehicle.py --add-param-file.

Hardware-specific params (SERVO mapping, GPS_*, ARMING_*, COMPASS_*) are
deliberately NOT carried into SITL — SITL's defaults are correct for its own
simulated hardware, and carrying arming/hw params over breaks the sim.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from param_guard import load_param_file, is_tunable  # noqa: E402

BEHAVIORAL_EXTRAS = {"PILOT_STEER_TYPE"}  # firmware behavior, safe + meaningful in SITL


def main():
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        sys.exit(2)
    params = load_param_file(sys.argv[1])
    kept = {n: v for n, v in sorted(params.items())
            if is_tunable(n) or n in BEHAVIORAL_EXTRAS}
    if not kept:
        print(f"# WARNING: no tunables found in {sys.argv[1]}", file=sys.stderr)
    for name, val in kept.items():
        print(f"{name} {val:g}")


if __name__ == "__main__":
    main()
