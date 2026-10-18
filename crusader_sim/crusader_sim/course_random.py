"""course_random — a course's random choices, made ONCE per bring-up.

    python3 -m crusader_sim.course_random PATH       # rewrites PATH; prints what it picked

Runs in: WSL, from gz_sim_up.sh, on the run's own copy of the course (the one
gz_sync just copied in from the Windows checkout, which keeps saying `random`).
The world (gen_world: the indicator lights) and the referee (task3_world: which
bay RoboCommand wants) both read that copy, so they always agree.

  dock `green_bay: random`   -> 1, 2 or 3; `lit_window.bay` follows it (the fire
                                is in the GREEN bay). SIM_GREEN_BAY=N in the
                                environment forces one, for a repeatable test.

Prints "green_bay N" (or nothing when there was nothing random).
"""
import os
import random
import sys

import yaml


def resolve(course, rng=random):
    """The course with its random choices made; [(what, value)] picked."""
    picked = []
    for e in course.get("elements", []):
        if e.get("type") != "dock" or str(e.get("green_bay", "")).lower() != "random":
            continue
        forced = os.environ.get("SIM_GREEN_BAY", "")
        bay = int(forced) if forced in ("1", "2", "3") else rng.randint(1, 3)
        e["green_bay"] = bay
        lw = dict(e.get("lit_window") or {"slot": 1, "colour": "red"})
        lw["bay"] = bay
        e["lit_window"] = lw
        picked.append(("green_bay", bay))
    return course, picked


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__.split("\n\n")[1].strip(), file=sys.stderr)
        return 2
    path = argv[0]
    with open(path, encoding="utf-8") as f:
        text = f.read()
    course = yaml.safe_load(text)
    course, picked = resolve(course)
    if not picked:
        return 0
    with open(path, "w", encoding="utf-8") as f:
        f.write("# THIS RUN'S COPY: random choices made by crusader_sim.course_random "
                + ", ".join(f"{k} {v}" for k, v in picked) + "\n")
        yaml.safe_dump(course, f, sort_keys=False)
    for k, v in picked:
        print(f"{k} {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
