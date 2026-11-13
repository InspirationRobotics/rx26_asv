"""plot_run — one Task 1 sim run, top-down: the course's buoys and the boat's TRUE path.

    python3 -m crusader_sim.plot_run --course task1_dense_s1 --track track.csv --out run.png
           [--title "..."] [--verdict goal.log]

Runs where matplotlib is (the WSL host has python3-matplotlib). The track is task1_goal's
--track-csv (t, x, y, yaw every 0.2 s). Red/green/blue/black dots are the buoys as the course
file has them (the truth), each with a 1 m ring for scale; the path is coloured by time (start
dark, end light), with an arrow every 10 s showing the direction. --verdict adds the judge's
VERDICT line and every WRONG SIDE / contact line from the goal log under the title.
"""
import argparse
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if os.path.dirname(HERE) not in sys.path:
    sys.path.insert(0, os.path.dirname(HERE))

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from crusader_sim import course as C  # noqa: E402

COLOUR = {"flash_red": "#d62728", "flash_green": "#2ca02c", "flash_blue": "#1f77b4",
          "steady_blue": "#17becf", "off": "#222222"}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--course", required=True)
    ap.add_argument("--track", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="")
    ap.add_argument("--verdict", default="", help="the run's goal log (task1_goal output)")
    a = ap.parse_args()

    course = C.load(a.course)
    with open(a.track, encoding="utf-8") as f:
        rows = [(float(r["t"]), float(r["x"]), float(r["y"])) for r in csv.DictReader(f)]
    fig, ax = plt.subplots(figsize=(9, 11), dpi=110)
    for name, x, y, state, _side, _up in C.buoys(course):
        ax.add_patch(plt.Circle((x, y), 1.0, fill=False, ls=":", lw=0.6, color="#999999"))
        ax.plot(x, y, "o", ms=9, color=COLOUR.get(state, "#888888"), mec="black", mew=0.6)
        ax.annotate(name, (x, y), xytext=(5, 5), textcoords="offset points", fontsize=8)
    fc = course.get("field_centre")
    if fc:
        ax.plot(fc["x"], fc["y"], "+", ms=14, mew=2, color="#9467bd")
        ax.annotate("field centre", (fc["x"], fc["y"]), xytext=(6, -12),
                    textcoords="offset points", fontsize=8, color="#9467bd")
    if rows:
        t0, t1 = rows[0][0], rows[-1][0]
        cmap = plt.get_cmap("viridis")
        for (ta, xa, ya), (_tb, xb, yb) in zip(rows, rows[1:]):
            ax.plot([xa, xb], [ya, yb], "-", lw=2.0,
                    color=cmap((ta - t0) / max(1e-6, t1 - t0)))
        last = -1e9
        for (ta, xa, ya), (_tb, xb, yb) in zip(rows, rows[1:]):
            if ta - last >= 10.0 and math.hypot(xb - xa, yb - ya) > 0.02:
                last = ta
                ax.annotate("", xy=(xb, yb), xytext=(xa, ya),
                            arrowprops=dict(arrowstyle="->", color="black", lw=1.2))
        ax.plot(rows[0][1], rows[0][2], "s", ms=8, color="black")
        ax.annotate(f"start (0 s) .. end ({t1 - t0:.0f} s)", (rows[0][1], rows[0][2]),
                    xytext=(8, -4), textcoords="offset points", fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    lines = []
    if a.verdict and os.path.exists(a.verdict):
        with open(a.verdict, encoding="utf-8", errors="replace") as f:
            for ln in f:
                if "VERDICT" in ln or ("[judge]   buoy" in ln and "WRONG" in ln) or "CONTACT" in ln:
                    lines.append(ln.strip().replace("[judge] ", ""))
    ax.set_title("\n".join([a.title or a.course] + lines[:8]), fontsize=8, loc="left")
    fig.tight_layout()
    fig.savefig(a.out)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
