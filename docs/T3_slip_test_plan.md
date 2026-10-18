# Task 3 in the practice slip: test plan (2026-10-16)

## The commands

On the Jetson, in any new terminal (in one that was already open: `source ~/.bash_aliases`):

| Command | What it does |
|---|---|
| `slip` | What the LiDAR makes of the slip, one line a second. No tree, nothing moves. Ctrl-C stops it |
| `t3 start` | Starts the Task 3 launch with the slip tree. Leave it running; Ctrl-C ends it |
| `t3 go` | Starts a run: checks everything, resets the autonomy drop, sends the goal |
| `t3 steps` | The step lines as they happen: APPROACHING, IN THE SLIP, DOCKED, ... |
| `t3 log` | The tree's log, live, only the useful lines |
| `t3` | This list |

`t3 start fire` runs the old `cannon_fire` tree (no slip) instead.

## What the slip tree does

1. **Approach.** The camera points the bow at the bay and brings the boat to the slip's
   mouth (about 2.4 m from the wall).
2. **Hand-over.** Once the LiDAR has seen the whole slip for 1 s (both walls and the back
   wall), it takes over: it centers the boat between the walls, squares the bow, and holds
   **1.1 m from the LiDAR to the back wall**.
3. **Fire and code.** Docked, report, back off to **1.4 m** to watch the lights, move in to
   spray, back out to read the code.
4. **If the slip is lost for 2 s**, it backs out to the mouth and tries again. After three
   tries it stops and the boat is yours.

These distances are measured from the LiDAR, so they're 0.32 m shorter than the same spot in
`cannon_fire`. 1.1 m here is about where you sprayed from on 10-11 (1.4 m in `cannon_fire`).

**Taking the boat back:** SC out of MANUAL, SB e-stop, or the autonomy drop (SD).

## 1. The slip

Measured: 1.5 m wide, 2 m walls, about 0.4 m above the water. Nothing to change: that's
what the code expects.

## 2. LiDAR check (boat tied in the slip, about 10 min)

Run `slip`. Pull the boat to three spots, taping from the **LiDAR unit** to the back wall:
the mouth (about 2.4 m), the middle (about 1.6 m) and deep in (about 1.1 m).

```
OK  back wall 1.62 m | width 1.49 m | slip centre 0.03 m LEFT | slip points 1.2 deg LEFT | walls L R back
```

At each spot:

- [ ] The line starts with `OK`. If it starts with `--`, write down what it says.
- [ ] `back wall` matches your tape to within about 5 cm.
- [ ] `width` reads about 1.5 m.
- [ ] Push the boat about 20 cm toward the left wall: `slip centre` moves about 0.20 m
  toward RIGHT.
- [ ] Turn the bow about 10 deg right: `slip points` reads about 10 deg LEFT.

Ctrl-C `slip` when done. `t3 start` runs its own copy, and two would fight.

## 3. On the water

**Before the first run:** in the Tuning tab, `strafe.fire_range_m` and
`strafe.watch_range_m` should be **-1** (use the tree's numbers). A 1.4 left over from
`cannon_fire` would hold the hull about 0.3 m farther out.

Three terminals:

1. `t3 start` (leave it running)
2. `t3 steps` (to watch)
3. `t3 go` for each run

| Run | Start | Pass |
|---|---|---|
| 1. Straight in | about 4 m out, lined up with the slip | Hands over at the mouth ("IN THE SLIP"), goes in without touching a wall, DOCKED: centered within about 10 cm, square within about 6 deg, 1.1 m +- 8 cm |
| 2. Off to one side | about 4 m out, 0.5 m to one side, about 20 deg off | Still finds the slip and centers |
| 3. Push | docked | Push it about 0.3 m sideways: back within about 5 s, no hunting back and forth |
| 4. Full fire | docked, light RED | WATCHING, ON FIRE (about 2.5 s after RED), MOVING IN, water until GREEN, BACKING OUT, CODE |

If it stops at the mouth and never goes in, `t3 log` shows
"waiting for the LiDAR to see the slip (...)". Write down what's in the brackets.

## 4. Afterwards

Tell Claude which runs passed. It can read the logs off the boat itself:
- the step lines
- the `slot:` lines (distance, center offset, squareness and stick commands, once a second)
- the `code, window ...` lines from run 4
