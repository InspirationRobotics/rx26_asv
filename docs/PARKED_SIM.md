# Parked: simulation + autoresearch orchestrator

The off-board half of this repo is parked on the **`sim/orchestrator`** branch so `main`
carries only what runs on Crusader. Nothing was deleted — the branch is pushed, and the
state at the moment of parking is tagged `sim-parked-2026-07-26`.

## What moved

| Path | What it is |
|---|---|
| `orchestrator/` | episode runner + backends (kinematic / SITL / Gazebo), evaluator + keep rule, level1 / level1_5 / level2, scenarios, gate scripts G0/G3/G4/G5 |
| `docker/` | ArduPilot Rover SITL install + launch, Gazebo variant |
| `tools/sim/` (except `mock_robocommand.py`) | scenario→SDF world generation, hydro coefficient fitting, venue check, `crusader_omnix` model |
| `config/crusader_params.yaml` `por_usv:` | Proof-of-Readiness acceptance thresholds (PoR is complete; the scorer is parked with them) |
| `rx26_asv/api/common/config.py` `por_usv_kwargs()` | ditto |
| `.github/workflows/ci.yml` orchestrator job | orchestrator tests, G0 smoke, gates G3/G4/G5 |

`tools/sim/mock_robocommand.py` **stayed on main** — `tests/test_robocomms_integration.py`
imports it, and it is a comms fixture rather than a simulator.

## Why it merges back cleanly

Git resolves "deleted on one side, untouched on the other" as *deleted*, silently and with
no conflict. A plain removal on `main` would therefore mean the files never come back on a
later merge, in either direction.

To defuse that, `main`'s removal commit was merged into `sim/orchestrator` with
`-s ours` — recorded as an ancestor, not applied. Both tips now share a merge base in which
these paths are absent, so `sim/orchestrator` reads as *adding* them and an ordinary merge
does the obvious thing.

Consequences worth knowing:

- **Do not `git revert` the removal commit on `main`** as a way to bring the sim back. Merge
  the branch instead; reverting will fight the merge base.
- The removal commit is deliberately deletion-only, so it stays easy to reason about.

## Keeping it mergeable

The orchestrator imports the *real* cores (`apf_core`, `occupancy_core`, `api/mission/*`) and
reads `config/crusader_params.yaml`. It rots against boat progress rather than with it. Merge
`main` into the branch and run its suite after any change to those paths — the **U+S** rows in
[CHANGE_IMPACT_MAP.md](CHANGE_IMPACT_MAP.md):

```bash
git switch sim/orchestrator && git merge main && python -m pytest orchestrator/tests tests -q
```

Gazebo world tests need their worlds generated first (`.sdf` files are build artifacts):

```bash
python tools/sim/scenario_to_world.py --all
```

## Bringing it back

```bash
git switch main && git merge sim/orchestrator
```

Then restore the orchestrator CI job in `.github/workflows/ci.yml` (the `unit` job replaced
it; the branch's copy has the full version) and re-add the `docker/sitl` shell scripts to the
`tools-lint` `bash -n` line.
