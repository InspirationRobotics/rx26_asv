# `crusader_common` — shared plumbing

The library every other package builds on. **No nodes, no vehicle policy.** If a
change here would only make sense on Crusader, it belongs in a domain package instead.

| Module | What it is |
|---|---|
| `config.py` | Loads `crusader_params.yaml` so node code declares defaults *from* the file the launch system loads. Read its header before touching path resolution. |
| `param_utils.py` | `declare_from_config` + range validation. Rejects a YAML key with no declared posture at node start. |
| `node_main.py` | `run_node` — the lifecycle wrapper, with deterministic teardown. |
| `stream_cache.py` | Freshness-gated value cache. The thing that makes silence stay silent. |
| `drop_latch.py` | The autonomy-drop latch state machine (Gate G1). |
| `geo.py` | lat/lon ↔ local XY, body↔world rotation, ground speed from NED cm/s. |

## Dependency direction

Everything depends on `crusader_common`; `crusader_common` depends on nothing of ours
except, at *runtime only*, `crusader_bringup`'s share dir for the params file. It has no
build dependency on bringup, and it must never import a domain package. If you find
yourself wanting to, the thing you need is a parameter or a function argument.

## The params-file lookup is the one subtle thing here

`config.py` resolves `crusader_params.yaml` in this order: `$CRUSADER_PARAMS` →
`crusader_bringup`'s installed share dir (via ament) → the source tree.

**Do not replace that with `Path(__file__).parents[N]`.** In the install space this module
lives under `install/crusader_common/lib/python3.10/site-packages/` while the config lives
under `install/crusader_bringup/share/` — different packages, and `lib/` and `share/` are
siblings. No fixed parent count reaches it. Getting this wrong kills every node at
`rclpy.init()`, which is exactly what happened on 2026-07-29.

CI asserts the file still resolves from a real install space, not just from the source tree.

## Change impact

| You changed | Then |
|---|---|
| `config.py` path resolution | `python3 tools/scripts/check_config.py`, then a full `rebuild.sh` and confirm nodes start — this is the highest-blast-radius file in the repo |
| `stream_cache.py` | every consumer's staleness behaviour; re-check the watchdog's gateway-down path |
| `drop_latch.py` | the Gate G1 procedure (`docs/G1_bench_procedure.md`) |
| `geo.py` | anything doing position math — currently only `telemetry_bridge`'s ground speed |
