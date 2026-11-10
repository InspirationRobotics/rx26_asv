# crusader_groundstation

The ground station, in two halves joined by one TCP link (`gcs_wire.py`, port 14559):

| Where | What | Run |
|---|---|---|
| boat | `gcs_bridge`: state at 2 Hz, the map layers, nodes, the BT, logs, radio, OCS; starts and stops nodes, sets parameters, runs missions, records, powers off | `core.launch.py` |
| laptop | `laptop/gcs_app.py`: stdlib only, the page on http://localhost:8150 | `tools/scripts/BOAT_GUI.cmd` |
| boat | `ocs_client`: heartbeat and task reports (`/crsd/robocommand/report`) to the OCS; commands to `/crsd/ocs_command`; echo on `/crsd/ocs_traffic` | Nodes tab |

Protected core nodes cannot be stopped from the laptop; power is locked while armed
(`node_registry.py`). Camera and LiDAR pictures cross the link only while their tab is open.
The MANUAL field on the map publishes `/crsd/passage_plan` when Ekko cannot fly.

## The RunStart gate and the tier declaration

Every mission the Mission tab starts is held by `bt_runner_node` until RoboCommand's RunStart
reaches the boat through the OCS. While it waits, `status.mission.state` is `WAITING_RUN_START`
and a banner on every tab offers **Release without RoboCommand (bench)** (op `run_release`, which
calls `/crsd/bt_runner/release_run_start`).

The **Declaration** row (Task 1-4, each none / core / advanced / disruptive) is saved on the
mission's root element as `declare_task1..4`; a mission without them derives the row from its task
blocks. **Send tiers to OCS** (op `ocs_tiers`) goes `gcs_bridge` -> `/crsd/ocs_tiers` ->
`ocs_client` -> one `ocs_tiers` control frame (no seq, no receipt, not retried; refused while the
link is down). The OCS answers with an `ocs_run` snapshot, which `gcs_bridge` caches from
`/crsd/ocs_command` and puts in the status frame as `ocs_run` (with its age, and the last `reply`
with its own age). A declaration that disagrees with the blocks is a warning, never a refusal.

Survey templates live in `crusader_bt/behavior_trees/templates/` (never runnable by name, listed in
the catalogue as `templates`); **New from template** opens one as an unsaved draft centred on the
boat.
