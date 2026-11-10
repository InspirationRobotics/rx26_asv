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

## Auto-record

A mission sent from the **Mission tab** (`run_mission`) records itself. Set it up on the Record
tab: the fps box and topic ticks plus the camera and **Auto-record Mission-tab runs** boxes,
then **Save as auto-record settings** (saves, starts nothing). Defaults: on, camera at 1 Hz, no
bag. The file is `record_settings.json` beside `record_dir` (on the bind mount; override with
the `record_settings_file` param). Ops: `record_settings` `{action: get}` / `{action: set,
settings: {auto, frame_hz, camera, topics}}`. The REC tile shows `auto: <mission>`.

- Starts when `bt_status` goes RUNNING for that mission, not while it is WAITING_RUN_START, and
  only if nothing is recording. Task 1 / Task 3 panel missions never auto-record.
- Stops when that mission leaves RUNNING for any reason (done, cancel, abort, bt_runner gone),
  and only while the running session is still the auto one.
- A recording you start by hand is never started over or stopped by a mission. If you stop the
  auto recording by hand it is not restarted for that mission; pressing Start while it runs
  makes it yours. A new Mission-tab run arriving while the old auto recording is still open
  stops it, then starts a fresh one.
- Logic and tests: `record_settings.py`, `test/test_record_settings.py`.
