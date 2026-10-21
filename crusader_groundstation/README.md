# crusader_groundstation

The ground station, in two halves joined by one TCP link (`gcs_wire.py`, port 14559):

| Where | What | Run |
|---|---|---|
| boat | `gcs_bridge`: state at 2 Hz, the map layers, nodes, the BT, logs, radio, OCS; starts and stops nodes, sets parameters, runs missions, records, powers off | `core.launch.py` |
| laptop | `laptop/gcs_app.py`: stdlib only, the page on http://localhost:8090 | `tools/scripts/BOAT_GUI.cmd` |
| boat | `ocs_client`: heartbeat and task reports (`/crsd/robocommand/report`) to the OCS; commands to `/crsd/ocs_command`; echo on `/crsd/ocs_traffic` | Nodes tab |

Protected core nodes cannot be stopped from the laptop; power is locked while armed
(`node_registry.py`). Camera and LiDAR pictures cross the link only while their tab is open.
The MANUAL field on the map publishes `/crsd/passage_plan` when Ekko cannot fly.
