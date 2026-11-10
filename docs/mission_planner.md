# Mission planner — competition tree, task subtrees, block editor

Branch `feat/mission-planner`. Design of 2026-10-17. Handbook refs are to the mirror in
`Boat/handbook/` (refreshed 2026-10-17; Section 4 scoring was published 2026-10-15).

## What the handbook fixes

| Rule | Where | Consequence here |
|---|---|---|
| Heartbeat at 2 Hz for every active vehicle, **at all times** in the course area | 3.4.11, 5.3 item 8.1 | The heartbeat *transport* stays in `ocs_client` (it must flow with no mission running). The tree owns the heartbeat's **`current_task`** field. |
| A transition **into** a task value starts that attempt; a transition **to `TASK_NONE`** ends it and moves to the next | 3.4.11 | Every task subtree is wrapped in `TaskAttempt`, which sets the token on entry and `TASK_NONE` on every exit (success, failure, halt). |
| Vehicles position-hold after switching to autonomous; begin only once the OCS has verified `RunStart` | 3.4.10 hint | `bt_runner_node` holds station at its own RunStart gate (every goal, before the first tick) until RoboCommand's `run_start` reaches the boat. |
| `RunStart` is issued only once every declared vehicle reports `STATE_AUTO` | 3.4.10 steps 5-9, 5.3 item 8.2 | The boat must already be armed + GUIDED, or armed with the tree ticking (`ocs_client` reports either as AUTO: Task 3 drives in MANUAL), and "ticking" while it waits: the gate publishes `/crsd/autonomy_active` every tick, which is what makes `ocs_client` report AUTO. |
| Course elements activate only on a task-attempt heartbeat after `RunStart`; Task 1 beacons stay on only while the attempt is reported | 5.3 item 8.3, 3.3.2 | `current_task` must stay `TASK_SAFE_PASSAGE` for the whole Task 1 subtree. |
| "SoS activates challenge by sending heartbeat messages" is 25 pts per task | Section 4 | Each task attempt must be a clean `TASK_NONE -> TASK_x -> TASK_NONE` transition. |

## The run, end to end

```
OCS: declare run  ->  boat: operator arms, GUIDED, START mission (heartbeat STATE_AUTO, TASK_NONE)
                  ->  bt_runner gate: loads the tree, holds position, ticks nothing (bt_status WAITING_RUN_START)
RoboCommand: RunStart -> OCS verifies -> relays {"run_start": {...}} -> ocs_client -> /crsd/ocs_command
                  ->  gate opens (home and the timeout clock restart here) -> waypoints / TaskAttempt(Task 1) / TaskAttempt(Task 3) ...
```

The gate is in the runner, so no mission can skip it and none needs a `WaitForRunStart` block.
A RunStart counts if it arrived after the OCS's latest declaration, however long before START:
`standby_auto` (a new declaration) and the `run_end` directive both clear the latch, and the
`run_start_max_age_s` parameter (default 3600) is the backstop. The mission timeout does not run
while the goal waits.

Bench testing with no RoboCommand: call the Trigger service `/crsd/bt_runner/release_run_start`
(the boat GUI's "Release without RoboCommand (bench)" button does), which starts the ONE goal that
is waiting and logs a WARN. The sim scripts call it themselves. Nothing else changes.

## Tree layout

```
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <ReactiveSequence name="guard">
      <IsAutonomous/>                                  main-tree guard: the pilot can always take it back
      <Sequence name="mission">
        <NavigateTo target="fix" lat=".." lon=".."/>   "GPS waypoint" block, anywhere in any tree
        <TaskAttempt token="TASK_SAFE_PASSAGE">        heartbeat current_task, main tree
          <SubTree ID="Task1Advanced" _autoremap="true" orbit_points="8" .../>
        </TaskAttempt>
        <TaskAttempt token="TASK_COORDINATED_LOGISTICS">
          <SubTree ID="Task3Disruptive" _autoremap="true" .../>
        </TaskAttempt>
      </Sequence>
    </ReactiveSequence>
  </BehaviorTree>
  <BehaviorTree ID="Task1Advanced"> ... task-specific guards and reports live here ... </BehaviorTree>
</root>
```

* **Task subtrees** live in `crusader_bt/behavior_trees/lib/<name>.xml`, one `<BehaviorTree>` each,
  with a `<TreeNodesModel><SubTree ID=..>` block declaring its tunables (name, default, one-line
  description). Their task-specific guard band (e.g. `PassagePlanFresh`, `ResolveBuoyStates`,
  `PublishSafePassageReport`) and task reports stay **inside** the subtree. `IsAutonomous` and
  `SetTask` move **out** (to the main tree / `TaskAttempt`).
* **Every `<SubTree>` carries `_autoremap="true"`** or every leaf in it throws "no 'ctx' on the
  blackboard" (crusader_bt/README.md, "Subtrees need _autoremap").
* **Tunables are SubTree attributes.** Inside the subtree a leaf reads `{orbit_points}`; the
  caller writes `orbit_points="8"`. Writers (wrappers, the GUI) always write every declared port
  explicitly, so nothing depends on BT.CPP applying model defaults.
* **A tier belongs to the tree, never to the goal.** There is one tree per task per tier
  (`task1_advanced`, `task1_disruptive`, `task3_core`, `task3_advanced`, `task3_disruptive`); no
  tree file can do more than one tier, and `RunMission` carries no tier. Each standalone file is
  a thin `Main` that `<include>`s the body its tiers share (`lib/common/`, once) and then its own
  lib subtree. There is no Task 1 Core tree.
* **A mission saved by the GUI is self-contained**: every subtree it uses is inlined, no
  `<include>`. The file that ran is the whole record of what ran.

## New BT nodes (crusader_bt)

| Node | Kind | Ports | Contract |
|---|---|---|---|
| `WaitForRunStart` | action | `max_age_s` (30), `timeout_s` (0 = forever), `hold` (true), out `run_id` | Legacy: the runner's gate now does this for every goal, and the leaf SUCCEEDS at once once the gate has opened, so an old saved mission that still has it does not wait twice. Without the gate it was RUNNING, holding station like `HoldStation`, until a `run_start` received no earlier than `max_age_s` before this node started (30 s: a RunStart that just beat the START button counts, one from an earlier run does not). FAILURE only after `timeout_s` > 0. |
| `TaskAttempt` | decorator | `token` (RxTask name) | Sets `current_task` = token on its first tick, `TASK_NONE` when the child finishes or is halted. Returns the child's status. |
| `HoldHeading` | action | `heading_deg` (required, deg true 0-360), `duration_s` (30, > 0), `radius_m` (2, > 0), `return_speed_mps` (0.3, at most 0.4) | Turns to `heading_deg` and holds it for `duration_s`, at the point where the previous block left the boat; SUCCESS when the time is up, counted from its start. FAILURE without a fresh pose at the start, or after 5 s of a stale one. See below. |

### HoldHeading (Task 3 surveying / data collection)

Place it after a GPS waypoint, normally inside a `TaskAttempt` so the heartbeat keeps reporting
the task. On its first tick it takes the boat's position as the hold point P. GUIDED's position
setpoint ignores yaw, so it cannot point the bow; the leaf uses the heading-and-speed command
(`/crsd/guided_heading_speed`, MAVLink SET_ATTITUDE_TARGET, **GUIDED only**) instead, re-sent
every tick because the bridge stops the boat 0.5 s after the last one. Speed 0 turns the boat
to the heading and stops it, but that command does **not** hold position, so wind or current
will push the boat off P. The leaf therefore watches the distance to P:

* inside `radius_m`: command (`heading_deg`, 0 m/s);
* beyond `radius_m`: command (bearing to P, min(`return_speed_mps`, 0.3 x distance)), so it
  turns toward P and creeps back; it keeps returning until it is within `radius_m / 2`, then
  turns back to `heading_deg` (the hysteresis stops it chattering at the edge). The bridge
  clamps speed at 0.4 m/s whatever the port says.

Like `NavigateTo` it sends nothing when `publish_setpoints` is false (the log says `[NOT SENT]`).
Halting stops the commands; the bridge deadman then stops the boat. The map draws an arrow at the
GPS point the hold belongs to (the nearest GPS waypoint before it in mission order). The
Mission tab validates the ports: `heading_deg` in [0, 360), `duration_s`, `radius_m`,
`return_speed_mps` above 0 (errors), and `return_speed_mps` above 0.4 (warning).
Template: `crusader_bt/behavior_trees/templates/survey_t3_hold.xml`. The decision is
`include/crusader_bt/hold_heading.hpp`, tested by `test/test_hold_heading.cpp`. Not yet
verified on the water: how ArduRover answers the held heading.

The runner latches the newest `run_start` (`run_id`, receive time) from `/crsd/ocs_command` and
**does not reset it per goal**. `tree_file` becomes a live parameter, read when a goal is
accepted (like the `nav_*` knobs): the GUI sets it, then sends the goal.

## Mission planner tab (ground station)

A "Mission" tab beside the existing ones (the Task 1 tab stays for testing). Scratch-style:

* **Palette** (left): "Presets" (GPS waypoint, and Hold heading when the boat has the leaf),
  one group per task with its tier blocks, then the registered leaves grouped Controls,
  Decorators, Actions, Conditions. Hovering a block shows its description and ports.
* **Workspace** (centre): the selected tree as nested blocks; controls and decorators are
  C-shaped and hold children. Drag from the palette or within the workspace to insert or move;
  drop on the bin or press Delete to remove. Tabs switch between `Main` and every subtree in the
  mission, so a waypoint can go into any tree.
* **Inspector** (right): the selected block's ports, with defaults, descriptions and choices.
  A SubTree block shows its declared tunables (e.g. Task 1 orbit points).
* **Map** (right, under the inspector): an offline canvas (no tiles) with the boat, the known
  field, and the mission's GPS waypoints. A click sets the selected waypoint's lat/lon, or adds a
  new GPS waypoint after the selected block.
* **Toolbar**: mission picker (pre-built + saved), New, Save, Save as, Validate, XML view,
  Download / Upload, Start (timeout), Stop.

The editor's storage format **is** BT.CPP XML: load = parse XML into blocks, save = serialise.

### Where the library lives and what each side does

`crusader_groundstation/mission_xml.py` (pure Python, no ROS) turns blocks into mission XML and
back and validates a mission against the boat's catalogue; `mission_store.py` holds the
catalogue, the built-in/uploaded/template lists and the files. `gcs_bridge` serves them on the
boat (`mission_catalogue`, `mission_get`, `mission_upload`, `mission_delete`, `mission_template`,
`run_mission`, `cancel_mission`); the laptop's `laptop/gcs_app.py` answers `mission_build` and
`mission_open` itself and forwards the rest.

## Not changed

The `SafePassage` action interface, `ocs_client`'s transport and heartbeat rules, the RC e-stop
as the only safety path, `publish_setpoints` defaulting to false.
