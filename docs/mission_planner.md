# Mission planner — competition tree, task subtrees, block editor

Branch `feat/mission-planner`. Design of 2026-10-17. Handbook refs are to the mirror in
`Boat/handbook/` (refreshed 2026-10-17; Section 4 scoring was published 2026-10-15).

## What the handbook fixes

| Rule | Where | Consequence here |
|---|---|---|
| Heartbeat at 2 Hz for every active vehicle, **at all times** in the course area | 3.4.11, 5.3 item 8.1 | The heartbeat *transport* stays in `ocs_client` (it must flow with no mission running). The tree owns the heartbeat's **`current_task`** field. |
| A transition **into** a task value starts that attempt; a transition **to `TASK_NONE`** ends it and moves to the next | 3.4.11 | Every task subtree is wrapped in `TaskAttempt`, which sets the token on entry and `TASK_NONE` on every exit (success, failure, halt). |
| Vehicles position-hold after switching to autonomous; begin only once the OCS has verified `RunStart` | 3.4.10 hint | `WaitForRunStart` holds station until RoboCommand's `run_start` reaches the boat. |
| `RunStart` is issued only once every declared vehicle reports `STATE_AUTO` | 3.4.10 steps 5-9, 5.3 item 8.2 | The boat must already be armed + GUIDED (that is what `ocs_client` reports as AUTO) and ticking the tree while it waits. |
| Course elements activate only on a task-attempt heartbeat after `RunStart`; Task 1 beacons stay on only while the attempt is reported | 5.3 item 8.3, 3.3.2 | `current_task` must stay `TASK_SAFE_PASSAGE` for the whole Task 1 subtree. |
| "SoS activates challenge by sending heartbeat messages" is 25 pts per task | Section 4 | Each task attempt must be a clean `TASK_NONE -> TASK_x -> TASK_NONE` transition. |

## The run, end to end

```
OCS: declare run  ->  boat: operator arms, GUIDED, START mission (heartbeat STATE_AUTO, TASK_NONE)
                  ->  tree: WaitForRunStart holds position
RoboCommand: RunStart -> OCS verifies -> relays {"run_start": {...}} -> ocs_client -> /crsd/ocs_command
                  ->  tree: WaitForRunStart SUCCESS -> waypoints / TaskAttempt(Task 1) / TaskAttempt(Task 3) ...
```

Bench testing with no RoboCommand: delete the `WaitForRunStart` block from the mission (or load
the `bench_*` mission). Nothing else changes.

## Tree layout

```
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <ReactiveSequence name="guard">
      <IsAutonomous/>                                  main-tree guard: the pilot can always take it back
      <Sequence name="mission">
        <WaitForRunStart/>                             remove for bench tests
        <NavigateTo target="fix" lat=".." lon=".."/>   "GPS waypoint" block, anywhere in any tree
        <TaskAttempt token="TASK_SAFE_PASSAGE">        heartbeat current_task, main tree
          <SubTree ID="Task1Global" _autoremap="true" orbit_points="8" .../>
        </TaskAttempt>
        <TaskAttempt token="TASK_COORDINATED_LOGISTICS">
          <SubTree ID="Task3Disruptive" _autoremap="true" .../>
        </TaskAttempt>
      </Sequence>
    </ReactiveSequence>
  </BehaviorTree>
  <BehaviorTree ID="Task1Global"> ... task-specific guards and reports live here ... </BehaviorTree>
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
* The existing standalone files (`task1_disruptive.xml`, `task1_global.xml`,
  `task1_safe_passage.xml`, `task3_disruptive.xml`) keep their names and behaviour: each becomes a
  thin `Main` that `<include>`s its lib subtree, so `TREE=...` in the sim and lake rigs is
  unchanged.
* **A mission saved by the GUI is self-contained**: every subtree it uses is inlined, no
  `<include>`. The file that ran is the whole record of what ran.

## New BT nodes (crusader_bt)

| Node | Kind | Ports | Contract |
|---|---|---|---|
| `WaitForRunStart` | action | `max_age_s` (30), `timeout_s` (0 = forever), `hold` (true), out `run_id` | RUNNING, holding station like `HoldStation`, until a `run_start` received no earlier than `max_age_s` before this node started (30 s: a RunStart that just beat the START button counts, one from an earlier run does not). FAILURE only after `timeout_s` > 0. |
| `TaskAttempt` | decorator | `token` (RxTask name) | Sets `current_task` = token on its first tick, `TASK_NONE` when the child finishes or is halted. Returns the child's status. |
| `SetTier` | action | `tier` core/advanced/disruptive | Sets the tier `TierIs`/`TierAtLeast` read, for a mission that runs tasks at different tiers. |

The runner latches the newest `run_start` (`run_id`, receive time) from `/crsd/ocs_command` and
**does not reset it per goal**. `tree_file` becomes a live parameter, read when a goal is
accepted (like the `nav_*` knobs): the GUI sets it, then sends the goal.

## Mission planner tab (ground station)

A "Mission" tab beside the existing ones (the Task 1 tab stays for testing). Scratch-style:

* **Palette** (left): blocks grouped by colour-coded category: Flow (Sequence, ReactiveSequence,
  Fallback, Parallel...), Decorators (ForceSuccess, Inverter, Retry, Timeout, Repeat...),
  RoboCommand (WaitForRunStart, TaskAttempt, SetTier), Navigation (GPS waypoint, Return home,
  Orbit a GPS point, Hold position), Tasks (every lib subtree), Conditions, and an "advanced"
  toggle for the rest of the registered leaves. Every block shows a one-line description.
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
  Download / Upload, Start (tier, timeout), Stop.

The editor's storage format **is** BT.CPP XML: load = parse XML into blocks, save = serialise.

### Where the library lives and what each server does

`crusader_groundstation/mission_lib.py` (pure Python, no ROS) is the one implementation:
catalogue, list, load (flattening `<include>`), validate, save. Both servers call it:

| Endpoint (POST, JSON) | Laptop `gcs_laptop.py` | Jetson / sim `gcs_node.py` |
|---|---|---|
| `/mission/catalogue`, `/mission/list`, `/mission/load`, `/mission/validate`, `/mission/save` | served locally from the laptop's checkout (`crusader_bt/behavior_trees`, `crusader_bt/missions`) | served from its own workspace |
| `/mission/start` `{xml, name, tier, timeout_s}`, `/mission/stop` | forwarded to the Jetson | validates, writes the XML to `~/.ros/crusader_missions/`, sets `bt_runner_node.tree_file`, sends the `SafePassage` goal / cancels it |

## Not changed

The `SafePassage` action interface, `ocs_client`'s transport and heartbeat rules, the RC e-stop
as the only safety path, `publish_setpoints` defaulting to false.
