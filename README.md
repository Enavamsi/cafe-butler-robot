# Café Butler Robot 🤖☕

A ROS2 + Nav2 + Behavior Tree implementation of a café delivery robot, built for the
Goat Robotics ROS Developer assessment. The robot collects orders from the kitchen
and delivers them to one or more tables, handling confirmations, timeouts, and
cancellations along the way — all 7 assessment milestones are implemented as
**different Behavior Tree XML files built from the same small set of generic C++
nodes**.

---

## Table of Contents

- [Overview](#overview)
- [Why a Behavior Tree](#why-a-behavior-tree)
- [Architecture](#architecture)
- [Repository Structure](#repository-structure)
- [Packages](#packages)
- [Behavior Tree Nodes](#behavior-tree-nodes)
- [Generic Order Flow](#generic-order-flow)
- [Milestones → Implementation](#milestones--implementation)
- [ROS2 Interfaces](#ros2-interfaces)
- [Prerequisites](#prerequisites)
- [Build](#build)
- [Configuration (Waypoints)](#configuration-waypoints)
- [Running](#running)
- [Testing with the GUI](#testing-with-the-gui)
- [Troubleshooting](#troubleshooting)
- [Known Limitations / Future Work](#known-limitations--future-work)
- [License](#license)

---

## Overview

The café's workflow: an order comes in for one or more tables → the robot leaves
its home position → goes to the kitchen to collect the food → delivers to the
table(s) → returns home. On top of that base flow, the assessment asks for:

- waiting for a human to confirm pickup/drop-off, with a timeout fallback
- different recovery routes depending on **where** a timeout happens
- cancelling an order (or a single table) mid-delivery
- handling **multiple** tables in one trip, skipping ones that don't confirm
  or get cancelled, without stopping the whole delivery

Rather than hand-coding 7 separate state machines, this project implements
**one reusable set of Behavior Tree nodes** and expresses each milestone as a
short, readable BT XML file built from those nodes.

## Why a Behavior Tree

- **Readable** — the XML tree reads almost like the milestone description itself.
- **Generic** — the nodes don't know about `table1`/`table2`/etc.; they operate on
  whatever waypoint name or table list they're given. Milestones 5–7 (multiple
  tables) reuse the *exact same* nodes as 1–4 (single table).
- **Reactive** — `ReactiveSequence` + a condition node lets the tree interrupt an
  in-progress Nav2 goal the instant a cancellation event arrives, instead of
  waiting for the current action to finish.
- **Composable** — new rules are usually just a different arrangement of
  `Sequence` / `Fallback` / `Inverter` around existing leaf nodes, not new code.

## Architecture

![Architecture diagram](docs/architecture.png)

Four moving parts:

1. **`cafe_butler_gui`** — a Tkinter GUI standing in for the humans in the
   workflow: the host placing an order, kitchen staff confirming pickup,
   customers confirming delivery, and anyone cancelling an order/table.
2. **`/butler_order` action** — carries the order (which tables) from the GUI
   to the robot, and reports feedback/results back.
3. **`butler_bt_navigator`** — the BT.CPP executor node. Loads the XML tree
   for whichever milestone you select and ticks it, resolving waypoint names
   and listening for confirm/cancel events via a shared `OrderTracker`.
4. **Nav2** — actually drives the robot between named waypoints
   (`home`, `kitchen`, `table1`, `table2`, `table3`) via `NavigateToPose`.

## Repository Structure

```
cafe_ros2_ws/
└── src/
    ├── cafe_bot_description_fixed/   # URDF, Gazebo world, map, RViz config, Nav2/EKF params
    ├── cafe_butler_interfaces/       # OrderTask.action (goal / feedback / result)
    ├── cafe_butler_bt/               # Behavior Tree nodes + executor node + milestone trees
    │   ├── include/cafe_butler_bt/
    │   │   ├── order_tracker.hpp     # tracks /confirmation and /cancel_order events
    │   │   ├── waypoint_server.hpp   # loads named waypoints from waypoints.yaml
    │   │   └── bt_nodes.hpp          # declarations of the 5 custom BT nodes
    │   ├── src/
    │   │   ├── bt_nodes.cpp          # node implementations + factory registration
    │   │   └── bt_executor_node.cpp  # action server that ticks the selected tree
    │   ├── behavior_trees/           # m1.xml ... m7.xml, full.xml
    │   ├── config/waypoints.yaml     # home/kitchen/table1-3 poses (map frame)
    │   └── launch/butler_bt.launch.py
    └── cafe_butler_gui/              # order / confirm / cancel simulator GUI
```

## Packages

| Package | Build type | Purpose |
|---|---|---|
| `cafe_bot_description_fixed` | ament_cmake | Robot URDF, Gazebo café world, map, RViz config, Nav2/EKF params. |
| `cafe_butler_interfaces` | ament_cmake | Defines the `OrderTask` action. |
| `cafe_butler_bt` | ament_cmake (C++) | The Behavior Tree nodes, the executor node, and the milestone XML trees. |
| `cafe_butler_gui` | ament_python | GUI simulating host orders and kitchen/table confirm/cancel button presses. |

## Behavior Tree Nodes

Five custom nodes, registered once in `bt_nodes.cpp`, are reused across every
milestone:

| Node | Type | Ports | What it does |
|---|---|---|---|
| `NavigateToWaypoint` | Stateful action | `waypoint` (string) | Sends a Nav2 `NavigateToPose` goal for a named waypoint; `RUNNING` until arrival; cancels the Nav2 goal if halted. |
| `WaitForConfirmation` | Stateful action | `location`, `timeout` | Polls `OrderTracker` until someone confirms at `location`, or fails after `timeout` seconds. |
| `IsTableCancelled` | Condition | `table` | Succeeds if that table (or the whole order, via `"all"`) has been cancelled. |
| `ForEachTable` | Control node | — | Iterates the `tables_queue` from the action goal; ticks its one child once per table. Child `SUCCESS` → table marked delivered; child `FAILURE` → table marked **skipped**, and the loop keeps going regardless. This single node is what makes 1-table and N-table orders share the same subtree. |
| `PublishFeedback` | Sync action | `state` | Writes a human-readable state string that the executor forwards as ROS action feedback. |

All ROS-facing state — the Nav2 action client, the `/confirmation` and
`/cancel_order` subscriptions — lives in `NavigateToWaypoint`, `OrderTracker`,
and `WaypointServer` respectively. The XML trees themselves only ever reason
about pure BT logic (Sequence / Fallback / Inverter / ReactiveSequence).

## Generic Order Flow

This is the general shape shared by milestones 2–7 — kitchen and per-table
confirmation with timeouts, cancellation checks, and skip-and-continue
behaviour for multiple tables. Milestone 1 is the same flow with the
confirmation/cancellation checks removed.

![Generic order flow](docs/generic_flow.png)

## Milestones → Implementation

| # | Behaviour | Tree file | Key mechanism |
|---|---|---|---|
| 1 | Basic delivery, no confirmations | `m1.xml` | Plain `Sequence`: kitchen → table → home. |
| 2 | Wait for confirmation at kitchen & table; any timeout → home | `m2.xml` | `Fallback` around a `Sequence` of `WaitForConfirmation` nodes. |
| 3 | Kitchen timeout → home; table timeout → **via kitchen**, then home | `m3.xml` | Nested `Fallback` gives the table-timeout branch a different route than the kitchen-timeout branch. |
| 4 | Cancel while en route → home (kitchen leg) or via kitchen (table leg) | `m4.xml` | `ReactiveSequence` + `IsTableCancelled` interrupts `NavigateToWaypoint` mid-flight. |
| 5 | Multiple tables, one kitchen visit, deliver to all | `m5.xml` | `ForEachTable` loops over the goal's table list. |
| 6 | Skip a table that never confirms, continue to the rest, kitchen before home | `m6.xml` | Same loop; `WaitForConfirmation` failing just means `ForEachTable` marks that table skipped and moves on. |
| 7 | Skip a **cancelled** table, continue to the rest | `m7.xml` | Same loop; `Inverter(IsTableCancelled)` gates the per-table subtree. |
| bonus | All rules combined | `full.xml` | One production-style tree combining cancellation, confirmation/timeout, and multi-table looping — no per-milestone code, just composition. |

Milestones 1–4 are single-table trees (they read `current_table` directly,
seeded from the first table in the goal); milestones 5–7 use `ForEachTable`
over the full `tables_queue`. Both shapes share the same five nodes.

## ROS2 Interfaces

### Action: `/butler_order` (`cafe_butler_interfaces/action/OrderTask`)

| Field | Type | Meaning |
|---|---|---|
| `Goal.tables` | `string[]` | Tables to serve, e.g. `["table1"]` or `["table1","table2","table3"]`. |
| `Goal.order_id` | `string` | Free-form identifier for logging. |
| `Feedback.state` | `string` | Current step, e.g. `MOVING_TO_KITCHEN`, `WAITING_TABLE_CONFIRM`. |
| `Feedback.current_table` | `string` | Table currently being handled. |
| `Result.success` | `bool` | `true` if the tree finished in the `SUCCESS` branch. |
| `Result.delivered_tables` | `string[]` | Tables successfully served. |
| `Result.skipped_tables` | `string[]` | Tables skipped (timeout or cancellation). |

### Topics

| Topic | Type | Payload |
|---|---|---|
| `/confirmation` | `std_msgs/String` | `"kitchen"`, `"table1"`, `"table2"`, or `"table3"` |
| `/cancel_order` | `std_msgs/String` | A table name, or `"all"` to cancel the whole order |

Both topics are consumed internally by `OrderTracker` — the Behavior Tree
nodes never touch ROS topics directly, they just ask `OrderTracker` "has
table1 been confirmed?" or "was table2 cancelled?".

## Prerequisites

- Ubuntu 22.04 + ROS2 Humble
- Nav2 (`ros-humble-navigation2`, `ros-humble-nav2-bringup`)
- BehaviorTree.CPP **v4** (`ros-humble-behaviortree-cpp` — *not* `-v3`)
- `python3-tk` (for the GUI)
- A workspace with `colcon`

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-behaviortree-cpp python3-tk
```

## Build

```bash
cd ~/cafe_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Sanity-check the executable actually got installed and can find its library:

```bash
ros2 pkg executables cafe_butler_bt        # should list butler_bt_navigator
ldd install/cafe_butler_bt/lib/cafe_butler_bt/butler_bt_navigator | grep cafe_butler
```

## Configuration (Waypoints)

`cafe_butler_bt/config/waypoints.yaml` defines `home`, `kitchen`, `table1`,
`table2`, `table3` as `[x, y]` positions plus full quaternion orientations,
under the `butler_bt_navigator` parameter namespace:

```yaml
butler_bt_navigator:
  ros__parameters:
    waypoint_names: ["home", "kitchen", "table1", "table2", "table3"]
    waypoints:
      home:
        position: [-10.49, 5.72]
        orientation: [0.0, 0.0, 0.81, 0.9]   # keep this a unit quaternion!
      kitchen:
        position: [-4.6, 1.22]
        orientation: [0.0, 0.0, 0.69, 0.71]
      # ...
```

Adding `table4` is a **config-only** change: add its position/orientation
here and append it to `waypoint_names` — no code or tree edits needed.

Confirmation timeouts (default 15s) are set per-node inside each XML tree and
can be tuned per milestone without touching C++.

## Running

```bash
# 1) bring up the robot + Nav2 stack (map, AMCL/SLAM, controller, etc.)
ros2 launch cafe_bot_description_fixed bringup.launch.py
ros2 launch cafe_bot_description_fixed nav.launch.py     # Nav2, with a saved map

# 2) start the BT executor for a chosen milestone
ros2 launch cafe_butler_bt butler_bt.launch.py milestone:=m5

# 3) start the order / confirm / cancel simulator GUI
ros2 run cafe_butler_gui order_simulator_gui
```

`milestone` accepts any file name (without `.xml`) in `behavior_trees/`:
`m1` … `m7`, or `full`.

## Testing with the GUI

The GUI (`order_simulator_gui`) has three sections:

1. **Host** — tick one or more tables, click **Send Order** to fire the
   `/butler_order` action goal. **Cancel goal (ROS level)** issues a
   ROS-level action cancel.
2. **Kitchen** — **Confirm** (food handed to robot) and **Cancel whole
   order**.
3. **Per table** — **Confirm receipt** and **Cancel this table**.

A status/feedback panel streams the action's feedback (`state`,
`current_table`) live as the tree runs, so you can watch exactly which
branch of the tree fired.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `package 'cafe_butler_bt' found ... but libexec directory ... does not exist` | `CMakeLists.txt` is missing an `install(TARGETS ...)` block, so the executable compiles but never gets installed. | Add `install(TARGETS butler_bt_navigator RUNTIME DESTINATION lib/${PROJECT_NAME})`. |
| `error while loading shared libraries: libcafe_butler_bt_nodes.so: cannot open shared object file` | The shared library was installed to `lib/cafe_butler_bt/` (executable's folder), which isn't on `LD_LIBRARY_PATH`. | Install the library to `lib/` (`LIBRARY DESTINATION lib`) and only the executable to `lib/${PROJECT_NAME}`. |
| Build fails looking for `behaviortree_cpp_v3` symbols/headers | `behaviortree_cpp` (v4) and `behaviortree_cpp_v3` are separate apt packages; this project targets **v4**. | `ros2 pkg list \| grep -i behaviortree` and `dpkg -l \| grep behaviortree` to confirm `ros-humble-behaviortree-cpp` (v4) is installed, not just `-v3`. |
| Nav2 goal never completes / `Nav2 action server not available` | Nav2 bringup not running, or no map loaded. | Launch `nav2_bringup`/your nav launch file with a saved map before starting `butler_bt_navigator`. |

## Known Limitations / Future Work

- Confirmation timeouts are fixed in the XML; could be promoted to launch
  arguments for runtime tuning.
- The action's failure `message` is currently generic (`"Order failed"`);
  the specific fallback branch that fired could be threaded through the
  blackboard for richer diagnostics.
- Only one order can be in flight at a time — queuing a second order while
  one is running isn't handled yet.
- Nav2's own bringup (AMCL, planner, controller) is assumed to already be
  running with a saved map; it isn't launched by `cafe_butler_bt` itself.


