# Café Butler Robot — Behavior Tree Implementation

Implements the 7 milestones from the ROS Developer assessment using
**BehaviorTree.CPP v4** + **Nav2**, on top of the existing
`experiment8b_myrobot` / `experiment8b_myrobot_bringup` simulation
(URDF, Gazebo café world, SLAM, EKF).

## Why a Behavior Tree, and why it's generic

Every milestone is really the *same* workflow (home → kitchen → table(s) →
home) with extra rules layered on: waiting for people, timeouts,
cancellations, and multiple tables. Instead of writing 7 separate state
machines, this implementation has:

- **5 reusable BT leaf/control nodes** written once in C++ (`cafe_butler_bt`
  package) that know nothing about "milestones" — they only know how to
  navigate to a *named* waypoint, wait for a confirmation at a *named*
  location, check if a *named* table was cancelled, and iterate over
  *whatever* tables were requested.
- **8 small BT XML files** (one per milestone + one bonus "full" tree) that
  compose those same nodes differently. Adding milestone 8 tomorrow, or a
  4th table, means writing/adjusting an XML tree and a line in
  `waypoints.yaml` — never touching the C++ code.

This is the "generic, not hardcoded" approach the brief asks for: the
algorithm doesn't know about `table1`/`table2`/`table3` specifically, it
only knows about "whatever tables are in the goal".

## Package layout

```
cafe_butler_ws/src/
├── cafe_butler_interfaces/        # OrderTask.action (goal/result/feedback)
├── cafe_butler_bt/                # the BT nodes + executor node + trees
│   ├── include/cafe_butler_bt/
│   │   ├── order_tracker.hpp      # subscribes /confirmation, /cancel_order
│   │   ├── waypoint_server.hpp    # loads named waypoints from params
│   │   └── bt_nodes.hpp           # declarations of the 5 custom BT nodes
│   ├── src/
│   │   ├── bt_nodes.cpp           # node implementations + registration
│   │   └── bt_executor_node.cpp   # action server that ticks the chosen tree
│   ├── behavior_trees/            # milestone1_*.xml ... milestone7_*.xml,
│   │                               milestone_full_generic.xml
│   ├── config/waypoints.yaml      # home/kitchen/table1-3 poses (map frame)
│   └── launch/butler_bt.launch.py # ros2 launch cafe_butler_bt butler_bt.launch.py milestone:=...
└── cafe_butler_gui/                # Tkinter GUI: host order + kitchen/table
                                     # confirm buttons + cancel buttons
```

## The 5 custom BT nodes

| Node | Type | Ports | What it does |
|---|---|---|---|
| `NavigateToWaypoint` | Stateful action | `waypoint` (string) | Sends a Nav2 `NavigateToPose` goal for a named waypoint, RUNNING until arrival, cancels the Nav2 goal if halted. |
| `WaitForConfirmation` | Stateful action | `location`, `timeout` | Polls `OrderTracker` until someone confirms at `location`, SUCCESS/FAILURE on confirm/timeout. |
| `IsTableCancelled` | Condition | `table` | SUCCESS if that table (or the whole order) was cancelled. |
| `ForEachTable` | Control node | — | Iterates `tables_queue` from the blackboard; ticks its one child once per table; child SUCCESS → "delivered", child FAILURE → "skipped", and it keeps going either way. This is what makes 1-table and N-table orders use the same subtree. |
| `PublishFeedback` | Sync action | `state` | Writes a human-readable state string that the executor node forwards as ROS action feedback (so the GUI can show what the robot is doing). |

All ROS-facing state (topic subscriptions, Nav2 action client) lives in
`OrderTracker` / `WaypointServer` / `NavigateToWaypoint`, not scattered
across the tree — the XML trees only ever reason about pure BT logic.

## Milestone → XML mapping

| Milestone | File | Key idea |
|---|---|---|
| 1 | `milestone1_basic_delivery.xml` | home→kitchen→table→home, no confirmations. |
| 2 | `milestone2_wait_and_timeout.xml` | Confirmation needed at kitchen **and** table; any timeout aborts straight home. |
| 3 | `milestone3_kitchen_vs_table_timeout.xml` | Kitchen timeout → home; table timeout → kitchen **then** home (nested `Fallback`). |
| 4 | `milestone4_cancel_handling.xml` | `ReactiveSequence` + `IsTableCancelled` interrupts an in-progress `NavigateToWaypoint`; cancel-at-kitchen-leg → home, cancel-at-table-leg → kitchen then home. |
| 5 | `milestone5_multi_table.xml` | One kitchen visit, `ForEachTable` delivers to every table in the goal, then home. |
| 6 | `milestone6_skip_on_no_confirmation.xml` | Same loop, each table now requires confirmation; a timed-out table is auto-skipped by `ForEachTable`'s FAILURE handling; kitchen visited again before home. |
| 7 | `milestone7_skip_cancelled_table.xml` | Same loop, `Inverter(IsTableCancelled)` skips a cancelled table but still serves the rest. |
| bonus | `milestone_full_generic.xml` | All of the above rules combined into one production-grade tree. |

Because milestones 1–4 use a single implicit table and 5–7 use the table
*list* from the goal, the executor seeds the blackboard with both
`current_table` (= first table, for 1–4) and `tables_queue` (= full list,
for `ForEachTable` in 5–7), so the same nodes serve both shapes.

## ROS interfaces

- **Action** `/butler_order` (`cafe_butler_interfaces/action/OrderTask`)
  Goal: `string[] tables`, `string order_id`.
  Feedback: `state`, `current_table`.
  Result: `success`, `delivered_tables[]`, `skipped_tables[]`, `message`.
- **Topic** `/confirmation` (`std_msgs/String`) — payload `"kitchen"`,
  `"table1"`, `"table2"`, or `"table3"`.
- **Topic** `/cancel_order` (`std_msgs/String`) — payload a table name or
  `"all"`.

The GUI (`cafe_butler_gui`) is the only thing that talks on these — it
stands in for the host, kitchen staff, and customers.

## Build & run

```bash
# from the workspace root
colcon build --symlink-install
source install/setup.bash

# 1) bring up the robot + Nav2 stack (map, AMCL/SLAM, controller, etc.)
#    using your existing bringup launch, e.g.:
ros2 launch experiment8b_myrobot_bringup bringup.launch.xml
# ... plus your Nav2 bringup (nav2_bringup navigation_launch.py) once a map exists

# 2) start the BT executor for a given milestone
ros2 launch cafe_butler_bt butler_bt.launch.py milestone:=milestone5_multi_table

# 3) start the order/confirm/cancel simulator GUI
ros2 run cafe_butler_gui order_simulator_gui
```

Then in the GUI: tick tables → "Send Order", and use the Kitchen/Table
"Confirm"/"Cancel" buttons to exercise the milestone's rules while watching
the feedback log and the robot in RViz/Gazebo.

## A note on the waypoints you provided

`config/waypoints.yaml` uses your coordinates as-is. One flag: `home`'s
orientation quaternion is `[0.0, 0.0, 8.1, 0.9]` — `qz = 8.1` isn't a valid
unit quaternion component (should be in `[-1, 1]`), so Nav2 may reject or
mis-orient the final `home` pose. It looks like a typo for `0.81` — worth
double-checking against wherever these were captured (e.g. an RViz
"Publish Pose" tool) before relying on the robot's final heading at home.

## Suggested git workflow (per the brief's "documented approach for each milestone")

```
git checkout -b milestone-1 && ...commit milestone1 tree + any node changes... && git push
git checkout -b milestone-2 && ...
```
Each PR/commit message can point at the relevant XML file and reference this
README's mapping table, so a reviewer can cross a milestone off by reading
one file plus a couple of paragraphs.

## Known simplifications / next steps

- `WaitForConfirmation` timeouts are hardcoded in the XML (`timeout="15.0"`);
  trivial to promote to a launch argument if you want it tunable at runtime.
- Nav2 (AMCL/planner/controller) bringup itself isn't included here since
  your `experiment8b_myrobot_bringup` package already has SLAM/EKF; you'll
  need a `navigation_launch.py` (from `nav2_bringup`) with a saved map for
  `NavigateToPose` to actually be servable.
- The action's `message` field on failure is generic ("Order failed"); it
  would be easy to thread a more specific reason (which fallback branch
  fired) through the blackboard if you want richer diagnostics.
