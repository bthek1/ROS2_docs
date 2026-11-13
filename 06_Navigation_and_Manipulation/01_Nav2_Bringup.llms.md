# Nav2 Bringup

What Nav2 is made of, the lifecycle manager that starts it in order, the parameter file that configures all of it, and the bring-up failures worth recognising.

Author

Benedict Thekkel

## What Nav2 Is

Nav2 is not a node. It is a dozen lifecycle nodes, a behaviour tree that coordinates them, and one large parameter file.

                     nav2_bt_navigator          runs the behaviour tree, serves NavigateToPose
                       /        |       \
         planner_server   controller_server   behavior_server
         (global path)    (local velocities)  (spin, backup, wait)
                |                 |
         global_costmap     local_costmap      both fed by sensors via costmap layers
                |
           map_server  +  amcl                 the map, and the pose in it

         waypoint_follower, velocity_smoother, collision_monitor   optional, commonly used
         lifecycle_manager                     brings all of the above up in a defined order

The request path for a single goal: an action client sends `NavigateToPose` to `bt_navigator`, which ticks a behaviour tree; the tree calls `ComputePathToPose` on `planner_server` and `FollowPath` on `controller_server`; `controller_server` publishes `/cmd_vel`. Each of those is an action server in its own right, which is why each can be exercised alone when something is wrong.

``` bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=nav2.yaml
ros2 launch nav2_bringup localization_launch.py map:=my_map.yaml use_sim_time:=true
```

`navigation_launch.py` starts the navigation stack; `localization_launch.py` starts map server plus AMCL. They are separate because SLAM replaces the second one.

------------------------------------------------------------------------

## The Lifecycle Manager

Every Nav2 server is a lifecycle node ([../01_Core_Concepts/04_Executors_Lifecycle_and_Composition.ipynb](../01_Core_Concepts/executors_lifecycle_and_composition.llms.md)), and `lifecycle_manager` is what configures and activates them in a fixed order.

``` yaml
lifecycle_manager_navigation:
  ros__parameters:
    autostart: true
    node_names: ["controller_server", "smoother_server", "planner_server",
                 "behavior_server", "bt_navigator", "waypoint_follower",
                 "velocity_smoother"]
    bond_timeout: 4.0
```

This is the best thing about Nav2’s architecture from a debugging standpoint: **bring-up is ordered and observable**, so a stack that fails to start names the node that refused.

``` bash
ros2 lifecycle nodes
ros2 lifecycle get /planner_server
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```

Two mechanisms to understand:

- **`autostart: true`** activates everything at launch. Setting it false and activating manually is how you bring up a stack piece by piece when diagnosing.
- **The bond.** Each managed node holds a heartbeat with the manager. If a node dies or stalls longer than `bond_timeout`, the manager **deactivates the whole stack** deliberately, so a half-dead navigation system stops the robot rather than driving it with a stale plan. A stack that shuts itself down during heavy CPU load is usually the bond timing out rather than a crash, and raising `bond_timeout` is the fix only after confirming the node really is just slow.

------------------------------------------------------------------------

## The Parameter File

One YAML configures the whole stack, keyed by node name. The structure matters more than any individual value:

``` yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    default_nav_to_pose_bt_xml: ""        # empty: use the built-in tree

planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    failure_tolerance: 0.3
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
```

Three structural points that account for most parameter-file pain:

- **Plugin names are parameter namespaces.** `controller_plugins: ["FollowPath"]` declares a name, and the block called `FollowPath` configures it. Renaming one without the other leaves a plugin with default parameters and no error. The behaviour tree also refers to controllers by this name.
- **Keys must match the fully qualified node name.** Under a namespace the keys need the namespace too, or `/**:` as a wildcard. A namespaced bringup silently running on defaults is this. See [../01_Core_Concepts/02_Interfaces_and_Parameters.ipynb](../01_Core_Concepts/interfaces_and_parameters.llms.md).
- **Start from `nav2_bringup`’s shipped `nav2_params.yaml`** and change what you need. It is long because the stack is configurable, and writing one from scratch means discovering which of the hundred defaults you needed.

`progress_checker` and `goal_checker` deserve a mention because they produce confusing behaviour when mistuned: a robot that aborts mid-corridor is usually the progress checker concluding it has not moved `required_movement_radius` within `movement_time_allowance`, which happens legitimately when the robot is slow or the allowance is short.

------------------------------------------------------------------------

## Sending Goals

``` bash
# a single goal
ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"

# a route through waypoints
ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints "{poses: [...]}"
```

From Python, the `nav2_simple_commander` API is the practical interface and wraps the action clients:

``` python
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

nav = BasicNavigator()
nav.waitUntilNav2Active()              # blocks until the lifecycle manager reports active
nav.goToPose(goal_pose)
while not nav.isTaskComplete():
    feedback = nav.getFeedback()       # distance remaining, time elapsed
nav.getResult()                        # TaskResult.SUCCEEDED / CANCELED / FAILED
```

`waitUntilNav2Active()` is the call that saves the most time: sending a goal before the stack is active fails in a way that looks like a rejected goal.

Note that `-f` on `ros2 action send_goal` streams feedback, which is how you tell “planning and moving” from “accepted and stalled”. Goal states and preemption are in [../01_Core_Concepts/01_Services_and_Actions.ipynb](../01_Core_Concepts/services_and_actions.llms.md); Nav2’s servers preempt rather than queue, which is what you want for a single base.

------------------------------------------------------------------------

## Bring-up Failures

Nav2’s failures are mostly not in Nav2. In the order worth checking:

``` bash
ros2 lifecycle get /bt_navigator               # is the stack actually active
ros2 run tf2_tools view_frames                 # map -> odom -> base_link, once each
ros2 topic hz /scan /odom /tf /cmd_vel
ros2 topic echo /cmd_vel --once                # is anything being commanded
ros2 topic echo /local_costmap/costmap --once   # is the costmap populated
```

| Symptom | Cause |
|----|----|
| nodes stay `unconfigured` | parameter file keys do not match node names |
| stack activates then deactivates | bond timeout: a node stalled, often under CPU load |
| goal rejected immediately | stack not active, or goal frame is not `map` |
| “Failed to get robot pose” | no `map -> base_link` chain; usually no localiser or no initial pose |
| plan found, robot does not move | `/cmd_vel` not reaching the base, or a velocity smoother or collision monitor blocking it |
| robot spins in place | costmap says it is surrounded; check sensor frames and the inflation radius |
| “No valid path found” next to an obstacle | footprint or inflation larger than the free space; see [02_Costmaps_Planners_and_Controllers.ipynb](../06_Navigation_and_Manipulation/costmaps_planners_and_controllers.llms.md) |
| aborts partway with no obstacle | progress checker; `required_movement_radius` too large or allowance too short |
| works in simulation, not on hardware | `use_sim_time` left true, or sensor QoS incompatible |

Two of those are worth singling out. **`/cmd_vel` is a chain, not a topic**: in a default Jazzy bringup `controller_server` publishes to `velocity_smoother` and possibly through `collision_monitor` before anything reaches the base, so a robot that will not move with a valid plan is often a renamed topic mid-chain. And **Nav2 is a heavy CPU consumer**: MPPI on four cores alongside SLAM and a perception pipeline is enough to cause the bond timeouts above, which is a capacity problem presenting as a reliability one.

------------------------------------------------------------------------

Back to top
