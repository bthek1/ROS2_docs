# MoveIt 2 Overview

What MoveIt 2 is made of, the planning scene and collision objects, the SRDF and what the Setup Assistant generates, and how a plan reaches the hardware.

Author

Benedict Thekkel

## The Architecture

MoveIt 2 is to manipulation what Nav2 is to navigation: a stack that turns “put the gripper there” into joint trajectories, while avoiding collisions.

      your code  --- MoveGroupInterface / moveit_py / MoveGroup action --->
                             move_group node
                           /       |        \
              planning pipeline  planning scene  trajectory execution
              (OMPL, Pilz, ...)  (robot + world  (joint_trajectory_controller
                                  + collisions)   via ros2_control)
                                  |
                            kinematics plugin (IK/FK)

`move_group` is the central node. It holds the planning scene, calls a planner, validates the result, and hands the trajectory to a controller. Everything else is a plugin: the planner, the kinematics solver, the collision checker, and the controller interface.

``` bash
sudo apt install ros-jazzy-moveit
ros2 launch moveit2_tutorials demo.launch.py        # the Panda demo, if installed
```

The practical consequence of that plugin structure: **MoveIt is configured, not programmed.** A new arm needs a config package, not code, and most problems are in YAML rather than in C++.

------------------------------------------------------------------------

## The Planning Scene

The planning scene is MoveIt’s model of the world at this instant: the robot’s current state, plus everything it must not hit.

It has three parts:

- **The robot model**, from the URDF and SRDF.
- **The robot state**, from `/joint_states`.
- **The world**: collision objects you add, plus an **octomap** built from sensor data if a sensor is configured.

``` python
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

box = CollisionObject()
box.id = "table"
box.header.frame_id = "base_link"          # a frame tf2 knows about
primitive = SolidPrimitive()
primitive.type = SolidPrimitive.BOX
primitive.dimensions = [1.0, 0.6, 0.02]    # metres
box.primitives = [primitive]
box.primitive_poses = [Pose(position=Point(x=0.5, y=0.0, z=0.3))]
box.operation = CollisionObject.ADD
scene_pub.publish(box)                     # /planning_scene or the PlanningSceneInterface
```

Four facts that account for most collision surprises:

- **A plan is only as good as the scene.** MoveIt avoids what it knows about. An unmodelled table is not avoided, and the arm will plan straight through it.
- **`ATTACHED` objects move with the gripper.** After a grasp, attach the object so the planner accounts for its shape; forget to, and the arm swings the held object into the table.
- **The octomap remembers.** Sensor-built occupancy persists until cleared, so a hand that passed through the workspace leaves an obstacle behind. `/clear_octomap` is the service.
- **The gripper sees itself.** Self-collision is checked using the SRDF’s disable list; a missing entry makes a valid pose unreachable, and a spurious one lets the arm collide with itself.

``` bash
ros2 service call /clear_octomap std_srvs/srv/Empty
ros2 topic echo /monitored_planning_scene --once
```

------------------------------------------------------------------------

## The SRDF

The URDF says what the robot is. The **SRDF** says how to use it, and it is what the Setup Assistant mostly generates.

``` xml
<robot name="my_arm">
  <group name="arm">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>
  <group name="gripper">
    <link name="finger_left"/>
    <link name="finger_right"/>
  </group>

  <group_state name="home" group="arm">
    <joint name="shoulder" value="0"/>
    <joint name="elbow" value="-1.57"/>
  </group_state>

  <end_effector name="hand" parent_link="tool0" group="gripper"/>

  <disable_collisions link1="base_link" link2="shoulder_link" reason="Adjacent"/>
  <disable_collisions link1="forearm" link2="wrist" reason="Never"/>
</robot>
```

| Element | Is |
|----|----|
| `group` | a named set of joints you plan for (“arm”, “gripper”) |
| `group_state` | a named pose (“home”, “ready”), usable as a planning target |
| `end_effector` | which group is the gripper, and where it attaches |
| `virtual_joint` | how the robot attaches to the world (fixed to a table, or floating on a mobile base) |
| `disable_collisions` | pairs never checked, with a reason |

The `disable_collisions` list is the part that matters and the part that is machine-generated: the Setup Assistant samples random configurations and disables pairs that are adjacent, always colliding, or never colliding. **It must be regenerated when the URDF changes.** A stale list after adding a gripper is a classic cause of an arm that declares every pose to be in self-collision.

`virtual_joint` is the other recurring trap: an arm on a mobile base needs a `floating` or `planar` virtual joint to `base_link`, and a fixed one pins the arm to the world so the planner fights the base’s motion.

------------------------------------------------------------------------

## The Setup Assistant and the Config Package

``` bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

It takes a URDF and produces a `*_moveit_config` package:

    my_arm_moveit_config/
      config/
        my_arm.srdf                      groups, poses, collision pairs
        kinematics.yaml                  which IK plugin per group
        joint_limits.yaml                velocity and acceleration limits for planning
        moveit_controllers.yaml          how to reach the ros2_control controllers
        ompl_planning.yaml               planner parameters
        initial_positions.yaml
      launch/
        demo.launch.py, move_group.launch.py, moveit_rviz.launch.py

`moveit_controllers.yaml` is the bridge to hardware, and the names must match the controllers that `ros2_control` actually loads:

``` yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
moveit_simple_controller_manager:
  controller_names:
    - arm_controller
  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints: [shoulder, elbow, wrist_1, wrist_2, wrist_3]
```

**This is where a plan that looks fine in RViz fails to move the robot.** The controller name, its action namespace and its joint list must all match `ros2_control`’s configuration; see [../04_Simulation_and_Hardware/02_ros2_control.ipynb](../04_Simulation_and_Hardware/ros2_control.llms.md). The usual symptom is MoveIt reporting that it found no controller for the joints it planned for.

Two further points on `joint_limits.yaml`: MoveIt’s limits are used for time parameterisation, so they can be *lower* than the URDF’s to produce gentler motion, and leaving acceleration limits unset produces trajectories the controller cannot track.

------------------------------------------------------------------------

## Planning and Executing

``` python
# C++ is the first-class API; moveit_py is the Python binding
move_group = MoveGroupInterface(node, "arm")
move_group.set_pose_target(target_pose)
plan = move_group.plan()
if plan:
    move_group.execute(plan)
```

The stages a request passes through, because each is a place it can fail:

1.  **Planning request adapters** fix up the start state (for example, a joint slightly outside its limit).
2.  **The planner** searches for a collision-free path (see [05_Kinematics_and_Motion_Planning.ipynb](../06_Navigation_and_Manipulation/kinematics_and_motion_planning.llms.md)).
3.  **Time parameterisation** turns the path into a trajectory honouring velocity and acceleration limits.
4.  **Validation** re-checks the trajectory against the current scene.
5.  **Execution** sends a `FollowJointTrajectory` goal to the controller.

``` bash
ros2 topic echo /display_planned_path           # what RViz draws
ros2 action list | grep -i trajectory
ros2 topic echo /joint_states --once
```

| Symptom | Cause |
|----|----|
| “Unable to sample any valid states” | goal in collision, or unreachable; check the scene and IK |
| plans but will not execute | controller name or action namespace mismatch |
| execution aborts immediately | trajectory violates the controller’s constraints, or `joint_states` stale |
| every pose reports self-collision | stale or missing `disable_collisions` after a URDF change |
| arm plans through a table | the table is not in the planning scene |
| plan jitters or takes odd routes | normal for sampling planners; see the smoothing note in the next notebook |
| “No controller found for joints” | `moveit_controllers.yaml` joint list does not match the group |

RViz’s MotionPlanning panel is the main interactive tool: drag the interactive marker to a goal, plan, and watch it. It is also the fastest way to tell a kinematics problem (the marker turns red, no IK solution) from a planning problem (IK fine, no collision-free path).

------------------------------------------------------------------------

Back to top
