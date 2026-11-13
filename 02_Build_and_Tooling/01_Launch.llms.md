# Launch

Launch files in Python, YAML and XML; substitutions and conditions; namespaces and remapping across a whole bringup; and composable node containers.

Author

Benedict Thekkel

## What Launch Is For

A robot is tens of nodes with parameters, remappings and namespaces. `ros2 run` starts one. `ros2 launch` starts a described system, and the description is code rather than a script: launch builds a set of actions, then executes and supervises them.

Three formats are supported. **Python** is the default and the only one with full expressive power (loops, conditionals, computed paths, event handlers). **YAML** and **XML** are declarative and easier to read for a flat list of nodes. Python is what most real bringups use, and what the rest of this notebook shows.

``` python
# launch/bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="my_pkg", executable="drive", name="drive",
             parameters=[{"max_speed": 0.8}],
             remappings=[("cmd_vel", "/robot1/cmd_vel")],
             output="screen"),
        Node(package="rviz2", executable="rviz2", output="log"),
    ])
```

The function **must** be named `generate_launch_description` and return a `LaunchDescription`. `output="screen"` is what makes a node’s stdout visible; the default sends it to a log file, which is why a crashing node can look like it simply did nothing.

The launch file must also be installed (`data_files` in `setup.py`, or `install(DIRECTORY launch ...)` in CMake), or `ros2 launch my_pkg bringup.launch.py` will not find it. See [00_Workspaces_and_Packages.ipynb](../02_Build_and_Tooling/workspaces_and_packages.llms.md).

------------------------------------------------------------------------

## Substitutions

A launch file is evaluated in two stages: building the description, then executing it. Values that are not known at build time are **substitutions**, resolved at execution. Using a plain Python string where a substitution is needed is the most common launch bug, and it usually shows up as a literal `LaunchConfiguration` object in a path.

``` python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  TextSubstitution, EnvironmentVariable, Command)
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim_time")
    robot   = LaunchConfiguration("robot_name")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                             description="use /clock instead of wall time"),
        DeclareLaunchArgument("robot_name", default_value="robot1"),

        Node(package="my_pkg", executable="drive",
             namespace=robot,
             parameters=[
                 PathJoinSubstitution([FindPackageShare("my_pkg"), "config", "drive.yaml"]),
                 {"use_sim_time": use_sim},
             ]),
    ])
```

| Substitution | Resolves to |
|----|----|
| `LaunchConfiguration("x")` | the value of launch argument `x` |
| `FindPackageShare("pkg")` | that package’s `share/` directory |
| `PathJoinSubstitution([a, b])` | a path, built from other substitutions |
| `EnvironmentVariable("HOME")` | an environment variable |
| `Command(["xacro ", path])` | the stdout of a shell command |
| `TextSubstitution(text="1")` | a literal, where a substitution type is required |

`Command` is how a xacro file becomes a `robot_description` parameter at launch time:

``` python
robot_description = Command(["xacro ", PathJoinSubstitution(
    [FindPackageShare("my_description"), "urdf", "robot.urdf.xacro"])])
```

Note the trailing space in `"xacro "`. The list is concatenated without separators, so omitting it produces `xacro/path/to/file` and an unhelpful “command not found”. See [../03_Spatial_and_Temporal/01_Robot_Description.ipynb](../03_Spatial_and_Temporal/robot_description.llms.md).

Launch arguments are passed on the command line as `name:=value`:

``` bash
ros2 launch my_pkg bringup.launch.py use_sim_time:=true robot_name:=robot2
ros2 launch my_pkg bringup.launch.py --show-args      # what this file accepts
```

------------------------------------------------------------------------

## Conditions, Includes and Groups

Conditions take a substitution, not a Python boolean, because the value is not known until execution:

``` python
from launch.conditions import IfCondition, UnlessCondition

Node(package="rviz2", executable="rviz2",
     condition=IfCondition(LaunchConfiguration("gui"))),
Node(package="my_pkg", executable="headless_monitor",
     condition=UnlessCondition(LaunchConfiguration("gui"))),
```

`if LaunchConfiguration("gui"):` in plain Python is always true (it is an object), which silently starts both nodes. That is the condition trap.

Composing bringups from other packages’ launch files:

``` python
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

GroupAction([
    PushRosNamespace("robot1"),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])),
        launch_arguments={"use_sim_time": "true",
                          "params_file": params}.items()),
])
```

`GroupAction` scopes what is inside it, so `PushRosNamespace` affects only that group. This is the multi-robot pattern: the same included launch file, instantiated twice under two namespaces.

Two scoping rules to keep in mind. `launch_arguments` values must be **strings or substitutions**, never Python objects, and an argument the included file does not declare is silently ignored rather than rejected. And parameters in a YAML file are keyed by the node’s **fully qualified** name, so pushing a namespace means the YAML keys need the namespace too, or a `/**:` wildcard; this is the usual reason a namespaced bringup ignores its parameter file. See [../01_Core_Concepts/02_Interfaces_and_Parameters.ipynb](../01_Core_Concepts/interfaces_and_parameters.llms.md).

Other actions worth knowing:

``` python
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit

ExecuteProcess(cmd=["ros2", "bag", "record", "-a"], output="screen"),
TimerAction(period=5.0, actions=[Node(...)]),        # crude startup ordering
RegisterEventHandler(OnProcessExit(target_action=calib, on_exit=[Node(...)])),
```

`TimerAction` is the usual way people order a bringup and it is a guess, not a guarantee. A lifecycle node plus a manager is the real answer for anything that must be up before the next thing starts; see [../01_Core_Concepts/04_Executors_Lifecycle_and_Composition.ipynb](../01_Core_Concepts/executors_lifecycle_and_composition.llms.md).

------------------------------------------------------------------------

## Namespaces and Remapping in a Bringup

Three mechanisms do overlapping jobs, and knowing which to reach for avoids a bringup with half-namespaced topics.

| Mechanism | Scope | Use for |
|----|----|----|
| `namespace=` on a `Node` | one node | placing a single node |
| `PushRosNamespace` in a `GroupAction` | everything in the group, includes too | multi-robot, whole subsystems |
| `remappings=` on a `Node` | one name on one node | connecting a node to a differently named topic |

``` python
Node(package="my_pkg", executable="drive", namespace="robot1",
     remappings=[("cmd_vel", "cmd_vel_nav"),        # relative: /robot1/cmd_vel_nav
                 ("/tf", "tf"), ("/tf_static", "tf_static")]),
```

The `/tf` remapping is the idiom worth copying. tf2 publishes to the absolute `/tf`, so two robots in separate namespaces would otherwise write into one shared transform tree and fight over frame names. Remapping `/tf` to the relative `tf` pushes it into the namespace.

Global topics that should **not** be namespaced: `/clock`, and usually `/map` when several robots share one map. Everything else belongs inside.

------------------------------------------------------------------------

## Composable Node Containers

Launching components into one process, rather than a process per node, is what unlocks intra-process zero copy for large messages (see [../01_Core_Concepts/04_Executors_Lifecycle_and_Composition.ipynb](../01_Core_Concepts/executors_lifecycle_and_composition.llms.md)):

``` python
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

ComposableNodeContainer(
    name="perception_container",
    namespace="",
    package="rclcpp_components",
    executable="component_container_mt",          # mt: multi-threaded executor
    composable_node_descriptions=[
        ComposableNode(package="image_proc", plugin="image_proc::DebayerNode",
                       name="debayer",
                       extra_arguments=[{"use_intra_process_comms": True}]),
        ComposableNode(package="image_proc", plugin="image_proc::RectifyNode",
                       name="rectify",
                       remappings=[("image", "image_raw")],
                       extra_arguments=[{"use_intra_process_comms": True}]),
    ],
    output="screen")
```

`extra_arguments=[{"use_intra_process_comms": True}]` on **every** component in the chain is what enables the zero-copy path. Omit it on one and that hop quietly falls back to normal DDS, with no warning and no error: the pipeline still works, just with the copies you were trying to avoid.

`LoadComposableNodes` adds components to a container started elsewhere, which is how a driver package contributes into an application’s container:

``` python
LoadComposableNodes(
    target_container="/perception_container",
    composable_node_descriptions=[ComposableNode(...)])
```

Debugging tip: a component that throws during construction takes the **whole container** down, and with it every other node in it. When a container dies on startup, load the components one at a time (`ros2 component load`) to find which one.

------------------------------------------------------------------------

Back to top
