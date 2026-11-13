# Interfaces and Parameters

The type system - the standard message packages, and how to define custom messages, services and actions - plus per-node parameters, declaration, validation and YAML overrides.

Author

Benedict Thekkel

## The Standard Interface Packages

Messages are defined in `.msg` files and generated into C++ and Python at build time. Four packages cover most of what a robot needs, and using the standard type instead of inventing one is what makes a node interoperate with the rest of the ecosystem.

| Package | Holds | Examples |
|----|----|----|
| `std_msgs` | primitives and `Header` | `String`, `Bool`, `Float64`, `Header` |
| `geometry_msgs` | poses, twists, transforms | `Twist`, `PoseStamped`, `TransformStamped`, `Point` |
| `sensor_msgs` | sensor data | `LaserScan`, `Image`, `PointCloud2`, `Imu`, `JointState`, `CameraInfo` |
| `nav_msgs` | navigation | `Odometry`, `Path`, `OccupancyGrid` |

A few others appear constantly: `std_srvs` (`Empty`, `SetBool`, `Trigger`), `trajectory_msgs` (`JointTrajectory`), `diagnostic_msgs`, `tf2_msgs`.

**Avoid `std_msgs` for anything structured.** Publishing a robot pose as `std_msgs/Float64MultiArray` is legal and loses every property that makes the ecosystem work: no frame id, no timestamp, no tool that understands it. RViz2, tf2, Nav2 and rosbag2 all key off the proper types.

The `Stamped` suffix matters. `geometry_msgs/Pose` is bare numbers; `geometry_msgs/PoseStamped` carries a `Header` with `stamp` and `frame_id`, and without those a pose cannot be transformed into another frame. See [../03_Spatial_and_Temporal/00_tf2.ipynb](../03_Spatial_and_Temporal/tf2.llms.md).

``` bash
ros2 interface show sensor_msgs/msg/LaserScan     # the definition, with comments
ros2 interface list | grep -i image               # what exists
ros2 interface proto geometry_msgs/msg/Twist      # a YAML prototype to paste into pub
```

------------------------------------------------------------------------

## Defining Custom Interfaces

Custom types go in their own `ament_cmake` package, even in an otherwise pure-Python project, because the generators are CMake-based. A Python package cannot generate message code.

    my_interfaces/
      CMakeLists.txt
      package.xml
      msg/WheelState.msg
      srv/SetMode.srv
      action/Dock.action

`msg/WheelState.msg`:

    std_msgs/Header header
    float64 left_rad_s
    float64 right_rad_s
    uint8 MODE_IDLE=0          # constants are upper case
    uint8 MODE_DRIVE=1
    uint8 mode

`CMakeLists.txt` needs the generator invocation:

``` cmake
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelState.msg"
  "srv/SetMode.srv"
  "action/Dock.action"
  DEPENDENCIES std_msgs)
```

and `package.xml`:

``` xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>std_msgs</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Field types are the IDL set: `bool`, `byte`, `char`, `float32/64`, `int8/16/32/64` and unsigned variants, `string`, plus arrays (`float64[]` unbounded, `float64[9]` fixed, `float64[<=10]` bounded). Defaults are written after the name (`int32 count 5`); constants use `=`.

**Changing a published message definition is a breaking change across the whole graph.** There is no version negotiation: a publisher and subscriber built against different revisions of the same type name either fail to match or misinterpret bytes. Rebuild and redeploy every node that uses it, together.

------------------------------------------------------------------------

## Parameters

Parameters are per-node configuration values, readable and settable at runtime. Unlike ROS 1 there is no global parameter server; each node owns its own, and a parameter exists only while its node is alive.

``` python
class Drive(Node):
    def __init__(self):
        super().__init__("drive")
        self.declare_parameter("max_speed", 0.5)
        self.declare_parameter("frame_id", "base_link")
        self.max_speed = self.get_parameter("max_speed").value
```

**Declare every parameter.** Reading an undeclared parameter raises `ParameterNotDeclaredError`, and while `allow_undeclared_parameters=True` in the constructor suppresses that, it also removes the only place the node documents what it is configurable by. The declared default is the documentation.

Reacting to runtime changes needs an explicit callback:

``` python
from rcl_interfaces.msg import SetParametersResult

def __init__(self):
    ...
    self.add_on_set_parameters_callback(self.on_params)

def on_params(self, params):
    for p in params:
        if p.name == "max_speed":
            if p.value <= 0.0:
                return SetParametersResult(successful=False, reason="must be positive")
            self.max_speed = p.value
    return SetParametersResult(successful=True)
```

Returning `successful=False` rejects the change, and the caller’s `ros2 param set` reports the reason. A node that caches a parameter in `__init__` and registers no callback will silently ignore every later `ros2 param set`, which looks like the parameter not working.

Descriptors add constraints and help text, and they are what `rqt_reconfigure` renders:

``` python
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

self.declare_parameter(
    "max_speed", 0.5,
    ParameterDescriptor(description="m/s cap on forward velocity",
                        floating_point_range=[FloatingPointRange(from_value=0.0,
                                                                 to_value=2.0)]))
```

------------------------------------------------------------------------

## Parameter Files and the CLI

Parameters are normally supplied from YAML, keyed by node name then `ros__parameters`:

``` yaml
# config/drive.yaml
drive:
  ros__parameters:
    max_speed: 0.8
    frame_id: base_link
    gains: [1.0, 0.0, 0.1]

/**:                      # wildcard: applies to every node
  ros__parameters:
    use_sim_time: true
```

``` bash
ros2 run my_pkg drive --ros-args --params-file config/drive.yaml
ros2 run my_pkg drive --ros-args -p max_speed:=0.8
```

The node key must match the node’s **fully qualified name**, so a namespaced node needs `/robot1/drive:` rather than `drive:`. A mismatch is silent: the node starts with its declared defaults and the YAML is ignored. This is the most common parameter bug, and `ros2 param get` after startup is how to confirm the file was actually applied. The `/**:` wildcard form sidesteps it when a value really is global, `use_sim_time` being the usual case.

``` bash
ros2 param list                       # every node, every parameter
ros2 param get /drive max_speed
ros2 param set /drive max_speed 0.3
ros2 param describe /drive max_speed  # the descriptor, including ranges
ros2 param dump /drive                # current values as YAML, ready to save
```

`ros2 param dump` is the fast way to capture a tuned configuration into a file rather than retyping it.

------------------------------------------------------------------------

Back to top
