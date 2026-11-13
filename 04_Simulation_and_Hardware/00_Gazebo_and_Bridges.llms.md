# Gazebo and Bridges

The modern Gazebo (Harmonic and Ionic, not Gazebo Classic), ros_gz_bridge and the clock that everything depends on, sensor plugins, and authoring worlds and models in SDF.

Author

Benedict Thekkel

## Which Gazebo

The naming is genuinely confusing and picking the wrong one wastes days, because tutorials for each are indistinguishable at a glance.

| Name | Status | Pairs with |
|----|----|----|
| **Gazebo Classic** (`gazebo`, versions 1-11) | **end of life January 2025** | ROS 1, early ROS 2 |
| Ignition Gazebo (`ign`, Citadel to Fortress) | the rename generation | Foxy, Galactic |
| **Gazebo** (`gz`, Garden, **Harmonic**, Ionic) | current | Harmonic is the LTS paired with Jazzy |

For Jazzy the answer is **Gazebo Harmonic**, driven by the `gz` command and the `ros_gz` packages. Anything that says `gazebo_ros`, `<gazebo_ros>` plugins or `spawn_entity.py` is Classic and does not apply.

``` bash
sudo apt install ros-jazzy-ros-gz          # bridge, sim, interfaces
gz sim --versions
gz sim shapes.sdf                          # a built-in world
gz sim -r -v4 my_world.sdf                 # -r run immediately, -v4 verbose
gz sim -s my_world.sdf                     # server only, no GUI: what CI uses
```

Two commands worth knowing from the start, because they are how you tell a Gazebo problem from a ROS problem:

``` bash
gz topic -l                    # Gazebo's own topics, nothing to do with ROS
gz topic -e -t /clock          # echo one
gz service -l
```

If a value is present on `gz topic` and absent on `ros2 topic`, the bridge is the problem. If it is absent on both, the simulation is the problem. That split is the single most useful debugging habit here.

------------------------------------------------------------------------

## ros_gz_bridge

Gazebo and ROS 2 are separate middlewares. Nothing crosses between them unless a bridge is told to carry it, per topic and per type.

``` bash
# one topic, both directions
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

# ROS -> Gazebo only:  ]    Gazebo -> ROS only:  [
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

The syntax is `topic@ros_type@gz_type`, where the separator encodes direction: `@` is bidirectional, `[` means Gazebo to ROS, `]` means ROS to Gazebo. Getting the direction character wrong produces a bridge that starts cleanly and carries nothing.

A YAML config scales better than a command line, and is what a real bringup uses:

``` yaml
# config/bridge.yaml
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/model/my_robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/scan"
  gz_topic_name: "/lidar"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

``` python
Node(package="ros_gz_bridge", executable="parameter_bridge",
     parameters=[{"config_file": PathJoinSubstitution(
         [FindPackageShare("my_bringup"), "config", "bridge.yaml"])}],
     output="screen")
```

**Bridge the clock first, and bridge it before anything else.** Every node in a simulated system runs with `use_sim_time: true` and reads `/clock`; without that one topic, every node’s clock sits at zero, every `tf2` lookup fails with an extrapolation error, and the failure looks like a transform problem rather than a missing bridge. See [../03_Spatial_and_Temporal/02_Conventions_and_Time.ipynb](../03_Spatial_and_Temporal/conventions_and_time.llms.md).

**Do not bridge images through `parameter_bridge`.** Use `ros_gz_image`’s `image_bridge`, which understands the image transports; the generic bridge will carry raw frames and saturate the graph.

``` bash
ros2 run ros_gz_image image_bridge /camera/image_raw
```

The other recurring trap is **topic names**: Gazebo’s sensor topics are nested under the model and link (`/world/empty/model/my_robot/link/lidar_link/sensor/lidar/scan`), not the short name you hoped for. `gz topic -l` is how you find the real one, and the bridge config is where you map it to something sane.

------------------------------------------------------------------------

## Spawning a Robot

The robot description stays a single URDF/xacro; Gazebo converts it to SDF when spawning. See [../03_Spatial_and_Temporal/01_Robot_Description.ipynb](../03_Spatial_and_Temporal/robot_description.llms.md).

``` python
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node

# start Gazebo with a world
ExecuteProcess(cmd=["gz", "sim", "-r", world_path], output="screen"),

# robot_state_publisher, from the same xacro the real robot uses
Node(package="robot_state_publisher", executable="robot_state_publisher",
     parameters=[{"robot_description": robot_description, "use_sim_time": True}]),

# spawn it from the /robot_description topic
Node(package="ros_gz_sim", executable="create",
     arguments=["-topic", "robot_description", "-name", "my_robot", "-z", "0.2"],
     output="screen"),
```

Spawning from `-topic robot_description` rather than `-file robot.sdf` is what keeps one description for simulation and hardware. `-z 0.2` matters: spawning at exactly ground level makes the robot intersect the floor, and the physics solver resolves that by launching it into the air.

Gazebo-specific additions live in `<gazebo>` tags inside the URDF, which ROS tools ignore and Gazebo lifts out during conversion:

``` xml
<gazebo reference="wheel_left">
  <mu1>1.0</mu1>            <!-- friction -->
  <mu2>1.0</mu2>
</gazebo>

<gazebo>
  <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>wheel_left_joint</left_joint>
    <right_joint>wheel_right_joint</right_joint>
    <wheel_separation>0.32</wheel_separation>
    <wheel_radius>0.05</wheel_radius>
    <topic>cmd_vel</topic>
  </plugin>
</gazebo>
```

**A link with no `<inertial>` block is ignored by physics.** RViz renders it happily, so a robot that looks right and refuses to move usually has missing or implausible inertia (a mass of 0, or an inertia tensor of zeros). Gazebo warns, in a wall of other output.

------------------------------------------------------------------------

## Sensors

Sensors are declared in the description and instantiated by Gazebo’s sensors system, then bridged across. The pattern is the same for all of them: a `<sensor>` element on a link, a plugin system loaded in the world, and a bridge entry.

``` xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_lidar">
    <update_rate>10</update_rate>
    <topic>lidar</topic>
    <gz_frame_id>lidar_link</gz_frame_id>
    <lidar>
      <scan><horizontal>
        <samples>360</samples>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal></scan>
      <range><min>0.12</min><max>12.0</max></range>
    </lidar>
  </sensor>
</gazebo>
```

| Sensor `type` | Publishes | ROS type |
|----|----|----|
| `gpu_lidar` | `/lidar` | `sensor_msgs/LaserScan` or `PointCloud2` |
| `camera` | `/camera/image`, `/camera/camera_info` | `Image`, `CameraInfo` |
| `rgbd_camera` | colour, depth and points | `Image`, `Image`, `PointCloud2` |
| `depth_camera` | depth only | `Image` |
| `imu` | `/imu` | `sensor_msgs/Imu` |
| `navsat` | `/navsat` | `sensor_msgs/NavSatFix` |
| `air_pressure`, `altimeter`, `magnetometer` | as named | various |

The world must load the systems that drive them, or the sensors exist in the description and publish nothing:

``` xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
```

**A missing system plugin is the commonest cause of a silent sensor.** There is no error saying “you declared a lidar but loaded no sensors system”; the topic simply never appears on `gz topic -l`.

Two further points on sensor realism. `<gz_frame_id>` must name the frame the robot’s tf tree actually has, or the data arrives stamped with a frame nothing can transform. And simulated sensors are **noise-free by default**, which flatters every filter downstream; add a `<noise>` block to get an honest picture, and see [01_Other_Simulators_and_Sim_to_Real.ipynb](../04_Simulation_and_Hardware/other_simulators_and_sim_to_real.llms.md).

------------------------------------------------------------------------

## Worlds and Models

A world is SDF: physics settings, lighting, the systems above, and models.

``` xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="room">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>

    <light type="directional" name="sun">
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table</uri>
      <pose>2 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

Fuel (`fuel.gazebosim.org`) hosts ready-made models, which is far faster than modelling a room. The first run downloads them, so a CI job needs them cached or vendored.

Practical notes:

- **`max_step_size` and the real-time factor are the performance dial.** A 1 ms step is accurate and expensive; 4 ms is usually fine for wheeled robots. Check the achieved factor in the GUI: a simulation running at 0.3x real time makes every timing-dependent behaviour wrong, and with `use_sim_time` it is not obviously broken, just slow.
- **`GZ_SIM_RESOURCE_PATH`** must include your meshes and models, or they silently render as nothing.
- **Run headless in CI** (`gz sim -s`). The GUI needs a GPU and is the usual reason a simulation test passes locally and fails on a runner.
- **Keep collision geometry primitive.** A visual mesh reused as collision geometry turns the physics step into a mesh-mesh intersection problem and destroys the real-time factor.

------------------------------------------------------------------------

Back to top
