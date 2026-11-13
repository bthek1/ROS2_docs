# Robot Description

URDF and xacro, what SDF is for, robot_state_publisher’s job, and forward kinematics computed from a parsed URDF to show what the joint chain actually means.

Author

Benedict Thekkel

## URDF

A URDF is an XML description of links (rigid bodies) and joints (the constraints between them). It is what `robot_state_publisher` turns into a tf tree, what RViz draws, and what MoveIt and `ros2_control` build their models from.

``` xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry><box size="0.4 0.3 0.1"/></geometry>
      <material name="grey"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.4 0.3 0.1"/></geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.07" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <link name="wheel_left"/>

  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.16 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

Three sub-elements of a link, with three different consumers:

| Element | Used by |
|----|----|
| `visual` | RViz, Gazebo rendering. Can be a mesh, can be decorative |
| `collision` | physics, MoveIt collision checking. Keep it simple: boxes and cylinders, not the visual mesh |
| `inertial` | physics only. Required by Gazebo, ignored by RViz |

Joint types, and which ones move:

| Type | Motion | Notes |
|----|----|----|
| `fixed` | none | not in `joint_states`, folded into the tree |
| `revolute` | rotation about `axis`, with limits | needs `<limit lower upper effort velocity>` |
| `continuous` | rotation, unlimited | wheels |
| `prismatic` | translation along `axis`, with limits | linear actuators |
| `floating` / `planar` | 6 or 3 DOF | rarely used, poorly supported |

Two things that cost hours when wrong:

- **`<origin>` on a joint is the parent-to-child transform at zero position**, and the joint’s motion is applied *after* it. Getting the `rpy` wrong puts the whole subtree in the wrong place, and because the subtree moves coherently it looks like a calibration error.
- **A `revolute` joint without `<limit>` fails to parse** in most consumers, with an error that names the file rather than the joint.

Checking a URDF before wiring it into anything:

``` bash
check_urdf /path/to/robot.urdf        # from liburdfdom-tools
ros2 run tf2_tools view_frames        # the tree it actually produced
```

------------------------------------------------------------------------

## Forward Kinematics from a Parsed URDF

The tf tree under a robot is exactly the joint chain with each joint’s transform applied in order. Parsing a small URDF and walking it numerically makes concrete what `robot_state_publisher` does with `joint_states` every cycle.

The arm below is a two-link planar manipulator, chosen because it has a closed-form solution to check against: with link lengths `l1` and `l2` and joint angles `q1`, `q2`, the tool sits at `(l1 cos q1 + l2 cos(q1+q2), l1 sin q1 + l2 sin(q1+q2))`.

------------------------------------------------------------------------

``` python
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R

URDF = """<?xml version="1.0"?>
<robot name="planar_arm">
  <link name="base_link"/>
  <link name="upper_arm"/>
  <link name="forearm"/>
  <link name="tool"/>

  <joint name="shoulder" type="revolute">
    <parent link="base_link"/><child link="upper_arm"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>

  <joint name="elbow" type="revolute">
    <parent link="upper_arm"/><child link="forearm"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>

  <joint name="wrist" type="fixed">
    <parent link="forearm"/><child link="tool"/>
    <origin xyz="0.4 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""

def parse_joints(urdf_text):
    "Pull out what kinematics needs: type, parent, child, origin and axis."
    root = ET.fromstring(urdf_text)
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in o.get("xyz", "0 0 0").split()] if o is not None else [0, 0, 0]
        rpy = [float(v) for v in o.get("rpy", "0 0 0").split()] if o is not None else [0, 0, 0]
        ax = j.find("axis")
        axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 1]
        joints[j.get("name")] = dict(type=j.get("type"),
                                     parent=j.find("parent").get("link"),
                                     child=j.find("child").get("link"),
                                     xyz=np.array(xyz, dtype=float),
                                     rpy=np.array(rpy, dtype=float),
                                     axis=np.array(axis, dtype=float))
    return joints

joints = parse_joints(URDF)
for name, j in joints.items():
    print(f"{name:9} {j['type']:10} {j['parent']:10} -> {j['child']:10} origin {j['xyz']}")
```

    shoulder  revolute   base_link  -> upper_arm  origin [0.  0.  0.1]
    elbow     revolute   upper_arm  -> forearm    origin [0.5 0.  0. ]
    wrist     fixed      forearm    -> tool       origin [0.4 0.  0. ]

``` python
def tf(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = R.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = xyz
    return T

def joint_tf(j, q):
    "The joint's fixed origin, then its motion about (or along) its axis."
    T = tf(j["xyz"], j["rpy"])
    if j["type"] in ("revolute", "continuous"):
        T = T @ tf([0, 0, 0], j["axis"] * q)      # rotate about the axis
    elif j["type"] == "prismatic":
        T = T @ tf(j["axis"] * q, [0, 0, 0])      # slide along it
    return T                                      # 'fixed' contributes only the origin

def fk(joints, q, tip="tool", base="base_link"):
    "Walk child->parent to the base, accumulating transforms. This is robot_state_publisher."
    by_child = {j["child"]: (name, j) for name, j in joints.items()}
    T, link = np.eye(4), tip
    while link != base:
        name, j = by_child[link]
        T = joint_tf(j, q.get(name, 0.0)) @ T
        link = j["parent"]
    return T

l1, l2 = 0.5, 0.4
for q1, q2 in [(0.0, 0.0), (np.pi / 2, 0.0), (0.3, -0.7), (np.pi / 4, np.pi / 4)]:
    T = fk(joints, {"shoulder": q1, "elbow": q2})
    closed_form = np.array([l1 * np.cos(q1) + l2 * np.cos(q1 + q2),
                            l1 * np.sin(q1) + l2 * np.sin(q1 + q2),
                            0.1])
    assert np.allclose(T[:3, 3], closed_form, atol=1e-12)
    print(f"q = ({q1:+.3f}, {q2:+.3f}) -> tool at {T[:3, 3].round(4)}  (matches closed form)")

# fully extended, the tool is exactly l1 + l2 from the shoulder axis
assert abs(np.linalg.norm(fk(joints, {"shoulder": 0, "elbow": 0})[:2, 3]) - (l1 + l2)) < 1e-12
print(f"\nfully extended reach: {l1 + l2} m")
```

    q = (+0.000, +0.000) -> tool at [0.9 0.  0.1]  (matches closed form)
    q = (+1.571, +0.000) -> tool at [0.  0.9 0.1]  (matches closed form)
    q = (+0.300, -0.700) -> tool at [ 0.8461 -0.008   0.1   ]  (matches closed form)
    q = (+0.785, +0.785) -> tool at [0.3536 0.7536 0.1   ]  (matches closed form)

    fully extended reach: 0.9 m

``` python
# The fixed `wrist` joint contributes its origin but has no state: it never appears in
# /joint_states, and asking for an angle on it changes nothing.
a = fk(joints, {"shoulder": 0.3, "elbow": -0.7})
b = fk(joints, {"shoulder": 0.3, "elbow": -0.7, "wrist": 1.57})
assert np.allclose(a, b)
print("fixed joints ignore their commanded value:", np.allclose(a, b))

# Tool orientation is the sum of the revolute angles, as expected for a planar chain.
for q1, q2 in [(0.0, 0.0), (0.3, -0.7), (1.0, 0.5)]:
    rpy = R.from_matrix(fk(joints, {"shoulder": q1, "elbow": q2})[:3, :3]).as_euler("xyz")
    assert abs(rpy[2] - (q1 + q2)) < 1e-12
    print(f"q = ({q1:+.2f}, {q2:+.2f}) -> tool yaw {rpy[2]:+.4f} rad = q1 + q2")
```

    fixed joints ignore their commanded value: True
    q = (+0.00, +0.00) -> tool yaw +0.0000 rad = q1 + q2
    q = (+0.30, -0.70) -> tool yaw -0.4000 rad = q1 + q2
    q = (+1.00, +0.50) -> tool yaw +1.5000 rad = q1 + q2

## robot_state_publisher

`robot_state_publisher` is the node that does the walk above, continuously. It takes the URDF as the `robot_description` parameter and subscribes to `/joint_states`, then publishes the whole tree: every movable joint on `/tf`, every fixed joint once on `/tf_static`.

``` python
# in a launch file
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

robot_description = Command([
    "xacro ",
    PathJoinSubstitution([FindPackageShare("my_description"), "urdf", "robot.urdf.xacro"])])

Node(package="robot_state_publisher", executable="robot_state_publisher",
     parameters=[{"robot_description": robot_description,
                  "use_sim_time": use_sim_time}]),
```

The trailing space in `"xacro "` is load-bearing: the list is concatenated without separators. See [../02_Build_and_Tooling/01_Launch.ipynb](../02_Build_and_Tooling/launch.llms.md).

What publishes `/joint_states` depends on the setup, and getting this wrong is why a robot appears collapsed at the origin in RViz:

| Source | When |
|----|----|
| `joint_state_publisher_gui` | testing a URDF by hand, with sliders |
| `joint_state_publisher` | placeholder, publishes zeros |
| `ros2_control` joint state broadcaster | a real robot or Gazebo |
| the driver itself | simple hardware |

**Nothing publishes `/joint_states` by default.** Without it `robot_state_publisher` has no joint positions, publishes no dynamic transforms, and RViz draws every link at the origin.

------------------------------------------------------------------------

## xacro

URDF has no variables, no arithmetic and no reuse, which makes a real robot’s URDF enormous and wrong in several places at once. xacro is the macro layer that fixes it, and essentially every real robot ships `.urdf.xacro` rather than `.urdf`.

``` xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="track" value="0.32"/>

  <xacro:macro name="wheel" params="side reflect">
    <link name="wheel_${side}">
      <visual>
        <geometry><cylinder radius="${wheel_radius}" length="0.03"/></geometry>
      </visual>
    </link>
    <joint name="wheel_${side}_joint" type="continuous">
      <parent link="base_link"/>
      <child link="wheel_${side}"/>
      <origin xyz="0 ${reflect * track / 2} 0" rpy="${-pi/2} 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <xacro:wheel side="left"  reflect="1"/>
  <xacro:wheel side="right" reflect="-1"/>

  <xacro:include filename="$(find my_description)/urdf/sensors.xacro"/>
</robot>
```

`${...}` evaluates arithmetic and has `pi` available. Properties are substituted textually, so a typo in a property name produces an unevaluated `${name}` in the output rather than an error.

Always expand and inspect before debugging the robot:

``` bash
xacro robot.urdf.xacro > /tmp/robot.urdf && check_urdf /tmp/robot.urdf
xacro robot.urdf.xacro use_sim:=true > /tmp/robot.urdf     # xacro args
```

This is the first debugging step for any description problem, because it separates a macro bug from a kinematics bug.

------------------------------------------------------------------------

## SDF, and When It Matters

SDF (Simulation Description Format) is Gazebo’s native format. It describes worlds, not just robots: lighting, physics parameters, multiple models, plugins and sensors.

The division of responsibility in practice:

- **URDF describes the robot**, and is what ROS 2 consumes. It cannot describe a world, closed kinematic loops, or multiple root links.
- **SDF describes the simulation**, and is what Gazebo consumes. Gazebo converts URDF to SDF internally when spawning a robot.
- **Gazebo-specific additions go in `<gazebo>` tags inside the URDF** (friction, sensor plugins, `ros2_control` hardware), which Gazebo lifts out during conversion and ROS 2 tools ignore.

So a single xacro file serves both, which is why robots ship one description rather than two. Writing SDF by hand is for worlds and static props. Details in [../04_Simulation_and_Hardware/00_Gazebo_and_Bridges.ipynb](../04_Simulation_and_Hardware/gazebo_and_bridges.llms.md).

------------------------------------------------------------------------

Back to top
