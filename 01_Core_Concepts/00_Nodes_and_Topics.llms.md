# Nodes and Topics

The process model and the streaming primitive: what a node is, how topics carry data between them, and the publisher/subscriber rules that decide whether messages actually arrive.

Author

Benedict Thekkel

## What a Node Is

ROS 2 is not an operating system. It is a set of libraries and conventions for writing robot software as many small processes that talk over a message bus. A robot becomes a graph of **nodes**, each doing one job: read the lidar, estimate the pose, plan a path, drive the wheels.

A node is a unit of computation with a name in the graph, not necessarily a process. One process can host several nodes (see [04_Executors_Lifecycle_and_Composition.ipynb](../01_Core_Concepts/executors_lifecycle_and_composition.llms.md)), and the same node class can be instantiated several times under different names. What a node owns is its publishers, subscriptions, service servers and clients, timers, and parameters.

``` python
import rclpy
from rclpy.node import Node

class Wheels(Node):
    def __init__(self):
        super().__init__("wheels")          # the graph name
        self.get_logger().info("up")

def main():
    rclpy.init()
    node = Wheels()
    try:
        rclpy.spin(node)                    # process callbacks until shutdown
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

`rclpy.spin` is the blocking call that hands control to the executor. A node that never spins never runs a callback, which is the commonest reason a freshly written subscriber prints nothing.

The node name must be unique in the graph. Two nodes with the same name is not an error the middleware rejects; it produces a graph where `ros2 node info` is ambiguous and parameter calls land on whichever responds. Remapping at launch (`--ros-args -r __node:=left_wheels`) is how one node class becomes two instances.

------------------------------------------------------------------------

## Topics

A topic is a named, typed, many-to-many stream. Publishers do not know who is listening and get no acknowledgement that anyone was. Subscribers do not know who published. This anonymity is the point: a logger can be attached to a running robot without the publisher being aware, and a simulated sensor can replace a real one by publishing the same type on the same name.

Use topics for continuous data: sensor readings, odometry, velocity commands, transforms. Do not use them for request/response or for anything where the sender needs to know the outcome; that is what services and actions are for ([01_Services_and_Actions.ipynb](../01_Core_Concepts/services_and_actions.llms.md)).

``` python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Drive(Node):
    def __init__(self):
        super().__init__("drive")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.sub = self.create_subscription(LaserScan, "scan", self.on_scan, 10)
        self.timer = self.create_timer(0.1, self.tick)   # 10 Hz

    def on_scan(self, msg: LaserScan):
        self.closest = min(msg.ranges)

    def tick(self):
        cmd = Twist()
        cmd.linear.x = 0.0 if getattr(self, "closest", 9.9) < 0.5 else 0.2
        self.pub.publish(cmd)
```

The `10` in both calls is the queue depth, the shorthand for a QoS profile with history `KEEP_LAST` and depth 10. Passing an integer is fine for most code; passing a real profile matters for sensor data and for latched configuration topics ([03_QoS_Profiles.ipynb](../01_Core_Concepts/qos_profiles.llms.md)).

**Publishing is asynchronous and lossy by default at the application level.** `publish()` hands the message to the middleware and returns. If no subscriber has matched yet, the message is gone unless durability says otherwise. A publisher created immediately before a single `publish()` call almost always sends into the void, because discovery has not completed; this is why one-shot publishing scripts need a short wait or a transient-local profile.

------------------------------------------------------------------------

## Names, Namespaces and Remapping

Every node, topic, service and action has a fully qualified name starting with `/`. A name written without a leading slash in code is **relative** and gets the node’s namespace prefixed, which is what makes a multi-robot launch possible without touching the source.

| Written in code | Node namespace | Resolves to |
|----|----|----|
| `cmd_vel` | `/` | `/cmd_vel` |
| `cmd_vel` | `/robot1` | `/robot1/cmd_vel` |
| `/cmd_vel` | `/robot1` | `/cmd_vel` (absolute, escapes the namespace) |
| `~/cmd_vel` | `/robot1` | `/robot1/<node_name>/cmd_vel` (private) |

Prefer relative names for everything a node owns and absolute names only for genuinely global topics (`/tf`, `/clock`). Hard-coding `/cmd_vel` in a driver is the single change that stops it working in a namespaced bringup.

Remapping rewrites names without editing code:

``` bash
ros2 run my_pkg drive --ros-args -r cmd_vel:=/robot1/cmd_vel -r __ns:=/robot1
```

The launch-file equivalents, and the interaction between `__ns`, `PushRosNamespace` and private names, are in [../02_Build_and_Tooling/01_Launch.ipynb](../02_Build_and_Tooling/launch.llms.md).

------------------------------------------------------------------------

## Inspecting a Live Graph

The graph is introspectable at runtime, which is the main debugging tool. Three commands answer most questions:

``` bash
ros2 node list                      # who is running
ros2 topic list -t                  # what streams exist, with types
ros2 topic info /scan --verbose     # every endpoint on one topic, with its QoS
```

`ros2 topic info --verbose` is the one worth memorising. It prints the QoS profile of each publisher and subscriber, which is how a silent incompatibility becomes visible: two endpoints on a topic, zero messages flowing, and the profiles side by side showing why.

``` bash
ros2 topic echo /scan               # print messages as they arrive
ros2 topic hz /scan                 # measure the actual rate
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

Note that `ros2 topic echo` is itself a subscriber with its own QoS, so it can fail to receive from a publisher whose profile is incompatible, and `echo` showing nothing is not proof that nothing is being published. The full tooling tour is in [../02_Build_and_Tooling/02_CLI_and_Introspection.ipynb](../02_Build_and_Tooling/cli_and_introspection.llms.md).

------------------------------------------------------------------------

## Differences from ROS 1

Worth stating once, because most ROS material online is ROS 1:

- **No `roscore`.** Discovery is peer-to-peer over DDS. Nodes find each other by multicast on the local subnet, so any two ROS 2 nodes on the same network with the same domain ID join the same graph. See [../07_Middleware_DDS/00_Discovery_and_RMW.ipynb](../07_Middleware_DDS/discovery_and_rmw.llms.md).
- **DDS transport** rather than a custom TCPROS protocol, which is where QoS comes from.
- **`colcon` and `ament`** replace `catkin`; see [../02_Build_and_Tooling/00_Workspaces_and_Packages.ipynb](../02_Build_and_Tooling/workspaces_and_packages.llms.md).
- **Parameters are per-node**, not entries in a global parameter server.
- **Message types are namespaced by kind**: `std_msgs/msg/String`, not `std_msgs/String`.
- **Python is `rclpy`, not `rospy`**, and the callback/spin model is explicit.

Migration of existing ROS 1 code, and the `ros1_bridge`, are covered in [../09_Ecosystem_and_Process/01_Release_and_Migration.ipynb](../09_Ecosystem_and_Process/release_and_migration.llms.md).

------------------------------------------------------------------------

Back to top
