# Client Libraries

The stack under a node: rclpy, rclcpp, and the rcl / rmw layers that make one ROS 2 API work over several DDS implementations.

Author

Benedict Thekkel

## The Layering

Everything a node does passes through the same five layers. Knowing which layer owns a behaviour is what turns an opaque failure into a searchable one.

       your node code
             |
      rclpy / rclcpp          language client library
             |                  node, publisher, executor, callback groups, parameters
            rcl                 C common core
             |                  names, QoS, time, graph API, parameter logic, init/shutdown
            rmw                 middleware abstraction (one API, several implementations)
             |
       rmw_fastrtps_cpp  /  rmw_cyclonedds_cpp  /  rmw_zenoh_cpp
             |
      DDS (or Zenoh) implementation
             |
       UDP multicast / UDP unicast / shared memory

Responsibilities, briefly:

| Layer | Owns | Typical symptom when it is the culprit |
|----|----|----|
| `rclpy` / `rclcpp` | executors, callback groups, the object model | deadlock, callback never runs |
| `rcl` | name resolution, QoS defaults, parameters, time | remapping does not apply, clock wrong |
| `rmw` | type support, QoS translation | incompatible-profile warnings |
| DDS impl | discovery, reliability, transport | nodes invisible, multicast blocked |
| network | packets | works locally, fails across machines |

The practical value of the diagram is triage. “My callback never fires” lives in the client library. “My node cannot see the other machine” lives in DDS and the network and is covered in [../07_Middleware_DDS/00_Discovery_and_RMW.ipynb](../07_Middleware_DDS/discovery_and_rmw.llms.md).

**Common behaviour lives in `rcl`, which is why the two client libraries agree.** Name resolution, QoS defaults and parameter semantics are implemented once in C; `rclpy` and `rclcpp` are bindings plus an execution model. That is also why the execution models differ while everything else matches: executors are the part each language library implements itself.

------------------------------------------------------------------------

## rclpy

The Python client library. Fast to write, and the right choice for coordination logic, mission scripting, experiments, and anything where development speed dominates.

``` python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__("talker")
        self.pub = self.create_publisher(String, "chatter", 10)
        self.create_timer(0.5, self.tick)
        self.i = 0

    def tick(self):
        self.pub.publish(String(data=f"hello {self.i}"))
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

`rclpy.try_shutdown()` rather than `shutdown()` in a `finally` avoids the “context already shut down” traceback on Ctrl-C, which is otherwise the noisiest part of every Python node’s exit.

What to know about rclpy specifically:

- **The GIL is the ceiling.** A multi-threaded executor gives concurrency for blocking I/O, not CPU parallelism. Heavy numeric work belongs in numpy (which releases the GIL) or in C++.
- **No intra-process zero copy.** Python components share a process but still serialise; see [../01_Core_Concepts/04_Executors_Lifecycle_and_Composition.ipynb](../01_Core_Concepts/executors_lifecycle_and_composition.llms.md).
- **Message objects are generated classes**, constructible with keyword arguments (`String(data="x")`) and mutable field by field. Nested fields must be assigned, not built by dict.
- **Conversion cost is real** for big messages. A `PointCloud2` handled field by field in Python is orders of magnitude slower than a numpy view over the buffer; see [../05_Perception/01_Point_Clouds_and_Lidar.ipynb](../05_Perception/point_clouds_and_lidar.llms.md).

------------------------------------------------------------------------

## rclcpp

The C++ client library. The right choice for anything in a control loop, anything processing images or point clouds at rate, and anything that needs composition.

``` cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Talker : public rclcpp::Node {
public:
  Talker() : Node("talker") {
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      auto msg = std_msgs::msg::String();
      msg.data = "hello " + std::to_string(i_++);
      pub_->publish(msg);
    });
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int i_{0};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
}
```

What C++ buys, beyond speed:

- **Intra-process zero copy** via `std::unique_ptr` publishing in a composed container. This is the reason perception pipelines are C++.
- **Real threading**, with no GIL.
- **Lifecycle and components** are first-class; the Python equivalents lag.
- **Deterministic destruction**, which matters for hardware handles.

The cost is build time and the CMake layer. A `CMakeLists.txt` for the above needs `find_package(rclcpp REQUIRED)`, `ament_target_dependencies`, and an `install(TARGETS ...)` stanza; forgetting the install is the C++ version of the missing `console_scripts` entry point.

**The normal split on a real robot**: C++ for drivers, controllers and perception; Python for bringup logic, mission state machines and tools. Both talk over the same topics, so the choice is per node rather than per project.

------------------------------------------------------------------------

## rcl, rmw and Swapping the Middleware

`rcl` is a C library, rarely used directly; it matters because it is where the behaviour shared by both client libraries lives. `rmw` is the interface that makes DDS swappable, and that swap is a single environment variable:

``` bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp     # default is rmw_fastrtps_cpp
ros2 doctor --report | grep -A2 "RMW MIDDLEWARE"
```

Each implementation is its own apt package (`ros-jazzy-rmw-cyclonedds-cpp`). **Every node in a graph must use interoperating implementations**, so change it everywhere or nowhere: a mixed graph produces nodes that are visible to some peers and not others, which is among the most confusing failures available. The implementations and their tuning are in [../07_Middleware_DDS/00_Discovery_and_RMW.ipynb](../07_Middleware_DDS/discovery_and_rmw.llms.md).

A note on why this layering exists at all: ROS 2 deliberately did not write a transport. It specified an abstraction and let vendors supply the implementation, which is how real-time DDS, shared-memory transports and now Zenoh arrived without changing a line of node code. The price is that QoS, discovery and configuration are vendor-shaped, so a DDS-level problem is debugged with vendor documentation rather than ROS documentation.

------------------------------------------------------------------------

Back to top
