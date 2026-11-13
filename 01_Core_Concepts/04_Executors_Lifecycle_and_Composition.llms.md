# Executors, Lifecycle and Composition

How callbacks actually get run - executors, callback groups and the deadlocks they cause - plus managed-startup lifecycle nodes and loading several nodes into one process.

Author

Benedict Thekkel

## The Execution Model

Creating a subscription does not make it run. Callbacks execute only while an **executor** has control, which is what `rclpy.spin(node)` hands over. The executor takes ready work from the middleware (an arrived message, an expired timer, a service request) and invokes the corresponding callback.

The default is the **single-threaded executor**: one thread, one callback at a time, in an order the executor chooses. Two consequences follow and they explain most mysterious behaviour in a first node:

- **A slow callback blocks everything.** A 500 ms image callback in a node that also runs a 20 Hz control timer means the timer fires at 2 Hz. There is no preemption.
- **A callback that waits on other ROS work deadlocks.** The executor cannot process the response while it is inside the callback that is waiting for it.

``` python
import rclpy
from rclpy.executors import MultiThreadedExecutor

rclpy.init()
a, b = SensorNode(), ControlNode()
ex = MultiThreadedExecutor(num_threads=4)
ex.add_node(a)                    # one executor can spin several nodes
ex.add_node(b)
try:
    ex.spin()
finally:
    ex.shutdown()
    a.destroy_node(); b.destroy_node()
    rclpy.shutdown()
```

`rclpy.spin_once(node, timeout_sec=0.1)` processes at most one ready callback and returns, which is how a node is driven from an existing loop (a GUI, a test) rather than handing the thread over.

------------------------------------------------------------------------

## Callback Groups

A callback group decides whether two callbacks may overlap. This is the control surface for concurrency, and a `MultiThreadedExecutor` alone changes nothing without it: every callback defaults to the node’s single **mutually exclusive** group, so a multi-threaded executor still serialises them.

- `MutuallyExclusiveCallbackGroup` - at most one callback from this group at a time. The default, and the safe choice: callbacks in one group need no locking against each other.
- `ReentrantCallbackGroup` - callbacks may run concurrently, including the same callback re-entered. Shared state now needs locking.

The canonical fix for the service-call deadlock from [01_Services_and_Actions.ipynb](../01_Core_Concepts/services_and_actions.llms.md):

``` python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class Bridge(Node):
    def __init__(self):
        super().__init__("bridge")
        self.cb_sub = MutuallyExclusiveCallbackGroup()
        self.cb_cli = ReentrantCallbackGroup()      # separate group for the client
        self.sub = self.create_subscription(Trigger, "in", self.on_msg, 10,
                                           callback_group=self.cb_sub)
        self.cli = self.create_client(SetBool, "set", callback_group=self.cb_cli)

    async def on_msg(self, msg):
        result = await self.cli.call_async(SetBool.Request(data=True))
        self.get_logger().info(f"got {result.success}")
```

Three things are load-bearing here: the client is in a **different** group from the subscription, the executor is multi-threaded, and the callback is `async` with `await` rather than `spin_until_future_complete`. Drop any one and the hang comes back.

**Timers are callbacks too.** A control timer that must keep its period belongs in its own mutually exclusive group, so a slow sensor callback in another group cannot delay it.

------------------------------------------------------------------------

## Lifecycle Nodes

A plain node starts doing its job in `__init__`, which makes startup order across a robot implicit and fragile. A **lifecycle node** (`LifecycleNode`) exposes a managed state machine so an orchestrator can bring a system up in a defined order.

    unconfigured --configure--> inactive --activate--> active
         ^                         |  ^                   |
         +------- cleanup ---------+  +---- deactivate ---+
                                      finalized <-- shutdown (from any state)

The contract is what makes it useful:

| Transition callback | Do here |
|----|----|
| `on_configure` | read parameters, allocate, create publishers (they start **deactivated**) |
| `on_activate` | start timers, enable publishing, open the device |
| `on_deactivate` | stop timers, stop publishing, keep allocations |
| `on_cleanup` | release everything allocated in configure |
| `on_shutdown` | final teardown |

``` python
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State

class Camera(LifecycleNode):
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.pub = self.create_lifecycle_publisher(Image, "image", 10)
        self.cam = open_device(self.get_parameter("device").value)
        return TransitionCallbackReturn.SUCCESS        # FAILURE or ERROR are the others

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.timer = self.create_timer(1/30, self.grab)
        return super().on_activate(state)              # do not forget the super() call
```

A `create_lifecycle_publisher` suppresses publishing while inactive, so a deactivated node is genuinely silent rather than publishing stale data. Returning `FAILURE` from `on_configure` leaves the node unconfigured and lets the orchestrator report which component refused to come up, instead of a crash loop.

``` bash
ros2 lifecycle nodes
ros2 lifecycle get /camera
ros2 lifecycle set /camera configure
ros2 lifecycle set /camera activate
```

Nav2 is built entirely from lifecycle nodes driven by a `lifecycle_manager`, which is why its bringup is ordered and why a half-started Nav2 reports exactly which node is stuck. See [../06_Navigation_and_Manipulation/01_Nav2_Bringup.ipynb](../06_Navigation_and_Manipulation/nav2_bringup.llms.md).

------------------------------------------------------------------------

## Composition and Intra-Process Communication

Nodes do not need one process each. A **component** is a node compiled into a shared library and loaded into a container process at runtime, and several components in one container can exchange messages without serialisation.

The gain is real and worth the trouble exactly where the data is large: a 5 MP image passed between two components in one container is an intra-process pointer hand-off rather than a serialise, copy, deserialise round trip. For a 10 Hz `Twist` it is noise.

``` bash
ros2 run rclcpp_components component_container_mt         # mt = multi-threaded
ros2 component load /ComponentManager image_pipeline image_proc::RectifyNode
ros2 component list
```

Two conditions have to hold for the zero-copy path to be taken, and missing either one silently falls back to the normal DDS path with no warning:

- Both endpoints are in the **same container process**.
- Intra-process communication is enabled (`use_intra_process_comms: True` in the component’s node options) and the publisher hands over ownership (`std::unique_ptr` in C++).

**Composition is a C++ feature in practice.** `rclpy` can load Python nodes into a shared process, but the intra-process zero-copy path is rclcpp-only, so a Python “component” gets the process-sharing and not the performance. Perception pipelines that need it are written in C++.

The launch-file side, `ComposableNodeContainer` and `LoadComposableNodes`, is in [../02_Build_and_Tooling/01_Launch.ipynb](../02_Build_and_Tooling/launch.llms.md).

------------------------------------------------------------------------

## Diagnosing Execution Problems

The symptoms are distinctive once the model above is clear:

| Symptom | Likely cause |
|----|----|
| callback never fires, endpoints match | node is not spinning, or spun in a thread that exited |
| callback never fires, endpoints visible but no data | QoS incompatibility, see [03_QoS_Profiles.ipynb](../01_Core_Concepts/qos_profiles.llms.md) |
| timer period drifts under load | slow callback in the same mutually exclusive group |
| node hangs on the first service call | synchronous call from inside a callback |
| hang only under load | reentrant group plus unprotected shared state |
| messages arrive in bursts | subscriber queue depth too large, or executor starved |

`ros2 topic hz` on the input and a log line at the top of the callback separate “not arriving” from “arriving but not processed”, which is the first fork in the diagnosis.

------------------------------------------------------------------------

Back to top
