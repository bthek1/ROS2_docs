# Services and Actions

The two request/response patterns: blocking calls for short operations, and goal-based actions with feedback and cancellation for anything slow.

Author

Benedict Thekkel

## Choosing Between the Three Patterns

Topics, services and actions are not interchangeable, and picking the wrong one is the most common design mistake in a first ROS 2 system.

| Pattern | Shape | Latency it suits | Gives you |
|----|----|----|----|
| Topic | many-to-many stream, fire and forget | continuous | no acknowledgement, no result |
| Service | one client to one server, blocking | milliseconds | exactly one response |
| Action | one client to one server, goal based | seconds to minutes | feedback, result, cancellation, status |

The rule of thumb: if the caller needs an answer, it is not a topic. If the answer takes longer than a control cycle, it is not a service. Anything a human would describe as a task (“go to the kitchen”, “pick up the block”, “calibrate the arm”) is an action.

------------------------------------------------------------------------

## Services

A service is a blocking request/response call. The definition is a `.srv` file with a request and a response separated by `---`:

    # AddTwoInts.srv
    int64 a
    int64 b
    ---
    int64 sum

Server side:

``` python
from example_interfaces.srv import AddTwoInts

class Adder(Node):
    def __init__(self):
        super().__init__("adder")
        self.srv = self.create_service(AddTwoInts, "add_two_ints", self.on_request)

    def on_request(self, request, response):
        response.sum = request.a + request.b
        return response                      # returning the filled response is required
```

Client side, and this is where the trap is:

``` python
class Caller(Node):
    def __init__(self):
        super().__init__("caller")
        self.cli = self.create_client(AddTwoInts, "add_two_ints")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for service")

    def add(self, a, b):
        req = AddTwoInts.Request(a=a, b=b)
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)   # only safe outside a callback
        return future.result().sum
```

**Never call a service synchronously from inside a callback.** `call_async` plus `spin_until_future_complete` inside a subscription callback deadlocks under the default single-threaded executor: the executor is already inside your callback and cannot process the response that would complete the future. The fixes are a reentrant callback group with a multi-threaded executor, or `await`ing the future from an async callback. Both are in [04_Executors_Lifecycle_and_Composition.ipynb](../01_Core_Concepts/executors_lifecycle_and_composition.llms.md), and this deadlock is the single most common ROS 2 hang.

The synchronous `cli.call(req)` exists but has the same constraint, which is why the async form is what the examples use.

Keep service handlers short. A handler that blocks for a second blocks the whole executor for a second, stalling every other callback in the node.

------------------------------------------------------------------------

## Actions

An action is a long-running goal with feedback and the ability to cancel. Internally it is three services (goal, cancel, result) plus two topics (feedback, status), which is why the CLI shows extra hidden topics under an action name.

The definition is a `.action` file with three sections:

    # Fibonacci.action
    int32 order        # goal
    ---
    int32[] sequence   # result
    ---
    int32[] partial    # feedback

Server side, the important parts being the execute callback and the cancellation check:

``` python
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from example_interfaces.action import Fibonacci

class FibServer(Node):
    def __init__(self):
        super().__init__("fib")
        self.srv = ActionServer(
            self, Fibonacci, "fibonacci",
            execute_callback=self.execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT)

    def execute(self, goal_handle):
        seq = [0, 1]
        for _ in range(goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()
            seq.append(seq[-1] + seq[-2])
            goal_handle.publish_feedback(Fibonacci.Feedback(partial=seq))
            time.sleep(0.1)
        goal_handle.succeed()
        return Fibonacci.Result(sequence=seq)
```

An execute callback that never checks `is_cancel_requested` produces an action that cannot be cancelled, which in a navigation stack means a robot that will not stop. Checking it every loop iteration is not optional.

Client side:

``` python
from rclpy.action import ActionClient

client = ActionClient(node, Fibonacci, "fibonacci")
client.wait_for_server()
goal_future = client.send_goal_async(
    Fibonacci.Goal(order=10),
    feedback_callback=lambda fb: print(fb.feedback.partial))
```

`send_goal_async` returns a future for the goal *handle*, not the result. Acceptance and completion are two separate awaits: first the handle, then `handle.get_result_async()`. Treating the first future as the result is a common mistake and yields an empty result object.

------------------------------------------------------------------------

## Goal States and Preemption

A goal moves through a state machine, and both `ros2 action` output and Nav2 behaviour only make sense with it in mind:

    ACCEPTED -> EXECUTING -> SUCCEEDED
                          -> CANCELED   (client asked, server agreed)
                          -> ABORTED    (server gave up)
             -> (rejected: never becomes a goal at all)

Whether a second goal preempts the first is **the server’s policy, not the framework’s**. The default `ActionServer` accepts concurrent goals and runs them in separate handles, which for a single robot base is almost never what is wanted; Nav2’s servers explicitly abort the running goal when a new one arrives. If a server is meant to do one thing at a time, the goal callback has to say so.

------------------------------------------------------------------------

## Inspecting Services and Actions

``` bash
ros2 service list -t                    # names with types
ros2 service type /add_two_ints
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"

ros2 action list -t
ros2 action info /fibonacci -t          # clients, servers, and the underlying topics
ros2 action send_goal -f /fibonacci example_interfaces/action/Fibonacci "{order: 5}"
```

`ros2 action send_goal -f` streams feedback, which makes it the fastest way to confirm a server is alive and actually progressing rather than accepting goals and stalling.

Note that `ros2 service call` blocks until the response arrives, so a hung call is evidence the server’s executor is blocked, not that the network is broken.

------------------------------------------------------------------------

Back to top
