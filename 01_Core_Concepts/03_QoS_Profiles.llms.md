# QoS Profiles

The delivery-guarantee settings DDS exposes through ROS 2, the built-in profiles, and the incompatibility table for the case where two endpoints exist on a topic and no messages flow.

Author

Benedict Thekkel

## Why QoS Exists

In ROS 1 a topic had one delivery behaviour: TCP, reliable, queued. ROS 2 runs on DDS, which parameterises delivery, and that parameterisation is exposed as a **QoS profile** on every publisher and subscriber.

This buys real capability. A camera stream can drop frames rather than buffer them, a latched map can be delivered to a subscriber that connects minutes later, and a wireless link can be told not to retry. It also introduces a failure mode that does not exist in ROS 1: **two endpoints on the same topic with the same type that never exchange a message**, because their profiles are incompatible.

The profile is fixed when the endpoint is created. There is no renegotiation, and no way to change it on a live publisher.

------------------------------------------------------------------------

## The Policies That Matter

Five of the policies come up in practice; the rest are rarely touched outside real-time work.

**Reliability** - `RELIABLE` - retry until acknowledged. Commands, transforms, configuration. - `BEST_EFFORT` - send once, accept loss. High-rate sensor data, anything over WiFi where a retry storm is worse than a dropped frame.

**Durability** - `VOLATILE` - a subscriber gets only what is published after it joins. - `TRANSIENT_LOCAL` - the publisher keeps the last `depth` messages and delivers them to late joiners. This is the ROS 2 equivalent of a ROS 1 latched topic, and it is how `/map`, `/robot_description` and static transforms reach nodes that start later.

**History and depth** - `KEEP_LAST` with depth N - a ring buffer of N. The normal choice. - `KEEP_ALL` - bounded only by middleware limits. Rarely correct; a slow subscriber turns it into unbounded memory growth.

**Deadline, Liveliness and Lifespan** - the ones to know about rather than use daily. `DEADLINE` declares a maximum gap between messages and fires a callback when it is missed, `LIVELINESS` detects a publisher that has stopped asserting it is alive, and `LIFESPAN` expires stale messages before delivery. Diagnostics and watchdogs are the usual consumers; see [../08_Testing_Deployment_Ops/02_Service_Management_and_Diagnostics.ipynb](../08_Testing_Deployment_Ops/service_management_and_diagnostics.llms.md).

------------------------------------------------------------------------

## The Built-in Profiles

`rclpy.qos` ships presets, and using the matching preset on both ends is the reliable way to avoid the incompatibility table below.

``` python
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
                       qos_profile_sensor_data, qos_profile_system_default)

# Sensor data: best effort, depth 5. Use on both the driver and every consumer.
self.sub = self.create_subscription(LaserScan, "scan", self.cb, qos_profile_sensor_data)

# Latched configuration: late joiners get the last value.
latched = QoSProfile(depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)
self.map_pub = self.create_publisher(OccupancyGrid, "map", latched)
```

| Preset | Reliability | Durability | Depth | For |
|----|----|----|----|----|
| default (an integer, e.g. `10`) | RELIABLE | VOLATILE | as given | general purpose |
| `qos_profile_sensor_data` | BEST_EFFORT | VOLATILE | 5 | lidar, camera, IMU |
| `qos_profile_services_default` | RELIABLE | VOLATILE | 10 | services |
| `qos_profile_parameters` | RELIABLE | VOLATILE | 1000 | parameter traffic |
| latched (hand-built, above) | RELIABLE | TRANSIENT_LOCAL | 1 | maps, robot_description |

Passing a bare integer is shorthand for the default profile with that depth, which is why most tutorial code interoperates: everything defaults to reliable and volatile.

------------------------------------------------------------------------

## The Incompatibility Table

Compatibility is one-directional. The rule is that the **subscriber may not demand more than the publisher offers**; a publisher offering more than the subscriber needs is always fine.

| Publisher offers | Subscriber requests | Match? | Symptom |
|----|----|----|----|
| BEST_EFFORT | RELIABLE | **no** | endpoints listed, zero messages |
| RELIABLE | BEST_EFFORT | yes | delivered, loss tolerated |
| VOLATILE | TRANSIENT_LOCAL | **no** | endpoints listed, zero messages |
| TRANSIENT_LOCAL | VOLATILE | yes | no history replay, live data fine |
| RELIABLE + VOLATILE | RELIABLE + VOLATILE | yes | the common default case |
| deadline 100 ms | deadline 50 ms | **no** | subscriber demands tighter timing |
| liveliness AUTOMATIC | liveliness MANUAL_BY_TOPIC | **no** | subscriber demands assertions |

History and depth are **not** part of matching. Mismatched depths always connect; they just buffer differently.

The failure is silent by design: DDS simply does not match the endpoints. What makes it diagnosable:

``` bash
ros2 topic info /scan --verbose     # prints every endpoint's full profile
ros2 doctor --report                # QOS COMPATIBILITY LIST section
```

`ros2 doctor --report` names the incompatible pair and the offending policy directly, which is faster than comparing two profile dumps by eye. Both sides of the pair being visible in `ros2 topic list` while `ros2 topic echo` stays silent is the signature of this problem, as distinct from a discovery failure (where the endpoints do not appear at all, see [../07_Middleware_DDS/00_Discovery_and_RMW.ipynb](../07_Middleware_DDS/discovery_and_rmw.llms.md)) or executor starvation (where messages arrive but callbacks do not run, see [04_Executors_Lifecycle_and_Composition.ipynb](../01_Core_Concepts/executors_lifecycle_and_composition.llms.md)).

One more asymmetry worth knowing: `ros2 topic echo` uses the default reliable profile, so it cannot subscribe to a best-effort-only publisher without `--qos-reliability best_effort`. An `echo` that prints nothing from a working sensor driver is usually this, not a broken driver.

------------------------------------------------------------------------

## Practical Rules

- **Use the same preset on both ends.** `qos_profile_sensor_data` on the publisher and a bare `10` on the subscriber is the incompatibility above, written by accident.
- **Latch anything a late joiner needs**: maps, robot descriptions, static transforms, configuration. Otherwise startup order becomes load-bearing.
- **Best effort for high-rate sensors, reliable for commands.** A dropped lidar scan is replaced in 100 ms; a dropped stop command is not.
- **Leave `KEEP_ALL` alone** unless there is a specific reason and a bounded producer.
- **Over WiFi or a VPN, prefer best effort** for streams. Reliable delivery over a lossy link produces retry storms that degrade everything else on the link.
- **Record what a topic’s profile is** in the package README. There is no way to discover the intended profile from a type name, and a consumer written against the wrong guess fails silently.

------------------------------------------------------------------------

Back to top
