# Conventions and Time

REP-103 and REP-105: which way the axes point and what the standard frames mean, then ROS 2 time - wall clock against simulated clock, stamp arithmetic, and what clock skew does to a transform lookup.

Author

Benedict Thekkel

## REP-103: Units and Axes

REP-103 fixes the conventions that let independently written nodes interoperate. They are not defaults to be overridden; a driver that ignores them is broken, not configurable.

**Units**: metres, radians, seconds, kilograms. Derived units follow (m/s, rad/s, N, Nm). No degrees, anywhere, in any message.

**Axis orientation**, for a body-fixed frame:

    x forward      y left      z up          (right handed)

Rotation about those axes, right-hand rule:

    roll about x      pitch about y      yaw about z

So a positive yaw turns the robot to its left, and a positive pitch tips the nose down only if you remember that y points left. Two frames are worth memorising because they are the ones that differ:

| Frame type | Convention | Used by |
|----|----|----|
| body / base | x forward, y left, z up | everything in ROS |
| optical / camera | z forward, x right, y down | images, OpenCV, `*_optical_frame` |

**Geographic frames** are ENU: x east, y north, z up. This matters when GPS enters the system, because most of the aerospace and marine world uses NED instead.

------------------------------------------------------------------------

``` python
import numpy as np

# ENU (ROS, REP-103) <-> NED (aerospace, marine, most IMU datasheets)
ENU_TO_NED = np.array([[0, 1, 0],
                       [1, 0, 0],
                       [0, 0, -1]], dtype=float)

print("det =", np.linalg.det(ENU_TO_NED))
assert np.isclose(np.linalg.det(ENU_TO_NED), 1.0)   # +1: both frames are right handed
assert np.allclose(ENU_TO_NED @ ENU_TO_NED, np.eye(3))  # the map is its own inverse

p_enu = np.array([3.0, 4.0, 5.0])      # 3 m east, 4 m north, 5 m up
p_ned = ENU_TO_NED @ p_enu
print("ENU", p_enu, "->  NED", p_ned, "  (north, east, down)")
assert np.allclose(p_ned, [4.0, 3.0, -5.0])
assert np.allclose(ENU_TO_NED @ p_ned, p_enu)
```

    det = 1.0
    ENU [3. 4. 5.] ->  NED [ 4.  3. -5.]   (north, east, down)

``` python
# Heading is where the sign errors live. ENU yaw is counter-clockwise from east;
# NED heading is clockwise from north. They are not the same number, and a
# mislabelled IMU feed shows up as a robot that turns the wrong way.
print("ENU yaw (deg)   NED heading (deg)   meaning")
for yaw_enu, meaning in [(0.0, "facing east"), (90.0, "facing north"),
                         (180.0, "facing west"), (-90.0, "facing south")]:
    heading = (90.0 - yaw_enu) % 360.0
    print(f"{yaw_enu:+9.1f}   {heading:14.1f}   {meaning}")
    assert 0.0 <= heading < 360.0

assert (90.0 - 0.0) % 360 == 90.0      # east in ENU is heading 090 in NED
assert (90.0 - 90.0) % 360 == 0.0      # north in ENU is heading 000 in NED
```

    ENU yaw (deg)   NED heading (deg)   meaning
         +0.0             90.0   facing east
        +90.0              0.0   facing north
       +180.0            270.0   facing west
        -90.0            180.0   facing south

## REP-105: What the Standard Frames Mean

REP-105 fixes the meaning of four frame names, and the contract is about guarantees rather than position.

| Frame | Guarantee | Who publishes the edge above it |
|----|----|----|
| `earth` | ECEF, for multi-map or multi-robot setups | rarely used |
| `map` | world-fixed, **no drift**, but **may jump** | AMCL, slam_toolbox, any global localiser |
| `odom` | **continuous and smooth**, drifts without bound | wheel odometry, an EKF |
| `base_link` | the robot body | nothing; it is the robot |

The `map -> odom` transform is the accumulated localisation correction. Read as a pair:

- **`odom -> base_link`** comes from dead reckoning. Smooth, differentiable, wrong over distance.
- **`map -> odom`** is the correction that makes `map -> base_link` globally right. It jumps whenever the localiser resamples.

The practical rule: **a control loop reads `odom`, a planner reads `map`.** Differentiating `map -> base_link` for velocity gives a spike of several m/s every time AMCL corrects, which downstream reads as a real motion.

A second consequence worth stating: **`map -> odom` must be published by exactly one node.** Running AMCL and an EKF that both publish it produces the flickering tree from [00_tf2.ipynb](../03_Spatial_and_Temporal/tf2.llms.md).

------------------------------------------------------------------------

## Wall Time and Simulated Time

Every node reads its clock through `node.get_clock()`, and what that returns depends on one parameter:

``` python
stamp = self.get_clock().now().to_msg()    # respects use_sim_time
```

- **`use_sim_time: false`** (default): the system clock.
- **`use_sim_time: true`**: the clock published on `/clock` by a simulator or by `ros2 bag play --clock`. The node’s notion of now is then whatever the simulation says, and it can run faster than real time, slower, or be paused.

``` bash
ros2 launch my_pkg bringup.launch.py use_sim_time:=true
ros2 bag play recording --clock            # publish /clock while replaying
ros2 topic echo /clock --once
```

**It has to be set on every node or none.** A mixed system has some nodes stamping with 2026 wall time and others with a simulation clock starting near zero, so every transform lookup across the boundary fails with extrapolation errors. The wildcard parameter form is the usual way to avoid a node being missed:

``` yaml
/**:
  ros__parameters:
    use_sim_time: true
```

Two further traps:

- **`time.sleep()` and `time.time()` ignore sim time.** Use `node.get_clock().sleep_for()` and `get_clock().now()`, or a node that is correct at 1x breaks when the simulation runs at 10x.
- **A paused simulation stops the clock.** A timeout written as a wall-clock deadline never fires, or fires immediately, depending on direction.

------------------------------------------------------------------------

``` python
NS = 1_000_000_000     # builtin_interfaces/Time is (int32 sec, uint32 nanosec)

def to_msg(seconds):
    "float seconds -> (sec, nanosec), the way a Time message stores an instant"
    ns = int(round(seconds * NS))
    return ns // NS, ns % NS

def to_float(sec, nanosec):
    return sec + nanosec / NS

print("12.25 s ->", to_msg(12.25))
assert to_msg(12.25) == (12, 250_000_000)
assert to_float(*to_msg(12.25)) == 12.25

# Integer nanoseconds accumulate exactly. Float seconds do not, which is why
# stamps are stored as two integers rather than one double.
float_clock, int_clock = 0.0, 0
for _ in range(100):              # a 10 Hz loop, 100 ticks
    float_clock += 0.1
    int_clock += 100_000_000
print(f"after 100 ticks of 0.1 s: int {int_clock / NS} s exactly, float {float_clock!r}")
assert int_clock == 10 * NS
assert float_clock != 10.0
print(f"float drift: {float_clock - 10.0:+.2e} s")
```

    12.25 s -> (12, 250000000)
    after 100 ticks of 0.1 s: int 10.0 s exactly, float 9.99999999999998
    float drift: -1.95e-14 s

``` python
# What clock skew does to a transform lookup. tf2 will interpolate within its buffer,
# but a stamp outside the published interval raises ExtrapolationException.
tolerance = 0.030      # 30 ms, a typical lookup timeout on a 30 Hz tf stream

print(f"{'skew':>10}   {'within 30 ms?':>14}   result")
for skew in [0.005, 0.020, 0.030, 0.050, 0.250, 1.0]:
    ok = skew <= tolerance
    result = "interpolated" if ok else "ExtrapolationException"
    print(f"{skew * 1000:8.1f} ms   {str(ok):>14}   {result}")

assert 0.020 <= tolerance < 0.050
# A node left on wall time while the rest of the system runs a sim clock starting
# near zero does not produce a small skew, it produces an absurd one:
wall_now = 1_789_000_000.0        # seconds since the epoch, a real timestamp
sim_now = 12.5                    # a simulation 12.5 s in
print(f"\nwall vs sim skew: {wall_now - sim_now:.0f} s")
assert (wall_now - sim_now) > 1e9
print("which is why the symptom is 'every lookup fails', not 'lookups are slightly off'")
```

          skew    within 30 ms?   result
         5.0 ms             True   interpolated
        20.0 ms             True   interpolated
        30.0 ms             True   interpolated
        50.0 ms            False   ExtrapolationException
       250.0 ms            False   ExtrapolationException
      1000.0 ms            False   ExtrapolationException

    wall vs sim skew: 1788999988 s
    which is why the symptom is 'every lookup fails', not 'lookups are slightly off'

## Stamping Rules

Three rules that prevent most time-related bugs:

- **Stamp at acquisition, not at publication.** A driver should stamp a frame with the time the sensor captured it. Stamping after 40 ms of processing tells every consumer the robot was somewhere it was not.
- **Propagate the stamp through a pipeline.** A node that consumes a scan and publishes a derived message copies `msg.header.stamp` rather than reading the clock again. Restamping is how a perception chain silently accumulates latency that no single node is responsible for.
- **Never stamp with zero.** `rclpy.time.Time()` as a stamp means “latest available” to tf2, which works until two sensors disagree and nothing can be synchronised. It also makes `ros2 bag` recordings useless for offline work.

The cost of getting this wrong scales with speed: at 1 m/s, a 50 ms stamp error is 5 cm of position error in anything transformed through the tree, and it appears as sensor miscalibration rather than as a timing bug.

Message synchronisation across two differently stamped streams is the subject of [03_Message_Synchronisation.ipynb](../03_Spatial_and_Temporal/message_synchronisation.llms.md).

------------------------------------------------------------------------

Back to top
