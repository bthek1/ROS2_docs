# CLI and Introspection

The `ros2` command reference, the graphical tools (rqt, rviz2), `ros2 doctor`, rosbag2 recording and replay, and the logging system.

Author

Benedict Thekkel

## The ros2 Command Reference

One entry point, with subcommands mirroring the concepts. These are the ones used constantly, with the flags that matter:

``` bash
# What is running
ros2 node list
ros2 node info /drive               # its publishers, subscribers, services, actions

# Topics
ros2 topic list -t                  # -t appends the message type
ros2 topic echo /scan               # print messages as they arrive
ros2 topic echo /scan --once        # exactly one message, then exit
ros2 topic echo /scan --field ranges --qos-reliability best_effort
ros2 topic hz /scan                 # measured publish rate
ros2 topic bw /image_raw            # measured bandwidth
ros2 topic info /scan --verbose     # every endpoint, with its full QoS profile
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"

# Services and actions
ros2 service list -t
ros2 service call /reset std_srvs/srv/Empty
ros2 action list -t
ros2 action info /navigate_to_pose -t
ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose "{...}"

# Parameters
ros2 param list
ros2 param get /drive max_speed
ros2 param set /drive max_speed 0.5
ros2 param dump /drive              # current values as YAML

# Interfaces
ros2 interface show geometry_msgs/msg/Twist
ros2 interface proto geometry_msgs/msg/Twist    # a prototype to paste into topic pub

# Packages and execution
ros2 pkg list
ros2 pkg prefix my_pkg              # where it is installed
ros2 pkg executables my_pkg         # what `ros2 run` can start
ros2 run my_pkg drive --ros-args -p max_speed:=0.3 -r cmd_vel:=/robot1/cmd_vel
ros2 launch my_pkg bringup.launch.py use_sim_time:=true
```

Three of these repay singling out. `ros2 topic info --verbose` prints QoS profiles and is how a silent incompatibility is found (see [../01_Core_Concepts/03_QoS_Profiles.ipynb](../01_Core_Concepts/qos_profiles.llms.md)). `ros2 interface proto` generates the YAML skeleton for a message, which beats hand-writing nested braces for `ros2 topic pub`. And `ros2 topic pub --once` with a zero `Twist` is the manual stop button for a runaway base.

`--ros-args` separates ROS arguments from the node’s own. Everything after it belongs to ROS; the `--` separator is needed before any arguments the program itself parses.

------------------------------------------------------------------------

## Graphical Tools

``` bash
rviz2                               # 3D visualisation
rqt                                 # plugin host
rqt_graph                           # the node/topic graph
ros2 run rqt_plot rqt_plot          # time series of numeric fields
ros2 run rqt_console rqt_console    # log viewer with filters
ros2 run rqt_reconfigure rqt_reconfigure   # live parameter editing
ros2 run tf2_tools view_frames      # PDF of the transform tree
```

**RViz2** is the main one. It renders topics through display plugins: `LaserScan`, `PointCloud2`, `Image`, `Path`, `Map`, `RobotModel`, `TF`. Two things reliably confuse people new to it:

- **The Fixed Frame must exist in the transform tree**, or nothing renders and the status shows a frame error. `map` is the usual choice once SLAM is running, `odom` before that.
- **Display QoS defaults to reliable**, so a best-effort sensor topic shows nothing until the display’s QoS is changed in its properties. This is the same trap as `ros2 topic echo`.

Save a configured session as a `.rviz` file and ship it in the package’s `share/` directory, then launch it with `rviz2 -d $(ros2 pkg prefix my_pkg)/share/my_pkg/rviz/robot.rviz`. Reconfiguring RViz by hand on every run is wasted time.

**`rqt_graph`** answers “is this node actually connected to that one”. Turn off “Debug” and leave “Dead sinks” and “Leaf topics” on while hunting a missing connection, otherwise the graph is unreadable on a real robot.

------------------------------------------------------------------------

## ros2 doctor

``` bash
ros2 doctor                  # checks, with warnings
ros2 doctor --report         # the full report
ros2 doctor --report-failed  # only what failed
```

The report has sections worth knowing: `NETWORK CONFIGURATION` (interfaces, whether multicast is available), `PLATFORM INFORMATION`, `RMW MIDDLEWARE`, `ROS 2 INFORMATION` (distribution and whether it is EOL), `TOPIC LIST` (publisher/subscriber counts) and **`QOS COMPATIBILITY LIST`**, which names incompatible endpoint pairs and the offending policy directly.

That last section is the fastest diagnosis for the “two endpoints, no messages” case, which is why it is worth remembering that `ros2 doctor` exists at all. Most people discover it after an hour of comparing profile dumps by eye.

------------------------------------------------------------------------

## rosbag2

Recording is how a field failure becomes a reproducible test case.

``` bash
ros2 bag record -a                              # everything (watch the disk)
ros2 bag record /scan /odom /tf /tf_static      # named topics, the normal case
ros2 bag record -a -x "/image_raw.*"            # all but a regex
ros2 bag record -o morning_run --compression-mode file --compression-format zstd

ros2 bag info morning_run
ros2 bag play morning_run
ros2 bag play morning_run --rate 0.5 --loop
ros2 bag play morning_run --topics /scan        # replay a subset
ros2 bag play morning_run --remap /scan:=/scan_replay
```

A bag is a directory: `metadata.yaml` plus one or more storage files (sqlite3 by default, mcap in newer distributions and preferable for large recordings).

Three things that make replay work, all of them learned the hard way:

- **Record `/tf_static` or the replay has no transform tree.** It is published once with a transient-local profile, so a recording started after the publisher still captures it, but a replay that omits it leaves every transform lookup failing.
- **Set `use_sim_time: true` on consumers and play with `--clock`** if timing matters. Otherwise nodes stamp against wall time while the data carries recorded stamps, and anything time-sensitive (tf2 lookups, message filters) fails in ways that look like a logic bug. See [../03_Spatial_and_Temporal/02_Conventions_and_Time.ipynb](../03_Spatial_and_Temporal/conventions_and_time.llms.md).
- **Record best-effort topics with the matching QoS override** (`--qos-profile-overrides-path`), or the recorder’s reliable subscription never matches the publisher and the bag silently contains nothing for that topic.

Images are the disk problem: raw 1080p at 30 Hz is roughly 180 MB/s. Record the compressed transport instead, or a subsampled rate.

------------------------------------------------------------------------

## Logging

Every node has a logger, named after the node, with five levels.

``` python
self.get_logger().debug("per-cycle detail")
self.get_logger().info("state change")
self.get_logger().warn("recoverable oddity")
self.get_logger().error("operation failed")
self.get_logger().fatal("cannot continue")
```

Throttling matters in a callback that runs at sensor rate, where an unthrottled `info` both floods the console and costs real time:

``` python
self.get_logger().info("waiting for transform", throttle_duration_sec=2.0)
self.get_logger().info("first scan received", once=True)
```

Level control, at launch or at runtime:

``` bash
ros2 run my_pkg drive --ros-args --log-level debug
ros2 run my_pkg drive --ros-args --log-level drive:=debug --log-level rclcpp:=warn
ros2 service call /drive/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: drive, level: 10}]}"
```

Logs go to the console and to `~/.ros/log/<timestamp>/`, one directory per run, and nothing cleans that up: a robot running many launches a day accumulates gigabytes there. It is worth a cron job or a `tmpfiles.d` rule.

Useful environment variables:

``` bash
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
```

Adding `{file_name}:{line_number}` to the format is the cheapest way to find which of twenty identical “failed to transform” lines is firing.

------------------------------------------------------------------------

Back to top
