# Drivers and micro-ROS

Writing a sensor or actuator driver: serial and CAN transports, what a driver owes the rest of the graph, micro-ROS for microcontrollers, and what real-time actually requires.

Author

Benedict Thekkel

## What a Driver Owes the Graph

A driver is the boundary between a device and ROS 2, and the same handful of obligations come up whatever the device is. Getting these right is most of what separates a driver that works on a bench from one that works on a robot.

- **Stamp at acquisition.** The stamp belongs to the moment the measurement was taken, not the moment the message was published. See [../03_Spatial_and_Temporal/02_Conventions_and_Time.ipynb](../03_Spatial_and_Temporal/conventions_and_time.llms.md).
- **Set `frame_id`, always.** A `sensor_msgs` message with an empty `frame_id` cannot be transformed and is unusable downstream.
- **Use the standard message type**, not a custom one that happens to fit. A lidar publishes `LaserScan`, not `Float32MultiArray`; see [../01_Core_Concepts/02_Interfaces_and_Parameters.ipynb](../01_Core_Concepts/interfaces_and_parameters.llms.md).
- **Publish SI units** per REP-103, converted from whatever the device reports.
- **Choose QoS deliberately.** Sensor streams are best effort; see [../01_Core_Concepts/03_QoS_Profiles.ipynb](../01_Core_Concepts/qos_profiles.llms.md).
- **Be a lifecycle node.** Opening the device in `on_configure` and streaming from `on_activate` makes bring-up ordered and lets an orchestrator report which device failed.
- **Reconnect, do not die.** A USB device that re-enumerates, or a CAN bus that goes bus-off, should be recovered from rather than crash the node. A driver that exits on the first read error turns a glitch into a mission failure.
- **Publish diagnostics.** `diagnostic_updater` is how a fleet operator learns a sensor is degraded rather than absent; see [../08_Testing_Deployment_Ops/02_Service_Management_and_Diagnostics.ipynb](../08_Testing_Deployment_Ops/service_management_and_diagnostics.llms.md).

**Check whether the driver already exists before writing one.** `ros2 pkg list | grep -i <vendor>` and the vendor’s GitHub first: most lidars, IMUs, cameras and CAN interfaces have a maintained driver, and replacing a maintained one is rarely the right trade.

------------------------------------------------------------------------

## Serial

Serial is the common case for small sensors, microcontrollers and cheap motor controllers.

``` python
import serial                      # pyserial
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

class WheelDriver(LifecycleNode):
    def on_configure(self, state):
        self.declare_parameter("device", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.ser = serial.Serial(self.get_parameter("device").value,
                                 self.get_parameter("baud").value,
                                 timeout=0.05)        # never timeout=None
        self.pub = self.create_lifecycle_publisher(JointState, "joint_states", 10)
        return TransitionCallbackReturn.SUCCESS
```

Four points that matter more than the protocol:

- **Always set a timeout.** `timeout=None` blocks forever, and inside a callback or a control loop that is a hang with no diagnostic. A short timeout plus a retry is correct.
- **Use a stable device path.** `/dev/ttyUSB0` is assigned in enumeration order, so two USB devices swap identities across a reboot. A udev rule keyed on the serial number gives `/dev/my_wheel_controller`, and it is the difference between a robot that boots reliably and one that sometimes drives its arm with wheel commands.

&nbsp;

    # /etc/udev/rules.d/99-robot.rules
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{serial}=="A6004byk", SYMLINK+="my_wheel_controller"

- **Frame the protocol.** A length-prefixed or delimited frame with a checksum; `readline()` on binary data will desynchronise eventually and then decode garbage as plausible values.
- **Permissions.** The user must be in `dialout`, or the open fails with a message that reads like a missing device.

------------------------------------------------------------------------

## CAN

CAN is the bus for motor controllers, industrial actuators and vehicles. On Linux it is SocketCAN, so it is an ordinary network interface.

``` bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
candump can0                                 # from can-utils
cansend can0 123#DEADBEEF
ip -details -statistics link show can0       # error counters, bus state
```

``` bash
sudo apt install ros-jazzy-ros2-socketcan
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0
```

`ros2_socketcan` gives `can_msgs/Frame` in and out, which keeps the bus plumbing out of your node; the node then only encodes and decodes frames. For CANopen devices, `ros2_canopen` handles the protocol layer and integrates with `ros2_control`.

What to know:

- **Bitrate must match every device on the bus**, and a mismatch presents as error frames rather than silence.
- **Termination matters.** 120 ohm at both ends; missing termination works on a short bench cable and fails intermittently on a robot, which is the worst failure mode available.
- **Watch the bus state.** Enough errors put a controller into `bus-off`, where it stops participating until reset. Read the counters in `ip -details link show` and publish them as diagnostics rather than discovering bus-off as a robot that stopped responding.
- **CAN is not a stream.** Each frame is 8 bytes (64 with CAN FD); anything larger needs a transport protocol on top, which is what CANopen and J1939 provide.

------------------------------------------------------------------------

## micro-ROS

micro-ROS puts a ROS 2 node on a microcontroller: an ESP32, STM32, Teensy or RP2040, with kilobytes of RAM rather than gigabytes.

      MCU                                    Linux host
      micro-ROS node                         micro_ros_agent
      rmw_microxrcedds  --- serial/UDP ---    bridges into the DDS graph

The MCU does not speak DDS. It speaks **XRCE-DDS**, a lightweight client protocol, to an **agent** on a host, and the agent is what appears in the ROS 2 graph. No agent, no topics, and a micro-ROS node that looks dead is usually an agent that was never started.

``` bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Supported on FreeRTOS, Zephyr, NuttX and bare metal, with `micro_ros_arduino` and an ESP-IDF component for the hobbyist path.

The constraints that shape the code:

- **Memory is the binding limit.** Publishers, subscribers and message sizes are declared up front and statically allocated; a `String` message with a 2 KB buffer may not fit alongside anything else. Tune the middleware’s memory profile rather than hoping.
- **No `rclcpp` or `rclpy`.** It is `rclc`, a C API with an explicit executor, and the programming model is closer to an embedded main loop than to a ROS 2 node.
- **Transport is serial or UDP.** Serial is simpler and slower; UDP over Wi-Fi on an ESP32 is convenient and adds the reliability questions of any wireless link.
- **Time synchronisation is manual.** `rmw_uros_sync_session` sets the MCU clock against the agent; without it, every stamp from the MCU is meaningless, and `message_filters` and tf2 both fail in confusing ways.

**When not to use it.** If the MCU is a sensor or motor interface and a Linux host is present anyway, a framed serial protocol plus a host-side driver node is simpler, easier to debug with `candump`-style tools, and has no agent to keep alive. micro-ROS earns its complexity when the MCU needs to be a first-class participant: running a control loop, publishing several topics, subscribing to commands, and surviving the host rebooting.

MCU hardware itself, boards and toolchains, is covered in the sibling site rather than here: [ESP32, Pico and microcontrollers](https://bthek1.github.io/Hardware_Tools/Embedded_Systems/microcontrollers.html) and [Raspberry Pi 5](https://bthek1.github.io/Hardware_Tools/Embedded_Systems/raspberry_pi_5.html).

------------------------------------------------------------------------

## Real-Time

“Real-time” means a **bounded worst case**, not a fast average. A loop that completes in 1 ms on average and 40 ms occasionally is not real-time, and the occasional case is what breaks a balancing robot.

Stock Linux gives no bound. The pieces that move toward one, in order of effect:

- **`PREEMPT_RT`.** Mainline since 6.12, so a modern kernel can be configured for it; it makes most kernel code preemptible and turns worst-case latency from milliseconds into tens of microseconds.
- **Scheduling policy.** `SCHED_FIFO` at a modest priority for the control thread, above the default but below the kernel’s own threads.
- **Lock memory, disable paging.** `mlockall(MCL_CURRENT | MCL_FUTURE)`, and pre-fault the stack. A page fault in the control loop is a multi-millisecond stall.
- **No allocation in the loop.** Covered in [02_ros2_control.ipynb](../04_Simulation_and_Hardware/ros2_control.llms.md) and worth repeating: allocation can take a lock and can fault.
- **CPU isolation and affinity.** `isolcpus` plus pinning the control thread, so the loop does not share a core with the rest of the system. On a 4-core machine this is a real cost and still usually worth it.
- **Interrupt affinity.** Move device interrupts off the isolated core.
- **Turn off frequency scaling and deep C-states** for that core; wake-up latency from a deep idle state is measurable.

On the ROS 2 side:

- **Keep the real-time path out of the middleware.** `ros2_control`’s loop does not publish from `read`/`write`; it hands data to non-real-time publishers.
- **Use a real-time-friendly RMW configuration.** Both Fast DDS and Cyclone DDS can be configured for static allocation and no discovery traffic during operation; see [../07_Middleware_DDS/00_Discovery_and_RMW.ipynb](../07_Middleware_DDS/discovery_and_rmw.llms.md).
- **Measure, do not assume.** `cyclictest` for kernel latency, and a histogram of your own loop period. The number that matters is the maximum over hours, not the mean over a minute.

And the honest caveat: **most robots do not need hard real-time.** A 50 Hz navigation stack with a few milliseconds of jitter is fine. Hard real-time is for balancing, legged locomotion, force control and anything where a missed cycle is a fall. Paying its cost without that requirement buys complexity and nothing else.

------------------------------------------------------------------------

Back to top
