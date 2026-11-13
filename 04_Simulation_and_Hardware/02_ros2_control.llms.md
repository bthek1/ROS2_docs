# ros2_control

The real-time control framework: hardware interfaces, the controller manager and its lifecycle, the standard controllers and broadcasters, and why mock hardware is the fastest way to test a controller.

Author

Benedict Thekkel

## What It Is For

A robot needs a loop that reads joint states, computes commands and writes them, at a fixed rate, without a missed cycle. A plain ROS 2 node cannot promise that: topics are asynchronous, executors are not real-time, and a subscriber callback can be delayed arbitrarily.

`ros2_control` is the answer. It provides a **real-time control loop** in one process, with two swappable halves:

      controllers            diff_drive, joint_trajectory, broadcasters
           |                 read state interfaces, write command interfaces
     controller_manager      owns the loop, runs at `update_rate`
           |
     resource_manager        claims and arbitrates interfaces
           |
     hardware_interface      your plugin: read() and write()
           |
      real hardware          CAN, EtherCAT, serial, or a simulator

The payoff is that **controllers and hardware are independent**. `diff_drive_controller` does not know whether it is driving Gazebo, mock hardware or a CAN bus, and a new robot needs a hardware plugin rather than new controllers. This is also what makes the simulation-to-hardware boundary clean; see [01_Other_Simulators_and_Sim_to_Real.ipynb](../04_Simulation_and_Hardware/other_simulators_and_sim_to_real.llms.md).

Everything inside `controller_manager` runs in one process and one thread. Topics exist only at the edges: commands in, states out.

------------------------------------------------------------------------

## Declaring Hardware in the URDF

The hardware and its interfaces are declared in the robot description, inside a `<ros2_control>` block. This is the contract between the plugin and the controllers.

``` xml
<ros2_control name="MyRobotSystem" type="system">
  <hardware>
    <plugin>my_robot_hardware/MyRobotSystem</plugin>
    <param name="device">/dev/ttyUSB0</param>
    <param name="baud_rate">115200</param>
  </hardware>

  <joint name="wheel_left_joint">
    <command_interface name="velocity">
      <param name="min">-10</param>
      <param name="max">10</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="wheel_right_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

Three hardware `type`s, and the distinction is about direction:

| Type | Has | Example |
|----|----|----|
| `system` | multiple joints, commands and states | a mobile base, an arm |
| `actuator` | one joint, commands and states | a single motor controller |
| `sensor` | states only, no commands | an IMU, a force/torque sensor |

Interface names are conventions that controllers rely on: `position`, `velocity`, `effort` for joints, plus arbitrary names for sensors (`orientation.x`, `force.z`). **A controller that asks for an interface the hardware does not declare fails to activate**, with a message naming the missing interface; that is the most common bring-up error and it is a good error.

Swapping in simulation or mock hardware is a change of the `<plugin>` line only:

``` xml
<plugin>mock_components/GenericSystem</plugin>      <!-- no physics, echoes commands to states -->
<plugin>gz_ros2_control/GazeboSimSystem</plugin>    <!-- Gazebo Harmonic -->
```

------------------------------------------------------------------------

## Writing a Hardware Interface

A hardware plugin is a C++ class (there is no Python equivalent, by design: this code runs in the real-time loop) implementing a lifecycle plus `read` and `write`.

``` cpp
class MyRobotSystem : public hardware_interface::SystemInterface {
  CallbackReturn on_init(const HardwareInfo & info) override;        // parse params, size vectors
  CallbackReturn on_configure(const State &) override;               // open the device
  CallbackReturn on_activate(const State &) override;                // enable motors
  CallbackReturn on_deactivate(const State &) override;              // disable motors
  std::vector<StateInterface> export_state_interfaces() override;
  std::vector<CommandInterface> export_command_interfaces() override;
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
};
```

The lifecycle is the same state machine as a lifecycle node ([../01_Core_Concepts/04_Executors_Lifecycle_and_Composition.ipynb](../01_Core_Concepts/executors_lifecycle_and_composition.llms.md)): allocate in `on_configure`, energise in `on_activate`, and make sure `on_deactivate` leaves the motors safe.

**`read()` and `write()` are the real-time path, and the rules there are strict:**

- **No allocation.** No `new`, no `std::vector` growth, no `std::string` construction. Size everything in `on_init`.
- **No blocking.** No mutex that a non-real-time thread holds, no logging in the common path, no file or network I/O that can stall. A blocking serial read inside `write()` is the classic way to destroy the loop rate.
- **No exceptions** escaping; return `return_type::ERROR` instead.
- **Bounded time, every cycle.** A 10 ms budget at 100 Hz means the worst case, not the average.

Serial and CAN transports usually need their own thread with a lock-free handoff, because a device read cannot be made to fit the budget reliably. See [03_Drivers_and_micro_ROS.ipynb](../04_Simulation_and_Hardware/drivers_and_micro_ros.llms.md).

------------------------------------------------------------------------

## The Controller Manager and Its Lifecycle

`controller_manager` is the node that owns the loop. It is configured by a YAML file that sets the rate and lists the controllers.

``` yaml
controller_manager:
  ros__parameters:
    update_rate: 100        # Hz, the control loop rate

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["wheel_left_joint"]
    right_wheel_names: ["wheel_right_joint"]
    wheel_separation: 0.32
    wheel_radius: 0.05
    odom_frame_id: odom
    base_frame_id: base_link
    enable_odom_tf: true
    use_stamped_vel: false
```

Controllers have the lifecycle states `unconfigured`, `inactive` and `active`, and only an `active` controller claims interfaces and runs. Loading and activating them:

``` bash
ros2 control list_hardware_interfaces          # what exists, and who claims it
ros2 control list_controllers                  # state of each
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control set_controller_state diff_drive_controller inactive
ros2 control switch_controllers --activate diff_drive_controller
```

In a launch file, the `spawner` does load-configure-activate in one step:

``` python
Node(package="controller_manager", executable="spawner",
     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]),
Node(package="controller_manager", executable="spawner",
     arguments=["diff_drive_controller", "-c", "/controller_manager"]),
```

Four things that account for most bring-up time:

- **`joint_state_broadcaster` must be active or there is no tf tree.** It is what turns state interfaces into `/joint_states`, which `robot_state_publisher` needs. Forget it and the robot appears collapsed at the origin with no error anywhere; see [../03_Spatial_and_Temporal/01_Robot_Description.ipynb](../03_Spatial_and_Temporal/robot_description.llms.md).
- **Two controllers cannot claim the same command interface.** Activating a second one fails with a resource-conflict message. This is deliberate, and it is how you switch between position and velocity control safely: deactivate one, activate the other, in a single `switch_controllers` call.
- **`update_rate` must be achievable.** If `read` plus `write` plus the controllers exceed the period, the loop overruns and the log fills with missed-cycle warnings. Start at 50 Hz and raise it with evidence.
- **Spawner ordering matters.** A controller spawned before the hardware is active fails; the convention is to chain spawners on `OnProcessExit` of the previous one rather than trusting a `TimerAction`.

------------------------------------------------------------------------

## The Standard Controllers

Most robots need no custom controller at all.

| Controller | For |
|----|----|
| `joint_state_broadcaster` | **always**: state interfaces to `/joint_states` |
| `diff_drive_controller` | differential-drive base: `Twist` in, wheel velocities and odometry out |
| `ackermann_steering_controller`, `tricycle_controller` | other base geometries |
| `joint_trajectory_controller` | arms: a `JointTrajectory` action with interpolation |
| `forward_command_controller` | pass a value straight to an interface; good for bring-up |
| `position_controllers/JointGroupPositionController` | position commands on a group |
| `velocity_controllers/JointGroupVelocityController` | velocity commands on a group |
| `effort_controllers/JointGroupEffortController` | torque commands |
| `imu_sensor_broadcaster` | an IMU state interface to `sensor_msgs/Imu` |
| `force_torque_sensor_broadcaster` | F/T sensor to `WrenchStamped` |

`diff_drive_controller` is worth one note: it publishes `/odom` **and** the `odom -> base_link` transform when `enable_odom_tf: true`. If an EKF (`robot_localization`) is also publishing that edge, you get the flickering-tree failure from [../03_Spatial_and_Temporal/00_tf2.ipynb](../03_Spatial_and_Temporal/tf2.llms.md). Turn it off in the controller when fusing.

`joint_trajectory_controller` is the interface MoveIt 2 drives, which is why an arm brought up with it works with MoveIt without further plumbing.

------------------------------------------------------------------------

## Testing Without Hardware

`mock_components/GenericSystem` is the fastest feedback loop available: it echoes commands straight back as states, so the whole stack runs with no physics engine and no robot.

``` xml
<ros2_control name="MockSystem" type="system">
  <hardware>
    <plugin>mock_components/GenericSystem</plugin>
    <param name="calculate_dynamics">true</param>   <!-- integrate velocity into position -->
  </hardware>
  ...
</ros2_control>
```

``` bash
ros2 launch my_bringup bringup.launch.py use_mock_hardware:=true
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.2}}}"
ros2 topic echo /joint_states
ros2 topic echo /odom
```

This answers, in seconds and with no GPU: are the controllers loading, are the interfaces claimed, is the YAML keyed correctly, does odometry integrate in the right direction, does the tf tree appear. Those are most bring-up bugs. What it cannot answer is anything about dynamics, because there are none.

Debugging order when a controller will not activate:

``` bash
ros2 control list_hardware_interfaces     # does the interface exist, and is it claimed
ros2 control list_controllers             # state, and the type that failed to load
ros2 param get /controller_manager update_rate
ros2 topic echo /joint_states --once      # is the broadcaster active
```

| Symptom | Cause |
|----|----|
| controller stuck `unconfigured` | YAML key does not match the controller name, so it has no parameters |
| activation fails naming an interface | hardware does not declare it, or another controller holds it |
| no `/joint_states` | `joint_state_broadcaster` not spawned |
| robot collapsed at origin in RViz | same |
| missed-cycle warnings | `update_rate` too high, or blocking I/O in `read`/`write` |
| plugin not found | missing `pluginlib` export, or the package was not sourced |

------------------------------------------------------------------------

Back to top
