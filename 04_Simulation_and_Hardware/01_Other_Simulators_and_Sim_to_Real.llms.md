# Other Simulators and Sim to Real

Isaac Sim and Webots against Gazebo, how to choose, and the sim-to-real gap: what transfers from a simulation unchanged and what has to be earned on hardware.

Author

Benedict Thekkel

## Webots

Webots is the lightweight option: one binary, no separate bridge process, and a Python or C++ controller API wrapped for ROS 2 by `webots_ros2`.

``` bash
sudo apt install ros-jazzy-webots-ros2
ros2 launch webots_ros2_universal_robot multirobot_launch.py
```

What it is good at: running on a laptop without a discrete GPU, starting in seconds, and shipping a large library of working robot models. The `webots_ros2_driver` node maps a Webots robot to ROS 2 topics from a URDF-like description, so a teaching or algorithm project gets a working robot quickly.

What it is not: the ecosystem default. Nav2, MoveIt and `ros2_control` examples are written against Gazebo, so Webots means translating them. Its physics is ODE-based and adequate rather than exacting, and contact-rich manipulation is not its strength.

------------------------------------------------------------------------

## Isaac Sim

NVIDIA’s simulator, built on Omniverse: photorealistic rendering, PhysX, and a focus on synthetic data generation and reinforcement learning.

Where it wins:

- **Photorealistic sensor simulation.** If the model is trained on images, rendering quality is the sim-to-real gap, and Isaac is a different class from Gazebo here.
- **Synthetic data at scale**, with domain randomisation and automatic ground-truth labelling.
- **Massively parallel RL** through Isaac Lab: thousands of environments on one GPU.
- **Isaac ROS** gives GPU-accelerated perception nodes (NITROS, VSLAM, nvblox) that pair with it.

The costs are real and should be checked before committing:

- **RTX-class GPU required**, and a generous one. On a 12 GB card Isaac Sim alone is most of the budget, which leaves little for the model being tested. It is not a background process.
- **Closed source**, under NVIDIA’s licence, NVIDIA hardware only.
- **USD, not SDF or URDF.** Conversion exists (and improves each release) but is a step, and the imported robot usually needs fixing up.
- **Heavy.** Startup is minutes, not seconds, and headless CI needs a GPU runner.

------------------------------------------------------------------------

## Choosing

| If the constraint is | Use |
|----|----|
| Nav2, MoveIt, `ros2_control` tutorials working out of the box | **Gazebo Harmonic** |
| no discrete GPU, or a fast iteration loop | **Webots** |
| training a vision model that must transfer | **Isaac Sim** |
| reinforcement learning at scale | **Isaac Lab** |
| CI on a cheap runner | **Gazebo headless** (`gz sim -s`) |
| testing controller logic only, no physics | **`ros2_control` mock hardware**, no simulator at all |

That last row is the one people skip. If the question is whether a controller commands the right joint velocities, `mock_components/GenericSystem` answers it in a second with no physics engine at all. See [02_ros2_control.ipynb](../04_Simulation_and_Hardware/ros2_control.llms.md).

Gazebo remains the default for this site. Nothing below is Gazebo-specific.

------------------------------------------------------------------------

## What Transfers, and What Does Not

The useful way to think about a simulator is as a test of **structure**, not of behaviour.

Transfers essentially unchanged:

- **The graph.** Node names, topics, services, namespaces, QoS profiles.
- **Launch structure and parameters.** The same launch files with `use_sim_time` flipped.
- **Kinematics and the tf tree.** A URDF that is right in simulation is right on hardware, which is exactly why one description serves both.
- **Logic.** State machines, behaviour trees, planners, anything that reasons about geometry.
- **Interfaces.** A `ros2_control` controller written against a simulated hardware interface runs against a real one.

Does not transfer, and each one has bitten somebody:

| Simulated | Reality |
|----|----|
| friction and contact | the dominant error for wheeled and legged robots; tyre slip, compliance and backlash are crudely modelled at best |
| motor dynamics | a velocity command is met instantly in simulation; a real motor has inertia, current limits, a gearbox and thermal derating |
| sensor noise | simulated sensors are clean unless noise is configured, so filters are tuned against a problem that does not exist |
| latency | USB, serial and network delays, and driver buffering, are mostly absent |
| timing | simulation time can be paused and is uniform; wall time is neither |
| lighting | the camera’s automatic exposure, motion blur and rolling shutter are usually unmodelled |
| failure | cables come loose, SD cards corrupt, Wi-Fi links die, sensors brown out |

The last row is worth stating plainly: **a simulation has no failure modes except the ones you write**, and most robot time is spent on failures. Experience on this site’s own hardware bears that out; the public [piros2](https://github.com/bthek1/piros2) project log records two Wi-Fi link deaths where the machine stayed up, continued writing journals, and was simply unreachable, which is not a state any simulator produces.

------------------------------------------------------------------------

## Practices That Narrow the Gap

- **One description, one launch tree.** The same xacro and the same launch files with a `use_sim_time` argument. A simulation-only launch file diverges within weeks, and then the simulation is testing itself.
- **One hardware interface boundary.** `ros2_control` with a simulated plugin on one side and real hardware on the other means controllers, their parameters and their tuning are shared. This is the highest-value structural decision available.
- **Add noise on purpose.** Configure sensor noise in the description, and make it worse than you expect. A filter tuned on clean data is tuned for nothing.
- **Record bags on both, and compare the same topics.** A bag from simulation and a bag from hardware, run through the same analysis, tells you where the gap actually is rather than where you assume. See [../02_Build_and_Tooling/02_CLI_and_Introspection.ipynb](../02_Build_and_Tooling/cli_and_introspection.llms.md).
- **Measure end-to-end latency on hardware early.** It is the quantity simulation models worst and the one that breaks control loops.
- **Do not tune gains in simulation** beyond getting them stable. Friction and motor dynamics are the two things simulation gets most wrong, and they are exactly what gains compensate for.
- **Randomise what you cannot measure.** If friction is a guess, train or test across a range of guesses rather than committing to one.

The honest summary: **simulation catches integration bugs, not physical ones.** That is a large fraction of the bugs, and it catches them without breaking hardware, which is why it is worth the setup. It is not a substitute for a robot.

------------------------------------------------------------------------

Back to top
