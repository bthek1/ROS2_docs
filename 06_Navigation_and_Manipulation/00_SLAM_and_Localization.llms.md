# SLAM and Localization

Building a map and knowing where you are in it: slam_toolbox for 2D, RTAB-Map and visual SLAM for 3D, AMCL, and fusing odometry sources with robot_localization.

Author

Benedict Thekkel

## The Two Jobs, and the Frames That Encode Them

**SLAM** builds a map while estimating the pose in it. **Localization** estimates the pose in a map that already exists. They are different problems and different nodes, and a robot normally does SLAM once and localization forever after.

Both publish the same edge: `map -> odom`. Nothing else should.

    map  --(SLAM or AMCL: the correction, jumps)-->  odom
    odom --(wheel odometry or an EKF: smooth, drifts)-->  base_link

That split is REP-105 and it is the single most useful thing to keep straight, because it tells you which node to suspect. A pose that jumps is the localiser correcting; a pose that drifts is odometry doing its job. See [../03_Spatial_and_Temporal/02_Conventions_and_Time.ipynb](../03_Spatial_and_Temporal/conventions_and_time.llms.md).

**Exactly one node may publish `map -> odom`.** Running SLAM and AMCL together, or AMCL and an EKF configured to publish it, gives the flickering tree from [../03_Spatial_and_Temporal/00_tf2.ipynb](../03_Spatial_and_Temporal/tf2.llms.md). The same applies to `odom -> base_link`: if `diff_drive_controller` publishes it and an EKF also does, turn one off. See [../04_Simulation_and_Hardware/02_ros2_control.ipynb](../04_Simulation_and_Hardware/ros2_control.llms.md).

------------------------------------------------------------------------

## slam_toolbox

The default for 2D lidar SLAM on Jazzy. It replaces gmapping and cartographer for most wheeled robots, and it does both mapping and localization.

``` bash
sudo apt install ros-jazzy-slam-toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

``` yaml
slam_toolbox:
  ros__parameters:
    mode: mapping                  # or localization
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    resolution: 0.05
    max_laser_range: 12.0
    minimum_travel_distance: 0.3   # only add a node after moving this far
    minimum_travel_heading: 0.3
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
```

Modes worth knowing: `mapping` builds a new map; `localization` loads a serialised pose graph and tracks within it; `lifelong` keeps updating a map while running, which is attractive and expensive.

Saving, and the distinction that catches people:

``` bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: my_map}}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: my_map}"
```

`save_map` writes the `.pgm` plus `.yaml` that AMCL and Nav2’s map server consume. `serialize_map` writes slam_toolbox’s own pose graph (`.posegraph` plus `.data`), which is what `localization` mode needs. **They are not interchangeable**, and discovering that at the point of switching to localization is the usual way to learn it.

Three practical points:

- **Odometry quality sets map quality.** slam_toolbox corrects odometry; it cannot rescue odometry that is wrong in scale or sign. Drive a 2 m square and check `odom` before blaming SLAM.
- **`minimum_travel_distance` controls node density.** Too small and the graph bloats; too large and loop closure has nothing to match.
- **Loop closure is what makes a map metric.** A long corridor traversed once produces a map that looks fine and is skewed. Drive loops deliberately.

------------------------------------------------------------------------

## AMCL

Localization in an existing map, by particle filter: `nav2_amcl` maintains a cloud of pose hypotheses, weights them by how well the scan matches the map, and resamples.

``` yaml
amcl:
  ros__parameters:
    min_particles: 500
    max_particles: 2000
    laser_model_type: likelihood_field
    max_beams: 60
    update_min_d: 0.25            # resample after this much motion
    update_min_a: 0.2
    alpha1: 0.2                   # rotation noise from rotation
    alpha2: 0.2                   # rotation noise from translation
    alpha3: 0.2                   # translation noise from translation
    alpha4: 0.2                   # translation noise from rotation
    set_initial_pose: false
    tf_broadcast: true
```

What goes wrong, in order of frequency:

- **No initial pose, so the filter never converges.** AMCL starts ignorant. Give it a pose with the RViz “2D Pose Estimate” tool, `set_initial_pose` plus `initial_pose` parameters, or a `/initialpose` publication. A robot that localises nowhere has usually just never been told where it is.
- **The `alpha*` motion-noise parameters are wrong for the robot.** Too low and the filter is overconfident and cannot recover from a bump; too high and the pose is permanently fuzzy. They are the main tuning knobs and they are robot-specific.
- **The map does not match the world.** Furniture moved, or the map was built at a different lidar height. AMCL will confidently localise to the map’s version of reality.
- **Symmetry.** A long empty corridor is genuinely ambiguous along its length, and no tuning fixes that; the particle cloud will stretch, correctly.

`ros2 topic echo /amcl_pose` gives the pose with its covariance, which is the honest measure of whether it has converged. Watching the particle cloud in RViz is the fast version.

------------------------------------------------------------------------

## Odometry Sources and robot_localization

Wheel odometry alone drifts, mostly in heading, and heading error turns into position error linearly with distance. Fusing it with an IMU fixes most of that. `robot_localization` provides an EKF for this.

``` yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true              # true for a wheeled ground robot: locks z, roll, pitch
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom             # this choice decides which edge is published

    odom0: /wheel/odometry
    odom0_config: [false, false, false,    # x, y, z
                   false, false, false,    # roll, pitch, yaw
                   true,  true,  false,    # vx, vy, vz
                   false, false, true,     # vroll, vpitch, vyaw
                   false, false, false]    # ax, ay, az
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, true,      # yaw
                  false, false, false,
                  false, false, true,      # vyaw
                  true,  false, false]     # ax
```

Two decisions dominate the configuration:

- **`world_frame` chooses the edge.** `world_frame: odom` fuses continuous sources and publishes `odom -> base_link`. `world_frame: map` fuses global sources (GPS, a localiser) and publishes `map -> odom`. A common setup runs two EKF instances, one of each. Setting `world_frame: map` on the instance that also has `publish_tf: true` while AMCL is running is how you get two publishers of the same edge.
- **Fuse velocities from wheels, not positions.** Wheel odometry’s absolute position is an integral of its own errors; its velocity is a measurement. The `odom0_config` above reflects that: velocities true, positions false.

Things that cost time here: **covariances that are left at zero or at defaults**, which makes the filter trust a source absolutely; an **IMU whose yaw is in the wrong convention** (REP-103 is counter-clockwise from east, most datasheets are clockwise from north, see [../03_Spatial_and_Temporal/02_Conventions_and_Time.ipynb](../03_Spatial_and_Temporal/conventions_and_time.llms.md)); and **fusing two sources that are not independent**, such as an IMU and a wheel odometry that already has the IMU’s yaw folded in, which makes the filter overconfident.

------------------------------------------------------------------------

## Visual and RGB-D SLAM

When there is no lidar, or the map needs to be 3D, the camera becomes the sensor. `rtabmap_ros` is the mature ROS 2 option, taking RGB-D or stereo plus odometry and producing a 3D map with loop closure.

``` bash
sudo apt install ros-jazzy-rtabmap-ros
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=base_link approx_sync:=false
```

Notes from actually running this on modest hardware (the public [piros2](https://github.com/bthek1/piros2) project, a Raspberry Pi 5 streaming to a dev box, with monocular neural depth rather than an RGB-D sensor):

- **A motionless scene produces a one-node map, and that is correct.** RTAB-Map merges lookalike frames, so a static bag yields a single graph node rather than a map. Measured odometry quality in the 447 to 563 range on such a bag is the pipeline working, not failing.
- **Its 5-second no-data warnings are a watchdog**, not a fault, when synced pairs arrive at 1 to 2 Hz. Its `delay=` figure is the age of the data, which on a replayed bag is the bag’s age.
- **`camera_info` with an all-zero `k` makes a recording unusable for mapping**, and nothing says so until RTAB-Map refuses to initialise. Record `camera_info` and check it; see [../05_Perception/00_Images_and_Calibration.ipynb](../05_Perception/images_and_calibration.llms.md).
- **Looping a bag teleports odometry.** Play once (`ros2 bag play` without `--loop`) when feeding SLAM, or the jump between the end and the start is interpreted as motion.
- **Exact sync needs a deep queue.** With `approx_sync:=false` and the default queue of 5, pairing was a coin toss: 0 to 6 odometry updates on a replay where a 30-deep queue deterministically gave
  24. Queue depth is not a tuning nicety here. See [../03_Spatial_and_Temporal/03_Message_Synchronisation.ipynb](../03_Spatial_and_Temporal/message_synchronisation.llms.md).
- **Monocular depth has no absolute scale.** A neural depth model gives relative depth, so a scale factor must be calibrated against a measured distance, and it wobbles per frame (about 4% on a static scene in that project). A 2.50 m wall read as 9.30 m until the scale was fixed. That wobble, not the pose estimate, was the limit on map quality.
- **Pure-rotation visual odometry is a trap worth naming.** The essential matrix is degenerate under pure rotation, so rotation-only estimators are used instead; but a hand-held “rotation only” pan actually contains translation (0.9 m of arm arc, measured), which smears the map radially.

For 3D mapping on better hardware the alternatives are `ORB-SLAM3`, and NVIDIA’s `isaac_ros_visual_slam` if the Isaac stack is already in play; see [../04_Simulation_and_Hardware/01_Other_Simulators_and_Sim_to_Real.ipynb](../04_Simulation_and_Hardware/other_simulators_and_sim_to_real.llms.md).

------------------------------------------------------------------------

## Choosing, and Diagnosing

| Situation | Use |
|----|----|
| 2D lidar, flat floors | `slam_toolbox` to map, then AMCL or slam_toolbox `localization` |
| RGB-D or stereo, 3D map needed | `rtabmap_ros` |
| no lidar, monocular camera only | visual SLAM, and expect to calibrate scale |
| outdoor with GPS | `robot_localization` with `navsat_transform_node` |
| known map, just need a pose | AMCL; do not run SLAM |

Diagnosing, in order:

``` bash
ros2 run tf2_tools view_frames            # is map -> odom -> base_link present, once each
ros2 topic hz /scan /odom /tf
ros2 topic echo /amcl_pose                # pose and covariance
ros2 topic echo /odom --field pose.pose.position
```

| Symptom | Likely cause |
|----|----|
| map drifts and shears over a long run | no loop closure; drive loops |
| pose jumps constantly in a static robot | localiser resampling on a poor scan match, or `alpha*` too high |
| pose never converges | no initial pose given |
| robot position in RViz lags reality | stamps, or `use_sim_time` set on some nodes only |
| map built but localization mode fails to load it | `save_map` output used where a serialised pose graph is required |
| SLAM produces a one-node map | the robot never moved far enough, by design |
| two poses flickering | two publishers of `map -> odom` |

The conceptual background on SLAM algorithms (graph optimisation, loop closure, scan matching) is covered in study notes kept outside this site and deliberately not linked here; this notebook is about the ROS 2 side.

------------------------------------------------------------------------

Back to top
