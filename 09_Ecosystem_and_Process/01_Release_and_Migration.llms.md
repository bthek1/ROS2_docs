# Release and Migration

Releasing a package with bloom so others can apt-install it, what the buildfarm does with it, and migrating ROS 1 code including the ros1_bridge.

Author

Benedict Thekkel

## Releasing a Package

Publishing to `packages.ros.org` so that `apt install ros-jazzy-my-pkg` works is a defined process, not a favour from anybody. `bloom` automates it.

What has to be true first:

- The package builds from a clean checkout with `rosdep install` alone.
- `package.xml` is complete and has a real `<maintainer>`, a `<license>` and a `<description>`.
- The repository has a version tag, and the source is on a public git host.
- Tests pass, because the buildfarm runs them.

``` bash
sudo apt install python3-bloom
catkin_generate_changelog            # from catkin_pkg: turns commits into CHANGELOG.rst
# edit CHANGELOG.rst into something a human would read
catkin_prepare_release               # bumps versions, commits, tags
bloom-release --rosdistro jazzy --track jazzy my_repo
```

`bloom-release` does not upload a binary. It opens a **pull request against the `rosdistro` repository** adding your package to the distribution index. Once that is merged, the buildfarm takes over.

Two points about the metadata, since they are what reviewers reject:

- **The version lives in `package.xml`** and `catkin_prepare_release` is what bumps it. Hand-editing it and forgetting the tag produces a release that cannot be built.
- **`CHANGELOG.rst` is read by users.** `catkin_generate_changelog` produces a list of commit subjects, which is a starting point, not a changelog.

------------------------------------------------------------------------

## The Buildfarm

Once the `rosdistro` PR is merged, `build.ros.org` does the rest:

1.  **Source build (`Ssrc`)**: makes a source package from your tag.
2.  **Binary build (`Pbin`)**: builds a `.deb` for every supported architecture in REP-2000, runs the tests, and publishes to the apt repository.
3.  **Devel and PR jobs**: build your branch on every push, and optionally every pull request.
4.  **Documentation jobs**: generate API docs if configured.

The buildfarm is also the mechanism that keeps the ecosystem honest, and the ways it fails are instructive:

- **It builds in a clean container with only declared dependencies.** A package that builds on your machine and not there has an incomplete `package.xml`, every time.
- **It builds for every architecture**, including arm64. Code that assumes x86 intrinsics fails there and nowhere else.
- **Tests run, and failures block the release.** Flaky integration tests become a release problem rather than an irritation; see [../08_Testing_Deployment_Ops/00_Testing_and_Linting.ipynb](../08_Testing_Deployment_Ops/testing_and_linting.llms.md).
- **A broken dependency breaks you.** If a package you depend on fails to build, yours does not get published either.
- **Sync is periodic.** A successful build reaches `packages.ros.org` at the next sync, which is days to weeks, not minutes. `ROS_REPO=testing` is how to get it sooner.

Most packages never need releasing. A private robot project is fine as a git checkout in a workspace, and releasing is for code other people will depend on.

------------------------------------------------------------------------

## Migrating from ROS 1

ROS 1 reached end of life with Noetic in May 2025, so migration is now maintenance rather than a choice. The changes that affect code, beyond the build system:

| ROS 1                   | ROS 2                                       |
|-------------------------|---------------------------------------------|
| `roscore`               | nothing; peer-to-peer DDS discovery         |
| `catkin`                | `colcon` with `ament`                       |
| `rospy`                 | `rclpy`, with explicit spin and an executor |
| global parameter server | per-node parameters                         |
| `std_msgs/String`       | `std_msgs/msg/String` (namespaced by kind)  |
| `rospy.Time.now()`      | `node.get_clock().now()`                    |
| `.launch` XML           | `.launch.py` (XML and YAML also supported)  |
| nodelets                | components                                  |
| `tf`                    | `tf2`                                       |
| dynamic_reconfigure     | parameters plus a set-parameters callback   |
| `actionlib`             | `rclpy.action` / `rclcpp_action`            |

The conversions that take the most rework, in order:

- **The callback and execution model.** `rospy.spin()` in ROS 1 hides threading; ROS 2 makes executors and callback groups explicit, and naive ports deadlock on the first service call from inside a callback. This is the single largest source of migration bugs; see [../01_Core_Concepts/04_Executors_Lifecycle_and_Composition.ipynb](../01_Core_Concepts/executors_lifecycle_and_composition.llms.md).
- **QoS.** ROS 1 topics were reliable and queued; ROS 2 profiles must match or nothing flows, silently. A ported node that publishes into the void is usually this; see [../01_Core_Concepts/03_QoS_Profiles.ipynb](../01_Core_Concepts/qos_profiles.llms.md).
- **Parameters.** A global parameter server becomes per-node declarations, so code that read another node’s parameters needs a service call or a redesign.
- **Launch files.** XML to Python is a rewrite, not a translation, because substitutions are evaluated in two stages; see [../02_Build_and_Tooling/01_Launch.ipynb](../02_Build_and_Tooling/launch.llms.md).
- **Time.** ROS 2 separates wall and simulated time per node, and `use_sim_time` must be set everywhere or nowhere; see [../03_Spatial_and_Temporal/02_Conventions_and_Time.ipynb](../03_Spatial_and_Temporal/conventions_and_time.llms.md).

Practical advice: **port node by node, newest and smallest first**, and keep the interfaces identical so the two halves can be bridged while the work proceeds. A whole-system rewrite has no working state in the middle.

------------------------------------------------------------------------

## ros1_bridge

`ros1_bridge` connects a ROS 1 graph to a ROS 2 graph, translating messages with matching definitions. It is a migration tool.

``` bash
# needs both distributions sourced, in this order
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

The constraints are significant and they are why this is a transition tool rather than an architecture:

- **It needs both distributions installed on one machine**, and the ROS 1 and ROS 2 Ubuntu releases must coincide. Noetic is 20.04; ROS 2 distributions on 20.04 are Foxy and Galactic, both long EOL. Bridging Noetic to Jazzy therefore means containers or a source build, and is genuinely awkward.
- **Custom messages require building the bridge from source** with both message sets available, so it can generate the mapping pairs.
- **Only matching definitions bridge.** A field added on one side means that type does not map.
- **`dynamic_bridge` bridges what it sees**, which means topics with a publisher and subscriber on opposite sides; `--bridge-all-topics` forces everything and costs bandwidth.
- **QoS is guessed.** The bridge picks a profile for the ROS 2 side, and latched ROS 1 topics need transient-local on the ROS 2 side to behave as expected.

Given that, the honest recommendation for new work is to **avoid the bridge if the remaining ROS 1 code can be ported instead**. Where it genuinely earns its place is a large, working ROS 1 system with one or two components that must move to ROS 2, and a deadline. For a long-lived boundary, writing a small translator node for the handful of topics that actually cross is usually less trouble than keeping two distributions co-installed.

------------------------------------------------------------------------

Back to top
