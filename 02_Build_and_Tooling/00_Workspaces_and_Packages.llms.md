# Workspaces and Packages

Installing ROS 2, then the build layer: workspaces and overlays, package manifests for ament_python and ament_cmake, colcon, and rosdep.

Author

Benedict Thekkel

## Installing ROS 2

The Debian packages are the normal path on Ubuntu. Add the apt repository, then install either the `desktop` metapackage (core plus RViz2, demos and tutorials) or `ros-base` for a headless robot.

``` bash
# Repository setup (once)
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install (Jazzy on Ubuntu 24.04)
sudo apt update && sudo apt install ros-jazzy-desktop -y

# Build tooling
sudo apt install ros-dev-tools python3-colcon-common-extensions -y
```

Every shell needs the distribution sourced before `ros2` exists:

``` bash
source /opt/ros/jazzy/setup.bash
```

Putting that in `~/.bashrc` is what most people do, and it is also the cause of a whole class of confusing bugs once there is more than one distribution or more than one workspace on the machine. The symptom is a package that builds against the wrong headers or a node that imports a stale Python module; the cure is to source explicitly per shell and keep `~/.bashrc` clean on any machine with two distributions.

**The distribution is pinned to an Ubuntu release**, so the choice is mostly decided by what is on the robot. Jazzy Jalisco is the LTS for Ubuntu 24.04 and what this site targets; the full distribution and support-window table is in [../09_Ecosystem_and_Process/00_Distros_REPs_and_rosdep.ipynb](../09_Ecosystem_and_Process/distros_reps_and_rosdep.llms.md).

------------------------------------------------------------------------

## Workspaces and Overlays

Source code lives in a **workspace**: a directory with a `src/` folder holding packages. `colcon build` produces `build/`, `install/` and `log/` as siblings.

``` bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Sourcing a workspace’s `install/setup.bash` **overlays** it on whatever was sourced before, and the last one sourced wins for any package present in more than one layer:

    /opt/ros/jazzy           underlay (the distribution)
      ~/ros2_ws/install      overlay  (your packages, and your rebuilt forks)

This is how a distribution package is replaced by a local fork: clone it into `src/`, build, and the overlay shadows the apt version. It is also how a stale build quietly wins, because nothing warns that the overlay contains an older copy of a package that apt has since updated.

Three rules keep overlays sane:

- **Never build a workspace with another workspace of your own sourced**, unless the layering is deliberate. `colcon build` bakes discovered dependency paths into the result.
- **One distribution per shell.** Sourcing Humble and Jazzy into one shell produces failures that look like compiler bugs.
- **`rm -rf build install log` is the first debugging step** for any inexplicable build error. colcon’s incremental state is not always right after a package is renamed, removed, or has its dependencies changed.

`--symlink-install` links Python files into `install/` rather than copying them, so editing a Python node takes effect without a rebuild. C++ still needs one. Adding a new file, an entry point, or a dependency also needs a rebuild either way.

------------------------------------------------------------------------

## Packages and Manifests

``` bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_pkg      # Python
ros2 pkg create --build-type ament_cmake my_cpp_pkg   # C++
ros2 pkg create --build-type ament_cmake my_interfaces  # custom messages, always cmake
```

A Python package is an ordinary Python package plus ROS metadata:

    my_pkg/
      package.xml          # name, version, dependencies (the ROS manifest)
      setup.py             # entry points: maps an executable name to a main()
      setup.cfg
      resource/my_pkg      # empty marker file the ament index looks for
      my_pkg/
        __init__.py
        drive.py

`package.xml` is what `rosdep` and the buildfarm read:

``` xml
<package format="3">
  <name>my_pkg</name>
  <version>0.1.0</version>
  <description>Drives the wheels.</description>
  <maintainer email="me@example.com">Me</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>                      <!-- build and run -->
  <depend>geometry_msgs</depend>
  <exec_depend>ros2launch</exec_depend>       <!-- run only -->
  <test_depend>ament_copyright</test_depend>

  <export><build_type>ament_python</build_type></export>
</package>
```

`setup.py` is where an executable name comes from, and forgetting the entry point is why `ros2 run my_pkg drive` reports “no executable found” on a package that built fine:

``` python
entry_points={"console_scripts": ["drive = my_pkg.drive:main"]},
data_files=[("share/" + package_name, ["package.xml"]),
            ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
            ("share/" + package_name + "/config", glob("config/*.yaml"))],
```

**Files not listed in `data_files` are not installed**, so a launch file or YAML that works when run from the source tree vanishes under `ros2 launch`. The `ament_cmake` equivalent is an `install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})` line.

The `resource/my_pkg` marker is not decoration: the ament index uses it to find the package, and deleting it makes the package invisible to `ros2 pkg list` after a build.

------------------------------------------------------------------------

## colcon

`colcon` builds every package in `src/` in dependency order, in parallel.

``` bash
colcon build                                  # everything
colcon build --packages-select my_pkg         # one package
colcon build --packages-up-to my_pkg          # it and its dependencies
colcon build --packages-skip-build-finished   # skip what is already built
colcon build --symlink-install
colcon build --event-handlers console_direct+  # stream compiler output live
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`--packages-select` is the difference between a 4 second and a 4 minute iteration loop on a real robot workspace, and `console_direct+` is how a compiler error becomes readable instead of buffered into a summary.

Testing:

``` bash
colcon test --packages-select my_pkg
colcon test-result --verbose                  # the failures, in detail
```

`colcon test` reports a pass even when tests were not discovered, so `colcon test-result` is the command that actually tells you what ran. Test conventions are in [../08_Testing_Deployment_Ops/00_Testing_and_Linting.ipynb](../08_Testing_Deployment_Ops/testing_and_linting.llms.md).

On a 4-core machine an unrestricted parallel build of a large workspace will exhaust RAM and be killed by the OOM reaper, which presents as an unexplained compiler crash:

``` bash
MAKEFLAGS="-j2" colcon build --parallel-workers 2
```

------------------------------------------------------------------------

## rosdep

`rosdep` resolves the dependencies declared in `package.xml` onto system packages, so a checkout becomes buildable without reading anyone’s README.

``` bash
sudo rosdep init          # once per machine
rosdep update             # as the normal user, refreshes the rules
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

`--ignore-src` stops it trying to apt-install packages that are present in the workspace, `-r` continues past errors, `-y` accepts. This is also the exact command a CI job runs, so a dependency that is installed on the developer machine but missing from `package.xml` passes locally and fails in CI. Declaring dependencies is not bookkeeping; it is what makes the build reproducible.

A dependency with no rosdep rule (a pip-only package, a vendor SDK) needs either a local rules file or documented manual installation. The mapping database and release process are covered in [../09_Ecosystem_and_Process/00_Distros_REPs_and_rosdep.ipynb](../09_Ecosystem_and_Process/distros_reps_and_rosdep.llms.md).

------------------------------------------------------------------------

Back to top
