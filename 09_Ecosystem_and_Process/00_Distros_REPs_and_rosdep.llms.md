# Distros, REPs and rosdep

The release cycle and what an LTS commitment actually means, how REPs define the conventions the rest of this site relies on, and what rosdep is doing underneath.

Author

Benedict Thekkel

## The Release Cycle

ROS 2 releases once a year, in May, on World Turtle Day. Each release is pinned to one Ubuntu LTS or interim release, and the pairing is not a recommendation: the binaries are built against that Ubuntu’s system libraries.

Every other year the release is an **LTS** with five years of support; the years between get about 18 months. The distribution table, and which one this site targets, is on the [index page](../index.llms.md).

What the support commitment actually means, which is narrower than it sounds:

- **Core packages get bug fixes and security patches** for the support period, through `ros-<distro>-*` apt packages.
- **No new features** after the release, and no API breaks. This is the point of an LTS.
- **Third-party packages are not covered.** A community package may be maintained for one distribution and abandoned for the next, and the LTS promise says nothing about it. Checking that your dependencies actually have a release for the distribution you are choosing is the real compatibility question.
- **End of life means the apt repository stops being updated**, not that it disappears immediately. Staying on an EOL distribution is viable and steadily more painful.

``` bash
printenv ROS_DISTRO
ros2 doctor --report | grep -A3 "ROS 2 INFORMATION"    # warns if the distro is EOL
apt list --installed | grep -c ^ros-jazzy              # how many ROS packages are installed
```

**Rolling Ridley** is the development branch: always current, no stability promise, and the place new packages land first. Develop against an LTS and test against Rolling if you maintain a package; do not run a robot on it.

------------------------------------------------------------------------

## Upgrading, and the Trap Worth Knowing

A distribution upgrade is a re-test, not an apt command. Message definitions change, default parameter values change, and deprecated APIs are removed.

The trap is specific and it has been hit in practice: **`packages.ros.org` is pinned by Ubuntu suite**, so upgrading the operating system silently changes the ROS distribution. On the public [piros2](https://github.com/bthek1/piros2) project, an Ubuntu release upgrade replaced all 303 `ros-jazzy-*` packages with the next distribution’s and moved Python 3.12 to 3.14 in the same step, leaving one machine on a different ROS version from its robot. Two consequences followed:

- **A missing RMW package makes every `rclpy` process exit 1**, almost silently, which under a test runner reads as the test framework’s fault. See [../07_Middleware_DDS/00_Discovery_and_RMW.ipynb](../07_Middleware_DDS/discovery_and_rmw.llms.md).
- **Python version changes break the dependency set**, not just the ROS packages - a library with no wheel for the new interpreter can no longer share a process with `rclpy` at all.

So: **upgrade the OS deliberately, knowing it upgrades ROS**, and never upgrade one machine of a pair. Cross-distribution graphs are possible, since the wire protocol is DDS, but message definitions drift between releases, so treat interoperability as something to measure rather than assume.

A sane upgrade order: read the release notes for removals; build the workspace on the new distribution in a container or a spare machine; run the test suite; then upgrade a robot.

------------------------------------------------------------------------

## REPs

REPs (ROS Enhancement Proposals) are the specifications that make independently written nodes interoperate. They are worth knowing by number because the ones that matter are cited constantly.

| REP | Defines | Where it appears here |
|----|----|----|
| **REP-103** | units (SI), axis conventions (x forward, y left, z up), ENU for geographic frames | [../03_Spatial_and_Temporal/02_Conventions_and_Time.ipynb](../03_Spatial_and_Temporal/conventions_and_time.llms.md) |
| **REP-105** | the meaning of `earth`, `map`, `odom`, `base_link` and their guarantees | [../06_Navigation_and_Manipulation/00_SLAM_and_Localization.ipynb](../06_Navigation_and_Manipulation/slam_and_localization.llms.md) |
| REP-107 | diagnostics conventions | [../08_Testing_Deployment_Ops/02_Service_Management_and_Diagnostics.ipynb](../08_Testing_Deployment_Ops/service_management_and_diagnostics.llms.md) |
| REP-117 | meaning of out-of-range sensor readings (`inf` and `NaN`) | [../05_Perception/01_Point_Clouds_and_Lidar.ipynb](../05_Perception/point_clouds_and_lidar.llms.md) |
| REP-140 | `package.xml` format 2 | [../02_Build_and_Tooling/00_Workspaces_and_Packages.ipynb](../02_Build_and_Tooling/workspaces_and_packages.llms.md) |
| REP-144 | package naming rules |  |
| REP-2000 | **which distribution targets which platforms and dependency versions** | this notebook |
| REP-2004 | package quality levels, 1 to 5 |  |

Two of those repay a direct read. **REP-2000** is the authoritative answer to “what Ubuntu, what Python, what DDS version does this distribution use”, which is otherwise guesswork. **REP-2004** defines the quality levels that core packages advertise: level 1 means version-stable, documented, tested with coverage and a declared maintainer, while level 4 or 5 is essentially “it exists”. Checking a dependency’s quality level before building on it is a cheap way to avoid a surprise.

The design documents at `design.ros2.org` complement these: they explain *why* ROS 2 works as it does (the DDS choice, the QoS model, the executor design) rather than specifying conventions.

------------------------------------------------------------------------

## rosdep

`rosdep` maps the dependency names in `package.xml` onto the actual packages of whatever platform you are on. It is the reason a checkout is buildable without reading anyone’s README.

``` bash
sudo rosdep init            # once per machine: installs the default sources list
rosdep update               # as the normal user: fetches the rules
rosdep install --from-paths src --ignore-src -r -y
rosdep resolve python3-numpy   # what a key maps to here
rosdep keys --from-paths src --ignore-src     # what this workspace needs
```

The mechanism, because understanding it makes the failures obvious:

1.  `package.xml` names a **rosdep key**, for example `python3-numpy` or `libopencv-dev`.
2.  The rules come from YAML in the `rosdistro` repository, keyed by platform:

``` yaml
python3-numpy:
  ubuntu:
    noble: [python3-numpy]
  fedora: [python3-numpy]
```

3.  `rosdep` resolves the key for the current platform and invokes the package manager.

Where it goes wrong:

- **“Cannot locate rosdep definition for X”** means the key does not exist, usually a typo or a pip-only dependency that has no system package. A ROS package dependency resolves automatically from the distribution index, so this error on a `ros-*` name usually means that package has no release for your distribution.
- **`rosdep update` must be run as your user**, not with sudo, or the cache lands in root’s home and your user still has none.
- **A stale cache** after a new dependency is released; `rosdep update` again.
- **pip dependencies are second-class.** `<exec_depend>python3-pip-name</exec_depend>` works only if a rosdep key exists. Local rules can be added:

``` yaml
# /etc/ros/rosdep/sources.list.d/50-my-rules.list
yaml file:///home/me/my-rosdep-rules.yaml
```

Keeping `package.xml` honest is what makes this work, and the check is mechanical: **a CI job that runs `rosdep install` from a clean container is the only reliable test** that the manifest is complete. A dependency installed on your machine and missing from the manifest passes locally forever. See [../08_Testing_Deployment_Ops/00_Testing_and_Linting.ipynb](../08_Testing_Deployment_Ops/testing_and_linting.llms.md).

------------------------------------------------------------------------

Back to top
