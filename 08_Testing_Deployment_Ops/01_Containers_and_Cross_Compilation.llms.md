# Containers and Cross-Compilation

Docker for ROS 2 and the networking boundary DDS has to cross, dev containers, and building arm64 images for a robot from an x86 workstation.

Author

Benedict Thekkel

## The Images

``` bash
docker pull ros:jazzy-ros-core          # rclcpp, rclpy, the CLI. Smallest
docker pull ros:jazzy-ros-base          # + colcon, rosdep, build tools. The usual base
docker pull osrf/ros:jazzy-desktop      # + RViz2, rqt, demos. Large
```

`ros:*` images are the official minimal ones; `osrf/ros:*` add the desktop tooling. For a robot, build `FROM ros:jazzy-ros-base` and install only what the package needs.

``` dockerfile
FROM ros:jazzy-ros-base AS build
WORKDIR /ws
COPY src/ src/
# rosdep first, in its own layer, so a source change does not reinstall apt packages
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y \
 && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/jazzy/setup.sh \
 && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

FROM ros:jazzy-ros-base
COPY --from=build /ws/install /ws/install
COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "my_bringup", "robot.launch.py"]
```

``` bash
#!/bin/bash
# ros_entrypoint.sh
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
exec "$@"
```

Three points specific to ROS in containers:

- **The entrypoint must source both setups.** A container that runs `ros2` without sourcing fails with “command not found”, and one that sources only `/opt/ros` fails to find your packages.
- **Order the layers by change frequency.** `rosdep install` in its own layer before `COPY src/` means a code change rebuilds in seconds rather than reinstalling dependencies.
- **Multi-stage keeps the runtime small**, but copying only `install/` drops the build tools, so anything that compiles at runtime (some Python extensions) needs them in the final stage.

------------------------------------------------------------------------

## Networking Is the Real Problem

A container is a network namespace, and **DDS discovery has to cross it**. This is the thing that makes ROS 2 in Docker harder than most applications, and it is worth deciding deliberately rather than discovering.

``` bash
# the pragmatic answer for a robot: share the host's network entirely
docker run --network host --ipc host --pid host my_robot_image
```

| Option | Effect |
|----|----|
| `--network host` | no namespace; DDS behaves exactly as on the host. Simplest, least isolated |
| `--ipc host` | shared memory works, which Fast DDS and Cyclone both use for local transport |
| bridge (the default) | the container has its own subnet; discovery to the host or other machines needs work |

**Without `--ipc host`, shared-memory transport fails and falls back to UDP loopback**, usually silently and slower. With bridge networking, a container advertises an address on `172.17.x.x` that nothing outside can route to, which is the same class of failure as the multi-homed interface problem in [../07_Middleware_DDS/00_Discovery_and_RMW.ipynb](../07_Middleware_DDS/discovery_and_rmw.llms.md) - and note that the Docker bridges a container creates are *also* what confuses DDS on the **host**.

If isolation matters and host networking is not acceptable, the workable combinations are a discovery server with a fixed address, explicit `ROS_STATIC_PEERS`, or the Zenoh RMW whose router model crosses a namespace boundary cleanly. See [../07_Middleware_DDS/01_Multi_Machine_and_Zenoh.ipynb](../07_Middleware_DDS/multi_machine_and_zenoh.llms.md).

**Containers are not always the right answer for a robot.** The trade is a network boundary plus device passthrough (`--device /dev/ttyUSB0`, `--device /dev/video0`, and `--gpus all` for CUDA) against reproducibility. Two projects on this site’s own hardware went the other way deliberately: [piros2](https://github.com/bthek1/piros2) and [ros2_pi](https://github.com/bthek1/ros2_pi) run ROS 2 natively from apt with Ansible providing reproducibility, and this superproject removed its own Docker path in favour of a venv plus systemd. Configuration management gives much of what containers give without putting a namespace between the nodes and the hardware. Containers earn their place for **fleet deployment** - shipping a versioned, identical image to many robots - more than for a single robot on a bench.

------------------------------------------------------------------------

## Dev Containers

A dev container keeps the toolchain out of your machine while letting the editor drive it.

``` json
// .devcontainer/devcontainer.json
{
  "name": "ROS 2 Jazzy",
  "image": "osrf/ros:jazzy-desktop",
  "runArgs": ["--network=host", "--ipc=host",
              "--volume=/tmp/.X11-unix:/tmp/.X11-unix",
              "--env=DISPLAY=${localEnv:DISPLAY}"],
  "workspaceMount": "source=${localWorkspaceFolder},target=/ws/src/pkg,type=bind",
  "workspaceFolder": "/ws/src/pkg",
  "customizations": {
    "vscode": {
      "extensions": ["ms-python.python", "ms-iot.vscode-ros", "ms-vscode.cpptools"]
    }
  },
  "postCreateCommand": "rosdep update && rosdep install --from-paths /ws/src --ignore-src -r -y"
}
```

Two practical notes: **GUI applications need the X socket and `DISPLAY`** mounted as above (Wayland needs the Wayland socket instead), and RViz additionally needs GPU access to render usefully. And **the workspace mount should be `src/pkg`, not the workspace root**, so `build/` and `install/` stay inside the container rather than littering the host with artefacts built for a different environment.

------------------------------------------------------------------------

## Cross-Compilation for arm64

A Raspberry Pi or Jetson is arm64; a workstation is usually x86_64. Building on the target is simplest and can be very slow, so there are three options.

**1. Build on the target.** No cross-compilation, no surprises, slow. For a Pi 5 and a handful of packages this is often the right answer, and it is what the projects cited above do.

**2. Emulated build with buildx.** One command, correct results, roughly five to ten times slower than native:

``` bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
docker buildx create --use --name robot
docker buildx build --platform linux/arm64 -t me/robot:arm64 --load .
```

**3. A real cross-toolchain**, via `ros_cross_compile` or colcon mixins. Fastest, and the most work to set up; worth it for a large workspace built often.

``` bash
pip install ros_cross_compile
ros_cross_compile ./ws --arch aarch64 --os ubuntu --rosdistro jazzy
```

What goes wrong:

- **Anything with a binary wheel or a vendor SDK.** A Python dependency with no arm64 wheel compiles from source inside the emulator, and a closed-source x86 SDK simply cannot be used. Check the dependency list before choosing an approach.
- **CUDA on Jetson is not generic arm64.** It needs the L4T base images and a matching JetPack version; a plain `ubuntu:24.04` arm64 image will not do.
- **`-march=native` is poison** in a cross or emulated build: it optimises for the build machine.
- **Test on the target.** An emulated build that passes its tests under QEMU can still fail on real hardware over timing and floating-point differences, and QEMU’s timing is nothing like a Pi’s.

A final note on where the time actually goes: for most small robot workspaces, the honest comparison is a 15-minute native build on the Pi against an hour of setting up cross-compilation. Reach for option 3 when the build is in a CI loop, not before.

------------------------------------------------------------------------

Back to top
