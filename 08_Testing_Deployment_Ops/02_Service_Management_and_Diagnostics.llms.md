# Service Management and Diagnostics

Making a robot start itself: systemd units for a ROS 2 launch, robot_upstart, and the diagnostics system that tells an operator which sensor is degraded rather than merely absent.

Author

Benedict Thekkel

## systemd, by Hand

A robot has to come up without anyone logging in. systemd is the mechanism, and a ROS 2 launch needs three things a normal service does not: the distribution and workspace sourced, the right user, and a restart policy.

The wrapper script exists because `ExecStart` cannot source anything:

``` bash
#!/bin/bash
# /usr/local/bin/robot-start.sh
set -e
source /opt/ros/jazzy/setup.bash
source /home/robot/ws/install/setup.bash
exec ros2 launch my_bringup robot.launch.py
```

``` ini
# /etc/systemd/system/robot.service
[Unit]
Description=Robot bringup
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/usr/local/bin/robot-start.sh
User=robot
WorkingDirectory=/home/robot/ws
EnvironmentFile=-/home/robot/ws/.env
Restart=always
RestartSec=5
TimeoutStopSec=20
KillMode=mixed

[Install]
WantedBy=multi-user.target
```

``` bash
sudo systemctl daemon-reload
sudo systemctl enable --now robot
systemctl status robot
journalctl -u robot -f
journalctl -u robot -b -1        # the previous boot, where a crash's real cause usually is
```

This repository runs a working example of the same pattern, which is worth reading as a reference rather than a tutorial: [`infra/ansible/roles/jupyter_service/templates/jupyter-lab.service.j2`](https://github.com/bthek1/Knowledge/blob/main/infra/ansible/roles/jupyter_service/templates/jupyter-lab.service.j2) is a templated unit with `After=network.target`, an explicit `User=` and `WorkingDirectory=`, `Restart=always`, `WantedBy=multi-user.target`, and `EnvironmentFile=-` where the leading dash makes the file optional so the unit still starts when it is absent. The role is applied by Ansible on every machine, which is the point: **the unit is generated from configuration, not hand-edited on the robot.**

Details that matter on a robot specifically:

- **`network-online.target`, not `network.target`.** The latter means “networking is being configured”, not “an address exists”. DDS binding to an interface that has no address yet is a startup race that appears as a robot that works after a manual restart. A Wi-Fi robot may need to wait longer still.
- **`KillMode=mixed` plus a generous `TimeoutStopSec`.** A launch file starts many processes; the default kill mode can leave orphans holding `/dev/ttyUSB0` or `/dev/video0`, and the next start then fails on a busy device.
- **`Restart=always` is not always right.** A node that crashes on malformed configuration will restart-loop forever. `StartLimitIntervalSec` and `StartLimitBurst` bound it, and a brake-engaging `ExecStopPost` is worth considering for anything that moves.
- **The environment is not a login shell.** `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI` live in `~/.bashrc` for interactive use and are **absent** under systemd. Put them in the unit or an `EnvironmentFile`, or the robot comes up on domain 0 with the wrong middleware. This is the single most common systemd-plus-ROS bug.

------------------------------------------------------------------------

## robot_upstart

`robot_upstart` generates the unit and the wrapper for you, which removes the boilerplate above.

``` bash
sudo apt install ros-jazzy-robot-upstart
ros2 run robot_upstart install my_bringup/launch/robot.launch.py \
  --job robot --rmw rmw_cyclonedds_cpp --symlink --user robot
sudo systemctl daemon-reload && sudo systemctl start robot
ros2 run robot_upstart uninstall robot
```

`--symlink` points the installed job at the workspace rather than copying, so editing the launch file takes effect without reinstalling. `--rmw` writes the middleware into the environment, which is exactly the variable most often missed.

It is a convenience wrapper, not a different mechanism: the result is a systemd unit, debugged with `journalctl` like any other. For a single robot it saves an hour; for a fleet, a templated unit under configuration management is better, because it is versioned and identical everywhere.

------------------------------------------------------------------------

## Diagnostics

An operator needs to know that the lidar is returning sparse scans, not merely that a topic exists. `diagnostic_updater` is how a node reports its own health, aggregated into one tree.

``` python
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus

class LidarDriver(Node):
    def __init__(self):
        super().__init__("lidar")
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("hokuyo-0042")
        self.updater.add("connection", self.check_connection)

        # a frequency watchdog: warn if the rate leaves the expected band
        self.freq = diagnostic_updater.HeaderlessTopicDiagnostic(
            "scan", self.updater,
            diagnostic_updater.FrequencyStatusParam({"min": 9.0, "max": 11.0}, 0.1, 10))

    def check_connection(self, stat):
        if not self.device.is_open():
            stat.summary(DiagnosticStatus.ERROR, "device not open")
        elif self.error_count > 10:
            stat.summary(DiagnosticStatus.WARN, f"{self.error_count} read errors")
        else:
            stat.summary(DiagnosticStatus.OK, "connected")
        stat.add("port", self.port)
        stat.add("read errors", str(self.error_count))
        return stat

    def on_scan_published(self):
        self.freq.tick()            # call this on every publish
```

Everything lands on `/diagnostics`, and `diagnostic_aggregator` groups it into `/diagnostics_agg` for display:

``` yaml
analyzers:
  ros__parameters:
    sensors:
      type: diagnostic_aggregator/GenericAnalyzer
      path: Sensors
      contains: ["lidar", "camera", "imu"]
    motors:
      type: diagnostic_aggregator/GenericAnalyzer
      path: Motors
      contains: ["wheel", "controller_manager"]
```

``` bash
ros2 run rqt_robot_monitor rqt_robot_monitor
ros2 topic echo /diagnostics_agg
```

Why this is worth the effort, rather than logging:

- **A log is read after a failure; a diagnostic is read during one.** `rqt_robot_monitor` shows a tree where the degraded branch is yellow, which is information an operator can act on.
- **The three levels are a contract.** `OK`, `WARN` (degraded, still working) and `ERROR` (not working) let an operator or a supervisor distinguish “slower than expected” from “stopped”, and a behaviour tree can branch on it; see [../06_Navigation_and_Manipulation/03_Behaviour_Trees_and_Recovery.ipynb](../06_Navigation_and_Manipulation/behaviour_trees_and_recovery.llms.md).
- **Key-value pairs carry the detail** that makes a warning actionable: the port, the error count, the measured rate.
- **A frequency watchdog catches the failure that silence does not.** A sensor publishing at 2 Hz instead of 10 passes every “is the topic there” check and breaks everything downstream.

Use `WARN` honestly. A robot whose diagnostics are permanently yellow has no diagnostics, because nobody looks any more.

------------------------------------------------------------------------

## What to Check When a Robot Will Not Start

In order, because each rules out a layer:

``` bash
systemctl status robot                       # did the unit start at all
journalctl -u robot -b --no-pager | head -50  # the first error, not the last
journalctl -u robot -b -1                    # the previous boot, if it restart-looped
systemctl show robot -p Environment          # is ROS_DOMAIN_ID actually set
sudo -u robot env | grep -E '^ROS_|^RMW_'    # what the service user would see
ls -l /dev/ttyUSB0 /dev/video0               # permissions and presence
groups robot                                 # dialout, video
```

| Symptom | Cause |
|----|----|
| unit active, no topics | environment missing: wrong domain or RMW under systemd |
| works manually, fails at boot | started before the network had an address |
| device busy on restart | orphaned processes; `KillMode` and `TimeoutStopSec` |
| permission denied on a device | service user not in `dialout` or `video` |
| restart loop | configuration error; read the *first* error in the journal |
| topics appear then vanish | a node crashing and the unit restarting |
| no journal entries at all | `ExecStart` path wrong, or the wrapper is not executable |

The recurring theme is that **a service is not a login shell**: it has no `~/.bashrc`, a different environment, a different user and possibly no `HOME`. Most “it works when I run it myself” reports are that one fact.

------------------------------------------------------------------------

Back to top
