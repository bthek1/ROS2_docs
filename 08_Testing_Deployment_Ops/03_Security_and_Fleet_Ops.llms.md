# Security and Fleet Ops

SROS2: what DDS security actually gives you and what it costs, then the tools for watching and driving robots remotely - Foxglove, rosbridge and the Zenoh bridge.

Author

Benedict Thekkel

## The Default Is No Security At All

A ROS 2 graph is **unauthenticated and unencrypted by default**. Any process that can reach the network can list every topic, subscribe to the camera, and publish to `/cmd_vel`. There is no password, and `ROS_DOMAIN_ID` is not a security boundary - it is a port number.

That is a reasonable default for a bench and an unreasonable one for a robot on a corporate network or anything reachable from the internet. Two ways to address it, and they are not equivalent:

- **A perimeter**: a VPN or an isolated VLAN. Easy, and it makes the graph private to everything inside the perimeter. It does not authenticate nodes to each other, so one compromised machine has full control.
- **SROS2**: authentication, access control and encryption inside DDS itself, so each node proves who it is and is permitted only what its policy allows.

Most robots should do the first. The second is for when a node’s privileges actually need limiting, or a standard requires it.

------------------------------------------------------------------------

## SROS2

SROS2 wraps the DDS Security specification: a certificate authority, a keystore of per-node identities, and XML permissions saying which topics each node may read and write.

``` bash
sudo apt install ros-jazzy-sros2 ros-jazzy-rmw-fastrtps-cpp

# a keystore with its own CA
ros2 security create_keystore demo_keystore
ros2 security create_enclave demo_keystore /talker
ros2 security create_enclave demo_keystore /listener

export ROS_SECURITY_KEYSTORE=$PWD/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce        # Permissive logs violations; Enforce refuses
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker
```

Permissions are generated from an observed graph, which is the only practical way to write them:

``` bash
# watch a working system, then turn what it did into a policy
ros2 security generate_policy policy.xml
ros2 security create_permission demo_keystore /talker policy.xml
```

What it gives you, and the cost of each:

| Gives | Costs |
|----|----|
| authentication: a node proves its identity with a certificate | a keystore to distribute, and certificates that expire |
| access control: per-topic read/write permissions | a policy file that must be regenerated whenever the graph changes |
| encryption of traffic | CPU and latency, which matters on a Pi-class machine |

The practical difficulties, in the order they are met:

- **`Enforce` breaks everything until the policy is complete**, and the failure mode is a node that discovers nothing, which is indistinguishable from the discovery problems in [../07_Middleware_DDS/00_Discovery_and_RMW.ipynb](../07_Middleware_DDS/discovery_and_rmw.llms.md). Start in `Permissive`, collect the violations, then switch.
- **Policies go stale.** Adding a topic means regenerating and redistributing permissions. This is the reason most projects abandon SROS2: it is maintenance on every change.
- **Key distribution is the real problem.** Every robot needs its keys, the CA key must be protected, and certificate expiry becomes a fleet-wide outage if unmanaged.
- **RMW support varies.** Fast DDS is the best-supported path; check before committing.
- **Encryption costs CPU**, and on a four-core robot already running Nav2 and perception that is a real budget item.

Honest summary: **SROS2 is the correct answer to a requirement, not a default to adopt.** If the requirement is “nobody outside should reach the robot”, a VPN is less work and fails less confusingly. If it is “this node must not be able to command motion”, SROS2 is the only thing that provides it.

------------------------------------------------------------------------

## Remote Visualisation

Running RViz against a robot across a network is usually a bandwidth problem rather than a tooling one: RViz subscribes to everything it displays, so every display is another stream across the link. The arithmetic is in [../07_Middleware_DDS/01_Multi_Machine_and_Zenoh.ipynb](../07_Middleware_DDS/multi_machine_and_zenoh.llms.md).

**Foxglove** is the usual answer: a web or desktop visualiser that connects over a WebSocket, so the robot serves one connection rather than N DDS subscriptions.

``` bash
sudo apt install ros-jazzy-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

It also opens MCAP recordings, which makes the same tool work live and after the fact. Recording to MCAP rather than sqlite3 is worth doing for that reason alone; see [../02_Build_and_Tooling/02_CLI_and_Introspection.ipynb](../02_Build_and_Tooling/cli_and_introspection.llms.md).

**rosbridge** is the older, more general protocol: JSON over WebSocket, which any language can speak.

``` bash
sudo apt install ros-jazzy-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

JSON makes it easy to integrate and unsuitable for images or point clouds; it is for dashboards, buttons and telemetry, not for sensor streams.

``` javascript
// roslibjs
const ros = new ROSLIB.Ros({url: 'ws://robot:9090'});
new ROSLIB.Topic({ros, name: '/cmd_vel', messageType: 'geometry_msgs/msg/Twist'})
  .publish(new ROSLIB.Message({linear: {x: 0.2}}));
```

**Both are unauthenticated by default**, and a rosbridge server on a routable address is remote control of the robot by anyone who finds it. Bind to localhost behind an authenticating reverse proxy, or keep it inside a VPN.

------------------------------------------------------------------------

## Fleet Operations

Beyond one robot, the problems change shape: software versions drift, logs are on the robots, and diagnosing a fault means getting data off a machine that may be on a bad link.

What is worth having, roughly in order of value:

- **Configuration management, not manual setup.** Ansible or similar, so a robot’s state is declared and reproducible. The alternative is a fleet where every robot is subtly different and nobody knows how. This repository’s own `infra/ansible` roles are that pattern, and the projects cited in [../07_Middleware_DDS/01_Multi_Machine_and_Zenoh.ipynb](../07_Middleware_DDS/multi_machine_and_zenoh.llms.md) use it to keep two machines’ ROS environments in agreement - which, as that notebook says, is the single largest source of wasted time when done by hand.
- **Versioned deployment.** A container image or a versioned apt repository, so “which software is on robot 7” has an answer. See [01_Containers_and_Cross_Compilation.ipynb](../08_Testing_Deployment_Ops/containers_and_cross_compilation.llms.md).
- **Diagnostics aggregated centrally**, so degradation is visible before failure; see [02_Service_Management_and_Diagnostics.ipynb](../08_Testing_Deployment_Ops/service_management_and_diagnostics.llms.md).
- **Log and bag shipping, with a budget.** Bags are large, and continuous upload over a robot’s link is usually impossible; record locally and upload on demand or on a trigger.
- **A Zenoh bridge for the wide-area hop.** `zenoh-bridge-ros2dds` lets each site keep DDS while the link between sites is Zenoh, which behaves far better over a WAN and does not require moving the whole fleet to a new RMW.
- **Remote access that survives a bad link.** A VPN plus `ssh` with short keepalives; one of the cited projects found that adding `-o BatchMode=yes -o ConnectTimeout=5` to every scripted `ssh` turned a dead link from a two-minute hang into a five-second failure, and that server-side `ClientAlive` settings are what stop a dropped session orphaning a process that holds the camera.

The recurring lesson from that project’s logs is worth stating plainly: **a robot that is unreachable is usually not a robot that has crashed.** Twice, a machine stayed up, kept writing journals and kept running its camera locally while being completely unreachable over Wi-Fi. The operational consequence is that the fleet tooling must distinguish “the robot is down” from “the link is down”, and that the previous boot’s journal (`journalctl -b -1`) is where the truth is after a recovery.

------------------------------------------------------------------------

Back to top
