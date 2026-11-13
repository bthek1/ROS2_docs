# Discovery and RMW

How nodes find each other without a master: domain IDs and the ports they imply, the discovery range settings, the RMW implementations and the rule that they must match, interface pinning, and the discovery server.

Author

Benedict Thekkel

## There Is No roscore

Nodes find each other by multicast DDS discovery, which means **any ROS 2 node on the same subnet joins the same graph by default**. Two people debugging robots on one office network will see each other’s topics.

That is the whole design: no central process to start, nothing to be a single point of failure, and a node can be restarted without the graph noticing. The cost is that the graph’s membership is a property of the network rather than of a configuration file, so when it is wrong the answer is in network configuration rather than in ROS.

Discovery happens in two stages. Participants announce themselves on a multicast address (SPDP), then exchange their publisher and subscriber lists with each peer directly (SEDP). **Discovery traffic grows with the square of the participant count**, which is why a graph of 200 nodes behaves differently from one of 20, and why composition into fewer processes ([../01_Core_Concepts/04_Executors_Lifecycle_and_Composition.ipynb](../01_Core_Concepts/executors_lifecycle_and_composition.llms.md)) helps at scale.

------------------------------------------------------------------------

## ROS_DOMAIN_ID

`ROS_DOMAIN_ID` partitions the network. Set it to the same integer on every machine that should talk, and a different one elsewhere:

``` bash
export ROS_DOMAIN_ID=42
```

The default is `0`, which is also everyone else’s default, so on a shared LAN stray nodes from other projects appear in `ros2 topic list`. Choosing a non-default domain per project is the cheapest isolation available.

**The domain is not a label, it is arithmetic on port numbers.** The RTPS specification derives ports from the domain:

    discovery multicast port = 7400 + 250 * domain_id
    user     multicast port = 7400 + 250 * domain_id + 1
    discovery unicast  port = 7400 + 250 * domain_id + 10 + 2 * participant_id

So domain 0 listens around 7400 and domain 42 listens around **17900**. Two consequences:

- **A `tcpdump` filter must match the domain.** `portrange 7400-7500` captures domain 0 only; for domain 42 it is `portrange 17900-18000`. A filter copied from a tutorial explains a lot of “no packets are being sent” conclusions that were really no packets being captured.
- **Stay in 0-101 on Linux.** Higher domains push the derived ports into the ephemeral range, where they collide with ordinary applications. The protocol allows up to 232; the practical ceiling is
  101. 

------------------------------------------------------------------------

## Limiting Discovery Range

To stop discovery leaving the machine entirely:

``` bash
export ROS_LOCALHOST_ONLY=1                        # Humble and earlier
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST     # Jazzy and later
```

`ROS_LOCALHOST_ONLY` is **deprecated from Iron onward** and replaced by `ROS_AUTOMATIC_DISCOVERY_RANGE`, which takes four values:

| Value            | Meaning                                                |
|------------------|--------------------------------------------------------|
| `OFF`            | no automatic discovery at all; only `ROS_STATIC_PEERS` |
| `LOCALHOST`      | this machine only                                      |
| `SUBNET`         | the local subnet (the default)                         |
| `SYSTEM_DEFAULT` | whatever the RMW’s own configuration says              |

`OFF` combined with explicit `ROS_STATIC_PEERS` gives fully deterministic discovery, which is the right tool when you want to eliminate the network as a variable. See [01_Multi_Machine_and_Zenoh.ipynb](../07_Middleware_DDS/multi_machine_and_zenoh.llms.md).

Note that these settings are read by the RMW at participant creation, so they must be set **before** a node starts, and changing them in one shell does not affect nodes already running in another.

------------------------------------------------------------------------

## RMW Implementations

The DDS implementation is swappable (`rmw_fastrtps_cpp` is the default, `rmw_cyclonedds_cpp` the common alternative) via `RMW_IMPLEMENTATION`. **Every node in a graph must use implementations that interoperate, so change it everywhere or nowhere.**

``` bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 doctor --report | grep -A2 "RMW MIDDLEWARE"
```

| Implementation | Notes |
|----|----|
| `rmw_fastrtps_cpp` | the Jazzy default (eProsima Fast DDS). Has the discovery server; XML profiles for tuning |
| `rmw_cyclonedds_cpp` | Eclipse Cyclone DDS. Smaller, and its interface selection is easier to control, which matters on a multi-homed machine |
| `rmw_zenoh_cpp` | not DDS at all; router-based, designed for lossy links and WAN. See [01_Multi_Machine_and_Zenoh.ipynb](../07_Middleware_DDS/multi_machine_and_zenoh.llms.md) |
| `rmw_connextdds` | RTI Connext, commercial, certifiable |

A mixed graph is the failure to watch for: different vendors are interoperable in principle, through the RTPS wire protocol, and in practice mixed graphs produce nodes visible to some peers and not others. Standardise per project.

One sharp edge worth knowing: **if `RMW_IMPLEMENTATION` names a package that is not installed, every process fails immediately**. In Python that surfaces as an `rclpy` process exiting with status 1 and very little explanation, which under a test runner looks like the test framework’s fault. This has been hit in practice on this site’s own hardware, after a distribution upgrade replaced the installed RMW packages; see the [piros2](https://github.com/bthek1/piros2) project notes.

------------------------------------------------------------------------

## Interface Pinning on a Multi-Homed Machine

This is the problem most likely to cost an evening, and it appears on any development machine that runs Docker, a VPN, or both.

A typical dev box has several interfaces that are not the LAN:

    docker0          172.17.0.1/16
    br-808fe3a7a7d5  172.18.0.1/16
    tailscale0       (no IPv4)
    wg0              10.8.0.3/32        # WireGuard
    enp6s18          192.168.2.109/24   # the only one that reaches the robot

By default the DDS implementation enumerates **every** interface and picks one, often a bridge or the VPN rather than the LAN. The node then advertises itself at an address the robot cannot route to. The symptoms are distinctive and none of them says “wrong interface”:

- discovery is slow or intermittent
- `ros2 topic list` shows the topic but `ros2 topic echo` sits silent
- it works in one direction only

**VPN interfaces make this worse than Docker alone**, because they are routable-looking but lead somewhere the robot is not. Pin the interface explicitly rather than relying on enumeration order.

Cyclone DDS, which is the easier one to control:

``` xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="enp6s18" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

``` bash
export CYCLONEDDS_URI=file:///home/me/.config/cyclonedds/cyclonedds.xml
```

Fast DDS uses its own XML profile and `FASTRTPS_DEFAULT_PROFILES_FILE`, with a `whitelist` of addresses inside a UDP transport descriptor.

Two hard-won notes on where that file lives and what breaks it:

- **Pin by interface name and know that names change.** A Proxmox guest’s interface renamed itself from `ens18` to `enp6s18` across an upgrade (same IP, new predictable-name scheme) and broke Cyclone’s pin, because the configured name no longer existed. The failure is silent: DDS falls back to choosing for itself.
- **Keep the config outside any directory that is synced or deleted.** A config templated into a workspace that is synced with `rsync --delete` vanishes on the next sync, after which DDS binds whatever interface it likes with no error anywhere.

Generating the file per host from one template, with the interface as the only variable, is what keeps two machines’ configs from drifting in structure while differing in the one field that should differ.

------------------------------------------------------------------------

## The Discovery Server

For larger graphs, Fast DDS offers a **discovery server**: a known process that participants register with, replacing multicast discovery with unicast to a rendezvous point.

``` bash
# the server
fastdds discovery --server-id 0 --ip-address 192.168.2.109 --port 11811

# every client
export ROS_DISCOVERY_SERVER=192.168.2.109:11811
ros2 daemon stop && ros2 daemon start
```

Worth it when: multicast is blocked or rate-limited, the graph is large enough that n-squared discovery traffic is measurable, or discovery must cross a subnet boundary.

The cost is a process that has to be running before anything else, and which becomes the thing that was forgotten when nothing can see anything. It can be made redundant (several servers, clients listing all of them), which is worth doing before depending on it.

------------------------------------------------------------------------

## Diagnosing

``` bash
# what can this machine actually see
ros2 node list
ros2 topic list

# the full environment and middleware report
ros2 doctor --report

# is anything publishing, and how fast
ros2 topic hz /scan
ros2 topic info /scan --verbose        # endpoints and their QoS

# are DDS packets crossing the wire at all (domain 42 -> 17900)
sudo tcpdump -i enp6s18 -n 'udp and portrange 17900-18000'

# what the environment is on a remote machine
ssh robot -t 'bash -lc "env | grep -E \"^ROS_|^RMW_|^CYCLONEDDS\""'
```

**Restart the daemon after changing any `ROS_*` or DDS environment variable:**

``` bash
ros2 daemon stop && ros2 daemon start
```

The `ros2` CLI talks to a background daemon that caches discovery state, and it will keep reporting the old view otherwise. This alone explains a large share of “I fixed it and nothing changed”.

Which layer owns the symptom:

| Symptom | Layer |
|----|----|
| node not in `ros2 node list` on either machine | the node, or the RMW is missing |
| node visible locally, not remotely | discovery: domain, discovery range, interface, or multicast |
| visible remotely, `echo` silent | either a wrong advertised address, or a QoS mismatch |
| both endpoints listed with profiles, no data | QoS: see [../01_Core_Concepts/03_QoS_Profiles.ipynb](../01_Core_Concepts/qos_profiles.llms.md) |
| data arrives, callback does not run | the executor: see [../01_Core_Concepts/04_Executors_Lifecycle_and_Composition.ipynb](../01_Core_Concepts/executors_lifecycle_and_composition.llms.md) |
| works after a long delay | discovery succeeded eventually; suspect multicast being rate-limited |

The distinction in the middle two rows is the one worth internalising, because it separates a network problem from a QoS problem, and `ros2 topic info --verbose` is what tells them apart.

------------------------------------------------------------------------

Back to top
