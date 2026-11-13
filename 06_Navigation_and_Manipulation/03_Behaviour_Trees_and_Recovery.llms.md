# Behaviour Trees and Recovery

Why Nav2 coordinates with a behaviour tree rather than a state machine, the node types and their exact semantics implemented and ticked here, Nav2’s default tree, and the recovery behaviours.

Author

Benedict Thekkel

## Why a Behaviour Tree

Navigation is a sequence of fallible steps: plan, follow, and when either fails, do something about it. Written as a state machine, every new failure mode adds transitions from every state, and the diagram becomes unmaintainable at about a dozen states.

A behaviour tree inverts that. Each node returns one of three statuses, and **composition is the only control flow**:

| Status    | Means                           |
|-----------|---------------------------------|
| `SUCCESS` | this node is done and succeeded |
| `FAILURE` | this node is done and failed    |
| `RUNNING` | still working, tick me again    |

The tree is **ticked** repeatedly, typically at 10 to 100 Hz. Each tick walks from the root, resuming where a `RUNNING` node left off. Adding a recovery means adding a branch, not rewiring transitions, and that is the whole reason Nav2 uses one.

Nav2 uses BehaviorTree.CPP, with trees written in XML.

------------------------------------------------------------------------

## The Node Types, Implemented

The semantics are simple enough to write in one cell, which is the quickest way to get them exactly right rather than approximately right.

------------------------------------------------------------------------

``` python
from enum import Enum

class Status(Enum):
    SUCCESS = "SUCCESS"; FAILURE = "FAILURE"; RUNNING = "RUNNING"
    def __str__(self): return self.value

class Node:
    def __init__(self, name): self.name = name
    def tick(self): raise NotImplementedError
    def halt(self): pass                     # called when a node is abandoned mid-flight

class Action(Node):
    "A leaf returning RUNNING for `ticks` ticks, then `result`."
    def __init__(self, name, result=Status.SUCCESS, ticks=0):
        super().__init__(name)
        self.result, self.ticks = result, ticks
        self.n = 0          # ticks in the current attempt; halt() resets this
        self.total = 0      # every tick ever, which halt() does not reset
    def tick(self):
        self.n += 1; self.total += 1
        return Status.RUNNING if self.n <= self.ticks else self.result
    def halt(self): self.n = 0

class Sequence(Node):
    "SUCCESS only if every child succeeds. Stops at the first FAILURE or RUNNING."
    def __init__(self, name, children):
        super().__init__(name); self.children, self.i = children, 0
    def tick(self):
        while self.i < len(self.children):
            s = self.children[self.i].tick()
            if s is Status.RUNNING: return s          # resume here on the next tick
            if s is Status.FAILURE: self.halt(); return s
            self.i += 1
        self.halt(); return Status.SUCCESS
    def halt(self):
        self.i = 0
        for c in self.children: c.halt()

class Fallback(Node):
    "SUCCESS at the first child that succeeds. FAILURE only if all of them fail."
    def __init__(self, name, children):
        super().__init__(name); self.children, self.i = children, 0
    def tick(self):
        while self.i < len(self.children):
            s = self.children[self.i].tick()
            if s is Status.RUNNING: return s
            if s is Status.SUCCESS: self.halt(); return s
            self.i += 1
        self.halt(); return Status.FAILURE
    def halt(self):
        self.i = 0
        for c in self.children: c.halt()

class Inverter(Node):
    "A decorator: flips SUCCESS and FAILURE, passes RUNNING through."
    def __init__(self, name, child): super().__init__(name); self.child = child
    def tick(self):
        s = self.child.tick()
        return {Status.SUCCESS: Status.FAILURE, Status.FAILURE: Status.SUCCESS}.get(s, s)
    def halt(self): self.child.halt()

class RetryUntilSuccessful(Node):
    "A decorator: re-tick a failing child up to num_attempts times."
    def __init__(self, name, child, num_attempts):
        super().__init__(name); self.child, self.max, self.tries = child, num_attempts, 0
    def tick(self):
        while self.tries < self.max:
            s = self.child.tick()
            if s is Status.RUNNING: return s
            if s is Status.SUCCESS: self.tries = 0; return s
            self.tries += 1
            self.child.halt()                 # reset the child before retrying
        self.tries = 0
        return Status.FAILURE
    def halt(self): self.tries = 0; self.child.halt()

print("control nodes: Sequence, Fallback; decorators: Inverter, RetryUntilSuccessful")
```

    control nodes: Sequence, Fallback; decorators: Inverter, RetryUntilSuccessful

``` python
# Sequence is AND, Fallback is OR.
assert Sequence("s", [Action("a"), Action("b")]).tick() is Status.SUCCESS
assert Sequence("s", [Action("a"), Action("b", Status.FAILURE), Action("c")]).tick() is Status.FAILURE
assert Fallback("f", [Action("a", Status.FAILURE), Action("b")]).tick() is Status.SUCCESS
assert Fallback("f", [Action("a", Status.FAILURE), Action("b", Status.FAILURE)]).tick() is Status.FAILURE
assert Inverter("i", Action("a")).tick() is Status.FAILURE
assert Inverter("i", Action("a", Status.FAILURE)).tick() is Status.SUCCESS
print("Sequence = AND, Fallback = OR, Inverter flips. All as expected.")

# Short circuit: nothing after a failure is ticked. This is why ordering matters.
never = Action("never_runs")
seq = Sequence("s", [Action("fails", Status.FAILURE), never])
assert seq.tick() is Status.FAILURE and never.total == 0
print(f"the node after a failure was ticked {never.total} times - the tree short circuits")

# Retry ticks the failing child repeatedly, then gives up.
flaky = Action("flaky", Status.FAILURE)
assert RetryUntilSuccessful("r", flaky, 3).tick() is Status.FAILURE
assert flaky.total == 3
print(f"RetryUntilSuccessful ticked the failing child {flaky.total} times before giving up")
```

    Sequence = AND, Fallback = OR, Inverter flips. All as expected.
    the node after a failure was ticked 0 times - the tree short circuits
    RetryUntilSuccessful ticked the failing child 3 times before giving up

``` python
# RUNNING is the part that makes a tree a controller rather than a function.
slow = Action("FollowPath", Status.SUCCESS, ticks=2)
seq = Sequence("nav", [Action("ComputePath"), slow, Action("UpdateGoal")])

trace = [seq.tick() for _ in range(4)]
print("four consecutive ticks:", [str(s) for s in trace])

# RUNNING on ticks 1-2 while FollowPath works, SUCCESS on tick 3 - then RUNNING again on
# tick 4, because a tree that completes resets and the next tick starts the behaviour over.
# That is why a BT is ticked continuously rather than called once.
assert trace == [Status.RUNNING, Status.RUNNING, Status.SUCCESS, Status.RUNNING]
print("note tick 4: finishing resets the tree, so ticking again restarts the behaviour")
```

    four consecutive ticks: ['RUNNING', 'RUNNING', 'SUCCESS', 'RUNNING']
    note tick 4: finishing resets the tree, so ticking again restarts the behaviour

## Nav2’s Tree, in Miniature

Nav2’s default `navigate_to_pose_w_replanning_and_recovery.xml` is, stripped to its shape, a `Fallback` whose first child is the navigation attempt and whose second is recovery:

``` xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
      <Sequence name="RecoveryActions">
        <ClearEntireCostmap name="ClearGlobalCostmap-Context"/>
        <Spin spin_dist="1.57"/>
        <Wait wait_duration="5.0"/>
        <BackUp backup_dist="0.30" backup_speed="0.05"/>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

Three Nav2-specific nodes carry most of the behaviour:

- **`RecoveryNode`** is a `Fallback` with a retry count: try the first child, on failure run the second, then try the first again, up to `number_of_retries`.
- **`PipelineSequence`** re-ticks earlier children while later ones run, which is what makes replanning concurrent with following rather than sequential.
- **`RateController`** throttles its child, so the planner runs at 1 Hz while the controller runs at 20 Hz.

The blackboard (`{goal}`, `{path}`) is the shared key-value store nodes pass data through.

------------------------------------------------------------------------

``` python
def nav_tree(compute=Status.SUCCESS, follow=Status.SUCCESS, recovery=Status.SUCCESS):
    "Nav2's shape: attempt, else recover."
    return Fallback("NavigateRecovery", [
        Sequence("NavigateWithReplanning", [
            Action("ComputePathToPose", compute),
            Action("FollowPath", follow)]),
        Sequence("RecoveryActions", [
            Action("ClearCostmap"), Action("Spin"), Action("BackUp"), Action("Wait", recovery)])])

print("happy path           ->", nav_tree().tick())
print("FollowPath fails     ->", nav_tree(follow=Status.FAILURE).tick(), "(recovery ran)")
print("planner fails        ->", nav_tree(compute=Status.FAILURE).tick(), "(recovery ran)")
print("both branches fail   ->", nav_tree(follow=Status.FAILURE, recovery=Status.FAILURE).tick())

assert nav_tree().tick() is Status.SUCCESS
assert nav_tree(follow=Status.FAILURE).tick() is Status.SUCCESS
assert nav_tree(follow=Status.FAILURE, recovery=Status.FAILURE).tick() is Status.FAILURE
print("\nthe tree reports success of the BEHAVIOUR, not of the first attempt -")
print("which is exactly why a navigation goal can succeed after a recovery nobody asked for")
```

    happy path           -> SUCCESS
    FollowPath fails     -> SUCCESS (recovery ran)
    planner fails        -> SUCCESS (recovery ran)
    both branches fail   -> FAILURE

    the tree reports success of the BEHAVIOUR, not of the first attempt -
    which is exactly why a navigation goal can succeed after a recovery nobody asked for

## Recovery Behaviours

`behavior_server` hosts the recoveries the tree calls. They are actions, so each can be invoked directly, which is the fastest way to test one:

| Behaviour | Does |
|----|----|
| `Spin` | rotate in place by `spin_dist`, to re-observe surroundings |
| `BackUp` | reverse a short distance |
| `DriveOnHeading` | move along a fixed heading, ignoring the planner |
| `Wait` | do nothing for a duration; lets a dynamic obstacle move |
| `ClearEntireCostmap` | discard accumulated obstacle data |
| `AssistedTeleop` | operator drives, collision checking stays on |

``` bash
ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: 1.57}"
ros2 action send_goal /backup nav2_msgs/action/BackUp "{target: {x: -0.3}, speed: 0.05}"
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
```

Two cautions that matter on a real robot. **`BackUp` and `DriveOnHeading` move without a plan**, and most robots have no rear sensor, so reversing is genuinely blind; keep `backup_dist` small. And **`ClearEntireCostmap` discards real obstacles as well as phantom ones**, so a tree that clears on every failure can clear an obstacle and then drive into it. Clearing is a remedy for a sensor or frame problem, and the better fix is the sensor or frame. See [02_Costmaps_Planners_and_Controllers.ipynb](../06_Navigation_and_Manipulation/costmaps_planners_and_controllers.llms.md).

------------------------------------------------------------------------

## Waypoint Following, and Customising

`waypoint_follower` drives a list of poses, with plugins that run at each arrival:

``` bash
ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints "{poses: [...]}"
```

``` yaml
waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false          # skip a waypoint that fails rather than abandoning the route
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 2000
```

`stop_on_failure: false` is usually what an inspection route wants: one unreachable waypoint should not end the mission.

Writing a custom tree, in order of how much work it is:

- **Swap the XML.** Set `default_nav_to_pose_bt_xml` to your own file. This covers most needs: changing the recovery order, adding a `Wait`, throttling differently.
- **Compose existing nodes.** The shipped condition nodes (`IsBatteryLow`, `GoalUpdated`, `IsStuck`, `TransformAvailable`) plus decorators cover a lot without any C++.
- **Write a plugin.** A C++ `BT::ActionNodeBase` or `BT::ConditionNode` registered with the factory, for a genuinely new action such as calling your own service.

``` bash
ros2 run groot2 Groot2      # visualise and edit trees, and watch one tick live
ros2 topic echo /behavior_tree_log
```

`/behavior_tree_log` is the debugging tool to know about: it reports every node’s status transition per tick, which turns “navigation failed” into the specific node that returned `FAILURE` first.

------------------------------------------------------------------------

Back to top
