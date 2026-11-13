# Costmaps, Planners and Controllers

The costmap layers and the inflation model, the global planners and local controllers Nav2 ships, and a costmap, an A\* plan and a pure-pursuit controller built and plotted here to show what the parameters do.

Author

Benedict Thekkel

## The Costmap

A costmap is a grid where each cell carries a cost from 0 to 255. It is built from **layers**, each of which writes into the grid in order, and the layer list is the main thing to configure.

| Layer | Writes |
|----|----|
| `static_layer` | the map from `map_server`, loaded once |
| `obstacle_layer` | live 2D obstacles from a `LaserScan` or `PointCloud2` |
| `voxel_layer` | live 3D obstacles, with a ray-traced clearing model |
| `inflation_layer` | a cost gradient around everything the layers above marked |
| `range_layer` | sonar or infrared |
| `denoise_layer` | removes isolated speckle before inflation |

Nav2 keeps **two** costmaps, and they answer different questions:

- **global_costmap**: the whole map, updated slowly (1 Hz), used for planning a route.
- **local_costmap**: a rolling window a few metres across, updated fast (5 to 20 Hz), used for avoiding what is in front of the robot now.

``` yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom          # odom, not map: the local window must not jump
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          obstacle_max_range: 2.5
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

**`global_frame: odom` on the local costmap is deliberate.** The local window must be continuous, and `map` jumps when the localiser corrects; a local costmap in `map` shifts its obstacles sideways every correction. That is the REP-105 contract between the two frames, described in [../03_Spatial_and_Temporal/02_Conventions_and_Time.ipynb](../03_Spatial_and_Temporal/conventions_and_time.llms.md) and tracked through the transform tree in [../03_Spatial_and_Temporal/00_tf2.ipynb](../03_Spatial_and_Temporal/tf2.llms.md); which node publishes which edge is in [00_SLAM_and_Localization.ipynb](../06_Navigation_and_Manipulation/slam_and_localization.llms.md).

Two more traps in that block. **`clearing: true` needs the sensor’s own frame to be right**, because clearing ray-traces from the sensor origin; a wrong `frame_id` leaves phantom obstacles that never clear. And **`obstacle_max_range` must be below the sensor’s range**, or out-of-range returns mark obstacles at the maximum distance.

------------------------------------------------------------------------

## Inflation, and Why It Decides Whether a Path Exists

The inflation layer is where most “Nav2 will not plan through my doorway” problems live. It has three regions:

    distance from obstacle      cost
    0 (the obstacle itself)      254   LETHAL_OBSTACLE
    0 < d <= inscribed_radius    253   INSCRIBED_INFLATED_OBSTACLE  (robot centre cannot go here)
    inscribed < d <= inflation   252 * exp(-cost_scaling_factor * (d - inscribed_radius))
    d > inflation_radius           0   free

The planner refuses cells at or above 253, so **`inscribed_radius` sets which gaps are passable** and `cost_scaling_factor` only shapes the preference for staying clear. Those two being confused is the usual cause of a robot that refuses a gap it physically fits, or one that scrapes door frames.

Let us build one and see.

------------------------------------------------------------------------

``` python
import numpy as np
from scipy.ndimage import distance_transform_edt

LETHAL, INSCRIBED, FREE = 254, 253, 0
RES = 0.05                       # m per cell, as in the YAML above
H, W = 60, 100                   # 3.0 m x 5.0 m

occ = np.zeros((H, W), dtype=np.uint8)
occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = 1     # room walls
occ[1:59, 40] = 1                                        # a dividing wall, full height
occ[22:38, 40] = 0                                       # with a 16-cell (80 cm) doorway
occ[15:20, 70:75] = 1                                    # a block in the far room
print(f"grid {H}x{W} at {RES} m = {H*RES} x {W*RES} m, {int(occ.sum())} occupied cells")

def inflate(occ, res, inscribed_radius=0.15, inflation_radius=0.45, cost_scaling_factor=3.0):
    """Nav2's inflation_layer: lethal on the obstacle, INSCRIBED out to the robot's
    inscribed radius, then an exponential decay out to inflation_radius."""
    dist = distance_transform_edt(occ == 0) * res        # metres to the nearest obstacle
    cost = np.zeros(occ.shape, dtype=np.uint8)
    cost[occ == 1] = LETHAL
    cost[(dist > 0) & (dist <= inscribed_radius)] = INSCRIBED
    decay = (dist > inscribed_radius) & (dist <= inflation_radius)
    cost[decay] = ((INSCRIBED - 1) * np.exp(-cost_scaling_factor *
                                            (dist[decay] - inscribed_radius))).astype(np.uint8)
    return cost, dist

cost, dist = inflate(occ, RES)
print(f"lethal {int((cost == LETHAL).sum())}, inscribed {int((cost == INSCRIBED).sum())}, "
      f"gradient {int(((cost > 0) & (cost < INSCRIBED)).sum())}, free {int((cost == 0).sum())}")
assert cost[occ == 1].min() == LETHAL
```

    grid 60x100 at 0.05 m = 3.0 x 5.0 m, 383 occupied cells
    lethal 383, inscribed 832, gradient 2644, free 2141

``` python
from pyecharts.globals import CurrentConfig

# assets.pyecharts.org serves an expired TLS certificate from most nodes (2026-11-11),
# so charts failed to load. jsDelivr mirrors the same assets, pinned to one commit.
CurrentConfig.ONLINE_HOST = "https://cdn.jsdelivr.net/gh/pyecharts/pyecharts-assets@f2328d558550046fc5449672a6850e848e141590/assets/v6/"

from pyecharts.charts import HeatMap
from pyecharts import options as opts

data = [[int(c), int(r), int(cost[r, c])] for r in range(H) for c in range(W) if cost[r, c] > 0]
(HeatMap(init_opts=opts.InitOpts(width="900px", height="560px"))
 .add_xaxis([str(c) for c in range(W)])
 .add_yaxis("cost", [str(r) for r in range(H)], data,
            label_opts=opts.LabelOpts(is_show=False))
 .set_global_opts(
     title_opts=opts.TitleOpts(title="Inflated costmap",
                               subtitle="254 lethal, 253 inscribed (planner refuses), then a decay"),
     visualmap_opts=opts.VisualMapOpts(min_=1, max_=254, orient="horizontal",
                                       pos_left="center", pos_bottom="0%"),
     xaxis_opts=opts.AxisOpts(name="cell x", splitline_opts=opts.SplitLineOpts(is_show=False)),
     yaxis_opts=opts.AxisOpts(name="cell y"),
     tooltip_opts=opts.TooltipOpts(is_show=True))
 ).render_notebook()
```

## The Global Planner

The global planner searches the global costmap for a route. Nav2 ships several:

| Plugin | Method | Suits |
|----|----|----|
| `NavfnPlanner` | Dijkstra or A\* on the grid | the default; fast, ignores robot shape |
| `SmacPlanner2D` | A\* on the grid, smoothed | a cleaner 2D path |
| `SmacPlannerHybrid` | Hybrid-A\* with motion primitives | car-like robots, honours a turning radius |
| `SmacPlannerLattice` | state lattice | arbitrary kinematics |
| `ThetaStarPlanner` | any-angle A\* | fewer needless turns |

`NavfnPlanner` plans for a point, so a path it produces can be infeasible for a long robot. The Smac hybrid and lattice planners are the ones that respect kinematics, at more CPU.

A\* over the costmap above, refusing any cell at or above 253:

------------------------------------------------------------------------

``` python
import heapq

def astar(cost, start, goal, max_cost=INSCRIBED):
    "8-connected A*, refusing cells at or above max_cost and preferring low-cost ones."
    H, W = cost.shape
    h = lambda a, b: np.hypot(a[0] - b[0], a[1] - b[1])
    nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    openq, came, g = [(h(start, goal), 0.0, start, None)], {}, {start: 0.0}
    while openq:
        _, gc, cur, parent = heapq.heappop(openq)
        if cur in came:
            continue
        came[cur] = parent
        if cur == goal:
            break
        for dr, dc in nbrs:
            nxt = (cur[0] + dr, cur[1] + dc)
            if not (0 <= nxt[0] < H and 0 <= nxt[1] < W) or cost[nxt] >= max_cost:
                continue
            ng = gc + np.hypot(dr, dc) * (1.0 + cost[nxt] / 255.0)
            if ng < g.get(nxt, np.inf):
                g[nxt] = ng
                heapq.heappush(openq, (ng + h(nxt, goal), ng, nxt, cur))
    if goal not in came:
        return None
    path, n = [], goal
    while n is not None:
        path.append(n); n = came[n]
    return path[::-1]

start, goal = (30, 10), (30, 90)
path = astar(cost, start, goal)
length = sum(np.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])) * RES
print(f"path: {len(path)} cells, {length:.3f} m (straight line would be "
      f"{np.hypot(goal[0]-start[0], goal[1]-start[1]) * RES:.3f} m)")

assert path[0] == start and path[-1] == goal
# the stability criterion: the plan never enters inflated space the planner must refuse
assert all(cost[c] < INSCRIBED for c in path)
print(f"highest cost on the path: {int(max(cost[c] for c in path))} (INSCRIBED is {INSCRIBED})")

# the wall spans the room, so every path must cross column 40 - inside the doorway
crossings = [c for c in path if c[1] == 40]
print(f"cells on the wall column: {crossings} -> all inside the doorway (rows 22-37): "
      f"{all(22 <= r < 38 for r, _ in crossings)}")
assert crossings and all(22 <= r < 38 for r, _ in crossings)
```

    path: 81 cells, 4.000 m (straight line would be 4.000 m)
    highest cost on the path: 119 (INSCRIBED is 253)
    cells on the wall column: [(30, 40)] -> all inside the doorway (rows 22-37): True

``` python
# inscribed_radius decides PASSABILITY. The doorway is 0.80 m wide, so the most
# clearance available inside it is 0.40 m, and a robot needing more is refused.
print(f"widest clearance inside the doorway: {dist[22:38, 40].max():.3f} m\n")
for r_in in [0.15, 0.25, 0.35, 0.45]:
    c2, _ = inflate(occ, RES, inscribed_radius=r_in, inflation_radius=max(r_in + 0.30, 0.45))
    p2 = astar(c2, start, goal)
    print(f"inscribed_radius {r_in:.2f} m (robot {2 * r_in:.2f} m wide) -> "
          f"{'path of ' + str(len(p2)) + ' cells' if p2 else 'NO VALID PATH'}")

c_wide, _ = inflate(occ, RES, inscribed_radius=0.45, inflation_radius=0.75)
assert astar(c_wide, start, goal) is None, "a 0.9 m robot must not fit a 0.8 m doorway"

# cost_scaling_factor does NOT change passability. Here it does not change the route
# at all, because with one doorway the route is geometrically forced.
print()
for csf in [1.0, 3.0, 10.0]:
    c3, _ = inflate(occ, RES, cost_scaling_factor=csf)
    p3 = astar(c3, start, goal)
    print(f"cost_scaling_factor {csf:5.1f} -> {len(p3)} cells, "
          f"mean clearance {np.mean([dist[c] for c in p3]):.3f} m  (unchanged: only one route exists)")
```

    widest clearance inside the doorway: 0.400 m

    inscribed_radius 0.15 m (robot 0.30 m wide) -> path of 81 cells
    inscribed_radius 0.25 m (robot 0.50 m wide) -> path of 81 cells
    inscribed_radius 0.35 m (robot 0.70 m wide) -> path of 81 cells
    inscribed_radius 0.45 m (robot 0.90 m wide) -> NO VALID PATH

    cost_scaling_factor   1.0 -> 81 cells, mean clearance 0.645 m  (unchanged: only one route exists)
    cost_scaling_factor   3.0 -> 81 cells, mean clearance 0.645 m  (unchanged: only one route exists)
    cost_scaling_factor  10.0 -> 81 cells, mean clearance 0.645 m  (unchanged: only one route exists)

``` python
# Where there IS a choice, cost_scaling_factor moves the route - and in the direction
# people usually get backwards. Cost decays as exp(-csf * distance), so a LOWER csf
# decays more slowly, keeps cost higher further out, and pushes the path WIDER.
open_room = np.zeros((H, W), dtype=np.uint8)
open_room[0, :] = open_room[-1, :] = open_room[:, 0] = open_room[:, -1] = 1
open_room[28:32, 48:52] = 1                      # a single 20 cm pillar mid-room

for csf in [0.5, 1.0, 3.0, 10.0]:
    c4, d4 = inflate(open_room, RES, inscribed_radius=0.15, inflation_radius=0.80,
                     cost_scaling_factor=csf)
    p4 = astar(c4, start, goal)
    clear = [d4[c] for c in p4]
    print(f"cost_scaling_factor {csf:5.1f} -> min clearance {min(clear):.3f} m, "
          f"mean {np.mean(clear):.3f} m")

c_lo, d_lo = inflate(open_room, RES, inflation_radius=0.80, cost_scaling_factor=0.5)
c_hi, d_hi = inflate(open_room, RES, inflation_radius=0.80, cost_scaling_factor=10.0)
lo = min(d_lo[c] for c in astar(c_lo, start, goal))
hi = min(d_hi[c] for c in astar(c_hi, start, goal))
assert lo > hi, "a lower cost_scaling_factor must give more clearance"
print(f"\nlower csf gives more clearance: {lo:.3f} m at csf 0.5 against {hi:.3f} m at csf 10")
```

    cost_scaling_factor   0.5 -> min clearance 0.450 m, mean 0.802 m
    cost_scaling_factor   1.0 -> min clearance 0.450 m, mean 0.802 m
    cost_scaling_factor   3.0 -> min clearance 0.450 m, mean 0.802 m
    cost_scaling_factor  10.0 -> min clearance 0.350 m, mean 0.737 m

    lower csf gives more clearance: 0.450 m at csf 0.5 against 0.350 m at csf 10

## The Local Controller

The controller turns the global path into velocity commands, at `controller_frequency`, while avoiding what the local costmap sees.

| Plugin | Method | Notes |
|----|----|----|
| `DWBLocalPlanner` | Dynamic Window, sampled trajectories scored by critics | the ROS 1 heritage option; many knobs |
| `MPPIController` | Model Predictive Path Integral, sampled and weighted | the current default choice; smooth, CPU hungry |
| `RegulatedPurePursuitController` | pure pursuit with speed regulation on curvature and proximity | simple, predictable, few parameters |
| `RotationShimController` | rotates to face the path, then hands to another controller | a wrapper, not a controller |

`RegulatedPurePursuit` is the one to start with on a differential-drive robot: it has few parameters and fails understandably. MPPI produces better paths and costs far more CPU, which matters on four cores alongside SLAM.

Pure pursuit is simple enough to write: pick the point on the path at a fixed lookahead distance, compute the curvature that reaches it, command that.

------------------------------------------------------------------------

``` python
pts = np.array([(c * RES, r * RES) for r, c in path])      # path in metres, (x, y)

def pure_pursuit(pts, lookahead=0.30, v=0.4, dt=0.05, omega_max=2.0, max_steps=2000):
    """Follow `pts` with a unicycle model. Curvature to the lookahead point is
    2 * y_err / L^2, with y_err the lateral offset in the robot's own frame."""
    x, y = pts[0]
    th = np.arctan2(pts[1, 1] - pts[0, 1], pts[1, 0] - pts[0, 0])
    traj, idx = [(x, y)], 0
    for _ in range(max_steps):
        while idx < len(pts) - 1 and np.hypot(pts[idx, 0] - x, pts[idx, 1] - y) < lookahead:
            idx += 1
        dx, dy = pts[idx, 0] - x, pts[idx, 1] - y
        y_err = -np.sin(th) * dx + np.cos(th) * dy
        L = max(np.hypot(dx, dy), 1e-6)
        omega = np.clip(2.0 * v * y_err / (L * L), -omega_max, omega_max)
        x += v * np.cos(th) * dt
        y += v * np.sin(th) * dt
        th += omega * dt
        traj.append((x, y))
        if idx == len(pts) - 1 and np.hypot(pts[-1, 0] - x, pts[-1, 1] - y) < 0.05:
            break
    return np.array(traj)

traj = pure_pursuit(pts)
cte = np.linalg.norm(traj[:, None, :] - pts[None, :, :], axis=2).min(axis=1)
print(f"{len(traj)} control steps, final gap to goal {np.hypot(*(traj[-1] - pts[-1])):.3f} m")
print(f"cross-track error: mean {cte.mean():.3f} m, max {cte.max():.3f} m")

assert np.hypot(*(traj[-1] - pts[-1])) < 0.10
assert cte.max() < 0.25
# and the executed trajectory, not just the plan, stays out of lethal space
cells = [(int(round(y / RES)), int(round(x / RES))) for x, y in traj]
worst = max(cost[r, c] for r, c in cells if 0 <= r < H and 0 <= c < W)
print(f"worst costmap value under the executed trajectory: {int(worst)} (lethal is {LETHAL})")
assert worst < LETHAL
```

    199 control steps, final gap to goal 0.040 m
    cross-track error: mean 0.012 m, max 0.020 m
    worst costmap value under the executed trajectory: 119 (lethal is 254)

``` python
from pyecharts.charts import Scatter

obstacles = [[round(c * RES, 3), round(r * RES, 3)]
             for r in range(H) for c in range(W) if occ[r, c]]
(Scatter(init_opts=opts.InitOpts(width="900px", height="520px"))
 .add_xaxis([p[0] for p in obstacles])
 .add_yaxis("obstacles", [p[1] for p in obstacles], symbol_size=3,
            label_opts=opts.LabelOpts(is_show=False))
 .add_yaxis("A* plan", [[round(x, 3), round(y, 3)] for x, y in pts], symbol_size=4,
            label_opts=opts.LabelOpts(is_show=False))
 .add_yaxis("pure pursuit", [[round(x, 3), round(y, 3)] for x, y in traj], symbol_size=3,
            label_opts=opts.LabelOpts(is_show=False))
 .set_global_opts(
     title_opts=opts.TitleOpts(title="Plan and execution through an 80 cm doorway",
                               subtitle=f"cross-track error max {cte.max():.3f} m"),
     xaxis_opts=opts.AxisOpts(type_="value", name="x (m)", min_=0, max_=5),
     yaxis_opts=opts.AxisOpts(type_="value", name="y (m)", min_=0, max_=3),
     tooltip_opts=opts.TooltipOpts(trigger="item"))
 ).render_notebook()
```

## Tuning, and What Each Symptom Means

The demonstrations above are the real relationships in miniature, and they map onto the usual complaints:

| Symptom | Parameter |
|----|----|
| “No valid path found” through a gap the robot fits | `inscribed_radius` / `robot_radius` too large |
| path hugs obstacles despite a large `inflation_radius` | `cost_scaling_factor` too **high**: it decays faster, so repulsion dies sooner |
| robot clips door frames | `inscribed_radius` too small, or footprint not modelled |
| path hugs walls | `cost_scaling_factor` too low, or `inflation_radius` too small |
| path takes absurd detours | `inflation_radius` too large, closing reasonable routes |
| robot oscillates along the path | controller lookahead too short, or `controller_frequency` too low |
| corners cut | lookahead too long |
| robot stops and spins often | local costmap full of phantom obstacles; check clearing and sensor frames |
| path fine, tracking poor | controller, not planner: check the velocity limits actually reach the base |

Two rules that follow from the measured results above. **`inscribed_radius` is a physical fact about the robot, not a tuning knob** - measure it and set it. And **`cost_scaling_factor` changes preference, not feasibility**: the sweeps above show it leaving a forced route untouched, and moving a route only where an alternative existed. Note its sign, which is easy to get backwards - cost decays as `exp(-csf * distance)`, so a *lower* factor decays more slowly and pushes the path *wider*. Raising it will never make an impassable gap passable, and lowering it will never let the robot through something it does not fit.

Finally, a capacity note: the local costmap at 5 Hz over a 3 m window plus MPPI at 20 Hz is a real CPU load. On a four-core machine running SLAM and perception as well, this is where the lifecycle bond timeouts in [01_Nav2_Bringup.ipynb](../06_Navigation_and_Manipulation/nav2_bringup.llms.md) come from.

------------------------------------------------------------------------

Back to top
