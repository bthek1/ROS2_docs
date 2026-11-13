# Message Synchronisation

message_filters: why two sensor streams never line up, the ExactTime and ApproximateTime policies, and the ApproximateTime matching worked through on synthetic streams to show what the slop parameter buys.

Author

Benedict Thekkel

## The Problem

Fusing two sensors means handling one message from each that describe the same instant. Nothing in ROS 2 arranges that for you. Two streams on one robot are never aligned, for reasons that do not go away:

- **Different rates.** A camera at 30 Hz and a lidar at 10 Hz share one stamp in three at best.
- **Different latencies.** USB, Ethernet and SPI sensors each add their own delay, and the driver’s own processing adds more.
- **Jitter.** A nominal 30 Hz camera delivers at 30 Hz on average, with milliseconds of variation per frame.

The naive approach, caching the last message from stream B in a member variable and reading it in stream A’s callback, is what most people write first. It works in the sense that it runs, and it silently pairs a fresh image with a lidar scan of unknown age. At 1 m/s, a 100 ms pairing error is 10 cm of disagreement between the two, which downstream reads as a calibration problem.

`message_filters` is the library that does it properly.

------------------------------------------------------------------------

## The Two Policies

``` python
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer

class Fusion(Node):
    def __init__(self):
        super().__init__("fusion")
        image = Subscriber(self, Image, "camera/image_raw")
        scan = Subscriber(self, LaserScan, "scan")

        # ExactTime: header.stamp must match bit for bit
        self.sync = TimeSynchronizer([image, scan], queue_size=10)

        # ApproximateTime: match within `slop` seconds
        self.sync = ApproximateTimeSynchronizer([image, scan], queue_size=10, slop=0.05)

        self.sync.registerCallback(self.on_pair)

    def on_pair(self, image, scan):
        # both arguments describe (approximately) the same instant
        ...
```

**`ExactTime` is for messages that were stamped by the same node**, typically an image and its `CameraInfo`, or a stereo pair from one driver. Anything else never matches.

**`ApproximateTime` is for everything else**, and `slop` is the whole decision. The implementation is a queue per input; a set is released when one message from each input falls within `slop`, and each message is used at most once.

Two parameters, two distinct failure modes:

- **`slop` too small**: few or no callbacks. The symptom is a fusion node that appears dead.
- **`slop` too large**: callbacks fire with badly matched data. Worse than the first case, because it looks like it is working.
- **`queue_size` too small**: messages are evicted before their partner arrives, so the match rate drops under load even with a generous slop.

------------------------------------------------------------------------

## Matching, Worked Through

A simplified ApproximateTime model on synthetic streams, to see how the match rate and the worst-case pairing error move with `slop`. The streams below are deliberately awkward: a 30 Hz camera with 5 ms of jitter, and a 10 Hz lidar with 15 ms of jitter that also runs 20 ms late.

------------------------------------------------------------------------

``` python
import numpy as np

rng = np.random.default_rng(0)
cam = np.sort(np.arange(0, 2, 1 / 30) + rng.normal(0, 0.005, 60))        # 30 Hz, 5 ms jitter
lidar = np.sort(np.arange(0, 2, 1 / 10) + 0.020 + rng.normal(0, 0.015, 20))  # 10 Hz, late, jittery
print(f"camera: {len(cam)} stamps over 2 s     lidar: {len(lidar)} stamps over 2 s")
print("first few camera stamps:", cam[:4].round(4))
print("first few lidar stamps :", lidar[:4].round(4))

def approximate_time(a, b, slop):
    """Greedy nearest-neighbour pairing within `slop`, each stamp used at most once.
    A simplified model of ApproximateTimeSynchronizer: the real one works on queues
    as messages arrive and can revise a pending set, but the slop behaviour matches."""
    pairs, used = [], set()
    for i, ta in enumerate(a):
        best, best_d = None, slop
        for j, tb in enumerate(b):
            if j in used:
                continue
            if abs(ta - tb) <= best_d:
                best, best_d = j, abs(ta - tb)
        if best is not None:
            used.add(best)
            pairs.append((i, best, ta - b[best]))
    return pairs

# ExactTime on jittered streams matches nothing at all
exact = [(i, j) for i, ta in enumerate(cam) for j, tb in enumerate(lidar) if ta == tb]
print("\nExactTime matches:", len(exact), "- as expected, since no two stamps are equal")
assert len(exact) == 0
```

    camera: 60 stamps over 2 s     lidar: 20 stamps over 2 s
    first few camera stamps: [0.0006 0.0327 0.0699 0.1005]
    first few lidar stamps : [0.0135 0.1025 0.2461 0.3126]

    ExactTime matches: 0 - as expected, since no two stamps are equal

``` python
rows = []
for slop in [0.005, 0.010, 0.025, 0.050, 0.100, 0.200]:
    pairs = approximate_time(cam, lidar, slop)
    dt = np.array([p[2] for p in pairs]) if pairs else np.array([0.0])
    rows.append((slop, len(pairs), np.abs(dt).max(), np.abs(dt).mean()))
    print(f"slop {slop * 1000:6.1f} ms -> {len(pairs):2d}/{len(lidar)} lidar scans matched, "
          f"worst pairing error {np.abs(dt).max() * 1000:5.1f} ms, "
          f"mean {np.abs(dt).mean() * 1000:4.1f} ms")

# properties the real policy also guarantees
wide = approximate_time(cam, lidar, 0.2)
narrow = approximate_time(cam, lidar, 0.005)
assert len(narrow) < len(wide), "a wider slop cannot match fewer pairs"
assert len(wide) <= len(lidar), "each message is consumed at most once"
assert all(abs(d) <= 0.2 for _, _, d in wide), "no pair exceeds the slop"
js = [j for _, j, _ in wide]
assert len(js) == len(set(js)), "no lidar scan is reused"
print("\nmatch count is monotonic in slop, and no message is reused")
```

    slop    5.0 ms ->  8/20 lidar scans matched, worst pairing error   4.3 ms, mean  2.4 ms
    slop   10.0 ms -> 13/20 lidar scans matched, worst pairing error   9.8 ms, mean  4.5 ms
    slop   25.0 ms -> 20/20 lidar scans matched, worst pairing error  25.0 ms, mean 10.2 ms
    slop   50.0 ms -> 20/20 lidar scans matched, worst pairing error  49.4 ms, mean 33.3 ms
    slop  100.0 ms -> 20/20 lidar scans matched, worst pairing error  99.5 ms, mean 78.9 ms
    slop  200.0 ms -> 20/20 lidar scans matched, worst pairing error 199.2 ms, mean 168.5 ms

    match count is monotonic in slop, and no message is reused

``` python
from pyecharts.globals import CurrentConfig

# assets.pyecharts.org serves an expired TLS certificate from most nodes (2026-11-11),
# so charts failed to load. jsDelivr mirrors the same assets, pinned to one commit.
CurrentConfig.ONLINE_HOST = "https://cdn.jsdelivr.net/gh/pyecharts/pyecharts-assets@f2328d558550046fc5449672a6850e848e141590/assets/v6/"

from pyecharts.charts import Line
from pyecharts import options as opts

slops_ms = [r[0] * 1000 for r in rows]
matched = [r[1] for r in rows]
worst_ms = [round(r[2] * 1000, 1) for r in rows]

(Line(init_opts=opts.InitOpts(width="820px", height="400px"))
 .add_xaxis([f"{s:g}" for s in slops_ms])
 .add_yaxis("lidar scans matched (of 20)", matched, yaxis_index=0,
            is_smooth=False, symbol_size=8)
 .add_yaxis("worst pairing error (ms)", worst_ms, yaxis_index=1,
            is_smooth=False, symbol_size=8)
 .extend_axis(yaxis=opts.AxisOpts(name="worst error (ms)", position="right"))
 .set_global_opts(
     title_opts=opts.TitleOpts(title="The slop trade-off",
                               subtitle="more matches, but each one pairs worse"),
     xaxis_opts=opts.AxisOpts(name="slop (ms)"),
     yaxis_opts=opts.AxisOpts(name="matched"),
     tooltip_opts=opts.TooltipOpts(trigger="axis"))
 ).render_notebook()
```

## Choosing the Slop

The curve above is the whole design decision, and it has no optimum in the abstract: wider slop buys more callbacks and pays in pairing error. What settles it is how far the robot moves in that time.

A usable rule: **slop times maximum speed should be below the accuracy the consumer needs.**

| Speed                   | Needed accuracy | Slop ceiling                 |
|-------------------------|-----------------|------------------------------|
| 0.2 m/s indoor robot    | 1 cm            | 50 ms                        |
| 1.0 m/s indoor robot    | 1 cm            | 10 ms                        |
| 5.0 m/s outdoor vehicle | 5 cm            | 10 ms                        |
| stationary arm          | 1 mm            | generous; rotation dominates |

Then check the match rate is acceptable at that slop, and if it is not, the fix is upstream: better stamping in the driver, a faster sensor, or hardware triggering. Raising the slop until the callbacks arrive is how a fusion node ends up quietly wrong.

Three alternatives to reach for instead of a very large slop:

- **Interpolate rather than match.** For a pose or a transform, tf2 already interpolates between samples, so a lookup at the image’s stamp beats pairing with the nearest pose message.
- **Hardware sync.** Cameras and lidars with a trigger input remove the problem rather than managing it, and `ExactTime` then works.
- **Let the slower stream drive.** If the lidar is the limiting sensor, run the pipeline at lidar rate and look up everything else at the scan’s stamp.

------------------------------------------------------------------------

## Diagnosing a Silent Synchroniser

A fusion callback that never fires is the usual complaint, and the causes are ordered:

``` bash
ros2 topic hz /camera/image_raw /scan       # are both streams alive, and at what rate
ros2 topic echo /scan --field header.stamp --once
ros2 topic echo /camera/image_raw --field header.stamp --once
```

| Observation | Cause |
|----|----|
| one stream’s stamps are all zero | driver not stamping; `ApproximateTime` cannot work at all |
| stamps differ by hours | one node on wall time, another on sim time |
| stamps differ by a constant offset | sensor latency; raise slop to just over the offset, or fix the driver |
| both streams fine, no callbacks | slop too small, or `queue_size` too small under load |
| callbacks fire, data disagrees | slop too large |
| callbacks stop under load | queue eviction; raise `queue_size` |

The zero-stamp case is worth checking first because it is common in cheap drivers and indistinguishable from a synchroniser bug. A stamp of zero means tf2 and `message_filters` have nothing to work with, and no amount of slop tuning helps. See [02_Conventions_and_Time.ipynb](../03_Spatial_and_Temporal/conventions_and_time.llms.md).

------------------------------------------------------------------------

Back to top
