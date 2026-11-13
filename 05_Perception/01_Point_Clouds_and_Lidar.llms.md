# Point Clouds and Lidar

PointCloud2 as a byte buffer: the field layout decoded with numpy structured dtypes, the depth-image and laser-scan conversions, and pointcloud_to_laserscan reimplemented over a synthetic cloud.

Author

Benedict Thekkel

## PointCloud2 Is a Byte Buffer

`sensor_msgs/PointCloud2` does not hold a list of points. It holds an opaque `uint8[] data` plus a description of how to read it, which is why it is fast and why naive code is slow and wrong.

    std_msgs/Header header
    uint32 height            1 for an unordered cloud, image rows for an organised one
    uint32 width             points per row
    PointField[] fields      name, offset, datatype, count - the layout
    bool is_bigendian
    uint32 point_step        bytes per point (often larger than the sum of the fields)
    uint32 row_step          point_step * width
    uint8[] data             height * row_step bytes
    bool is_dense            false if any point is NaN or Inf

The `PointField` datatype enum:

| Value | Type    | numpy |
|-------|---------|-------|
| 1     | INT8    | `i1`  |
| 2     | UINT8   | `u1`  |
| 3     | INT16   | `i2`  |
| 4     | UINT16  | `u2`  |
| 5     | INT32   | `i4`  |
| 6     | UINT32  | `u4`  |
| 7     | FLOAT32 | `f4`  |
| 8     | FLOAT64 | `f8`  |

Three properties decide how the buffer must be read:

- **`point_step` is usually bigger than the fields.** Drivers pad for alignment: a Velodyne point with x, y, z, intensity and ring occupies 18 bytes of content in a 32-byte stride. Assuming a packed layout reads the wrong bytes for every field after the gap.
- **`offset` is authoritative, not field order.** Fields can be declared in any order and the offsets need not be contiguous.
- **`height > 1` means organised.** A cloud straight from an RGB-D camera is a 2D grid, so it can be reshaped to the image and indexed by pixel. A lidar cloud has `height == 1`.

**Never iterate point by point in Python.** `read_points` in a loop over 100k points takes seconds; the structured-dtype view below takes microseconds and gives a numpy array.

------------------------------------------------------------------------

## Decoding and Encoding

``` python
import numpy as np

# sensor_msgs/PointField datatype constants
INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64 = 1, 2, 3, 4, 5, 6, 7, 8
NUMPY_OF = {INT8: "i1", UINT8: "u1", INT16: "i2", UINT16: "u2",
            INT32: "i4", UINT32: "u4", FLOAT32: "f4", FLOAT64: "f8"}

# A realistic Velodyne-style layout: note intensity at offset 16, not 12, and a
# 32-byte stride for 18 bytes of content.
fields = [("x", 0, FLOAT32), ("y", 4, FLOAT32), ("z", 8, FLOAT32),
          ("intensity", 16, FLOAT32), ("ring", 20, UINT16)]
point_step = 32

def dtype_for(fields, point_step):
    "A structured dtype that honours byte offsets, with itemsize pinned to point_step."
    return np.dtype({"names":    [f[0] for f in fields],
                     "formats":  [NUMPY_OF[f[2]] for f in fields],
                     "offsets":  [f[1] for f in fields],
                     "itemsize": point_step})

dt = dtype_for(fields, point_step)
print("itemsize:", dt.itemsize, "  names:", dt.names)
print("offsets :", [dt.fields[n][1] for n in dt.names])
assert dt.itemsize == point_step
```

    itemsize: 32   names: ('x', 'y', 'z', 'intensity', 'ring')
    offsets : [0, 4, 8, 16, 20]

``` python
# Build a cloud, encode it to the bytes that would go in msg.data, decode it back.
n = 1000
rng = np.random.default_rng(1)
cloud = np.zeros(n, dtype=dt)
cloud["x"] = rng.uniform(-10, 10, n).astype(np.float32)
cloud["y"] = rng.uniform(-10, 10, n).astype(np.float32)
cloud["z"] = rng.uniform(-1, 2, n).astype(np.float32)
cloud["intensity"] = rng.uniform(0, 255, n).astype(np.float32)
cloud["ring"] = rng.integers(0, 16, n).astype(np.uint16)

data = cloud.tobytes()            # this is msg.data
row_step = point_step * n         # msg.row_step, with height=1, width=n
print(f"{n} points -> {len(data)} bytes   point_step {point_step}   row_step {row_step}")
assert len(data) == row_step == n * point_step

# the subscriber side: one frombuffer call, no loop
decoded = np.frombuffer(data, dtype=dtype_for(fields, point_step), count=n)
for name in dt.names:
    assert np.array_equal(decoded[name], cloud[name]), name
print("round trip byte-exact for all", len(dt.names), "fields")

# xyz as a plain (n, 3) array, which is what downstream maths wants
xyz = np.stack([decoded["x"], decoded["y"], decoded["z"]], axis=1)
print("xyz:", xyz.shape, xyz.dtype)
assert xyz.shape == (n, 3) and xyz.dtype == np.float32
```

    1000 points -> 32000 bytes   point_step 32   row_step 32000
    round trip byte-exact for all 5 fields
    xyz: (1000, 3) float32

``` python
# Why the offsets matter: guessing a packed layout does NOT raise, it returns
# plausible nonsense for every field after the padding gap.
guessed = [("x", 0, FLOAT32), ("y", 4, FLOAT32), ("z", 8, FLOAT32),
           ("intensity", 12, FLOAT32)]          # wrong: intensity is at 16
bad = np.frombuffer(data, dtype=dtype_for(guessed, point_step), count=n)

print("x correct with the wrong layout? ", np.array_equal(bad["x"], cloud["x"]))
print("intensity, guessed offset 12:", bad["intensity"][:3].round(3))
print("intensity, true offset 16   :", cloud["intensity"][:3].round(3))
assert np.array_equal(bad["x"], cloud["x"])                        # xyz happens to be right
assert not np.array_equal(bad["intensity"], cloud["intensity"])    # intensity is garbage
print("\nno exception, no warning - read msg.fields, never assume")
```

    x correct with the wrong layout?  True
    intensity, guessed offset 12: [0. 0. 0.]
    intensity, true offset 16   : [ 29.393 224.042  76.331]

    no exception, no warning - read msg.fields, never assume

## pointcloud_to_laserscan

A 3D cloud is often reduced to a 2D `LaserScan`, because Nav2’s 2D costmaps, AMCL and `slam_toolbox` all consume scans. The `pointcloud_to_laserscan` package does it: keep the points in a height band, bin them by bearing, take the nearest hit per bin.

Reimplementing it makes the parameters concrete, and shows what information the reduction throws away.

------------------------------------------------------------------------

``` python
def to_laserscan(xyz, angle_min=-np.pi, angle_max=np.pi,
                 angle_increment=np.deg2rad(1.0),
                 range_min=0.1, range_max=30.0,
                 min_height=-0.2, max_height=0.5):
    """Flatten a cloud to ranges, the way pointcloud_to_laserscan does:
    slice a height band, bin by bearing, keep the closest hit in each bin."""
    nbins = int(round((angle_max - angle_min) / angle_increment))
    ranges = np.full(nbins, np.inf, dtype=np.float32)      # inf = no return

    band = (xyz[:, 2] >= min_height) & (xyz[:, 2] <= max_height)
    p = xyz[band]
    r = np.hypot(p[:, 0], p[:, 1])
    keep = (r >= range_min) & (r <= range_max)
    p, r = p[keep], r[keep]

    bearing = np.arctan2(p[:, 1], p[:, 0])
    idx = np.clip(((bearing - angle_min) / angle_increment).astype(int), 0, nbins - 1)
    np.minimum.at(ranges, idx, r)        # unbuffered: the closest hit wins
    return ranges

scan = to_laserscan(xyz)
hits = np.isfinite(scan)
print(f"{len(scan)} bins, {hits.sum()} with a return ({100 * hits.mean():.0f}% coverage)")
print(f"range min {scan[hits].min():.3f} m, max {scan[hits].max():.3f} m")
assert len(scan) == 360
assert scan[hits].min() >= 0.1 and scan[hits].max() <= 30.0

# a single known point lands in the bin and at the range it should
one = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
s = to_laserscan(one)
bin_straight_ahead = int(round((0.0 - (-np.pi)) / np.deg2rad(1.0)))
print(f"\npoint (1, 0, 0) -> bin {bin_straight_ahead}, range {s[bin_straight_ahead]}")
assert np.isclose(s[bin_straight_ahead], 1.0)
assert np.isfinite(s).sum() == 1

# and a point above the band is dropped entirely
overhead = np.array([[1.0, 0.0, 5.0]], dtype=np.float32)
assert np.isfinite(to_laserscan(overhead)).sum() == 0
print("a point 5 m up is discarded: the height band is what makes this lossy")
```

    360 bins, 171 with a return (48% coverage)
    range min 0.269 m, max 13.059 m

    point (1, 0, 0) -> bin 180, range 1.0
    a point 5 m up is discarded: the height band is what makes this lossy

``` python
from pyecharts.globals import CurrentConfig

# assets.pyecharts.org serves an expired TLS certificate from most nodes (2026-11-11),
# so charts failed to load. jsDelivr mirrors the same assets, pinned to one commit.
CurrentConfig.ONLINE_HOST = "https://cdn.jsdelivr.net/gh/pyecharts/pyecharts-assets@f2328d558550046fc5449672a6850e848e141590/assets/v6/"

from pyecharts.charts import Scatter
from pyecharts import options as opts

# the flattened scan in polar terms, plotted as cartesian hit positions
bearings = np.arange(len(scan)) * np.deg2rad(1.0) - np.pi
hx = (scan[hits] * np.cos(bearings[hits])).round(3).tolist()
hy = (scan[hits] * np.sin(bearings[hits])).round(3).tolist()

(Scatter(init_opts=opts.InitOpts(width="720px", height="520px"))
 .add_xaxis(hx)
 .add_yaxis("nearest return per 1 deg bin", hy, symbol_size=6,
            label_opts=opts.LabelOpts(is_show=False))
 .set_global_opts(
     title_opts=opts.TitleOpts(title="Cloud flattened to a LaserScan",
                               subtitle=f"{hits.sum()} of 360 bins have a return"),
     xaxis_opts=opts.AxisOpts(type_="value", name="x (m)", min_=-15, max_=15,
                              splitline_opts=opts.SplitLineOpts(is_show=True)),
     yaxis_opts=opts.AxisOpts(type_="value", name="y (m)", min_=-15, max_=15,
                              splitline_opts=opts.SplitLineOpts(is_show=True)),
     tooltip_opts=opts.TooltipOpts(trigger="item"))
 ).render_notebook()
```

## What the Reduction Costs, and the Parameters That Decide

The height band is the whole design. Set it wrong and the robot either cannot see obstacles or refuses to move:

| Parameter | Effect of getting it wrong |
|----|----|
| `min_height` | too low: the floor becomes an obstacle ring at every bearing |
| `max_height` | too high: a doorframe or low ceiling blocks the whole corridor |
| `range_min` | too small: the robot’s own chassis appears as an obstacle |
| `angle_increment` | too coarse: thin obstacles (chair legs, poles) fall between bins |
| `use_inf` | `false` replaces no-return with `range_max`, which some consumers read as a real hit |

What is lost and cannot be recovered: **overhangs and negative obstacles**. A table is seen as its legs, so a robot can plan under a surface it does not fit beneath; a downward step is not seen at all, because there is no return in the band. Both are reasons to keep a 3D layer in the costmap rather than relying only on the flattened scan.

Also worth remembering that **`inf` is meaningful**: it means “nothing within range on this bearing”, which is different from “an obstacle at `range_max`”. A consumer that does `np.nan_to_num` on a scan has converted free space into a wall.

------------------------------------------------------------------------

## Depth Images and Organised Clouds

An RGB-D camera publishes a depth image, and `depth_image_proc` turns it into a cloud. The conversion is the unprojection from [00_Images_and_Calibration.ipynb](../05_Perception/images_and_calibration.llms.md) applied per pixel, with the camera’s `k` and the depth value as the range.

``` bash
# depth image + camera_info -> PointCloud2, as a composable node
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/depth/image_rect_raw \
  -r camera_info:=/camera/depth/camera_info \
  -r points:=/camera/depth/points
```

Because the source is an image, the resulting cloud is **organised**: `height` and `width` match the image, so `xyz.reshape(height, width, 3)` indexes by pixel and a detection bounding box in the colour image maps straight onto the points inside it. That correspondence is the reason to keep the cloud organised rather than filtering it early.

Two things to expect:

- **Invalid pixels are NaN**, and `is_dense` is then `false`. Every consumer has to handle them; `np.isfinite` masking before any maths is the habit.
- **Depth encoding differs by driver**: `16UC1` is millimetres, `32FC1` is metres. The same trap as in the image notebook, and here it produces a cloud 1000 times too large.

Filtering in practice uses PCL through `pcl_ros` (voxel grid, passthrough, statistical outlier removal, RANSAC plane fitting) rather than hand-written numpy, because those are C++ nodes that run at sensor rate. The numpy form above is for understanding the data and for offline work.

------------------------------------------------------------------------

Back to top
