# Inference Node Integration

Wrapping a trained model as a ROS 2 node: where the inference runtime goes, how to keep a 30 Hz stream from queueing behind a 100 ms model, and what to publish so downstream nodes can use the result.

Author

Benedict Thekkel

## The Shape of an Inference Node

An inference node is a subscriber, a model, and a publisher, and the interesting parts are all about rate and memory rather than about the model.

``` python
class Detector(Node):
    def __init__(self):
        super().__init__("detector")
        self.declare_parameter("model_path", "")
        self.declare_parameter("conf_threshold", 0.5)

        self.bridge = CvBridge()
        self.session = self.load_model(self.get_parameter("model_path").value)

        # best effort, depth 1: drop stale frames rather than queue them
        self.sub = self.create_subscription(
            Image, "image_rect", self.on_image,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))
        self.pub = self.create_publisher(Detection2DArray, "detections", 10)

    def on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        boxes, scores, labels = self.infer(img)
        out = self.to_detection_array(boxes, scores, labels)
        out.header = msg.header          # the image's stamp and frame, not now
        self.pub.publish(out)
```

**`depth=1` with `BEST_EFFORT` is the single most important line.** With the default reliable, depth-10 profile, a model that takes 100 ms behind a 30 Hz camera builds a queue, and the node ends up processing frames from seconds ago while appearing to keep up. Dropping frames is correct behaviour for perception: the robot needs the current world, not every frame of history.

**`out.header = msg.header`** carries the acquisition stamp forward, so a consumer can transform the detection using the pose at capture time rather than the pose now. Restamping is the subtle version of the same latency bug.

------------------------------------------------------------------------

## Keeping the Callback Off the Executor

A 100 ms inference call inside a callback blocks the node’s executor for 100 ms, which stalls every other callback including timers and service responses. Three arrangements, in increasing order of effort:

| Arrangement | Good for |
|----|----|
| single node, own executor thread | one model, nothing else in the process |
| inference in its own callback group, multi-threaded executor | a node that also serves parameters, diagnostics, services |
| inference in a worker thread, callback only hands off the frame | long or variable inference times |

The worker-thread form, which is what most production nodes end up with:

``` python
def on_image(self, msg):
    with self.lock:
        self.latest = msg          # overwrite: only the newest frame matters
        self.have_frame.set()

def worker(self):                  # a plain threading.Thread
    while not self.stopping:
        self.have_frame.wait()
        with self.lock:
            msg, self.latest = self.latest, None
            self.have_frame.clear()
        if msg is not None:
            self.publish_result(msg, self.infer(msg))
```

Overwriting rather than queueing is deliberate, and it makes the drop policy explicit in the code instead of hidden in a QoS profile. Executors and callback groups are covered in [../01_Core_Concepts/04_Executors_Lifecycle_and_Composition.ipynb](../01_Core_Concepts/executors_lifecycle_and_composition.llms.md).

------------------------------------------------------------------------

## Runtimes

The model does not run in Python’s main loop if performance matters, and the runtime choice is mostly about where the robot is going.

| Runtime | Why | Cost |
|----|----|----|
| **ONNX Runtime** | one exported file runs on CPU, CUDA, TensorRT or ROCm; good default | a little slower than a vendor-optimised engine |
| **TensorRT** | fastest on NVIDIA, including Jetson | engine files are built per GPU and per TensorRT version, so they are not portable |
| **OpenVINO** | fastest on Intel CPU and iGPU | Intel only |
| **torch** directly | no export step, matches training exactly | largest dependency, slowest startup |

Two deployment facts worth knowing before choosing:

- **A TensorRT engine is not portable.** It is compiled for a specific GPU architecture and TensorRT version, so it must be built on the target or in a matching container, and the first build takes minutes. Shipping an `.onnx` and building the engine on first run is the usual compromise.
- **Warm up the model in `on_configure`, not on the first frame.** The first inference allocates workspace and can take ten times as long, which in a lifecycle node means a latency spike exactly when the robot starts moving. Running one dummy inference at startup moves the cost somewhere harmless.

A lifecycle node is the natural fit: load the model in `on_configure`, warm it up, and only start subscribing in `on_activate`.

------------------------------------------------------------------------

## What to Publish

Use the standard vision messages so that RViz plugins, trackers and Nav2 layers can consume the output without a custom bridge.

| Message | For |
|----|----|
| `vision_msgs/Detection2DArray` | 2D boxes with scores and class ids |
| `vision_msgs/Detection3DArray` | 3D boxes, after unprojection |
| `sensor_msgs/Image` (`mono8`) | segmentation masks |
| `geometry_msgs/PoseArray` | keypoints or object poses |
| `visualization_msgs/MarkerArray` | visualisation only, never as the data path |

`MarkerArray` is the one to be disciplined about: it is for RViz, and a downstream node that parses markers instead of `Detection3DArray` is coupling itself to a debug output.

Turning a 2D detection into something a planner can use needs depth, and the unprojection plus the optical-frame rotation from [00_Images_and_Calibration.ipynb](../05_Perception/images_and_calibration.llms.md):

``` python
# box centre -> a 3D point in the optical frame, then into base_link via tf2
u, v = box.center.position.x, box.center.position.y
z = depth_at(u, v)                       # metres, from the aligned depth image
ray = np.linalg.inv(K) @ np.array([u, v, 1.0])
p_optical = ray * z
```

Two rules that keep the result usable: **publish in the frame the data came from** and let tf2 do the rest, rather than transforming inside the inference node; and **publish an empty array when nothing is detected**, since a consumer cannot distinguish “no objects” from “node died” if the topic simply goes quiet.

------------------------------------------------------------------------

## Budgeting on a Small Machine

The machine these notes are written on is a 4-core container with 20 GB RAM and a 12 GB RTX 3060, which is a realistic robot compute budget and tighter than it sounds once a full stack is running.

- **Inference shares the GPU with nothing else, but the CPU with everything.** Pre- and post-processing (resize, normalise, NMS) is CPU work and often costs more than the forward pass. Do the resize on the GPU or accept the frame rate.
- **One large model live at a time.** Two 3 GB models plus a simulator does not fit in 12 GB, and the failure is a CUDA OOM mid-mission rather than at startup.
- **`DataLoader`-style worker processes are not free.** On 4 cores, a node spawning 8 workers starves the control loop.
- **Measure the end-to-end latency, not the inference time.** Stamp at acquisition and log the age of the message when the result is published; that number includes transport, queueing and post-processing, and it is the one a controller experiences.

``` python
age = (self.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds / 1e6
self.get_logger().info(f"end-to-end {age:.1f} ms", throttle_duration_sec=2.0)
```

If that number is larger than the inference time by more than a few milliseconds, the problem is queueing rather than the model, and the QoS profile at the top of this notebook is the fix.

------------------------------------------------------------------------

Back to top
