# xense-ros

ROS2 wrapper for the [Xense](https://xense.ai) tactile-visual sensor, built on **libxensesdk**.

> **Branch:** This repository uses `humble` as the default branch (for ROS2 Humble / Ubuntu 22.04 — coming soon). Active development for **ROS2 Jazzy / Ubuntu 24.04** lives on the [`jazzy`](../../tree/jazzy) branch.

---

## Packages

| Package | Description |
|---|---|
| `xense_camera` | Main driver node — streams tactile sensor data as ROS2 topics |
| `xense_msgs` | Custom message definitions for marker and force data |

---

## Prerequisites

- **Ubuntu 24.04** with **ROS2 Jazzy** installed
- **libxensesdk** built and installed to `/usr/local`
  ```bash
  cd ~/libxensesdk
  cmake -B build -DCMAKE_BUILD_TYPE=Release
  cmake --build build -j$(nproc)
  sudo cmake --install build
  ```
- System dependencies:
  ```bash
  sudo apt install -y nlohmann-json3-dev
  ```
- ROS2 dependencies:
  ```bash
  sudo apt install -y \
    ros-jazzy-rclcpp \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-tf2-ros
  ```

---

## Building

```bash
cd ~/xense-ros
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Usage

### Basic launch (rectified stream only, default)

```bash
ros2 launch xense_camera xense_launch.py
```

### Enable difference stream (single mode)

```bash
ros2 launch xense_camera xense_launch.py enable_diff:=true
```

### Enable difference stream (continuous mode)

```bash
ros2 launch xense_camera xense_launch.py enable_diff:=true diff_mode:=continuous
```

### Select a specific device by serial

```bash
ros2 launch xense_camera xense_launch.py device_serial:=OG000456
```

### Use a params file

```bash
ros2 launch xense_camera xense_launch.py \
  params_file:=$(ros2 pkg prefix xense_camera)/share/xense_camera/config/xense_params.yaml
```

---

## Topics

All topics are published under the node's namespace (default: `/xense_camera`).

Publishers use **RELIABLE QoS** (depth 10) — compatible with rviz2 and all standard ROS2 subscribers out of the box.

| Topic | Type | Encoding | Enabled by |
|---|---|---|---|
| `~/raw/image_raw` | `sensor_msgs/Image` | `bgr8` | `enable_raw:=true` |
| `~/rectified/image` | `sensor_msgs/Image` | `bgr8` | `enable_rectified` (default **on**) |
| `~/diff/image` | `sensor_msgs/Image` | `32FC1` or `bgr8` | `enable_diff:=true` |
| `~/camera_info` | `sensor_msgs/CameraInfo` | — | always |

### Diff stream modes

| `diff_mode` | SDK mode | Description |
|---|---|---|
| `single` | `SingleInference` | Reference frame is inferred **once at startup**. |
| `continuous` | `PerFrameInference` | Reference frame is inferred **on every frame**. |

### Stream dependencies

Enabling diff automatically adds the upstream rectified stream:

```
enable_raw               → Raw
enable_rectified         → Rectified
enable_diff              → Rectified + Diff
```

---

## Parameters

See [`xense_camera/config/xense_params.yaml`](xense_camera/config/xense_params.yaml) for the full list with descriptions.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `device_serial` | string | `""` | Sensor serial (empty = auto-detect) |
| `device_index` | int | `-1` | V4L2 index (-1 = auto-detect) |
| `enable_raw` | bool | `false` | Publish raw camera frames |
| `enable_rectified` | bool | `true` | Publish rectified frames |
| `enable_diff` | bool | `false` | Publish difference image on `~/diff/image` |
| `diff_mode` | string | `single` | `single` or `continuous` |
| `inference_backend` | string | `Auto` | Inference backend (`Auto`, `CPU`, `ONNX`, `MIGraphX`, …) |
| `use_gpu` | bool | `true` | Use GPU for inference |
| `camera_frame_id` | string | `xense_camera_link` | TF frame ID |
| `publish_tf` | bool | `true` | Publish world→camera static TF |

---

## Monitoring publish rate

The node logs the actual publish rate every 3 seconds at INFO level:

```
[fps] Node publish rate: 30.00 Hz (90 frames / 3.0 s)
```

Use this — or `ros2 topic hz /xense_camera/camera_info` — to verify frame rate. Avoid running `ros2 topic hz` directly on image topics: the Python subscriber may drop messages due to deserialisation overhead on large images (840 KB per frame at 30 Hz), producing a falsely low reading.

---

## Custom Messages (xense_msgs)

Stub definitions for future marker and force streaming support:

- `xense_msgs/ForceVector` — 3D force vector with magnitude
- `xense_msgs/ForceArray` — Stamped array of force vectors + resultant
- `xense_msgs/MarkerArray2D` — Stamped 2D marker grid positions
- `xense_msgs/MarkerArray3D` — Stamped 3D marker grid positions

---

## License

Apache-2.0
