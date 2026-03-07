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
- ROS2 dependencies:
  ```bash
  sudo apt install -y \
    ros-jazzy-rclcpp \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-tf2-ros \
    ros-jazzy-camera-info-manager
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

### Enable depth pipeline

```bash
ros2 launch xense_camera xense_launch.py enable_diff:=true enable_depth:=true
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

| Topic | Type | Encoding | Enabled by |
|---|---|---|---|
| `~/raw/image_raw` | `sensor_msgs/Image` | `bgr8` | `enable_raw:=true` |
| `~/rectified/image` | `sensor_msgs/Image` | `bgr8` | `enable_rectified` (default **on**) |
| `~/diff/image` | `sensor_msgs/Image` | `32FC1` | `enable_diff:=true` |
| `~/depth/image` | `sensor_msgs/Image` | `32FC1` | `enable_depth:=true` |
| `~/camera_info` | `sensor_msgs/CameraInfo` | — | always |

> **Note:** Depth values are in the range **[0.0, 1.0]** representing normalised tactile gel deformation — not metric scene depth.

### Stream dependencies

Enabling a stream automatically includes any upstream streams it requires:

```
enable_raw      → Raw
enable_rectified → Rectified
enable_diff     → Rectified + Diff
enable_depth    → Rectified + Diff + Depth
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
| `enable_diff` | bool | `false` | Publish difference image |
| `enable_depth` | bool | `false` | Publish tactile depth map |
| `diff_mode` | string | `SingleInference` | `SingleInference` or `PerFrameInference` |
| `inference_backend` | string | `Auto` | Inference backend |
| `use_gpu` | bool | `true` | Use GPU for inference |
| `camera_frame_id` | string | `xense_camera_link` | TF frame ID |
| `publish_tf` | bool | `true` | Publish world→camera static TF |

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
