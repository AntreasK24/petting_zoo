# mirte_apple_detector

A ROS 2 **C++** node (`rclcpp`) that detects **red** and **green apples** in a
compressed image stream using **classical colour + shape analysis** — no neural
network, no model to download. Red and green apples are classified separately.

Humans are handled by the separate `mirte_human_detector` package.

## How it works

Per frame: decode → blur → HSV → threshold a colour mask (red wraps hue 0, so
two ranges; green is one) → morphological clean-up → find contours → keep the
ones that are round (circularity), solid (solidity) and large enough (area) →
publish a bounding box per surviving blob.

## Topics

**Subscribes**
- `/camera/image_raw/compressed` (`sensor_msgs/CompressedImage`)

**Publishes**
- `detections/apples/red` (`vision_msgs/BoundingBox2DArray`) — red apples
- `detections/apples/green` (`vision_msgs/BoundingBox2DArray`) — green apples
- `detections/apples/annotated` (`sensor_msgs/Image`) — red/green boxes drawn.

## Build & run

```bash
cd ~/dev_ws
colcon build --packages-select mirte_apple_detector
source install/setup.bash
ros2 launch mirte_apple_detector apple_detector.launch.xml
```

Feed it a camera (e.g. `camera_ros`) publishing on
`/camera/image_raw/compressed`.

View:

```bash
ros2 run rqt_image_view rqt_image_view /detections/apples/annotated
```

## Tuning

Everything is in [`config/apple_detector.yaml`](config/apple_detector.yaml):

- **Colour** (`red` / `green`): each colour has `hue_ranges` (a flat list of
  `[lo, hi, ...]` hue pairs, 0-179) plus `sat_min` / `val_min` floors. Red
  defaults to `[0, 10, 170, 179]` with a high `sat_min` — strict, pure red only.
  Missed apples → widen a hue range or lower `sat_min` / `val_min`; other
  objects leaking in → tighten them. (Hue cheat-sheet: red 0-10 & 170-179,
  orange 11-25, yellow 25-34, green 35-85.)
- **Shape**: `min_circularity` (roundness, 1.0 = perfect circle), `min_solidity`
  (how filled-in), `min_area` / `max_area` (size in pixels).
- **Clean-up**: `blur_ksize`, `morph_ksize`.
- **Rate**: `max_rate` (Hz).

> Pure colour/shape is lighting-sensitive. Tune the HSV ranges under your
> actual lighting (a quick way: inspect pixel HSV values on a sample image).
