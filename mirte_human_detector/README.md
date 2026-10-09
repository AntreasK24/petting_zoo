# mirte_human_detector

A ROS 2 node that detects **humans** in a compressed image stream using
**YOLO-FastestV2** on the **NCNN** runtime. CPU-only (no GPU), ~0.5 MB model,
tuned for small ARM boards such as the Orange Pi 3 (quad Cortex-A53).

Apples are handled by the separate `mirte_apple_detector` package.

## Topics

**Subscribes**
- `/camera/image_raw/compressed` (`sensor_msgs/CompressedImage`)

**Publishes**
- `detections/humans` (`vision_msgs/BoundingBox2DArray`)
- `detections/humans/annotated` (`sensor_msgs/Image`) — input with green boxes
  drawn (toggle via `publish_debug_image`).

## Install & build

```bash
# model (~0.5 MB) + ncnn runtime
bash src/mirte_human_detector/scripts/download_model.sh
pip install --no-deps ncnn
cd ~/dev_ws
colcon build --packages-select mirte_human_detector
source install/setup.bash
```

## Run

```bash
ros2 launch mirte_human_detector human_detector.launch.xml
```

Feed it a camera (e.g. `camera_ros`) publishing on
`/camera/image_raw/compressed`.

View:

```bash
ros2 run rqt_image_view rqt_image_view /detections/humans/annotated
```

## Configure

Everything is in [`config/human_detector.yaml`](config/human_detector.yaml):

- `max_rate` — cap detection Hz (dropped frames skip decode; frees CPU).
- `ncnn_input_size` — multiple of 32. Lower = faster (352→224 ~2×, 352→192 ~2.5×).
- `num_threads` — CPU cores to use.
- `confidence_threshold` — raise to be stricter.
- `publish_debug_image` — turn the annotated image on/off.
