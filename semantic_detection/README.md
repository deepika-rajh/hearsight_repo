# semantic_detection

Standalone ROS 2 node that runs YOLOv8 object detection. Subscribes directly
to the RealSense driver's RGB color topic (`/camera/camera/color/image_raw`)
-- not via OKVIS -- and publishes `vision_msgs/Detection2DArray` plus an
annotated `~/debug_image` (boxes + class labels drawn on the frame -- add
an rviz2 Image display on that topic to see detections visually). No
dependency on OKVIS core -- ROS topics are the only integration point, so it
runs the same way against a live camera or a `ros2 bag play` recording.
Model load time and, per frame, the camera interval, preprocess/inference/
postprocess/total timings, and every detection's label/score/box are logged
line-by-line via `RCLCPP_INFO`.

## Layout

- `include/semantic_detection/detector/` -- `YOLODetector`: backend-agnostic
  letterbox preprocessing, output decoding, NMS. `Detection`: the result type.
- `include/semantic_detection/backends/` -- `InferenceBackend` interface,
  `CpuBackend` (OpenCV DNN, full-precision ONNX), `QnnHtpBackend` (Qualcomm
  QNN, HTP-quantized).
- `include/semantic_detection/utils/` -- `ImageUtils` (letterbox), `NMS`
  (per-class non-max suppression), `YamlParser` (Ultralytics data.yaml class
  names).
- `include/semantic_detection/ros/` -- `YoloNode`, the rclcpp node. `src/main.cpp`
  is just the entry point.
- `models/` -- model artifacts (ONNX export, QNN context binary/model
  library, class names yaml). Not meant for git; copy your own here.
- `config/yolo_params.yaml` -- all runtime parameters, including the
  `backend`-specific ones. See comments inline for what each means and
  where the HTP values came from.

## Backends

- `backend: "cpu"` -- OpenCV DNN running the ONNX export. Currently fails to
  load on this workspace's OpenCV 4.5.4 (Ultralytics' baked-in DFL decode
  subgraph trips the importer's shape inference); deprioritized in favor of
  the HTP path.
- `backend: "htp"` -- Qualcomm QNN, loading a pre-baked context binary
  (`qnn-context-binary-generator` output) once at startup and calling
  `graphExecute()` per frame in-process. Requires `QNN_SDK_ROOT` at
  configure time (see `CMakeLists.txt`); falls back to `cpu` at runtime if
  requested but not compiled in.

## Build

Native (CPU backend only, unless `-DQNN_SDK_ROOT` points at a sysroot with
the QNN headers/libs):

```
colcon build --packages-select semantic_detection
```

Cross-compiled for the QCS6490 devkit: see your QIR eSDK cross-compile
workflow; add `-DQNN_SDK_ROOT=$SDKTARGETSYSROOT/usr` to the existing
`--cmake-args` to build the HTP backend.
