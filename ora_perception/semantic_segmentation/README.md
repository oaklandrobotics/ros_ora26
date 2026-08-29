# Semantic Segmentation Node

ROS2 node for real-time semantic segmentation inference using ONNX Runtime.

## Overview

This node performs semantic segmentation on camera images and publishes segmentation masks, confidence maps, and colored overlays. It uses ONNX Runtime for efficient inference without requiring PyTorch or super-gradients at runtime.

## Quick Start
This package uses Git LFS to store the ONNX model. Pull the model with:
```bash
git lfs pull --include="ora_perception/semantic_segmentation/models/model.onnx"
```

You can now run the project with semantic segmentation using:
```bash
ros2 launch ora_launch robot.launch.py use_sim_time:=true
```

## Disabling Semantic Segmentation
The semantic segmentation node is enabled by default as it's configuration is resource intensive and requires CUDA. Alternatively, you can disable with:
```bash
ros2 launch ora_launch robot.launch.py use_semantic_segmentation:=false
```

You can disable semantic segmentation persistantly by adding an environment variable:
```bash
echo "export USE_SEMANTIC_SEGMENTATION=false" >> ~/.bashrc

source ~/.bashrc
```

## Topics

**Subscribed:**
- `/masked/image` (sensor_msgs/Image) - Input RGB camera images

**Published:**
- `/segmentation/mask` (sensor_msgs/Image) - Segmentation mask with class IDs (mono8)
- `/segmentation/confidence` (sensor_msgs/Image) - Per-pixel confidence (mono8, 0-255)
- `/segmentation/overlay` (sensor_msgs/Image) - Colored overlay visualization (bgr8)
- `/segmentation/label_info` (vision_msgs/LabelInfo) - Class mappings (latched)

## Model

The ONNX model (`models/model.onnx`) can be generated using the [Simple Segmentation Toolkit](https://github.com/pepisg/simple_segmentation_toolkit).

### Training Your Own Model

1. Capture training images from a real robot or from Gazebo, with varying lighting and environmental conditions
2. Use the Simple Segmentation Toolkit to label and train a model
3. Convert the trained model to ONNX format: `python3 convert_to_onnx.py`
4. Copy `model.onnx` to this package's `models/` directory

The ontology configuration (`config/ontology.yaml`) must match the classes used during training.

## Usage

```bash
ros2 run semantic_segmentation segmentation_node
```

All dependencies are included in the devcontainer.

