---
sidebar_position: 12
title: "باب 12: Isaac ROS Hardware-Accelerated Perception"
---

# باب 12: Isaac ROS Hardware-Accelerated Perception

## تعارف

Isaac ROS NVIDIA کے GPU-accelerated perception packages کا مجموعہ ہے جو ROS 2 کے ساتھ مکمل طور پر compatible ہے۔ یہ پیکجز real-time perception tasks کے لیے بہتر کارکردگی فراہم کرتے ہیں۔

## بنیادی تصور 1: Isaac ROS Packages

- **Isaac ROS Image Processing**: GPU-accelerated image operations
- **Isaac ROS DNN Inference**: TensorRT-based neural network inference
- **Isaac ROS Visual SLAM**: Real-time localization and mapping
- **Isaac ROS Depth Segmentation**: 3D scene understanding

## بنیادی تصور 2: GPU-Accelerated Vision

```python
# Isaac ROS کے ساتھ GPU-accelerated perception
from isaac_ros_dnn_image_encoder import DnnImageEncoderNode

encoder_node = DnnImageEncoderNode(
    parameters={
        'input_image_width': 640,
        'input_image_height': 480,
        'network_image_width': 224,
        'network_image_height': 224
    }
)
```

## بنیادی تصور 3: Visual SLAM

Isaac ROS Visual SLAM stereo cameras کا استعمال کرتے ہوئے real-time localization فراہم کرتا ہے:
- GPU-accelerated feature extraction
- Real-time pose estimation
- Loop closure detection
- 3D reconstruction

## اہم نکات

- Isaac ROS GPU acceleration استعمال کرتا ہے
- Real-time perception ممکن ہے
- ROS 2 کے ساتھ مکمل integration
- TensorRT inference optimization

## اگلے باب کا پیش نظارہ

باب 13 Nav2 navigation stack کو تلاش کرے گا۔
