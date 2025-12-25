---
sidebar_position: 18
title: "باب 18: Multi-Modal Interaction"
---

# باب 18: Multi-Modal Interaction

## تعارف

Multi-modal interaction human-robot communication کے مستقبل کی نمائندگی کرتا ہے۔ یہ باب visual، auditory، tactile، اور دیگر sensory inputs کو روبوٹک سسٹمز میں ملانے کو تلاش کرتا ہے۔

## بنیادی تصور 1: Multi-Modal Perception

```python
import numpy as np
from dataclasses import dataclass
from typing import Optional

@dataclass
class SensorData:
    visual: Optional[np.ndarray] = None
    audio: Optional[np.ndarray] = None
    tactile: Optional[np.ndarray] = None
    timestamp: float = 0.0

class MultiModalFusion:
    def __init__(self):
        self.modality_weights = {
            'visual': 0.5,
            'audio': 0.3,
            'tactile': 0.2
        }

    def fuse_inputs(self, sensor_data: SensorData):
        """Multiple modalities کو fuse کریں"""
        features = []

        if sensor_data.visual is not None:
            visual_features = self.process_visual(sensor_data.visual)
            features.append(('visual', visual_features))

        if sensor_data.audio is not None:
            audio_features = self.process_audio(sensor_data.audio)
            features.append(('audio', audio_features))

        return self.weighted_fusion(features)
```

## بنیادی تصور 2: Sensor Fusion Techniques

مختلف sensors سے ڈیٹا ملانا:
- **Early Fusion**: Raw data level پر fusion
- **Late Fusion**: Feature level پر fusion
- **Attention-based Fusion**: Dynamic weighting

## بنیادی تصور 3: ROS 2 Multi-Modal Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from audio_msgs.msg import Audio

class MultiModalNode(Node):
    def __init__(self):
        super().__init__('multi_modal_node')

        self.image_sub = self.create_subscription(
            Image, 'camera/image', self.image_callback, 10
        )

        self.fusion = MultiModalFusion()

    def image_callback(self, msg):
        # Visual processing
        pass
```

## اہم نکات

- Multi-modal systems زیادہ robust ہیں
- Sensor fusion accuracy بڑھاتا ہے
- Temporal alignment critical ہے
- Modality failures کو handle کرنا ضروری ہے

## اگلے باب کا پیش نظارہ

باب 19 capstone project کو تلاش کرے گا جہاں تمام concepts ملائے جائیں گے۔

