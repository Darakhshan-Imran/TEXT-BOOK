---
sidebar_position: 16
title: "باب 16: Voice-to-Action اور Whisper"
---

# باب 16: Voice-to-Action اور Whisper

## تعارف

Voice-to-action systems انسان اور روبوٹ کے درمیان ایک اہم interface ہیں۔ یہ باب audio input processing سے speech-to-text conversion، natural language understanding، اور command execution تک کے مکمل pipeline کو تلاش کرتا ہے۔

## بنیادی تصور 1: Speech Recognition

Whisper OpenAI کا multilingual speech recognition model ہے:

```python
import whisper
import numpy as np

class VoiceCommandProcessor:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.command_map = {}

    def transcribe_audio(self, audio_path):
        """Audio file کو text میں تبدیل کریں"""
        result = self.model.transcribe(audio_path)
        return result["text"]

    def register_command(self, phrase, action):
        """Voice command register کریں"""
        self.command_map[phrase.lower()] = action

    def process_command(self, text):
        """Text سے command شناخت کریں"""
        text_lower = text.lower()
        for phrase, action in self.command_map.items():
            if phrase in text_lower:
                return action
        return None
```

## بنیادی تصور 2: ROS 2 Integration

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        self.publisher = self.create_publisher(
            String, 'voice_commands', 10
        )
        self.get_logger().info('Voice control node started')

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
```

## بنیادی تصور 3: Command Mapping

Voice commands کو robotic actions میں تبدیل کرنا:
- **Intent Recognition**: صارف کا ارادہ سمجھنا
- **Entity Extraction**: اہم معلومات نکالنا
- **Action Planning**: عمل کی منصوبہ بندی

## اہم نکات

- Whisper multilingual speech recognition فراہم کرتا ہے
- Voice interfaces natural human-robot interaction ممکن بناتے ہیں
- Real-time processing low latency درکار ہے
- Safety commands کو priority ملنی چاہیے

## اگلے باب کا پیش نظارہ

باب 17 LLMs کو robot brains کے طور پر استعمال کرنے کو تلاش کرے گا۔

