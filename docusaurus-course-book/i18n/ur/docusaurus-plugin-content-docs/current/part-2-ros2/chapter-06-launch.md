---
sidebar_position: 6
title: "باب 6: Launch فائلز اور سسٹم انضمام"
---

# باب 6: Launch فائلز اور سسٹم انضمام

## تعارف

ROS 2 launch سسٹم پیچیدہ روبوٹک ایپلیکیشنز کو منظم اور شروع کرنے کے لیے ایک طاقتور فریم ورک فراہم کرتا ہے۔ Launch فائلز متعدد نوڈز، پیرامیٹرز، اور configurations کو ایک ہی مربوط سسٹم میں مربوط کرنے کے قابل بناتی ہیں۔

## بنیادی تصور 1: Launch فائل کی بنیادی ساخت

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_node',
            output='screen'
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener_node',
            output='screen'
        )
    ])
```

## بنیادی تصور 2: پیرامیٹرز اور Arguments

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='my_robot',
        description='روبوٹ کا نام'
    )

    robot_node = Node(
        package='my_robot_pkg',
        executable='robot_controller',
        name=LaunchConfiguration('robot_name'),
        parameters=[{
            'max_velocity': 1.0,
            'safety_distance': 0.5
        }],
        output='screen'
    )

    return LaunchDescription([robot_name_arg, robot_node])
```

## بنیادی تصور 3: Conditional Launching

```python
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

debug_node = Node(
    package='my_robot_pkg',
    executable='debug_monitor',
    condition=IfCondition(LaunchConfiguration('debug')),
    output='screen'
)
```

## اہم نکات

- ROS 2 launch فائلز Python پر مبنی ہیں
- Launch arguments لچکدار configuration ممکن بناتے ہیں
- Conditional launching مختلف ترتیبات کی حمایت کرتا ہے
- Event handlers اعمال کو مربوط کرتے ہیں

## اگلے باب کا پیش نظارہ

باب 7 Gazebo simulation environment کو تلاش کرے گا۔
