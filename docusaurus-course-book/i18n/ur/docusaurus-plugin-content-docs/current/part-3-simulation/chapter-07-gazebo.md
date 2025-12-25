---
sidebar_position: 7
title: "باب 7: Gazebo Physics Simulation"
---

# باب 7: Gazebo Physics Simulation

## تعارف

Gazebo ایک طاقتور، اوپن سورس 3D روبوٹ simulator ہے جو ROS کے ساتھ بغیر کسی رکاوٹ کے integrate ہوتا ہے۔ یہ حقیقت پسندانہ physics simulation، sensor simulation، اور ماحولیاتی ماڈلنگ فراہم کرتا ہے۔

## بنیادی تصور 1: Gazebo World Files

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="simple_world">
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## بنیادی تصور 2: ROS 2 Integration

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ])
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot']
    )

    return LaunchDescription([gazebo, spawn_robot])
```

## بنیادی تصور 3: Gazebo Plugins

Gazebo plugins روبوٹ صلاحیتوں کو بڑھاتے ہیں جیسے differential drive، sensors، اور controllers۔

## اہم نکات

- Gazebo حقیقت پسندانہ physics simulation فراہم کرتا ہے
- SDF فارمیٹ worlds اور ماڈلز بیان کرتا ہے
- ROS 2 integration gazebo_ros packages کے ذریعے ہے
- Plugins روبوٹ صلاحیتوں کو بڑھاتے ہیں

## اگلے باب کا پیش نظارہ

باب 8 sensor simulation کو تفصیل سے تلاش کرے گا۔
