---
sidebar_position: 8
title: "باب 8: سینسر Simulation"
---

# باب 8: سینسر Simulation

## تعارف

سینسر simulation روبوٹکس ڈیولپمنٹ کا ایک اہم پہلو ہے۔ Gazebo میں مختلف قسم کے سینسرز کو simulate کیا جا سکتا ہے جیسے کیمرے، LIDAR، IMU، اور depth sensors۔

## بنیادی تصور 1: کیمرہ Simulation

```xml
<sensor type="camera" name="front_camera">
  <update_rate>30.0</update_rate>
  <camera name="head">
    <horizontal_fov>1.3962634</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <frame_name>camera_link</frame_name>
  </plugin>
</sensor>
```

## بنیادی تصور 2: LIDAR Simulation

```xml
<sensor type="ray" name="lidar">
  <update_rate>10.0</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <frame_name>lidar_link</frame_name>
  </plugin>
</sensor>
```

## بنیادی تصور 3: IMU Simulation

```xml
<sensor type="imu" name="imu_sensor">
  <update_rate>100.0</update_rate>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <frame_name>imu_link</frame_name>
  </plugin>
</sensor>
```

## اہم نکات

- سینسر simulation حقیقی ہارڈویئر کے بغیر الگورتھم ٹیسٹنگ ممکن بناتا ہے
- Gazebo متعدد سینسر اقسام کی حمایت کرتا ہے
- سینسر noise اور غیر کاملیت کو simulate کیا جا سکتا ہے
- ROS topics پر سینسر ڈیٹا شائع ہوتا ہے

## اگلے باب کا پیش نظارہ

باب 9 Unity کے ساتھ high-fidelity visualization کو تلاش کرے گا۔
