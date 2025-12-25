---
sidebar_position: 5
title: "باب 5: URDF کے ساتھ روبوٹس کی تفصیل"
---

# باب 5: URDF کے ساتھ روبوٹس کی تفصیل

## تعارف

Unified Robot Description Format (URDF) ROS میں روبوٹ ماڈلز کی نمائندگی کے لیے معیاری XML پر مبنی فارمیٹ ہے۔ URDF روبوٹ kinematics، dynamics، بصری ظاہری شکل، اور جسمانی خصوصیات کی درست تفصیل ممکن بناتا ہے، جس سے ROS پر مبنی سسٹمز میں روبوٹس کو simulate، visualize، اور کنٹرول کرنا ممکن ہو جاتا ہے۔

URDF ROS میں روبوٹ ماڈلز کے لیے بنیادی نمائندگی کا کام کرتا ہے، روبوٹ geometry، kinematic chains، اور جسمانی خصوصیات کی وضاحت کے لیے ایک معیاری طریقہ فراہم کرتا ہے۔ یہ فارمیٹ متعدد degrees of freedom، sensors، اور actuators والے پیچیدہ روبوٹس کی تفصیل کی اجازت دیتا ہے۔

## بنیادی تصور 1: بنیادی URDF ساخت اور اجزاء

URDF ایک XML پر مبنی فارمیٹ ہے جو روبوٹ ماڈلز کو ان کے links، joints، اور متعلقہ خصوصیات کی تعریف کرکے بیان کرتا ہے۔ URDF کے بنیادی تعمیراتی بلاکس links اور joints ہیں، جو مل کر روبوٹ کی kinematic ساخت بناتے ہیں۔

```xml
<!-- مثال 1: Links اور Joints کے ساتھ بنیادی روبوٹ ماڈل -->
<?xml version="1.0"?>
<robot name="simple_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link - روبوٹ کی جڑ -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- پہلا بازو لنک -->
  <link name="arm_link_1">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
      <origin xyz="0 0 0.25"/>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
      <origin xyz="0 0 0.25"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.25"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Base سے پہلے بازو تک Joint -->
  <joint name="base_to_arm_1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link_1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="1.0"/>
  </joint>

</robot>
```

## بنیادی تصور 2: Joint کی اقسام اور خصوصیات

URDF متعدد joint اقسام کی حمایت کرتا ہے، ہر ایک مختلف kinematic تعلقات کی نمائندگی کرتا ہے:

- **Revolute**: گھومنے والا joint جو ایک محور کے گرد گردش کی اجازت دیتا ہے (حدود کے ساتھ)
- **Continuous**: گھومنے والا joint جو لامحدود گردش کی اجازت دیتا ہے
- **Prismatic**: سلائیڈنگ joint جو ایک محور کے ساتھ ترجمہ کی اجازت دیتا ہے
- **Fixed**: کوئی حرکت نہیں - links کو سختی سے جوڑتا ہے
- **Floating**: چھ degrees of freedom کی اجازت دیتا ہے
- **Planar**: ایک جہاز میں حرکت کی اجازت دیتا ہے

```xml
<!-- مثال 2: مختلف Joint اقسام -->
<!-- Revolute joint - گھومنے والا joint (محدود) -->
<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="upper_arm"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  <dynamics damping="0.1" friction="0.05"/>
</joint>

<!-- Prismatic joint - سلائیڈنگ joint -->
<joint name="lift_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="elevator"/>
  <origin xyz="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0.0" upper="0.5" effort="50" velocity="0.5"/>
</joint>

<!-- Fixed joint - مقررہ کنکشن -->
<joint name="sensor_mount" type="fixed">
  <parent link="end_effector"/>
  <child link="camera_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
```

## بنیادی تصور 3: Xacro میکروز

Xacro (XML Macros) URDF کو ماڈیولر، قابل دیکھ بھال کوڈ کے لیے میکروز، پیرامیٹرز، اور conditional logic کے ساتھ بڑھاتا ہے:

```xml
<!-- مثال 3: Xacro میکروز -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="modular_robot">

  <!-- پیرامیٹرز کی تعریف -->
  <xacro:property name="arm_length" value="0.5"/>
  <xacro:property name="arm_radius" value="0.05"/>

  <!-- دوبارہ قابل استعمال میکرو -->
  <xacro:macro name="arm_segment" params="name length radius parent">
    <link name="${name}_link">
      <visual>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
      </visual>
    </link>
    <joint name="${name}_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${name}_link"/>
      <axis xyz="0 0 1"/>
      <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
    </joint>
  </xacro:macro>

  <!-- میکرو کا استعمال -->
  <xacro:arm_segment name="arm_1" length="${arm_length}" radius="${arm_radius}" parent="base_link"/>
  <xacro:arm_segment name="arm_2" length="${arm_length * 0.8}" radius="${arm_radius}" parent="arm_1_link"/>

</robot>
```

## بنیادی تصور 4: Sensors اور Gazebo Extensions

URDF کو Gazebo plugins کے ذریعے sensor تعریفیں اور simulation خصوصیات شامل کرنے کے لیے بڑھایا جا سکتا ہے:

```xml
<!-- مثال 4: Sensors اور Gazebo Integration -->
<!-- کیمرہ سینسر -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
</link>

<gazebo reference="camera_link">
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
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=image</remapping>
      </ros>
      <camera_name>front_camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## عام غلطیاں

1. **Inertia Values**: غلط inertia values غیر مستحکم simulation رویے کا سبب بن سکتی ہیں
2. **Joint Limits**: جوڑوں کی حدود کی غلط تعریف خود collision یا غیر حقیقی حرکات کا باعث بن سکتی ہے
3. **Collision Geometry**: پیچیدہ collision geometry simulation کی کارکردگی کو سست کر سکتی ہے
4. **Mass Distribution**: غیر حقیقی mass تقسیم فزکس simulation کو متاثر کرتی ہے

## حقیقی دنیا کی ایپلیکیشنز

URDF روبوٹک ایپلیکیشنز کی وسیع رینج میں استعمال ہوتا ہے:
- صنعتی manipulation کے لیے روبوٹ بازو
- موبائل روبوٹ پلیٹ فارمز
- ہیومنائیڈ روبوٹس
- ڈرونز اور aerial vehicles
- سرجیکل روبوٹس

## مشقیں

1. ایک سادہ 2-DOF روبوٹ بازو ماڈل بنائیں
2. کیمرہ اور LIDAR sensors شامل کریں
3. Xacro استعمال کرتے ہوئے دوبارہ قابل استعمال میکروز بنائیں

## اہم نکات

- URDF روبوٹ ماڈلز کے لیے ROS کا معیاری فارمیٹ ہے
- Links اور joints روبوٹ ساخت کی بنیاد بناتے ہیں
- Xacro میکروز ماڈیولر اور قابل دیکھ بھال کوڈ ممکن بناتے ہیں
- Gazebo plugins simulation خصوصیات فراہم کرتے ہیں
- مناسب inertial خصوصیات درست simulation کے لیے اہم ہیں

## اگلے باب کا پیش نظارہ

باب 6 ROS 2 Launch فائلز کو تلاش کرے گا، یہ سیکھے گا کہ پیچیدہ multi-node سسٹمز کو کیسے configure اور شروع کیا جائے۔
