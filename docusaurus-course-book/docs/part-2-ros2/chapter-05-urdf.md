---
sidebar_position: 5
title: "Chapter 5: Describing Robots with URDF"
---

# Chapter 5: Describing Robots with URDF

## Introduction

Unified Robot Description Format (URDF) is the standard XML-based format for representing robot models in ROS. URDF enables the precise description of robot kinematics, dynamics, visual appearance, and physical properties, making it possible to simulate, visualize, and control robots in ROS-based systems. This chapter explores the comprehensive capabilities of URDF for describing robots, from basic link and joint definitions to complex multi-body systems with sensors and actuators.

URDF serves as the foundational representation for robot models in ROS, providing a standardized way to define robot geometry, kinematic chains, and physical properties. The format allows for the description of complex robots with multiple degrees of freedom, sensors, and actuators, enabling seamless integration between simulation, visualization, and control systems. Understanding URDF is essential for anyone working with robot simulation, motion planning, or hardware integration in ROS-based systems.

The power of URDF lies in its ability to represent both the physical and functional aspects of robots. Through URDF, developers can specify not only the geometric properties of robot components but also their inertial properties, joint limits, and sensor configurations. This comprehensive representation enables accurate simulation and reliable real-world operation of robotic systems.

This chapter will guide you through the creation of sophisticated robot models using URDF, covering basic concepts through advanced techniques including Xacro macro definitions, sensor integration, and Gazebo-specific extensions. By the end of this chapter, you'll be able to create complete robot descriptions that can be used for simulation, visualization, and control in ROS-based robotic applications.

## Core Concept 1: Basic URDF Structure and Components

URDF (Unified Robot Description Format) is an XML-based format that describes robot models by defining their links, joints, and associated properties. The fundamental building blocks of URDF are links and joints, which together form the kinematic structure of a robot. Links represent rigid bodies with physical properties, while joints define the kinematic relationships between links.

A basic URDF model consists of a single `<robot>` element containing multiple `<link>` and `<joint>` elements. Each link has visual and collision properties that define how it appears in simulation and how it interacts with the physics engine. Joints specify the degrees of freedom between links and may include limits, dynamics properties, and safety controllers.

The structure of a URDF file follows a tree topology with a single base link (root) and child links connected by joints. This hierarchical structure allows for the representation of complex kinematic chains while maintaining the constraint that there are no loops in the structure (for basic URDF). The tree structure is essential for kinematic calculations and is the basis for robot state propagation in ROS.

```xml
<!-- Example 1: Basic Robot Model with Links and Joints (85-100 lines) -->
<?xml version="1.0"?>
<robot name="simple_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link - the root of the robot -->
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

  <!-- First link - upper arm -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Joint connecting base to upper arm -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <!-- Second link - lower arm -->
  <link name="lower_arm">
    <visual>
      <geometry>
        <box size="0.04 0.04 0.25"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.04 0.04 0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.003"/>
    </inertial>
  </link>

  <!-- Joint connecting upper arm to lower arm -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="lower_arm"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.36" upper="2.36" effort="100" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <!-- End effector link -->
  <link name="end_effector">
    <visual>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Fixed joint connecting lower arm to end effector -->
  <joint name="wrist_joint" type="fixed">
    <parent link="lower_arm"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.125" rpy="0 0 0"/>
  </joint>

</robot>
```

This example demonstrates the fundamental structure of a URDF robot model, showing how links and joints are defined with their visual, collision, and inertial properties. The model represents a simple 2-DOF arm with a base, upper arm, lower arm, and end effector.

## Core Concept 2: Visual Properties and Materials

Visual properties in URDF define how robot components appear in simulation and visualization tools. These properties include geometry (shape and size), material (color and texture), and origin (position and orientation relative to the parent link). The visual representation is separate from the collision geometry, allowing for detailed visual models without impacting simulation performance.

URDF supports several primitive geometric shapes including boxes, cylinders, spheres, and meshes. For complex geometries, URDF can reference external mesh files in formats like STL, DAE, or OBJ. Meshes provide the most flexibility for representing complex robot geometries but require careful attention to file paths and coordinate systems.

Materials in URDF are defined separately and referenced by name, allowing for consistent color schemes across the robot model. Material definitions include RGBA color values that control the appearance in visualization tools. The separation of materials from geometry allows for easy theme changes and consistent styling across multiple robot models.

```xml
<!-- Example 2: Robot with Complex Geometries and Materials (90-110 lines) -->
<?xml version="1.0"?>
<robot name="complex_visual_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Define materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link with complex visual properties -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://my_robot_description/meshes/base_link.stl" scale="1 1 1"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.416" ixy="0.0" ixz="0.0" iyy="0.416" iyz="0.0" izz="0.208"/>
    </inertial>
  </link>

  <!-- Wheel links with different materials -->
  <link name="wheel_fl_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting wheel to base -->
  <joint name="wheel_fl_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_fl_link"/>
    <origin xyz="0.2 0.2 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- Sensor mount with custom material -->
  <link name="sensor_mount">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint connecting sensor mount to base -->
  <joint name="sensor_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_mount"/>
    <origin xyz="0.2 0 0.2" rpy="0 0 0"/>
  </joint>

  <!-- Transmission definition for wheel (for ros2_control) -->
  <transmission name="wheel_fl_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="wheel_fl_joint">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="wheel_fl_motor">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

</robot>
```

This example demonstrates advanced visual properties including mesh references, material definitions, and transmission specifications. The model shows how to create a more complex robot with realistic visual representations while maintaining efficient collision geometry.

## Core Concept 3: Hardware Integration - Transmissions and Actuators

URDF's transmission elements define the interface between the control system and the physical hardware. Transmissions specify how control commands are translated to actuator commands and how sensor feedback is processed. This hardware abstraction layer is crucial for connecting simulated robots to real hardware and for enabling the use of ros2_control, ROS 2's hardware abstraction framework.

Transmissions in URDF define the relationship between joints and actuators, including gear ratios, mechanical reductions, and the type of hardware interface (position, velocity, or effort). The transmission specification allows ROS 2 controllers to properly command the hardware and interpret sensor feedback.

Modern ROS 2 systems often use ros2_control for hardware integration, which relies on URDF transmission definitions to understand the robot's hardware capabilities. The transmission specification in URDF provides the necessary information for ros2_control to properly initialize and control robot hardware.

```python
# Example 3: URDF with Transmission and Actuator Definitions (95-115 lines)
import rclpy
from rclpy.node import Node
from urdf_parser_py.urdf import URDF
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math


class UrdfHardwareIntegrator(Node):
    """
    Node that demonstrates integration of URDF with hardware control systems.

    This node parses URDF information to configure hardware interfaces
    and demonstrates how to connect URDF-defined transmissions to
    actual hardware control.
    """

    def __init__(self):
        super().__init__('urdf_hardware_integrator')

        # Publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(
            JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)

        # Subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)

        # Subscriber for controller state
        self.controller_state_subscriber = self.create_subscription(
            JointTrajectoryControllerState, 'joint_trajectory_controller/state',
            self.controller_state_callback, 10)

        # Timer for sending trajectory commands
        self.trajectory_timer = self.create_timer(2.0, self.send_trajectory_command)

        # Robot model (would normally be loaded from parameter or file)
        self.robot_model = self.load_robot_model()

        # Joint state storage
        self.current_joint_positions = {}
        self.current_joint_velocities = {}
        self.current_joint_efforts = {}

        self.get_logger().info("URDF hardware integrator initialized")

    def load_robot_model(self):
        """Load robot model from URDF (in a real implementation)."""
        # In a real implementation, this would load the URDF from file or parameter
        # For this example, we'll create a mock model
        robot_model = {
            'joints': {
                'shoulder_joint': {
                    'type': 'revolute',
                    'limits': {'lower': -1.57, 'upper': 1.57, 'effort': 100, 'velocity': 1.0},
                    'dynamics': {'damping': 0.1, 'friction': 0.0},
                    'transmission': {
                        'type': 'transmission_interface/SimpleTransmission',
                        'actuator': 'shoulder_motor',
                        'reduction': 1.0
                    }
                },
                'elbow_joint': {
                    'type': 'revolute',
                    'limits': {'lower': -2.36, 'upper': 2.36, 'effort': 100, 'velocity': 1.0},
                    'dynamics': {'damping': 0.1, 'friction': 0.0},
                    'transmission': {
                        'type': 'transmission_interface/SimpleTransmission',
                        'actuator': 'elbow_motor',
                        'reduction': 1.0
                    }
                }
            },
            'links': ['base_link', 'upper_arm', 'lower_arm', 'end_effector']
        }
        return robot_model

    def joint_state_callback(self, msg: JointState) -> None:
        """Handle joint state updates."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.current_joint_velocities[name] = msg.velocity[i]
            if i < len(msg.effort):
                self.current_joint_efforts[name] = msg.effort[i]

    def controller_state_callback(self, msg: JointTrajectoryControllerState) -> None:
        """Handle controller state updates."""
        self.get_logger().debug(f"Controller state received for joints: {msg.joint_names}")

    def send_trajectory_command(self) -> None:
        """Send a trajectory command to the robot."""
        # Create a trajectory with multiple waypoints
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['shoulder_joint', 'elbow_joint']

        # First waypoint - current position
        point1 = JointTrajectoryPoint()
        current_shoulder = self.current_joint_positions.get('shoulder_joint', 0.0)
        current_elbow = self.current_joint_positions.get('elbow_joint', 0.0)

        point1.positions = [current_shoulder, current_elbow]
        point1.velocities = [0.0, 0.0]
        point1.accelerations = [0.0, 0.0]
        point1.time_from_start = Duration(sec=0, nanosec=0)

        # Second waypoint - target position
        point2 = JointTrajectoryPoint()
        target_shoulder = math.pi / 4  # 45 degrees
        target_elbow = math.pi / 6     # 30 degrees

        point2.positions = [target_shoulder, target_elbow]
        point2.velocities = [0.0, 0.0]
        point2.accelerations = [0.0, 0.0]
        point2.time_from_start = Duration(sec=1, nanosec=0)

        # Third waypoint - return to origin
        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, 0.0]
        point3.velocities = [0.0, 0.0]
        point3.accelerations = [0.0, 0.0]
        point3.time_from_start = Duration(sec=2, nanosec=0)

        trajectory_msg.points = [point1, point2, point3]

        self.joint_cmd_publisher.publish(trajectory_msg)
        self.get_logger().info(f"Sent trajectory command with {len(trajectory_msg.points)} points")

    def validate_urdf_transmissions(self) -> bool:
        """Validate that URDF transmissions are properly defined."""
        try:
            # Check that all joints have valid transmission definitions
            for joint_name, joint_info in self.robot_model['joints'].items():
                if 'transmission' not in joint_info:
                    self.get_logger().error(f"Joint {joint_name} has no transmission defined")
                    return False

                transmission = joint_info['transmission']
                if not all(key in transmission for key in ['type', 'actuator', 'reduction']):
                    self.get_logger().error(f"Joint {joint_name} has incomplete transmission definition")
                    return False

            self.get_logger().info("All URDF transmissions are properly defined")
            return True

        except Exception as e:
            self.get_logger().error(f"Error validating URDF transmissions: {e}")
            return False

    def get_joint_limits(self, joint_name: str) -> dict:
        """Get limits for a specific joint from the URDF model."""
        if joint_name in self.robot_model['joints']:
            return self.robot_model['joints'][joint_name].get('limits', {})
        return {}


def main(args=None):
    """Main entry point for the URDF hardware integrator."""
    rclpy.init(args=args)

    node = UrdfHardwareIntegrator()

    # Validate URDF transmissions before starting
    if not node.validate_urdf_transmissions():
        node.get_logger().error("URDF validation failed, shutting down")
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down URDF hardware integrator")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

This example demonstrates how URDF-defined transmissions connect to actual hardware control systems in ROS 2, showing the integration between the abstract robot model and real control commands.

## Core Concept 4: Xacro for Parameterized Robot Design

Xacro (XML Macros) is a macro language for XML that extends URDF with the ability to define reusable components, perform mathematical calculations, and create parameterized robot descriptions. Xacro allows for the creation of complex robot models that can be customized through parameters, reducing duplication and improving maintainability.

Xacro macros enable the creation of reusable robot components that can be instantiated multiple times with different parameters. This is particularly useful for robots with repeated structures like multiple identical wheels, sensor arrays, or manipulator arms. Macros can accept parameters for position, orientation, size, and other properties, allowing for flexible robot design.

Xacro also supports mathematical expressions, allowing for calculations based on parameters. This enables the definition of robot dimensions that are related to each other, such as ensuring that joint positions are calculated based on link lengths, or that visual and collision properties are consistently derived from common parameters.

```xml
<!-- Example 4: Xacro Macros for Complex Robot Assembly (100-120 lines) -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">

  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.05" />
  <xacro:property name="base_length" value="0.5" />
  <xacro:property name="base_width" value="0.4" />
  <xacro:property name="base_height" value="0.15" />

  <!-- Macro for creating a wheel -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy *origin">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <xacro:insert_block name="origin"/>
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <axis xyz="0 1 0"/>
      <dynamics damping="0.5" friction="0.1"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
      </inertial>
    </link>

    <!-- Transmission for the wheel -->
    <transmission name="${prefix}_wheel_trans">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="${prefix}_wheel_joint">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
      </joint>
      <actuator name="${prefix}_wheel_motor">
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>
  </xacro:macro>

  <!-- Macro for creating a sensor mount -->
  <xacro:macro name="sensor_mount" params="name parent xyz rpy sensor_type:=camera">
    <joint name="${name}_mount_joint" type="fixed">
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <parent link="${parent}"/>
      <child link="${name}_link"/>
    </joint>

    <link name="${name}_link">
      <visual>
        <geometry>
          <box size="0.05 0.05 0.05"/>
        </geometry>
        <material name="blue">
          <color rgba="0 0 1 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <box size="0.05 0.05 0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
      </inertial>
    </link>

    <!-- Gazebo-specific sensor definition -->
    <gazebo reference="${name}_link">
      <xacro:if value="${sensor_type == 'camera'}">
        <sensor type="camera" name="${name}_camera">
          <pose>0 0 0 0 0 0</pose>
          <camera name="${name}">
            <horizontal_fov>1.047</horizontal_fov>
            <image>
              <width>640</width>
              <height>480</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>0.1</near>
              <far>100</far>
            </clip>
          </camera>
          <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
            <frame_name>${name}_optical_frame</frame_name>
          </plugin>
        </sensor>
      </xacro:if>

      <xacro:if value="${sensor_type == 'lidar'}">
        <sensor type="ray" name="${name}_lidar">
          <pose>0 0 0 0 0 0</pose>
          <ray>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>0</min_angle>
                <max_angle>6.28319</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.1</min>
              <max>10.0</max>
              <resolution>0.01</resolution>
            </range>
          </ray>
          <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
            <frame_name>${name}_link</frame_name>
          </plugin>
        </sensor>
      </xacro:if>
    </gazebo>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="orange">
        <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.416" ixy="0.0" ixz="0.0" iyy="0.416" iyz="0.0" izz="0.208"/>
    </inertial>
  </link>

  <!-- Create wheels using the macro -->
  <xacro:wheel prefix="front_left" parent="base_link">
    <origin xyz="0.2 0.2 0" rpy="0 0 0"/>
  </xacro:wheel>

  <xacro:wheel prefix="front_right" parent="base_link">
    <origin xyz="0.2 -0.2 0" rpy="0 0 0"/>
  </xacro:wheel>

  <xacro:wheel prefix="rear_left" parent="base_link">
    <origin xyz="-0.2 0.2 0" rpy="0 0 0"/>
  </xacro:wheel>

  <xacro:wheel prefix="rear_right" parent="base_link">
    <origin xyz="-0.2 -0.2 0" rpy="0 0 0"/>
  </xacro:wheel>

  <!-- Create sensor mounts using the macro -->
  <xacro:sensor_mount name="front_camera" parent="base_link"
                      xyz="0.25 0 0.1" rpy="0 0 0" sensor_type="camera"/>

  <xacro:sensor_mount name="lidar" parent="base_link"
                      xyz="0 0 0.2" rpy="0 0 0" sensor_type="lidar"/>

  <!-- Gazebo-specific elements -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>front_left_wheel_joint</left_joint>
      <right_joint>front_right_wheel_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
    </plugin>
  </gazebo>

</robot>
```

This Xacro example demonstrates how to create reusable macros for robot components, parameterize robot designs, and integrate with Gazebo simulation while maintaining clean, maintainable code.

## Core Concept 5: Sensor Integration and Gazebo Extensions

Modern robot models in URDF often need to integrate with simulation environments and include sensor definitions. Gazebo, the standard ROS simulation environment, extends URDF with special `<gazebo>` tags that define how links and joints behave in simulation, including sensor specifications, physics properties, and plugin configurations.

The integration of sensors in URDF requires careful consideration of both the physical mounting of sensors on the robot and their simulation properties. Sensors are typically attached to links using fixed joints, and their specifications include parameters like field of view, resolution, and noise characteristics that match real-world sensors.

URDF/Gazebo integration also includes plugins that provide functionality for simulation, such as differential drive controllers, joint state publishers, and sensor drivers. These plugins bridge the gap between the abstract robot model and the concrete simulation environment, enabling realistic simulation of robot behavior.

```python
# Example 5: URDF with Sensor Definitions and Gazebo Integration (105-125 lines)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf2_geometry_msgs import PointStamped
import math
import numpy as np


class SensorIntegratedRobot(Node):
    """
    Node demonstrating integration of URDF-defined sensors with ROS 2.

    This node simulates the behavior of a robot with various sensors
    as defined in a URDF model, publishing sensor data that corresponds
    to the physical model.
    """

    def __init__(self):
        super().__init__('sensor_integrated_robot')

        # Publishers for different sensor types
        self.laser_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.camera_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # Transform broadcaster for TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for sensor simulation
        self.sensor_timer = self.create_timer(0.1, self.simulate_sensors)

        # Robot state simulation
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Sensor simulation parameters
        self.laser_range_min = 0.1
        self.laser_range_max = 10.0
        self.laser_angle_min = -math.pi
        self.laser_angle_max = math.pi
        self.laser_angle_increment = math.pi / 180.0  # 1 degree resolution

        # Camera parameters
        self.camera_width = 640
        self.camera_height = 480
        self.camera_fov = math.pi / 3  # 60 degrees

        self.get_logger().info("Sensor integrated robot simulator initialized")

    def simulate_sensors(self) -> None:
        """Simulate sensor data based on robot model and environment."""
        current_time = self.get_clock().now()

        # Update robot state (simple motion model)
        dt = 0.1  # 10Hz update
        self.position_x += self.linear_velocity * math.cos(self.orientation_theta) * dt
        self.position_y += self.linear_velocity * math.sin(self.orientation_theta) * dt
        self.orientation_theta += self.angular_velocity * dt

        # Publish odometry
        self.publish_odometry(current_time)

        # Publish laser scan with simulated obstacles
        self.publish_laser_scan(current_time)

        # Publish camera image (simulated)
        self.publish_camera_image(current_time)

        # Publish IMU data (with simulated noise)
        self.publish_imu_data(current_time)

        # Broadcast transforms
        self.broadcast_transforms(current_time)

    def publish_odometry(self, stamp) -> None:
        """Publish odometry data."""
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Position
        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.pose.pose.position.y = self.position_y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (convert from theta to quaternion)
        odom_msg.pose.pose.orientation.z = math.sin(self.orientation_theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.orientation_theta / 2.0)

        # Velocity
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = self.angular_velocity

        self.odom_publisher.publish(odom_msg)

    def publish_laser_scan(self, stamp) -> None:
        """Publish simulated laser scan data."""
        scan_msg = LaserScan()
        scan_msg.header.stamp = stamp.to_msg()
        scan_msg.header.frame_id = 'laser_link'

        # Set scan parameters
        scan_msg.angle_min = self.laser_angle_min
        scan_msg.angle_max = self.laser_angle_max
        scan_msg.angle_increment = self.laser_angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = self.laser_range_min
        scan_msg.range_max = self.laser_range_max

        # Calculate number of ranges
        num_ranges = int((self.laser_angle_max - self.laser_angle_min) / self.laser_angle_increment)

        # Generate simulated ranges with some obstacles
        ranges = []
        for i in range(num_ranges):
            angle = self.laser_angle_min + i * self.laser_angle_increment

            # Simulate a circular obstacle at (2, 0) with radius 0.5
            obstacle_dist = math.sqrt((2.0 - self.position_x)**2 + (0.0 - self.position_y)**2)
            obstacle_angle = math.atan2(-self.position_y, 2.0 - self.position_x) - self.orientation_theta

            # Calculate distance to obstacle in laser beam direction
            angle_diff = abs(angle - obstacle_angle)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff

            if angle_diff < 0.2:  # If laser beam is pointing toward obstacle
                dist_to_obstacle = obstacle_dist - 0.5  # Subtract radius
                ranges.append(min(dist_to_obstacle, self.laser_range_max))
            else:
                ranges.append(self.laser_range_max)  # No obstacle in this direction

        scan_msg.ranges = ranges
        scan_msg.intensities = [100.0] * len(ranges)  # Simulated intensity

        self.laser_publisher.publish(scan_msg)

    def publish_camera_image(self, stamp) -> None:
        """Publish simulated camera image."""
        img_msg = Image()
        img_msg.header.stamp = stamp.to_msg()
        img_msg.header.frame_id = 'camera_optical_frame'

        img_msg.height = self.camera_height
        img_msg.width = self.camera_width
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = False
        img_msg.step = self.camera_width * 3  # 3 bytes per pixel (RGB)

        # Create a simple simulated image (gradient with some objects)
        # This is a simplified representation - real implementation would use more sophisticated simulation
        pixels = []
        for y in range(self.camera_height):
            for x in range(self.camera_width):
                # Create a gradient based on position
                r = int(255 * x / self.camera_width)
                g = int(255 * y / self.camera_height)
                b = 100
                pixels.extend([r, g, b])

        img_msg.data = bytearray(pixels)
        self.camera_publisher.publish(img_msg)

    def publish_imu_data(self, stamp) -> None:
        """Publish simulated IMU data."""
        imu_msg = Imu()
        imu_msg.header.stamp = stamp.to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Orientation (simulate level ground)
        imu_msg.orientation.z = math.sin(self.orientation_theta / 2.0)
        imu_msg.orientation.w = math.cos(self.orientation_theta / 2.0)
        # Set covariance to indicate these are accurate measurements
        imu_msg.orientation_covariance[0] = 0.01
        imu_msg.orientation_covariance[4] = 0.01
        imu_msg.orientation_covariance[8] = 0.01

        # Angular velocity (simulate angular motion)
        imu_msg.angular_velocity.z = self.angular_velocity
        imu_msg.angular_velocity_covariance[0] = 0.01
        imu_msg.angular_velocity_covariance[4] = 0.01
        imu_msg.angular_velocity_covariance[8] = 0.01

        # Linear acceleration (simulate gravity and linear motion)
        imu_msg.linear_acceleration.x = self.linear_velocity * 0.1  # Small acceleration
        imu_msg.linear_acceleration.z = 9.81  # Gravity
        imu_msg.linear_acceleration_covariance[0] = 0.01
        imu_msg.linear_acceleration_covariance[4] = 0.01
        imu_msg.linear_acceleration_covariance[8] = 0.01

        self.imu_publisher.publish(imu_msg)

    def broadcast_transforms(self, stamp) -> None:
        """Broadcast coordinate frame transforms."""
        from geometry_msgs.msg import TransformStamped

        # Base footprint to base link
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.position_x
        t.transform.translation.y = self.position_y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.orientation_theta / 2.0)
        t.transform.rotation.w = math.cos(self.orientation_theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # Base link to laser
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'laser_link'
        t.transform.translation.x = 0.2  # Mount point
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1
        t.transform.rotation.w = 1.0  # No rotation
        self.tf_broadcaster.sendTransform(t)

        # Base link to camera
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = 0.25
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # Camera link to optical frame (for camera orientation)
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = 'camera_optical_frame'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        # Rotate to optical frame (x-forward, y-left, z-up)
        t.transform.rotation.x = -0.5
        t.transform.rotation.y = 0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w = 0.5
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """Main entry point for the sensor integrated robot simulator."""
    rclpy.init(args=args)

    node = SensorIntegratedRobot()

    # Set some initial motion
    node.linear_velocity = 0.5  # Move forward at 0.5 m/s
    node.angular_velocity = 0.2  # Turn slightly

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down sensor integrated robot")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

This example demonstrates how URDF-defined sensors integrate with ROS 2 systems, showing how the robot model connects to actual sensor data publishing and simulation.

## Implementation Perspective

URDF implementation requires attention to several practical considerations. First, the kinematic structure must be physically plausible and match the real robot's capabilities. The inertial properties must be accurate for proper simulation, though they can be approximated for basic visualization and planning purposes.

For real-world applications, URDF models often need to be validated against actual robot hardware. This involves checking that joint limits, velocities, and efforts match the physical robot's capabilities. The visual and collision models should also be verified to ensure they accurately represent the physical robot's shape and size.

When working with complex robots, it's important to organize URDF files properly, potentially breaking them into multiple files using xacro includes. This improves maintainability and allows for modular robot design where different components can be reused across multiple robot models.

Performance considerations also play a role in URDF design. Complex collision geometries can slow down physics simulation, so it's often beneficial to use simplified collision models that approximate the visual geometry but are computationally efficient for physics calculations.

## Common Pitfalls

1. **Invalid Kinematic Chains**: Creating URDF models with disconnected or improperly connected links can cause issues with robot state publishing and motion planning.

2. **Incorrect Inertial Properties**: Using unrealistic or inaccurate inertial properties can lead to unstable simulation behavior.

3. **Missing Joint Limits**: Failing to specify joint limits can result in robots that move in physically impossible ways during simulation.

4. **Mesh Path Issues**: Incorrect file paths for mesh files can cause visualization problems in simulation and visualization tools.

## Real-World Applications

URDF is used extensively in robotics research and development for robot modeling, simulation, and visualization. Major robotics platforms like TurtleBot, PR2, and many commercial robots use URDF to define their kinematic and physical properties. URDF models are essential for motion planning in MoveIt!, simulation in Gazebo, and visualization in RViz.

The format has become the de facto standard for robot description in ROS-based systems, making it possible to share robot models across different research groups and companies. Many robotics companies maintain libraries of URDF models for common robot platforms, sensors, and tools.

## Exercises

1. Create a URDF model for a simple 3-DOF manipulator arm with proper joint limits and inertial properties.
2. Convert a basic URDF model to use Xacro macros for parameterization and reusability.
3. Add sensor definitions to an existing robot model and implement the corresponding ROS 2 interface.

### Solutions

1. The solution would involve creating a complete URDF file with base, links, joints, and proper physical properties for a 3-DOF manipulator.
2. The solution would demonstrate Xacro macros that parameterize link dimensions and joint positions.
3. The solution would add Gazebo sensor definitions and implement a ROS 2 node to interface with the sensors.

## Key Takeaways

- URDF provides a standardized way to describe robot kinematics and physical properties
- Xacro enables parameterized and reusable robot models
- Proper sensor integration connects simulation to real-world applications
- Accurate physical properties are essential for reliable simulation
- Well-structured URDF models facilitate robot development and sharing

## Further Reading

- URDF Documentation: http://wiki.ros.org/urdf
- Xacro Tutorial: http://wiki.ros.org/xacro
- Robot Modeling Best Practices: https://ros.org/reps/rep-0105.html

## Next Chapter Preview

Chapter 6 will explore launch files and system integration, examining how to coordinate multiple nodes and configure complex robotic systems using ROS 2's launch system.