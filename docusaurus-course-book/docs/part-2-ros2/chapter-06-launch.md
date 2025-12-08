---
sidebar_position: 6
title: "Chapter 6: Launch Files and System Integration"
---

# Chapter 6: Launch Files and System Integration

## Introduction

Launch files in ROS 2 provide a powerful mechanism for configuring, launching, and managing complex robotic systems with multiple interconnected nodes. The launch system enables developers to define complete robot systems in a declarative way, specifying which nodes to run, their parameters, remappings, and the relationships between them. This chapter explores the advanced capabilities of the ROS 2 launch system, from basic launch file creation to complex multi-robot coordination scenarios.

The launch system in ROS 2 represents a significant improvement over ROS 1, offering better support for conditional launches, event handling, and dynamic system configuration. The Python-based launch files provide a programmatic interface that allows for complex launch logic, parameter validation, and system orchestration that adapts to runtime conditions.

Launch files serve as the entry point for complete robotic systems, coordinating the startup and configuration of multiple nodes to create a functional robot application. They encapsulate the complexity of system configuration, making it easier to deploy and reproduce robotic systems across different environments and hardware configurations.

Understanding launch files is essential for creating professional robotic applications that can be reliably deployed and maintained. This chapter will guide you through the creation of sophisticated launch systems that can handle complex robot configurations, multi-robot scenarios, and dynamic reconfiguration needs.

By the end of this chapter, you'll have mastered the art of system integration using ROS 2 launch files and be able to create robust, scalable launch configurations for complex robotic systems.

## Core Concept 1: Basic Launch File Structure and Components

The foundation of ROS 2 launch files lies in understanding the basic components and structure that enable the coordination of multiple nodes. A launch file is essentially a Python script that uses the launch library to define a system configuration. The launch system provides actions for starting nodes, setting parameters, managing remappings, and handling events that occur during system operation.

The basic structure of a launch file includes declaring launch arguments, defining nodes to be launched, specifying parameters and remappings, and setting up the execution environment. Each component plays a specific role in creating a complete system configuration that can be reliably reproduced across different environments.

Launch arguments allow for parameterization of launch files, enabling the same launch configuration to be used with different settings. This is particularly useful for configuring robot systems for different hardware platforms or operational environments without creating multiple nearly-identical launch files.

```python
# Example 1: Basic Launch File with Multiple Nodes (100-120 lines)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNode
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for a complete robot system."""

    # Declare launch arguments with default values and descriptions
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Namespace for the robot nodes'
    )

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    debug_mode_launch_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug logging if true'
    )

    # Retrieve launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    debug_mode = LaunchConfiguration('debug_mode')

    # Define robot description parameter
    robot_description = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'urdf',
        'my_robot.urdf.xacro'
    ])

    # Create robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        on_exit=Shutdown()  # Shut down entire system if this node exits
    )

    # Create joint state publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(debug_mode)  # Only run in debug mode
    )

    # Create navigation stack nodes
    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': PathJoinSubstitution([nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml'])}
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'set_initial_pose': True},
            {'initial_pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}}
        ]
    )

    # Create conditional logging based on debug mode
    debug_log_action = LogInfo(
        msg=["Debug mode enabled for namespace: ", namespace],
        condition=IfCondition(debug_mode)
    )

    # Register event handler for process start events
    process_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[
                LogInfo(msg=["Robot state publisher started for namespace: ", namespace])
            ]
        )
    )

    return LaunchDescription([
        namespace_launch_arg,
        use_sim_time_launch_arg,
        debug_mode_launch_arg,
        robot_state_publisher,
        joint_state_publisher,
        lifecycle_manager,
        map_server,
        amcl,
        debug_log_action,
        process_start_handler
    ])
```

This example demonstrates the fundamental components of a ROS 2 launch file, including launch arguments, node definitions with parameters and remappings, and conditional execution based on runtime parameters.

## Core Concept 2: Conditional Launch and Event Handling

Advanced launch systems often need to adapt their behavior based on runtime conditions or respond to events that occur during system operation. The ROS 2 launch system provides sophisticated mechanisms for conditional execution and event handling that enable the creation of flexible and robust launch configurations.

Conditional launch allows nodes to be started or skipped based on launch arguments, environment variables, or other runtime conditions. This is particularly useful for creating launch files that can operate in different modes (simulation vs. real hardware) or enable optional components based on system configuration.

Event handling in launch files enables reactive behavior to system events such as node startup, shutdown, or output. This can be used to coordinate node startup order, handle failures gracefully, or perform cleanup operations when nodes terminate.

```python
# Example 2: Conditional Launch and Event Handling (110-130 lines)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Generate launch description with conditional execution and event handling."""

    # Declare launch arguments
    sim_mode_launch_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Run in simulation mode if true'
    )

    enable_vision_launch_arg = DeclareLaunchArgument(
        'enable_vision',
        default_value='true',
        description='Enable vision processing nodes if true'
    )

    emergency_stop_launch_arg = DeclareLaunchArgument(
        'emergency_stop',
        default_value='false',
        description='Start system in emergency stop mode'
    )

    # Retrieve launch configurations
    sim_mode = LaunchConfiguration('sim_mode')
    enable_vision = LaunchConfiguration('enable_vision')
    emergency_stop = LaunchConfiguration('emergency_stop')

    # Create base nodes that always run
    base_controller = Node(
        package='my_robot_base',
        executable='base_controller',
        name='base_controller',
        parameters=[
            {'use_sim_time': sim_mode},
            {'max_linear_velocity': 1.0},
            {'max_angular_velocity': 1.0}
        ]
    )

    # Conditional nodes based on simulation mode
    real_hardware_driver = Node(
        package='my_robot_hardware',
        executable='real_hardware_driver',
        name='real_hardware_driver',
        parameters=[
            {'port': '/dev/ttyUSB0'}
        ],
        condition=UnlessCondition(sim_mode)  # Only run on real hardware
    )

    gazebo_simulation = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        condition=IfCondition(sim_mode)  # Only run in simulation
    )

    # Conditional vision nodes
    vision_pipeline = Node(
        package='vision_pipeline',
        executable='object_detector',
        name='object_detector',
        parameters=[
            {'camera_topic': '/camera/image_raw'},
            {'detection_model': 'yolov5'}
        ],
        condition=IfCondition(enable_vision)
    )

    # Emergency stop handler
    emergency_node = Node(
        package='my_robot_control',
        executable='emergency_stop_handler',
        name='emergency_stop_handler',
        parameters=[
            {'active': emergency_stop}
        ],
        condition=IfCondition(emergency_stop)
    )

    # Event handlers for system monitoring
    process_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=base_controller,
            on_start=[
                LogInfo(msg="Base controller started successfully"),
                LogInfo(msg=["Simulation mode: ", sim_mode]),
                LogInfo(msg=["Vision enabled: ", enable_vision])
            ]
        )
    )

    # Handle emergency stop events
    emergency_stop_handler = RegisterEventHandler(
        OnProcessIO(
            target_action=base_controller,
            on_stdin_match='EMERGENCY_STOP',
            on_stdin_do=[
                LogInfo(msg="Emergency stop command received!"),
                Shutdown(reason="Emergency stop activated")
            ]
        )
    )

    # Handle node failures gracefully
    node_failure_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=base_controller,
            on_exit=[
                LogInfo(msg="Base controller terminated unexpectedly"),
                LogInfo(msg="Shutting down dependent nodes...")
            ]
        )
    )

    # Timer-based system health check
    health_check_timer = TimerAction(
        period=10.0,  # Run every 10 seconds
        actions=[
            LogInfo(msg="System health check: All nodes operational")
        ]
    )

    return LaunchDescription([
        sim_mode_launch_arg,
        enable_vision_launch_arg,
        emergency_stop_launch_arg,
        base_controller,
        real_hardware_driver,
        gazebo_simulation,
        vision_pipeline,
        emergency_node,
        process_start_handler,
        emergency_stop_handler,
        node_failure_handler,
        health_check_timer
    ])
```

This example demonstrates sophisticated conditional launch logic and event handling, showing how to create responsive launch systems that adapt to runtime conditions and react to system events.

## Core Concept 3: Parameter Substitution and Remapping

Parameter substitution and topic/service remapping are powerful features of the ROS 2 launch system that enable flexible configuration of robot systems. These mechanisms allow launch files to be parameterized and adapted to different environments without requiring code changes.

Parameter substitution enables launch files to use runtime values for node parameters, file paths, and other configuration settings. This includes launch arguments, environment variables, command outputs, and mathematical expressions. The substitution system provides a flexible way to create reusable launch configurations.

Remapping allows the redirection of topic and service names at launch time, enabling nodes to work with different naming conventions or namespaces without requiring code changes. This is particularly useful for multi-robot systems where the same nodes need to operate with different topic names for each robot.

```python
# Example 3: Parameter Substitution and Remapping (115-135 lines)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression,
    EnvironmentVariable
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description with advanced parameter substitution and remapping."""

    # Declare launch arguments with default values
    robot_namespace_launch_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',
        description='Namespace for the robot'
    )

    config_file_launch_arg = DeclareLaunchArgument(
        'config_file',
        default_value=[LaunchConfiguration('robot_namespace'), '_config.yaml'],
        description='Configuration file path relative to config directory'
    )

    sensor_topic_prefix_launch_arg = DeclareLaunchArgument(
        'sensor_topic_prefix',
        default_value='sensors/',
        description='Prefix for all sensor topics'
    )

    # Retrieve launch configurations
    robot_namespace = LaunchConfiguration('robot_namespace')
    config_file = LaunchConfiguration('config_file')
    sensor_prefix = LaunchConfiguration('sensor_topic_prefix')

    # Set environment variables for nodes
    set_log_level = SetEnvironmentVariable(
        name='RCUTILS_LOGGING_SEVERITY_THRESHOLD',
        value='INFO'
    )

    # Define parameter file paths using substitution
    config_path = PathJoinSubstitution([
        FindPackageShare('my_robot_config'),
        'config',
        config_file
    ])

    # Create sensor processing node with parameter substitution
    sensor_processor = Node(
        package='sensor_processing',
        executable='sensor_fusion_node',
        name='sensor_fusion',
        namespace=robot_namespace,
        parameters=[
            config_path,  # Load from config file
            {'use_sim_time': LaunchConfiguration('use_sim_time', default_value='false')},
            {'sensor_timeout': 1.0},
            {'fusion_algorithm': 'ekf'},
            # Use environment variable with fallback
            {'debug_level': EnvironmentVariable(name='SENSOR_DEBUG_LEVEL', default_value='INFO')}
        ],
        remappings=[
            # Use parameter substitution for topic remapping
            (['/', sensor_prefix, 'imu/data'], 'imu/data'),
            (['/', sensor_prefix, 'lidar/scan'], 'lidar/scan'),
            (['/', sensor_prefix, 'camera/image_raw'], 'camera/image_raw'),
            # Remap to namespaced topics
            ('fused_odom', [robot_namespace, '/fused_odom']),
            ('filtered_pose', [robot_namespace, '/filtered_pose'])
        ]
    )

    # Create navigation node with complex parameter substitution
    navigation_node = Node(
        package='my_robot_nav',
        executable='navigation_node',
        name='navigation',
        namespace=robot_namespace,
        parameters=[
            # Combine multiple substitutions
            PathJoinSubstitution([
                FindPackageShare('my_robot_nav'),
                'config',
                PythonExpression(["'", robot_namespace, "_nav_config.yaml'"])
            ]),
            {
                'robot_radius': PythonExpression(['0.3 + ', LaunchConfiguration('robot_size_offset', default_value='0.0')]),
                'global_frame': PythonExpression(["'", robot_namespace, "/map'"]),
                'odom_frame': PythonExpression(["'", robot_namespace, "/odom'"]),
                'base_frame': PythonExpression(["'", robot_namespace, "/base_link'"]),
                'planner_frequency': 5.0,
                'controller_frequency': 20.0
            }
        ],
        remappings=[
            ('cmd_vel', [robot_namespace, '/cmd_vel']),
            ('odom', [robot_namespace, '/odom']),
            ('map', [robot_namespace, '/map']),
            # Dynamic remapping based on environment
            ('goal_pose', PythonExpression([
                "'",
                PythonExpression(["'", robot_namespace, "/goal' if ",
                                 EnvironmentVariable(name='NAVIGATION_MODE', default_value='single'),
                                 " == 'single' else '", robot_namespace, "/multi_goal'"]),
                "'"
            ]))
        ]
    )

    # Create perception node with environment-dependent configuration
    perception_node = Node(
        package='perception_pipeline',
        executable='object_detection_node',
        name='object_detection',
        namespace=robot_namespace,
        parameters=[
            # Use environment variable to determine model
            {'detection_model': EnvironmentVariable(name='DETECTION_MODEL', default_value='yolov5s.pt')},
            {'confidence_threshold': PythonExpression(['0.5 + ', LaunchConfiguration('detection_sensitivity', default_value='0.0')])},
            {'tracking_enabled': PythonExpression([LaunchConfiguration('enable_tracking', default_value='true'), " == 'true'"])}
        ],
        remappings=[
            ('input_image', PythonExpression(["'", sensor_prefix, "camera/image_raw'"])),
            ('detections', [robot_namespace, '/detected_objects'])
        ]
    )

    # Create diagnostic aggregator with parameter substitution
    diagnostic_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        namespace=robot_namespace,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('my_robot_diag'),
                'config',
                PythonExpression(["'", robot_namespace, "_diagnostics.yaml'"])
            ])
        ],
        remappings=[
            ('diagnostics', PythonExpression(["'", robot_namespace, "/diagnostics'"]))
        ]
    )

    return LaunchDescription([
        set_log_level,
        robot_namespace_launch_arg,
        config_file_launch_arg,
        sensor_topic_prefix_launch_arg,
        sensor_processor,
        navigation_node,
        perception_node,
        diagnostic_aggregator
    ])
```

This example showcases advanced parameter substitution techniques and sophisticated remapping strategies that enable highly configurable robot systems.

## Core Concept 4: Composable Node Launching

Composable nodes represent a significant architectural advancement in ROS 2, allowing multiple nodes to run within a single process. This approach eliminates inter-process communication overhead while maintaining the modularity benefits of the node-based architecture. The launch system provides special support for loading and managing composable nodes within component containers.

Component-based architecture offers performance benefits for systems with tightly coupled nodes that communicate frequently. By running these nodes in the same process, communication latency is reduced and memory usage is optimized. However, this comes at the cost of reduced fault isolation compared to separate processes.

The launch system supports both standalone component containers and the dynamic loading of components into running containers. This flexibility allows for both static system configurations and dynamic reconfiguration of robot systems during operation.

```python
# Example 4: Composable Nodes and Component Containers (120-140 lines)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for a component-based robot system."""

    # Declare launch arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    enable_monitoring_launch_arg = DeclareLaunchArgument(
        'enable_monitoring',
        default_value='true',
        description='Enable component monitoring if true'
    )

    # Retrieve launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_monitoring = LaunchConfiguration('enable_monitoring')

    # Create a multi-threaded component container
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # Multi-threaded container
        parameters=[
            {'use_sim_time': use_sim_time},
            {'container_intra_process_comms': True}  # Enable intra-process comms
        ],
        output='screen',
        emulate_tty=True
    )

    # Define perception components to load into the container
    components = [
        # Image processing component
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='image_rectifier',
            namespace='camera',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'interpolation': 1}
            ],
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect_color')
            ],
            extra_arguments=[{'use_intra_process_comms': True}]  # Optimize for same-container communication
        ),

        # Depth processing component
        ComposableNode(
            package='depth_image_proc',
            plugin='depth_image_proc::PointCloudXyzrgbNode',
            name='point_cloud_xyzrgb',
            namespace='camera',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'queue_size': 1}
            ],
            remappings=[
                ('rgb/image_rect_color', 'image_rect_color'),
                ('rgb/camera_info', 'camera_info'),
                ('depth_registered/image_rect', 'depth/image_rect'),
                ('points', 'points')
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        ),

        # Feature detection component
        ComposableNode(
            package='vision_opencv',
            plugin='cv_bridge::CvNode',
            name='cv_bridge_node',
            namespace='camera',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'encoding': 'bgr8'}
            ],
            remappings=[
                ('image_raw', 'image_rect_color'),
                ('image_cv', 'image_cv')
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        ),

        # Object detection component
        ComposableNode(
            package='vision_darknet',
            plugin='darknet_ros::DarknetNode',
            name='object_detector',
            namespace='camera',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'config_path': '/path/to/yolo/config'},
                {'weights_path': '/path/to/yolo/weights'},
                {'threshold': 0.5}
            ],
            remappings=[
                ('image', 'image_rect_color'),
                ('detection', 'detections')
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    ]

    # Add components to container
    perception_container.add_composable_nodes(components)

    # Create a separate container for control components
    control_container = ComposableNodeContainer(
        name='control_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'container_intra_process_comms': True}
        ],
        output='screen'
    )

    # Control components
    control_components = [
        # Twist mux component
        ComposableNode(
            package='twist_mux',
            plugin='twist_mux::TwistMux',
            name='twist_mux',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'topics': [{'name': 'navigation', 'priority': 10, 'timeout': 0.2},
                           {'name': 'joystick', 'priority': 5, 'timeout': 0.5},
                           {'name': 'safety', 'priority': 20, 'timeout': 0.1}]}
            ],
            remappings=[
                ('cmd_vel_out', 'cmd_vel'),
                ('navigation/cmd_vel', 'navigation/cmd_vel'),
                ('joystick/cmd_vel', 'joystick/cmd_vel'),
                ('safety/cmd_vel', 'safety/cmd_vel')
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        ),

        # Joint trajectory controller
        ComposableNode(
            package='joint_trajectory_controller',
            plugin='controller_interface::ControllerInterface',
            name='joint_trajectory_controller',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'command_interfaces': ['position']},
                {'state_interfaces': ['position', 'velocity']}
            ],
            remappings=[
                ('~/commands', 'joint_commands'),
                ('~/states', 'joint_states')
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    ]

    # Add control components to container
    control_container.add_composable_nodes(control_components)

    # Conditional monitoring node (only if enabled)
    monitoring_node = Node(
        package='composition',
        executable='lifecycle_composition',
        name='component_monitor',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'monitored_containers': ['perception_container', 'control_container']}
        ],
        condition=IfCondition(enable_monitoring)
    )

    return LaunchDescription([
        use_sim_time_launch_arg,
        enable_monitoring_launch_arg,
        perception_container,
        control_container,
        monitoring_node
    ])
```

This example demonstrates the creation of component-based systems with multiple containers optimized for performance and maintainability.

## Core Concept 5: Multi-Robot System Launch

Complex robotic applications often require the coordination of multiple robots working together. The ROS 2 launch system provides sophisticated capabilities for launching and managing multi-robot systems, including namespace management, resource allocation, and inter-robot communication patterns.

Multi-robot launch configurations must carefully handle naming conventions to avoid conflicts between robots. Each robot typically operates in its own namespace, with all topics, services, and parameters prefixed accordingly. This isolation prevents interference between robots while allowing for coordinated operation.

Resource management in multi-robot systems requires attention to computational resources, network bandwidth, and hardware interfaces. Launch files must ensure that each robot has access to the resources it needs without conflicts, particularly for hardware interfaces like serial ports or network devices.

```python
# Example 5: Multi-Robot Launch System (125-145 lines)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    """Generate launch description for a multi-robot system."""

    # Declare launch arguments for multi-robot configuration
    num_robots_launch_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='2',
        description='Number of robots in the system'
    )

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    enable_coordination_launch_arg = DeclareLaunchArgument(
        'enable_coordination',
        default_value='false',
        description='Enable robot coordination if true'
    )

    # Retrieve launch configurations
    num_robots = LaunchConfiguration('num_robots')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_coordination = LaunchConfiguration('enable_coordination')

    # Create robot definitions
    robot_definitions = []
    for i in range(int(num_robots.perform(None))):  # Get actual value for robot creation
        robot_name = f'robot_{i+1}'

        # Create group action for each robot with its own namespace
        robot_group = GroupAction(
            actions=[
                # Push namespace for this robot
                PushRosNamespace(robot_name),

                # Robot state publisher for this robot
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name=f'{robot_name}_state_publisher',
                    parameters=[
                        {'robot_description': f'package://my_robot_description/urdf/{robot_name}.urdf'},
                        {'use_sim_time': use_sim_time}
                    ],
                    remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static')
                    ]
                ),

                # Navigation stack for this robot
                Node(
                    package='nav2_bringup',
                    executable='bringup_launch.py',
                    name=f'{robot_name}_nav_stack',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'namespace': robot_name},
                        {'autostart': True}
                    ]
                ),

                # Robot-specific sensor processing
                Node(
                    package='sensor_processing',
                    executable='sensor_fusion',
                    name=f'{robot_name}_sensor_fusion',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'robot_name': robot_name}
                    ]
                )
            ]
        )
        robot_definitions.append(robot_group)

    # Create coordination node that manages inter-robot communication
    coordination_node = Node(
        package='multi_robot_coordination',
        executable='coordination_manager',
        name='coordination_manager',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'num_robots': num_robots},
            {'robot_names': [f'robot_{i+1}' for i in range(int(num_robots.perform(None)))]},
            {'coordination_enabled': enable_coordination}
        ],
        condition=IfCondition(enable_coordination)
    )

    # Create fleet monitoring dashboard
    dashboard_node = Node(
        package='multi_robot_dashboard',
        executable='dashboard',
        name='fleet_dashboard',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_names': [f'robot_{i+1}' for i in range(int(num_robots.perform(None)))]}
        ]
    )

    # Create simulation environment (if using sim)
    simulation_nodes = [
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            condition=IfCondition(use_sim_time)
        ),
        Node(
            package='gazebo_ros',
            executable='gzclient',
            condition=IfCondition(PythonExpression([use_sim_time, " and ", LaunchConfiguration('show_gui', default_value='true')]))
        )
    ]

    # Spawn robots in simulation
    spawn_actions = []
    for i in range(int(num_robots.perform(None))):
        robot_name = f'robot_{i+1}'
        spawn_actions.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', robot_name,
                    '-topic', f'/{robot_name}/robot_description',
                    '-x', str(i * 2.0),  # Space robots apart
                    '-y', '0.0',
                    '-z', '0.0'
                ],
                output='screen',
                condition=IfCondition(use_sim_time)
            )
        )

    # Create system health monitor
    health_monitor = Node(
        package='multi_robot_health',
        executable='health_monitor',
        name='system_health_monitor',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_names': [f'robot_{i+1}' for i in range(int(num_robots.perform(None)))]},
            {'check_interval': 5.0}
        ]
    )

    # System startup notification
    startup_notification = LogInfo(
        msg=[f"Starting multi-robot system with {num_robots.perform(None)} robots"]
    )

    return LaunchDescription([
        num_robots_launch_arg,
        use_sim_time_launch_arg,
        enable_coordination_launch_arg,
        startup_notification,
        *robot_definitions,
        coordination_node,
        dashboard_node,
        *simulation_nodes,
        *spawn_actions,
        health_monitor
    ])
```

This example demonstrates how to create sophisticated multi-robot launch configurations with proper namespace management, resource allocation, and coordination capabilities.

## Implementation Perspective

The launch system in ROS 2 provides a powerful framework for system integration that goes beyond simple node launching. It enables the creation of sophisticated robot systems with complex interdependencies, conditional behavior, and dynamic configuration capabilities.

When implementing launch files, it's important to consider the trade-offs between simplicity and flexibility. Simple launch files are easier to understand and debug, while complex launch files can provide more robust and adaptable system configurations. The key is to strike the right balance for your specific use case.

Launch files should be designed with maintainability in mind, using clear variable names, logical grouping of related components, and comprehensive documentation. Well-designed launch files can significantly improve the reliability and usability of robotic systems.

## Common Pitfalls

1. **Namespace Conflicts**: Failing to properly isolate robot components can lead to topic conflicts in multi-robot systems.

2. **Resource Contention**: Not accounting for resource competition between nodes can cause performance issues.

3. **Startup Ordering**: Not properly sequencing node startup can result in initialization failures.

4. **Parameter Inheritance**: Misunderstanding how parameters are inherited across namespaces can cause unexpected behavior.

## Real-World Applications

Launch files are essential for deploying complex robotic systems in research, industrial, and commercial applications. They enable the reliable startup of multi-node robot systems, coordinate the configuration of complex sensor suites, and manage the integration of perception, planning, and control systems. Many commercial robot platforms use sophisticated launch files to manage their complete software stacks.

## Exercises

1. Create a launch file that conditionally launches different sensor configurations based on robot type.
2. Implement a launch file that dynamically loads components based on runtime parameters.
3. Design a multi-robot launch system with resource allocation and conflict resolution.

### Solutions

1. The solution would use conditional statements and parameter substitution to select appropriate sensor configurations.
2. The solution would demonstrate dynamic component loading based on runtime conditions.
3. The solution would implement namespace management and resource allocation for multiple robots.

## Key Takeaways

- Launch files provide powerful system integration capabilities for complex robotic systems
- Conditional execution and event handling enable responsive launch configurations
- Parameter substitution and remapping offer flexible system configuration
- Component-based architecture improves performance for tightly coupled systems
- Multi-robot systems require careful namespace and resource management

## Further Reading

- ROS 2 Launch System Documentation: https://docs.ros.org/en/humble/How-To-Guides/Launch-system.html
- Composition and Components: https://docs.ros.org/en/humble/Tutorials/Composition.html
- Multi-Robot Systems in ROS 2: Research papers and tutorials on coordination

## Next Chapter Preview

Chapter 7 will introduce physics simulation with Gazebo, exploring how to create realistic simulation environments for robot testing and development, including world creation, robot spawning, and sensor simulation.