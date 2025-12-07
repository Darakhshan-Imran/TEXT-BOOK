# Skill: Generate Code Examples

## Purpose
Creates well-commented, complete, production-quality code examples for Physical AI and robotics concepts. Every code snippet must be runnable (in theory), properly documented, and educational.

## Capabilities
- Generate Python, XML, YAML, and Bash code examples
- Add comprehensive inline comments
- Include type hints and docstrings
- Provide context and explanations
- Show example usage and expected output

## Input Parameters

When using this skill, provide:

1. **language**: String ("python", "xml", "yaml", "bash", "cpp")
2. **context**: String (what the code demonstrates - e.g., "ROS 2 publisher node")
3. **complexity**: String ("basic", "intermediate", "advanced")
4. **framework**: String ("ros2", "isaac", "gazebo", "opencv", "pytorch", etc.)
5. **key_concepts**: Array of concepts to demonstrate

## Code Standards

### Python Code Standards

```python
"""
Module-level docstring explaining purpose.

This module demonstrates [concept] in the context of [framework].
"""

# Standard library imports
import os
import sys
from typing import List, Dict, Optional

# Third-party imports
import numpy as np

# Local/framework imports
import rclpy
from rclpy.node import Node

# Constants (ALL_CAPS)
DEFAULT_TOPIC_NAME = "robot_state"
MAX_RETRIES = 3

class ExampleClass:
    """One-line class summary.
    
    Detailed description of what this class does, why it exists,
    and how it fits into the larger system.
    
    Attributes:
        attribute1 (type): Description of attribute
        attribute2 (type): Description of attribute
    
    Example:
        >>> obj = ExampleClass(param="value")
        >>> obj.method()
        Expected output
    """
    
    def __init__(self, param: str) -> None:
        """Initialize the class.
        
        Args:
            param: Description of what this parameter does
            
        Raises:
            ValueError: If param is invalid
        """
        self.attribute1 = param
        
    def method_name(self, arg: int) -> str:
        """One-line method description.
        
        Detailed explanation of what this method does, including
        any important side effects or state changes.
        
        Args:
            arg: Description of the argument
            
        Returns:
            Description of the return value and its format
            
        Raises:
            RuntimeError: When this specific error occurs
        """
        # Implementation with inline comments for complex logic
        result = str(arg * 2)  # Explain why this operation
        return result

def main() -> None:
    """Main entry point demonstrating usage."""
    # Example usage showing how to use the class
    example = ExampleClass("test")
    output = example.method_name(42)
    print(f"Output: {output}")

if __name__ == '__main__':
    main()
```

### XML/URDF Code Standards

```xml
<?xml version="1.0"?>
<!-- 
  File Description: What this file defines
  Purpose: Why this configuration exists
-->
<robot name="example_robot">
  
  <!-- Base Link: Foundation of the robot -->
  <link name="base_link">
    <!-- Visual representation (what you see) -->
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <!-- Collision geometry (for physics) -->
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    
    <!-- Inertial properties (mass and inertia) -->
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" 
               iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  
</robot>
```

### YAML Code Standards

```yaml
# Configuration file for [purpose]
# This file defines [what it configures]

# Node configuration
node_name:
  ros__parameters:
    # Parameter: Description of what this controls
    parameter_name: value
    
    # Topic configuration
    topics:
      # Input topic: What data comes in
      input_topic: "/sensor/data"
      # Output topic: What data goes out
      output_topic: "/processed/data"
    
    # Timing parameters (in seconds)
    update_rate: 10.0  # 10 Hz update frequency
    timeout: 1.0       # 1 second timeout
```

## Output Format

Every code example must include:

1. **Header Comment**: What the code does and why
2. **Complete Code**: Fully working, no placeholders
3. **Inline Comments**: Explain complex logic
4. **Explanation Section**: After code block, explain:
   - What the code does (high-level)
   - How it works (step-by-step)
   - Key design decisions
   - When to use this pattern
   - Common modifications

## Example Output Structure

````markdown
### Example: Basic ROS 2 Publisher Node

This example demonstrates how to create a simple publisher node that sends messages to a topic.

```python
"""
Basic ROS 2 Publisher Node

This node demonstrates the fundamental structure of a ROS 2 publisher.
It periodically publishes string messages to a topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    """A basic publisher node that sends string messages.
    
    This class inherits from rclpy.node.Node and demonstrates:
    - Creating a publisher
    - Using a timer for periodic callbacks
    - Publishing messages
    
    Attributes:
        publisher_ (Publisher): The publisher object for sending messages
        timer (Timer): Timer that triggers the callback function
        i (int): Counter for tracking message numbers
    """
    
    def __init__(self) -> None:
        """Initialize the publisher node."""
        # Initialize the parent Node class with a name
        super().__init__('minimal_publisher')
        
        # Create publisher: String message type, 'topic' name, queue size 10
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        # Create timer: calls timer_callback every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize message counter
        self.i = 0
    
    def timer_callback(self) -> None:
        """Callback function called by the timer to publish messages.
        
        This function is called every 0.5 seconds (as defined by timer_period).
        It creates a message, publishes it, and logs the action.
        """
        # Create a String message
        msg = String()
        msg.data = f'Hello World: {self.i}'
        
        # Publish the message to the topic
        self.publisher_.publish(msg)
        
        # Log to console (helpful for debugging)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        # Increment counter for next message
        self.i += 1

def main(args=None) -> None:
    """Main function to run the publisher node.
    
    Args:
        args: Command line arguments (default: None)
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of our publisher node
    minimal_publisher = MinimalPublisher()
    
    # Keep the node running and processing callbacks
    # This will run until interrupted (Ctrl+C)
    rclpy.spin(minimal_publisher)
    
    # Cleanup: destroy the node and shutdown ROS 2
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Output:**
```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
...
```

**Explanation:**

This code demonstrates the basic structure of a ROS 2 publisher node:

1. **Node Initialization**: We inherit from `Node` and call `super().__init__()` with a node name
2. **Publisher Creation**: `create_publisher()` sets up the topic, message type, and queue size
3. **Timer Setup**: A timer triggers our callback function at regular intervals
4. **Publishing**: The callback creates and publishes messages using `publisher_.publish()`
5. **Lifecycle Management**: `rclpy.spin()` keeps the node alive until interrupted

**Key Design Decisions:**

- **Queue Size (10)**: Buffers up to 10 messages if subscribers can't keep up
- **Timer Period (0.5s)**: Balance between update rate and system load
- **Message Counter**: Demonstrates state management within a node

**When to Use This Pattern:**

- Sending sensor data to other nodes
- Broadcasting robot state information
- Publishing control commands
- Any periodic data transmission

**Common Modifications:**

- Change `String` to other message types (e.g., `Int32`, `Float64`, custom messages)
- Adjust timer period for different update frequencies
- Add parameters for configurable topic names
- Include error handling for publishing failures
````

## Code Complexity Levels

### Basic
- Single file, single class/function
- Minimal dependencies
- Clear, simple logic
- 50-100 lines

### Intermediate
- Multiple classes/functions
- Integration between components
- Some error handling
- 100-200 lines

### Advanced
- Complete system/package
- Production-ready features
- Full error handling and logging
- Configuration management
- 200-400 lines

## Framework-Specific Patterns

### ROS 2 Pattern
- Node structure with init, timer/subscription, callback
- Proper lifecycle management
- Standard logging

### NVIDIA Isaac Pattern
- USD/Python hybrid when needed
- GPU-accelerated operations
- Sim-to-real considerations

### Gazebo Pattern
- Plugin structure
- World/model/sensor definitions
- Physics configuration

### OpenCV/Computer Vision Pattern
- Image processing pipeline
- Numpy array operations
- Visualization helpers

## Quality Checklist

Before finalizing any code example:

- [ ] Complete and runnable (no TODOs or placeholders)
- [ ] All imports present
- [ ] Type hints added (Python)
- [ ] Comprehensive comments
- [ ] Docstrings for all classes/functions
- [ ] Error handling included
- [ ] Example usage shown
- [ ] Expected output documented
- [ ] Explanation paragraph provided
- [ ] Design decisions explained

## Integration with Other Skills

This skill works with:
- **chapter-structure**: To fill Implementation Guide sections
- **diagram-generator**: Code examples should reference diagrams
- **content-writer**: Explanations should match writing style