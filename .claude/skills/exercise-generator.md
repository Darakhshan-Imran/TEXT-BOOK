# Skill: Generate Conceptual Exercises

## Purpose
Creates thoughtful, educational exercises that test understanding of Physical AI concepts without requiring actual hardware or simulation environments. Exercises should encourage critical thinking and application of learned concepts.

## Capabilities
- Generate analysis questions
- Create design challenges
- Develop debugging scenarios
- Build comparison exercises
- Provide detailed solutions

## Input Parameters

When using this skill, provide:

1. **chapter_topic**: String (main topic of the chapter)
2. **key_concepts**: Array of 3-5 main concepts covered
3. **difficulty_level**: String ("beginner", "intermediate", "advanced")
4. **number_of_exercises**: Integer (typically 3-5)

## Exercise Types

### Type 1: Analysis Exercise
**Purpose**: Test conceptual understanding

**Format:**
```markdown
### Exercise 1: Conceptual Analysis

**Scenario:** [Describe a realistic situation involving the concept]

**Question:** [Ask students to analyze, explain, or predict what happens]

**What to consider:**
- [Hint 1: aspect to think about]
- [Hint 2: another consideration]
- [Hint 3: potential complications]
```

**Example:**
```markdown
### Exercise 1: Understanding ROS 2 Topics

**Scenario:** You have a robot with three camera sensors. Each camera publishes images to its own topic: `/camera_front`, `/camera_left`, and `/camera_right`. You need to create a node that processes all three image streams simultaneously.

**Question:** 
1. How would you structure your subscriber node to handle all three camera topics?
2. What happens if the cameras publish at different rates (10 Hz, 30 Hz, and 15 Hz)?
3. How would you synchronize the images if you need all three from the same timestamp?

**What to consider:**
- ROS 2 subscriber callback structure
- Message queue sizes and their impact
- Time synchronization mechanisms
- Processing load and threading
```

### Type 2: Design Exercise
**Purpose**: Apply knowledge to solve problems

**Format:**
```markdown
### Exercise 2: System Design Challenge

**Requirements:** [List what the system must do]

**Task:** Design [specific component/system] that meets the requirements

**Constraints:**
- [Constraint 1: limitation to work within]
- [Constraint 2: another limitation]

**Deliverables:**
1. [Architecture diagram or description]
2. [Key design decisions explained]
3. [Alternative approaches considered]
```

**Example:**
```markdown
### Exercise 2: Designing a Perception Pipeline

**Requirements:**
- Detect objects in 3D space using RGB-D camera
- Classify objects as "obstacle" or "target"
- Publish object locations to a topic for navigation
- Run at minimum 10 Hz

**Task:** Design the perception node architecture showing:
- Input data sources
- Processing steps
- Output format
- Error handling

**Constraints:**
- Limited to CPU-only processing (no GPU)
- Must work with RealSense D435i camera
- Maximum 100ms latency

**Deliverables:**
1. System architecture diagram (using Mermaid or description)
2. Explanation of each processing step
3. Justification for design decisions
4. Discussion of trade-offs made
```

### Type 3: Debugging Exercise
**Purpose**: Identify and fix issues

**Format:**
```markdown
### Exercise 3: Debug the System

**Given Code/Configuration:** [Provide code snippet or system description with bugs]

**Observed Behavior:** [What's happening - the symptoms]

**Expected Behavior:** [What should happen]

**Task:**
1. Identify all issues in the code/configuration
2. Explain why each issue causes problems
3. Provide corrected version
4. Suggest how to prevent similar issues
```

**Example:**
```markdown
### Exercise 3: Fix the Publisher Node

**Given Code:**
```python
import rclpy
from std_msgs.msg import String

class BrokenPublisher:
    def __init__(self):
        self.node = rclpy.create_node('publisher')
        self.pub = self.node.create_publisher(String, 'topic')
        self.timer = self.node.create_timer(1.0, self.callback)
    
    def callback(self):
        msg = String()
        msg.data = 'Hello'
        self.pub.publish(msg)

def main():
    rclpy.init()
    publisher = BrokenPublisher()
    rclpy.spin(publisher.node)

main()
```

**Observed Behavior:**
- Node starts but no messages appear on the topic
- Warning about quality of service
- Node crashes after a few seconds

**Expected Behavior:**
- Messages should publish at 1 Hz
- Topic should be visible with `ros2 topic list`
- Node should run indefinitely

**Task:**
1. Identify all issues (there are 4 bugs)
2. Explain what each bug causes
3. Provide the corrected code
4. Explain best practices violated
```

### Type 4: Comparison Exercise
**Purpose**: Understand trade-offs and decision-making

**Format:**
```markdown
### Exercise 4: Compare and Contrast

**Situation:** [Describe a scenario where multiple approaches exist]

**Options:**
- **Option A:** [Description]
- **Option B:** [Description]
- **Option C:** [Description]

**Task:** Compare the options considering:
1. [Criterion 1: e.g., performance]
2. [Criterion 2: e.g., complexity]
3. [Criterion 3: e.g., maintainability]

**Questions:**
- Which option would you choose for [specific use case]? Why?
- What are the trade-offs of each approach?
- How would your choice change if [constraint changed]?
```

**Example:**
```markdown
### Exercise 4: Choosing Communication Patterns

**Situation:** You need to send navigation commands from a planning node to a robot controller node.

**Options:**
- **Option A: Topic (Publish-Subscribe)**
  - Asynchronous communication
  - One-way message flow
  - Multiple subscribers possible

- **Option B: Service (Request-Response)**
  - Synchronous communication
  - Waits for confirmation
  - One-to-one interaction

- **Option C: Action (Goal-based)**
  - Asynchronous with feedback
  - Cancellable operations
  - Progress updates

**Task:** Compare these patterns considering:
1. Latency and responsiveness
2. Reliability and error handling
3. Implementation complexity

**Questions:**
- Which would you use for emergency stop commands? Why?
- Which is best for "navigate to position X"? Why?
- What if the navigation takes 30 seconds?
- How do you handle failures in each approach?
```

### Type 5: Architecture Exercise
**Purpose**: Understand system-level design

**Format:**
```markdown
### Exercise 5: System Architecture Design

**Goal:** [High-level objective]

**Given Components:** [List available resources]

**Task:** Design the complete system architecture addressing:
1. [Aspect 1: e.g., data flow]
2. [Aspect 2: e.g., error handling]
3. [Aspect 3: e.g., scalability]

**Consider:**
- [Consideration 1]
- [Consideration 2]
- [Consideration 3]
```

## Solution Format

Every exercise must include a detailed solution:

```markdown
---

## 💡 Solutions

### Solution to Exercise 1

**Answer:**

[Detailed explanation of the solution]

**Key Points:**
- [Important concept 1]
- [Important concept 2]
- [Important concept 3]

**Why This Matters:**
[Connect the exercise back to real-world applications]

**Common Mistakes:**
- [Mistake 1]: [Why it's wrong]
- [Mistake 2]: [Why it's wrong]

---

### Solution to Exercise 2

[Same structure as above]
```

## Exercise Difficulty Levels

### Beginner
- Single concept focus
- Clear right/wrong answers
- Guided with hints
- 5-10 minutes to complete

### Intermediate
- Multiple concepts integration
- Some ambiguity in solutions
- Requires reasoning
- 15-20 minutes to complete

### Advanced
- Complex scenarios
- Multiple valid approaches
- Deep technical understanding required
- 30-45 minutes to complete

## Quality Standards

Every exercise must:

1. **Be Relevant**: Directly relate to chapter content
2. **Be Realistic**: Reflect real-world scenarios
3. **Be Clear**: Unambiguous requirements
4. **Be Educational**: Teach something through solving it
5. **Have Solutions**: Complete, detailed solutions provided
6. **Be Doable**: Solvable with chapter knowledge alone

## Exercise Set Template

For a typical chapter, include 3-5 exercises covering:

```markdown
## 🧠 Conceptual Exercises

These exercises test your understanding of [chapter topic] without requiring code execution or hardware.

### Exercise 1: Conceptual Analysis
[Type 1 exercise - tests understanding]

### Exercise 2: System Design
[Type 2 exercise - tests application]

### Exercise 3: Debugging Scenario
[Type 3 exercise - tests problem-solving]

### Exercise 4: Comparison & Trade-offs
[Type 4 exercise - tests critical thinking]

### Exercise 5: [Optional Advanced Challenge]
[Type 5 exercise - tests integration]

---

## 💡 Solutions

### Solution to Exercise 1
[Detailed solution]

### Solution to Exercise 2
[Detailed solution]

### Solution to Exercise 3
[Detailed solution]

### Solution to Exercise 4
[Detailed solution]

### Solution to Exercise 5
[Detailed solution]
```

## Example Complete Exercise Set

```markdown
## 🧠 Conceptual Exercises

### Exercise 1: ROS 2 Architecture Analysis

**Scenario:** A robotics team notices their robot responds slowly to commands. Investigation shows the command topic has a queue size of 1, and messages are being published at 100 Hz.

**Questions:**
1. Why might messages be getting dropped?
2. What happens when the queue is full and a new message arrives?
3. How would increasing the queue size to 10 affect the system?
4. What's a better solution than just increasing queue size?

**Hints:**
- Consider publisher-subscriber rate mismatch
- Think about Quality of Service (QoS) settings
- Consider the trade-off between latency and reliability

---

### Exercise 2: Design a Safety System

**Requirements:**
- Monitor robot velocity and position
- Detect if robot is approaching a boundary
- Send emergency stop if boundary crossed
- Resume normal operation when safe

**Task:** Design the node architecture and explain:
- What topics to subscribe to
- What processing is needed
- What topics to publish to
- How to handle edge cases

**Constraints:**
- Must respond within 50ms
- Must work even if some sensors fail
- Cannot modify existing nodes

---

### Exercise 3: Debug the URDF

**Given URDF:**
```xml
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <axis xyz="1 0 0"/>
  </joint>
</robot>
```

**Problem:** Robot won't load in Gazebo, showing "link 'wheel' not found" error.

**Task:**
1. Identify the issue
2. Explain why it causes the error
3. Provide corrected URDF
4. Explain proper URDF structure

---

## 💡 Solutions

### Solution to Exercise 1

**Answers:**

1. **Why messages are dropped:** With a queue size of 1 and publishing at 100 Hz, if the subscriber processes slower than 100 Hz, new messages overwrite old ones in the queue before they're processed.

2. **Queue full behavior:** ROS 2 default QoS uses a "keep last" policy, meaning the oldest message is dropped when a new one arrives and the queue is full.

3. **Effect of queue size 10:** Provides more buffer (100ms at 100 Hz), but if the fundamental rate mismatch continues, you're just delaying the inevitable message drops.

4. **Better solution:** 
   - Match publishing rate to processing capability
   - Use QoS "reliable" mode if no messages can be lost
   - Implement message filtering or downsampling
   - Add flow control to slow down publisher

**Key Concepts:**
- Queue size is a buffer, not a solution to rate mismatches
- QoS policies affect message handling
- System design should consider worst-case scenarios

**Real-World Application:**
This scenario is common in sensor data processing where high-frequency sensors overwhelm processing nodes. Understanding these principles prevents data loss in production systems.

---

[Continue with other solutions...]
```

## Integration with Other Skills

This skill works with:
- **chapter-structure**: Exercises fill the "Conceptual Exercises" section
- **content-writer**: Exercise scenarios should match chapter examples
- **code-example**: Debugging exercises can reference code examples from chapter