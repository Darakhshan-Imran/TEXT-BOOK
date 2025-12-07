# Skill: Generate Mermaid Diagrams

## Purpose
Creates clear, professional Mermaid diagrams for visualizing Physical AI concepts, system architectures, data flows, and processes. All diagrams must be rendered in Docusaurus using Mermaid syntax.

## Capabilities
- Generate flowcharts for processes and algorithms
- Create sequence diagrams for component interactions
- Build architecture diagrams for system components
- Design state diagrams for robot states
- Make class diagrams for software structure

## Input Parameters

When using this skill, provide:

1. **diagram_type**: String ("flowchart", "sequence", "architecture", "state", "class")
2. **concept**: String (what to visualize - e.g., "ROS 2 message flow")
3. **components**: Array of component names to include
4. **complexity**: String ("simple", "detailed")
5. **title**: String (diagram title/caption)

## Mermaid Syntax Reference

### Flowchart (Process Flow)

**Use for:** Algorithms, decision trees, process flows, pipelines

```mermaid
flowchart TD
    A[Start] --> B{Decision Point?}
    B -->|Yes| C[Process A]
    B -->|No| D[Process B]
    C --> E[Combine Results]
    D --> E
    E --> F[End]
    
    style A fill:#e1f5e1
    style F fill:#ffe1e1
    style B fill:#fff4e1
```

**Basic Shapes:**
- `[Rectangle]` - Process/Step
- `{Diamond}` - Decision
- `([Rounded])` - Start/End
- `[[Subroutine]]` - Sub-process
- `[(Database)]` - Storage
- `((Circle))` - Connection point

**Arrows:**
- `-->` - Solid arrow
- `-.->` - Dotted arrow
- `==>` - Thick arrow
- `--text-->` - Labeled arrow

### Sequence Diagram (Interactions)

**Use for:** Node communication, API calls, message passing, time-based interactions

```mermaid
sequenceDiagram
    participant User
    participant Node A
    participant Node B
    participant Robot
    
    User->>Node A: Send Command
    activate Node A
    Node A->>Node B: Request Data
    activate Node B
    Node B-->>Node A: Return Data
    deactivate Node B
    Node A->>Robot: Execute Action
    activate Robot
    Robot-->>Node A: Action Complete
    deactivate Robot
    Node A-->>User: Confirmation
    deactivate Node A
```

**Interactions:**
- `->>` - Solid line message
- `-->>` - Dashed line response
- `-x` - Lost message
- `activate/deactivate` - Show active period

### Architecture Diagram (System Components)

**Use for:** System overview, component relationships, data flow

```mermaid
graph TB
    subgraph "Physical Layer"
        A[Sensors: Camera, LIDAR, IMU]
        B[Actuators: Motors, Servos]
    end
    
    subgraph "Middleware Layer"
        C[ROS 2 Core]
        D[Topics & Services]
    end
    
    subgraph "Application Layer"
        E[Perception Node]
        F[Planning Node]
        G[Control Node]
    end
    
    A -->|Raw Data| E
    E -->|Object Detection| F
    F -->|Path Plan| G
    G -->|Commands| B
    
    E -.->|Publish| D
    F -.->|Subscribe| D
    G -.->|Publish| D
    
    style A fill:#e3f2fd
    style B fill:#e3f2fd
    style C fill:#fff3e0
    style D fill:#fff3e0
    style E fill:#f3e5f5
    style F fill:#f3e5f5
    style G fill:#f3e5f5
```

**Subgraphs:**
- Group related components
- Show layers or modules
- Use descriptive names

### State Diagram (Robot States)

**Use for:** Robot state machines, mode transitions, error handling

```mermaid
stateDiagram-v2
    [*] --> Idle
    
    Idle --> Initializing: Power On
    Initializing --> Ready: Init Complete
    
    Ready --> Moving: Start Command
    Ready --> Idle: Shutdown
    
    Moving --> Processing: Reached Target
    Moving --> Emergency: Obstacle Detected
    
    Processing --> Ready: Task Complete
    Processing --> Error: Processing Failed
    
    Emergency --> Idle: Manual Reset
    Error --> Idle: Error Resolved
    
    note right of Emergency
        Safety state
        All motors stopped
    end note
```

**States:**
- `[*]` - Initial/final state
- `State --> State` - Transition
- `note` - Add explanations

### Class Diagram (Software Structure)

**Use for:** Code architecture, inheritance, relationships

```mermaid
classDiagram
    class Node {
        +String name
        +Publisher[] publishers
        +Subscription[] subscriptions
        +init()
        +spin()
        +destroy()
    }
    
    class PublisherNode {
        +Timer timer
        +int counter
        +timer_callback()
        +publish_message()
    }
    
    class SubscriberNode {
        +callback(msg)
        +process_data(data)
    }
    
    Node <|-- PublisherNode
    Node <|-- SubscriberNode
    
    PublisherNode --> Message: publishes
    SubscriberNode --> Message: receives
    
    class Message {
        +String data
        +Timestamp timestamp
    }
```

## Standard Color Scheme

Use consistent colors for clarity:

```
- Blue (#e3f2fd): Hardware/Physical components
- Orange (#fff3e0): Middleware/Communication
- Purple (#f3e5f5): Application/Software
- Green (#e1f5e1): Success/Start states
- Red (#ffe1e1): Error/End states
- Yellow (#fff4e1): Warning/Decision points
- Gray (#f5f5f5): Inactive/Background
```

## Output Format

Every diagram must include:

1. **Context Paragraph** (before diagram): Explain what the diagram shows
2. **Mermaid Code Block**: Complete, valid Mermaid syntax
3. **Caption** (after diagram): Title and brief description
4. **Explanation** (after diagram): Detailed walkthrough of components

## Example Output Structure

````markdown
### ROS 2 Publisher-Subscriber Communication

Understanding how nodes communicate is fundamental to ROS 2. The following diagram illustrates the message flow between a publisher node and subscriber node through a topic.

```mermaid
sequenceDiagram
    participant P as Publisher Node
    participant T as Topic (/sensor_data)
    participant S as Subscriber Node
    
    Note over P: Timer triggers callback
    P->>T: Publish message
    Note over T: Message queued
    T->>S: Deliver message
    Note over S: Process in callback
    S-->>S: Execute logic
```

**Figure 1:** ROS 2 Publisher-Subscriber Communication Pattern

**Explanation:**

This sequence diagram shows the asynchronous communication pattern in ROS 2:

1. **Publisher Node**: A timer triggers the callback function, which creates and publishes a message
2. **Topic**: Acts as a message broker, queuing messages for delivery
3. **Subscriber Node**: Receives the message and processes it in its callback function
4. **Asynchronous**: The publisher doesn't wait for subscribers - it publishes and continues

**Key Points:**
- Communication is one-way (publisher → subscriber)
- Topics can have multiple publishers and subscribers
- Messages are queued if subscribers are slow
- No direct connection between nodes - only through topics

**When to Use This Pattern:**
- Sensor data broadcasting
- State information sharing
- One-to-many communication
- Fire-and-forget messaging
````

## Diagram Complexity Guidelines

### Simple Diagrams
- 3-5 components/nodes
- Single layer or flow
- Clear, focused concept
- Minimal text labels

### Detailed Diagrams
- 6-15 components/nodes
- Multiple layers or subgraphs
- Complete system view
- Descriptive labels and notes

## Common Diagram Templates

### Template 1: ROS 2 Node Architecture
```mermaid
graph LR
    subgraph "Node Name"
        A[Subscribers] --> B[Processing Logic]
        B --> C[Publishers]
        D[Services] --> B
        B --> E[Action Servers]
    end
    
    F[External Topics] --> A
    C --> G[Output Topics]
```

### Template 2: Sensor Pipeline
```mermaid
flowchart LR
    A[Raw Sensor Data] --> B[Preprocessing]
    B --> C[Feature Extraction]
    C --> D[Algorithm Processing]
    D --> E[Output/Action]
    
    style A fill:#e3f2fd
    style E fill:#e1f5e1
```

### Template 3: Navigation Stack
```mermaid
graph TB
    A[Goal Position] --> B[Global Planner]
    B --> C[Local Planner]
    D[Sensor Data] --> E[Costmap]
    E --> C
    C --> F[Velocity Commands]
    F --> G[Robot Base]
```

### Template 4: State Machine
```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> Active: Start
    Active --> Idle: Stop
    Active --> Error: Failure
    Error --> Idle: Reset
```

## Best Practices

1. **Keep It Focused**: One concept per diagram
2. **Use Consistent Naming**: Match code and text references
3. **Add Legends**: Explain colors and symbols if not obvious
4. **Label Everything**: All nodes and edges should be labeled
5. **Direction Matters**: Left-to-right or top-to-bottom flow
6. **Highlight Critical Paths**: Use colors or thick lines
7. **Add Notes**: For complex sections, add explanatory notes
8. **Test Rendering**: Verify diagram displays correctly in Docusaurus

## Quality Checklist

Before finalizing any diagram:

- [ ] Valid Mermaid syntax (renders without errors)
- [ ] All components labeled clearly
- [ ] Consistent color scheme used
- [ ] Arrows show correct direction
- [ ] Context paragraph provided
- [ ] Caption/title included
- [ ] Detailed explanation written
- [ ] Appropriate complexity level
- [ ] Matches chapter content
- [ ] Mobile-friendly (not too wide)

## Troubleshooting

### Diagram Not Rendering
- Check Mermaid syntax validity
- Verify Docusaurus has `@docusaurus/theme-mermaid` plugin
- Ensure code block uses ```mermaid not ```markdown

### Diagram Too Complex
- Split into multiple simpler diagrams
- Use subgraphs to organize
- Focus on one aspect at a time

### Colors Not Showing
- Use `style` command: `style A fill:#color`
- Check color hex codes are valid

## Integration with Other Skills

This skill works with:
- **chapter-structure**: Diagrams go in architecture sections
- **code-example**: Visualize code structure and flow
- **content-writer**: Support text explanations with visuals