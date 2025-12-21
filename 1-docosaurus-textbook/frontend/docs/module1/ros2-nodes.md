# Understanding ROS 2 Nodes & Communication

Now that you've seen ROS 2 in action, it's time to understand **how nodes communicate**. In this lesson, you'll learn about the publisher-subscriber pattern, services, and message types—the building blocks of every ROS 2 system.

---

## What is a Node?

A **node** is an independent executable program (a process) that performs a specific task in your robot system.

**Examples of Nodes**:
- `camera_driver_node`: Reads from a camera, publishes images
- `object_detector_node`: Subscribes to images, publishes detected objects
- `motion_planner_node`: Plans robot movements
- `motor_controller_node`: Sends commands to motors

**Why Multiple Nodes?**
- **Modularity**: Each node does one thing well (Unix philosophy)
- **Fault Isolation**: If one node crashes, others keep running
- **Scalability**: Run nodes on different computers (distributed computing)
- **Reusability**: Swap out nodes without rewriting everything

---

## Communication Pattern 1: Topics (Publisher-Subscriber)

**Topics** are the most common way for nodes to communicate. They implement a **many-to-many, asynchronous** communication pattern.

### How Topics Work

```
┌───────────────┐                          ┌─────────────────┐
│  Publisher 1  │ ──┐                  ┌──>│  Subscriber 1   │
└───────────────┘   │                  │   └─────────────────┘
                    │   /sensor/data   │
┌───────────────┐   ├─────────────────>│   ┌─────────────────┐
│  Publisher 2  │ ──┘     (Topic)      └──>│  Subscriber 2   │
└───────────────┘                          └─────────────────┘
                                            ┌─────────────────┐
                                        └──>│  Subscriber 3   │
                                            └─────────────────┘
```

**Key Characteristics**:
- **Asynchronous**: Publishers don't wait for subscribers
- **Decoupled**: Publishers and subscribers don't know about each other
- **Many-to-Many**: Multiple publishers and subscribers can use the same topic
- **Typed**: Each topic has a specific message type

### Example: Temperature Sensor

**Publisher Node** (sends temperature data):

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class TemperatureSensor(Node):
    """Simulates a temperature sensor publishing data."""

    def __init__(self):
        super().__init__('temperature_sensor')

        # Create publisher on /temperature topic
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)

        # Publish every 1 second
        self.timer = self.create_timer(1.0, self.publish_temperature)

        self.get_logger().info('Temperature sensor started!')

    def publish_temperature(self):
        """Simulate temperature reading with noise."""
        temperature = 20.0 + random.uniform(-2.0, 2.0)  # 18-22°C

        msg = Float32()
        msg.data = temperature

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published temperature: {temperature:.2f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Subscriber Node** (receives temperature data):

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TemperatureMonitor(Node):
    """Monitors temperature and warns if too hot."""

    def __init__(self):
        super().__init__('temperature_monitor')

        # Subscribe to /temperature topic
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10
        )

        self.threshold = 21.5  # Warning threshold
        self.get_logger().info(f'Monitoring temperature (threshold: {self.threshold}°C)')

    def temperature_callback(self, msg):
        """Called whenever a temperature message arrives."""
        temp = msg.data

        if temp > self.threshold:
            self.get_logger().warn(f'⚠️  High temperature detected: {temp:.2f}°C')
        else:
            self.get_logger().info(f'✅ Temperature normal: {temp:.2f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Running the Example**:
```bash
# Terminal 1: Run sensor
python3 temperature_sensor.py

# Terminal 2: Run monitor
python3 temperature_monitor.py

# Terminal 3: Inspect the topic
ros2 topic echo /temperature
ros2 topic hz /temperature  # Check publish rate
```

---

## Communication Pattern 2: Services (Request-Reply)

**Services** enable **synchronous, one-to-one** communication, similar to function calls.

### When to Use Services

- **One-time actions**: "Start calibration," "Take a picture now"
- **Queries**: "What's the robot's current pose?"
- **Configuration**: "Change navigation speed to 0.5 m/s"

**Not for**: High-frequency data streams (use topics instead)

### Service Structure

```
Client Node                      Server Node
    |                                |
    |──── Request (parameters) ────>|
    |                                |
    |                          (Processing)
    |                                |
    |<──── Response (result) ───────|
    |                                |
```

### Example: Add Two Numbers Service

**Server Node**:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """Service that adds two integers."""

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Add Two Ints server ready!')

    def add_two_ints_callback(self, request, response):
        """Called when client makes a request."""
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Client Node**:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    """Client that calls the addition service."""

    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.get_logger().info('Service available!')

    def send_request(self, a, b):
        """Send a request to add two numbers."""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call service (blocks until response)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        self.get_logger().info(f'Result: {a} + {b} = {result.sum}')
        return result.sum


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()

    # Example usage
    node.send_request(5, 7)
    node.send_request(10, 20)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Message Types

ROS 2 uses **strongly-typed messages** to ensure data consistency.

### Common Message Packages

| Package | Purpose | Example Types |
|---------|---------|---------------|
| `std_msgs` | Basic types | `String`, `Int32`, `Float64`, `Bool` |
| `geometry_msgs` | Geometry/motion | `Point`, `Pose`, `Twist`, `Transform` |
| `sensor_msgs` | Sensor data | `Image`, `LaserScan`, `Imu`, `JointState` |
| `nav_msgs` | Navigation | `Odometry`, `Path`, `OccupancyGrid` |

### Example: geometry_msgs/Twist (Velocity Command)

```python
from geometry_msgs.msg import Twist

# Create a velocity command message
cmd_vel = Twist()

# Set linear velocity (forward/backward)
cmd_vel.linear.x = 0.5  # Move forward at 0.5 m/s
cmd_vel.linear.y = 0.0  # No lateral movement
cmd_vel.linear.z = 0.0  # No vertical movement (for ground robots)

# Set angular velocity (rotation)
cmd_vel.angular.x = 0.0  # No roll
cmd_vel.angular.y = 0.0  # No pitch
cmd_vel.angular.z = 0.3  # Turn left at 0.3 rad/s

# Publish it
publisher.publish(cmd_vel)
```

**Use Case**: Controlling a mobile robot or humanoid base.

---

## Quality of Service (QoS)

QoS policies control **how messages are delivered** (reliability vs. performance trade-off).

### Common QoS Profiles

| Profile | Reliability | Use Case |
|---------|-------------|----------|
| `Sensor Data` | Best-effort, volatile | High-frequency sensor streams (OK to drop messages) |
| `Services` | Reliable, transient-local | Service calls (must not lose request/response) |
| `Parameters` | Reliable, volatile | Configuration changes |
| `Default` | Reliable, volatile | General-purpose |

**Example**:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# High-frequency sensor (OK to drop old data)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Critical command (must not lose messages)
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Create publisher with QoS
self.publisher_ = self.create_publisher(Twist, '/cmd_vel', command_qos)
```

---

## Node Lifecycle Management

Nodes have a lifecycle for safe startup/shutdown:

```bash
# List running nodes
ros2 node list

# Get info about a node
ros2 node info /temperature_sensor

# Kill a node (sends SIGINT)
# Use Ctrl+C in the node's terminal
```

**Graceful Shutdown** (best practice):
```python
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Gracefully handle Ctrl+C
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

---

## Mini Exercise: Build a Heartbeat Monitor

**Challenge**: Create a system where a "heartbeat" node publishes "alive" messages, and a "watchdog" node monitors them.

**Requirements**:
1. **Heartbeat Node**: Publishes `std_msgs/String` with message "alive" every 1 second on `/heartbeat`
2. **Watchdog Node**: Subscribes to `/heartbeat`. If no message received for 3 seconds, print "⚠️  HEARTBEAT LOST!"

**Hints**:
- Use `self.create_timer()` for periodic publishing
- Use `self.get_clock().now()` to track last message time
- Store timestamp of last received message in callback

**Extension**: Add a service `/reset_watchdog` that resets the timer.

---

## Summary

You've learned the fundamental communication patterns in ROS 2:

✅ **Nodes** are independent processes that encapsulate specific functionality
✅ **Topics** enable asynchronous, many-to-many communication (pub-sub pattern)
✅ **Services** enable synchronous, request-reply communication
✅ **Messages** are strongly-typed data structures (e.g., `Float32`, `Twist`, `Image`)
✅ **QoS** policies control message delivery (reliability vs. performance)
✅ **Lifecycle management** ensures graceful startup and shutdown

**Next Up**: You'll learn how to integrate Python AI agents with ROS 2, combining machine learning with robot control!

Ready to bridge AI and robotics? Let's continue! 🧠🤖
