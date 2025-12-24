# Research: Skills and Subagents Framework - ROS 2 Integration Patterns

**Date**: 2025-12-19
**Status**: Complete
**Purpose**: Validate ROS 2 integration patterns for agent-based humanoid robotics systems with concurrent skill execution

---

## Executive Summary

This research document identifies the optimal ROS 2 integration patterns for the Skills and Subagents Framework targeting humanoid robotics. Five critical patterns are evaluated:

1. **ROS 2 Action Servers/Clients** for skill execution (P1 priority)
2. **Lifecycle Nodes** for agent lifecycle management (P2 priority)
3. **Message Passing Patterns** (Pub/Sub + Services) (P1 priority)
4. **Executor Architecture** for concurrent agent execution (P2 priority)
5. **Multi-Agent Coordination** patterns (P3 priority)

**Recommended Approach**: Hybrid pattern combining ROS 2 Actions (skill execution) + Lifecycle Nodes (lifecycle management) + Pub/Sub (inter-agent communication) + MultiThreadedExecutor (concurrent execution).

---

## Pattern 1: ROS 2 Actions for Skill Execution

### Technical Overview

ROS 2 Actions implement a long-running, goal-oriented communication pattern ideal for skills. They provide:
- **Goal**: Initial request with parameters
- **Feedback**: Periodic status updates during execution
- **Result**: Final outcome when complete
- **Cancellation**: Ability to preempt execution

### Why Actions Are Ideal for Skills

1. **Skill-Specific Pattern**: Actions model the skill lifecycle (start → execute → complete/fail)
2. **Feedback Loop**: Allows monitoring skill progress (e.g., "arm at 45%, gripper closing")
3. **Cancellation Support**: Enable timeout and graceful shutdown per FR-009
4. **Native ROS 2 Pattern**: Integrated into rclpy; no custom middleware needed
5. **Suitable for Concurrent Execution**: Multiple action servers run in parallel

### Code Example: Skill as Action

```python
from rclpy.action import ActionServer
import rclpy

class SkillActionServer(rclpy.node.Node):
    def __init__(self):
        super().__init__('skill_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            execute_callback=self.execute_skill_callback
        )

    async def execute_skill_callback(self, goal_handle):
        """Execute skill with feedback loop"""
        try:
            for progress in range(0, 101, 10):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return NavigateToPose.Result(success=False)

                feedback_msg = NavigateToPose.Feedback()
                feedback_msg.progress = progress
                goal_handle.publish_feedback(feedback_msg)
                await asyncio.sleep(1.0)

            goal_handle.succeed()
            result = NavigateToPose.Result()
            result.success = True
            return result
        except Exception as e:
            goal_handle.abort()
            return NavigateToPose.Result(success=False)
```

### Performance Characteristics

| Metric | Value | Notes |
|--------|-------|-------|
| Action round-trip latency | 10-50ms | Local ROS 2 domain |
| Feedback update rate | 1-100 Hz | Configurable |
| Concurrent actions | 10-100+ | CPU/memory limited |
| Timeout handling | Native | Built-in support |

### Suitability for Concurrent Agents

✅ **Excellent** — Each action server handles requests independently without blocking others.

### Real-World Examples

1. **MoveIt 2**: Trajectory execution actions
2. **Nav2**: NavigateToPose action for autonomous navigation
3. **TurtleBot4**: Movement and rotation actions

---

## Pattern 2: ROS 2 Lifecycle Nodes for Agent Management

### Technical Overview

Lifecycle Nodes implement safe state transitions for initialization and shutdown:

```
Unconfigured → Inactive → Active → Inactive → Unconfigured → Finalized
     ↓         ↑  ↑          ↑        ↓          ↓
  configure  on_activate on_deactivate cleanup  shutdown
```

### Why Lifecycle Nodes for Subagents

1. **Structured Initialization**: Separate resource allocation from activation
2. **Error Isolation**: Failed agent cleanup doesn't affect others
3. **Coordinated Shutdown**: All agents transition cleanly
4. **Resource Management**: Configure phase allocates skills and entities
5. **Monitoring**: Query state to track agent status

### Code Example: Subagent Lifecycle

```python
from rclpy_lifecycle.node import LifecycleNode
from rclpy_lifecycle.transitions import TransitionCallbackReturn

class SkillSubagent(LifecycleNode):
    def __init__(self, agent_id, skill_name):
        super().__init__(f'subagent_{agent_id}')
        self.agent_id = agent_id
        self.skill_name = skill_name

    def on_configure(self, state):
        """Allocate resources (Unconfigured → Inactive)"""
        try:
            self.skill_instance = self.load_skill(self.skill_name)
            self.subscription = self.create_subscription(
                Float64, f'/sensor_{self.agent_id}',
                self.sensor_callback, 10
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception:
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        """Start execution (Inactive → Active)"""
        self.timer = self.create_timer(0.1, self.execute_skill_loop)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Stop gracefully (Active → Inactive)"""
        self.destroy_timer(self.timer)
        if self.skill_instance:
            self.skill_instance.cleanup()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """Release all resources (Inactive → Unconfigured)"""
        self.destroy_subscription(self.subscription)
        if self.skill_instance:
            self.skill_instance.release()
        return TransitionCallbackReturn.SUCCESS
```

### Benefits for Concurrent Agents

| Benefit | Mechanism |
|---------|-----------|
| Safe init | Separate configure and activate |
| Isolation | Failed agent doesn't crash system |
| Monitoring | Query lifecycle state for status |
| Cleanup | on_cleanup prevents resource leaks |

### Suitability for Concurrent Agents

✅ **Excellent** — Each agent manages lifecycle independently; LifecycleManager coordinates transitions.

---

## Pattern 3: Pub/Sub Message Passing for Inter-Agent Communication

### Technical Overview

ROS 2 Pub/Sub provides asynchronous topic-based communication:
- **Loose coupling**: Agents don't know each other
- **Broadcast**: One publisher, many subscribers
- **Scalable**: Adding agents doesn't require changes

### Comparison with Other Patterns

| Pattern | Use Case | Latency | Coupling |
|---------|----------|---------|----------|
| **Pub/Sub** | Continuous streams, broadcasts | 10-50ms | Loose |
| **Services** | One-off requests/response | 10-50ms | Tight |
| **Actions** | Long-running goals | 10-50ms | Moderate |

### Code Example: Inter-Agent Communication

```python
class SubagentA(rclpy.node.Node):
    def __init__(self):
        super().__init__('subagent_a')

        # Publish status for other agents
        self.status_pub = self.create_publisher(String, '/status_a', 10)

        # Subscribe to commands
        self.command_sub = self.create_subscription(
            String, '/commands', self.command_callback, 10
        )

        # Monitor other agents
        self.status_b_sub = self.create_subscription(
            String, '/status_b', self.status_b_callback, 10
        )

    def publish_status(self):
        """Broadcast status"""
        status = {
            "agent_id": "a",
            "task": self.current_task,
            "timestamp": self.get_clock().now().to_msg()
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def command_callback(self, msg):
        """Receive commands"""
        cmd = json.loads(msg.data)
        self.current_task = cmd.get("task")
```

### QoS for Reliable Coordination

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Reliable for critical messages
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=10
)
self.status_pub = self.create_publisher(
    String, '/status', qos_profile=reliable_qos
)

# Best-effort for high-frequency telemetry
best_effort = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1
)
```

### Performance Characteristics

| Metric | Value | Notes |
|--------|-------|-------|
| Publish latency | 1-10ms | Domain socket |
| Callback latency | 5-20ms | Executor dependent |
| Message throughput | 10K+ msgs/sec | Per topic |
| Concurrent pub/sub | Unlimited | Multiplexed by ROS 2 |

### Suitability for Concurrent Agents

✅ **Excellent** — Pub/Sub is fundamentally asynchronous; scales to many agents.

---

## Pattern 4: Executor Architecture for Concurrent Execution

### Technical Overview

Executors manage callback execution. Types available:

| Executor | Threading | Concurrency | Best For |
|----------|-----------|-------------|----------|
| SingleThreaded | Single | Sequential | Lightweight |
| **MultiThreaded** | Thread pool | Parallel | **Multiple agents** ✅ |
| StaticSingleThreaded | Single | Sequential | Deterministic |
| Events | Event-driven | Efficient | Heterogeneous |

### Recommended: MultiThreadedExecutor

```python
def main():
    rclpy.init()

    # Create multiple subagents
    agents = []
    for i in range(5):
        agent = SkillSubagent(agent_id=i, skill_name=f'skill_{i}')
        agents.append(agent)

    # MultiThreadedExecutor for concurrent execution
    executor = MultiThreadedExecutor(num_threads=4)

    for agent in agents:
        executor.add_node(agent)

    # All agents execute in parallel without blocking
    executor.spin()
    rclpy.shutdown()
```

### Thread-Safe State Sharing

```python
from threading import Lock

class SubagentState:
    """Thread-safe state for callback coordination"""

    def __init__(self):
        self.lock = Lock()
        self.data = {}

    def get(self, key):
        with self.lock:
            return self.data.get(key)

    def set(self, key, value):
        with self.lock:
            self.data[key] = value

shared_state = SubagentState()

def sensor_callback(self, msg):
    """Called by executor thread T1"""
    shared_state.set('last_sensor', msg.data)

def execute_loop(self):
    """Called by executor thread T2"""
    value = shared_state.get('last_sensor')  # Thread-safe read
```

### Performance Characteristics

| Metric | Value | Notes |
|--------|-------|-------|
| Callback latency | 1-5ms | With 4 threads |
| Agent memory | ~10MB | Per Python process |
| Concurrent agents | 10-20 | CPU/memory dependent |
| Idle overhead | ~1% CPU | Negligible |

### Suitability for Concurrent Agents

✅ **Excellent** — MultiThreadedExecutor designed for parallel callback execution.

---

## Pattern 5: Multi-Agent Coordination

### Centralized Coordination

```
┌─────────────────┐
│ Coordinator     │
└─────────────────┘
    │   │   │   │
    ▼   ▼   ▼   ▼
[Nav][Grasp][Scan][Place]
```

**Use Case**: Complex skill sequences (e.g., fetch = navigate + detect + grasp + return)

```python
class SkillCoordinator(rclpy.node.Node):
    async def execute_fetch_object(self, target_pose):
        """Composite skill orchestration"""
        # Step 1: Navigate
        nav_result = await self.execute_action(self.nav_client, target_pose)
        if not nav_result.success:
            return False

        # Step 2: Detect (2 seconds)
        await asyncio.sleep(2.0)

        # Step 3: Grasp
        grasp_result = await self.execute_action(self.grasp_client, object_id)
        if not grasp_result.success:
            return False

        # Step 4: Return
        return await self.execute_action(self.nav_client, home_pose)
```

### Decentralized Coordination

```
[Perception] ──publish──> [Grasping]
   Agent A        Topic       Agent B
```

**Use Case**: Loosely-coupled agents (e.g., perception publishes detections, grasping subscribes)

```python
class PerceptionAgent(SkillSubagent):
    def detect_loop(self):
        detection = Detection()
        detection.object = "cup"
        self.detection_pub.publish(detection)

class GraspingAgent(SkillSubagent):
    def detection_callback(self, msg):
        self.pending_grasps.append(msg)
```

### Deadlock Prevention

**Timeout-Based Solution**:
```python
async def execute_with_timeout(self, action, timeout=5.0):
    goal = self._action_client.send_goal_async(action)
    done, _ = asyncio.wait([goal], timeout=timeout)
    if not done:
        goal.cancel_goal()  # Timeout: cancel and fail
        return False
    return goal.get_result_async()
```

**Resource Ordering (Banker's Algorithm)**:
```python
class ResourceManager:
    ORDER = {'arm': 0, 'gripper': 1, 'base': 2}

    def acquire_resources(self, agent_id, resources):
        """Acquire in canonical order to prevent deadlock"""
        ordered = sorted(resources, key=lambda r: self.ORDER[r])
        for res in ordered:
            self.acquire(agent_id, res)
```

---

## Integration Recommendations

### Recommended Hybrid Approach

**Tier 1 (P1): Foundation**
- **ROS 2 Actions** for skill execution (goal/feedback/result)
- **Lifecycle Nodes** for subagent lifecycle (configure/activate/cleanup)
- **MultiThreadedExecutor** for concurrent execution (no blocking)

**Tier 2 (P2): Coordination**
- **Pub/Sub (Topics)** for inter-agent communication (loose coupling)
- **Centralized Coordinator** for skill composition (complex sequences)

**Tier 3 (P3): Advanced**
- **Decentralized patterns** for event-driven coordination
- **Resource ordering** to prevent deadlocks

### Full Architecture Diagram

```
┌──────────────────────────────────────────┐
│ Skills & Subagents Framework             │
├──────────────────────────────────────────┤
│                                          │
│  ┌────────────────────────────────────┐ │
│  │ Skill Coordinator (Tier 2)         │ │
│  │ - Load/register skills             │ │
│  │ - Compose complex skills           │ │
│  │ - Orchestrate agents               │ │
│  └────────────────────────────────────┘ │
│                  ▲                       │
│  ┌───────────────┴──────────────────┐  │
│  │ MultiThreadedExecutor (Tier 1)   │  │
│  │                                  │  │
│  │ ┌───────┐ ┌───────┐ ┌───────┐  │  │
│  │ │Agent1 │ │Agent2 │ │Agent3 │  │  │
│  │ │       │ │       │ │       │  │  │
│  │ │Action │ │Action │ │Pub/   │  │  │
│  │ │Server │ │Server │ │Sub    │  │  │
│  │ │       │ │       │ │       │  │  │
│  │ │Lifecycle Lifecycle Lifecycle   │  │
│  │ │Node   │ │Node   │ │Node   │  │  │
│  │ └───────┘ └───────┘ └───────┘  │  │
│  └──────────────────────────────────┘  │
│                  ▲                      │
│                  │ ROS 2 Topics/Actions│
│                  ▼                      │
│  ┌──────────────────────────────────┐  │
│  │ Robot Drivers, Gazebo, Sensors   │  │
│  └──────────────────────────────────┘  │
│                                         │
└─────────────────────────────────────────┘
```

---

## Implementation Roadmap

### Phase 1: Foundation (Weeks 1-2)
- [ ] Skill definition interface (FR-001)
- [ ] ROS 2 Action-based executor (FR-004)
- [ ] Subagent LifecycleNode (FR-005, FR-013)
- [ ] MultiThreadedExecutor setup
- [ ] Unit tests (5+ concurrent agents)

### Phase 2: Coordination (Weeks 3-4)
- [ ] Inter-agent Pub/Sub communication (FR-008)
- [ ] Skill registry with composition (FR-003, FR-011)
- [ ] Centralized coordinator
- [ ] Timeout/cancellation (FR-009)
- [ ] Integration tests

### Phase 3: Robustness (Weeks 5-6)
- [ ] Failure handling and recovery (FR-010)
- [ ] Resource limits and monitoring (FR-015)
- [ ] Execution logging (FR-014)
- [ ] Performance optimization
- [ ] Acceptance tests

---

## Performance Benchmarks

From spec success criteria:

- **SC-001**: Define & execute skill in <5 min → Actions + Lifecycle
- **SC-002**: 10+ concurrent subagents → MultiThreadedExecutor
- **SC-003**: Failure detection in <1s → Built-in action timeout
- **SC-004**: Message delivery 99.9% <100ms → Reliable QoS + local sockets
- **SC-007**: Subagent crash doesn't affect system → Lifecycle isolation

---

## Conclusion

**Recommended Integration Approach**:

1. **ROS 2 Actions** for skill execution (goal → feedback → result)
2. **LifecycleNodes** for subagent management (safe init/cleanup)
3. **MultiThreadedExecutor** for concurrent execution (no blocking)
4. **Pub/Sub (Topics)** for inter-agent communication (loose coupling)
5. **Timeout-based prevention** for deadlocks (every blocking op has timeout)

This hybrid approach scales to 10-20 concurrent agents with clear semantics for skills, lifecycle, and coordination.

---

## References

### Official ROS 2 Documentation

1. ROS 2 Actions: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Actions.html
2. ROS 2 Lifecycle: https://design.ros2.org/articles/node_lifecycle.html
3. ROS 2 Executors: https://docs.ros.org/en/humble/Concepts/Intermediate/ROS-2-Executors.html
4. rclpy: https://docs.ros.org/en/humble/p/rclpy/
5. ROS 2 QoS: https://docs.ros.org/en/humble/Concepts/Intermediate/About-ROS-2-QoS.html

### Key Projects Using These Patterns

1. Nav2: https://github.com/ros-planning/navigation2
2. MoveIt2: https://github.com/ros-planning/moveit2
3. Gazebo-ROS 2: https://github.com/gazebosim/ros_gz

---

**Status**: ✅ Complete | **Date**: 2025-12-19 | **Next**: Implement Phase 1 foundation
