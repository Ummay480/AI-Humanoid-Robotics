# Quickstart Guide: Skills and Subagents Framework

**Version**: 1.0.0
**Last Updated**: 2025-12-19
**Estimated Time**: 15 minutes

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Basic Concepts](#basic-concepts)
4. [Your First Skill](#your-first-skill)
5. [Running a Subagent](#running-a-subagent)
6. [Skill Composition](#skill-composition)
7. [Inter-Agent Communication](#inter-agent-communication)
8. [Next Steps](#next-steps)

---

## Prerequisites

Before starting, ensure you have:

- **Python 3.11+** installed
- **Basic Python knowledge** (functions, async/await)
- **Text editor** or IDE (VS Code recommended)
- **Terminal** access

**Optional**:
- **ROS 2 Humble** (for robotics integration)
- **Docker** (for running examples in containers)

---

## Installation

### Step 1: Clone or Install the Framework

```bash
# Option A: Install from PyPI (when published)
pip install skills-subagents-framework

# Option B: Install from source
git clone https://github.com/your-org/skills-subagents.git
cd skills-subagents
pip install -e .
```

### Step 2: Verify Installation

```bash
python -c "from skills_framework import SkillRegistry, SubagentManager; print('Installation successful!')"
```

---

## Basic Concepts

### Skills
Reusable capabilities your robot can perform (e.g., "navigate", "detect objects", "grasp").

### Subagents
Autonomous workers that execute skills concurrently.

### Skill Registry
Central catalog of all available skills.

### Messages
Communication between subagents (events and direct messages).

---

## Your First Skill

### Define a Simple Skill

Create `my_first_skill.py`:

```python
import asyncio
from skills_framework import SkillRegistry, SkillMetadata

# Define skill metadata
@SkillRegistry.register(SkillMetadata(
    name="greet",
    version="1.0.0",
    timeout=10,
    parameters_schema={
        "type": "object",
        "properties": {
            "name": {"type": "string"}
        },
        "required": ["name"]
    }
))
async def greet_skill(name: str) -> dict:
    """
    A simple greeting skill.

    Args:
        name: Person to greet

    Returns:
        dict with greeting message
    """
    print(f"Hello, {name}!")
    await asyncio.sleep(1)  # Simulate work
    return {
        "status": "success",
        "message": f"Hello, {name}!"
    }

# Test the skill
async def main():
    # Get skill from registry
    skill = SkillRegistry.get_skill("greet")

    # Execute it
    result = await skill(name="Alice")
    print(f"Result: {result}")

if __name__ == "__main__":
    asyncio.run(main())
```

### Run It

```bash
python my_first_skill.py
```

**Expected Output**:
```
Hello, Alice!
Result: {'status': 'success', 'message': 'Hello, Alice!'}
```

---

## Running a Subagent

### Spawn and Execute a Skill

Create `my_first_agent.py`:

```python
import asyncio
from skills_framework import SkillRegistry, SubagentManager
from my_first_skill import greet_skill  # Import to register

async def main():
    # Create skill registry and agent manager
    registry = SkillRegistry()
    manager = SubagentManager(max_agents=10)

    # Spawn a subagent
    agent_id = await manager.spawn_subagent(registry)
    print(f"✅ Spawned agent: {agent_id}")

    # Execute a skill
    result = await manager.agents[agent_id].execute_skill(
        "greet",
        {"name": "Bob"}
    )
    print(f"✅ Result: {result}")

    # Check agent status
    status = manager.get_agent_status(agent_id)
    print(f"✅ Agent status: {status['state']}")

if __name__ == "__main__":
    asyncio.run(main())
```

### Run It

```bash
python my_first_agent.py
```

**Expected Output**:
```
✅ Spawned agent: a7f3c2e1-8d4b-4a9c-9e2f-1b3c4d5e6f7a
Hello, Bob!
✅ Result: {'status': 'success', 'message': 'Hello, Bob!'}
✅ Agent status: completed
```

---

## Skill Composition

### Define Dependent Skills

Create `navigation_skills.py`:

```python
import asyncio
from skills_framework import SkillRegistry, SkillMetadata

@SkillRegistry.register(SkillMetadata(
    name="move-forward",
    version="1.0.0",
    timeout=30,
    parameters_schema={
        "type": "object",
        "properties": {
            "distance": {"type": "number"}
        },
        "required": ["distance"]
    }
))
async def move_forward_skill(distance: float) -> dict:
    """Move robot forward by specified distance."""
    print(f"🤖 Moving forward {distance} meters...")
    await asyncio.sleep(2)  # Simulate movement
    return {"status": "success", "distance_traveled": distance}

@SkillRegistry.register(SkillMetadata(
    name="turn",
    version="1.0.0",
    timeout=30,
    parameters_schema={
        "type": "object",
        "properties": {
            "angle": {"type": "number"}
        },
        "required": ["angle"]
    }
))
async def turn_skill(angle: float) -> dict:
    """Turn robot by specified angle in degrees."""
    print(f"🤖 Turning {angle} degrees...")
    await asyncio.sleep(1)
    return {"status": "success", "angle_turned": angle}

@SkillRegistry.register(SkillMetadata(
    name="navigate-square",
    version="1.0.0",
    timeout=120,
    dependencies=["move-forward", "turn"],  # Requires other skills
    parameters_schema={
        "type": "object",
        "properties": {
            "side_length": {"type": "number"}
        },
        "required": ["side_length"]
    }
))
async def navigate_square_skill(side_length: float) -> dict:
    """Navigate in a square pattern."""
    print(f"🤖 Navigating square with side {side_length}m...")

    # Get dependency skills from registry
    move = SkillRegistry.get_skill("move-forward")
    turn = SkillRegistry.get_skill("turn")

    # Execute composite behavior
    for i in range(4):
        await move(distance=side_length)
        await turn(angle=90)

    return {"status": "success", "pattern": "square", "perimeter": side_length * 4}
```

### Execute Composite Skill

```python
import asyncio
from skills_framework import SkillRegistry, SubagentManager
from navigation_skills import *

async def main():
    registry = SkillRegistry()
    manager = SubagentManager()

    # Check dependencies
    from skills_framework import DependencyResolver
    resolver = DependencyResolver(registry)

    # Get execution order
    order = resolver.topological_sort(["navigate-square"])
    print(f"✅ Execution order: {order}")

    # Spawn agent and execute
    agent_id = await manager.spawn_subagent(registry)
    result = await manager.agents[agent_id].execute_skill(
        "navigate-square",
        {"side_length": 2.0}
    )
    print(f"✅ Result: {result}")

if __name__ == "__main__":
    asyncio.run(main())
```

**Expected Output**:
```
✅ Execution order: ['move-forward', 'turn', 'navigate-square']
🤖 Navigating square with side 2.0m...
🤖 Moving forward 2.0 meters...
🤖 Turning 90 degrees...
🤖 Moving forward 2.0 meters...
🤖 Turning 90 degrees...
🤖 Moving forward 2.0 meters...
🤖 Turning 90 degrees...
🤖 Moving forward 2.0 meters...
🤖 Turning 90 degrees...
✅ Result: {'status': 'success', 'pattern': 'square', 'perimeter': 8.0}
```

---

## Inter-Agent Communication

### Publish/Subscribe Events

Create `message_example.py`:

```python
import asyncio
from skills_framework import SkillRegistry, SubagentManager, EventBus, Message

async def main():
    # Create event bus
    bus = EventBus()

    # Define event handler
    async def on_navigation_complete(msg: Message):
        print(f"📨 Received event from {msg.sender_id}: {msg.data}")

    # Subscribe to topic
    await bus.subscribe("navigation.complete", on_navigation_complete)

    # Publish event
    await bus.publish(Message(
        topic="navigation.complete",
        sender_id="agent-1",
        data={"location": "kitchen", "status": "success"}
    ))

    # Allow async handlers to run
    await asyncio.sleep(0.1)

if __name__ == "__main__":
    asyncio.run(main())
```

**Expected Output**:
```
📨 Received event from agent-1: {'location': 'kitchen', 'status': 'success'}
```

### Direct Agent Messaging

```python
import asyncio
from skills_framework import MessageBroker, SkillRequest, SkillResponse
import uuid

async def main():
    broker = MessageBroker()

    # Register two agents
    await broker.register_agent("agent-A")
    await broker.register_agent("agent-B")

    # Agent A sends request to Agent B
    async def agent_a():
        request = SkillRequest(
            request_id=str(uuid.uuid4()),
            skill_name="detect",
            params={"area": "kitchen"},
            reply_to="agent-A"
        )
        await broker.send_message("agent-B", request)
        print("📤 Agent A sent request")

        # Wait for response
        response = await broker.receive_message("agent-A")
        print(f"📥 Agent A received: {response.result}")

    # Agent B handles request
    async def agent_b():
        request = await broker.receive_message("agent-B")
        print(f"📥 Agent B received request: {request.skill_name}")

        # Process and respond
        response = SkillResponse(
            request_id=request.request_id,
            status="success",
            result={"objects": ["cup", "plate"]}
        )
        await broker.send_message(request.reply_to, response)
        print("📤 Agent B sent response")

    # Run both concurrently
    await asyncio.gather(agent_a(), agent_b())

if __name__ == "__main__":
    asyncio.run(main())
```

**Expected Output**:
```
📤 Agent A sent request
📥 Agent B received request: detect
📤 Agent B sent response
📥 Agent A received: {'objects': ['cup', 'plate']}
```

---

## Next Steps

### 1. Define Real Robot Skills

Create skills that integrate with your robot's hardware:

```python
import rospy  # Example with ROS 2
from geometry_msgs.msg import Twist

@SkillRegistry.register(SkillMetadata(
    name="ros-navigate",
    version="1.0.0"
))
async def ros_navigate_skill(x: float, y: float) -> dict:
    """Navigate using ROS 2 action client."""
    # Your ROS 2 integration code here
    pass
```

### 2. Load Skills from YAML

Define skills declaratively:

```yaml
# skills/navigation.yaml
name: navigate
version: 1.0.0
description: Navigate to location
module: robot_skills.navigation
function: navigate_skill
parameters_schema:
  type: object
  properties:
    location:
      type: string
timeout: 60
dependencies: []
tags: [navigation, motion]
```

Load at runtime:

```python
from skills_framework import ConfigurableSkillRegistry

registry = ConfigurableSkillRegistry("skills/navigation.yaml")
```

### 3. Add Error Handling

Implement retry logic and fallbacks:

```python
@SkillRegistry.register(SkillMetadata(
    name="robust-navigate",
    version="1.0.0",
    max_retries=3
))
async def robust_navigate_skill(location: str) -> dict:
    try:
        # Attempt navigation
        result = await navigate(location)
        return result
    except NavigationError as e:
        # Fallback strategy
        return await navigate_alternative_path(location)
```

### 4. Monitor Execution

Track skill performance:

```python
# View execution history
executions = manager.list_executions(skill_name="navigate")
for exec in executions:
    print(f"{exec.started_at}: {exec.status} ({exec.duration_ms}ms)")
```

### 5. Build a Web Dashboard

Create a monitoring interface:

```python
from fastapi import FastAPI
from skills_framework import SkillRegistry, SubagentManager

app = FastAPI()

@app.get("/agents")
def list_agents():
    return manager.list_agents()

@app.get("/skills")
def list_skills():
    return SkillRegistry.list_skills()
```

---

## Common Patterns

### Pattern 1: Sequential Skill Chain

```python
async def execute_chain(agent_id, skill_chain):
    for skill_name in skill_chain:
        result = await manager.agents[agent_id].execute_skill(skill_name, {})
        print(f"✅ Completed {skill_name}: {result}")
```

### Pattern 2: Parallel Skill Execution

```python
async def execute_parallel(agent_ids, skill_name, params):
    tasks = [
        manager.agents[aid].execute_skill(skill_name, params)
        for aid in agent_ids
    ]
    results = await asyncio.gather(*tasks)
    return results
```

### Pattern 3: Skill with Timeout

```python
async def execute_with_timeout(agent_id, skill_name, params, timeout_sec):
    try:
        result = await asyncio.wait_for(
            manager.agents[agent_id].execute_skill(skill_name, params),
            timeout=timeout_sec
        )
        return result
    except asyncio.TimeoutError:
        print(f"❌ Skill {skill_name} timed out after {timeout_sec}s")
        return None
```

---

## Troubleshooting

### Skill Not Found

**Error**: `ValueError: Skill 'navigate' not found`

**Solution**: Ensure the skill module is imported before accessing the registry:
```python
from my_skills import navigate_skill  # This registers the skill
```

### Agent Already Running

**Error**: `RuntimeError: Agent already executing a skill`

**Solution**: Wait for current skill to complete or cancel it:
```python
await manager.agents[agent_id].cancel()
```

### Circular Dependency

**Error**: `ValueError: Circular dependency detected`

**Solution**: Check your skill dependencies with:
```python
resolver.detect_cycles("my-skill")  # Returns True if circular
```

---

## Resources

- **API Reference**: See `contracts/agent-api.yaml`
- **Data Model**: See `data-model.md`
- **Examples**: See `examples/` directory
- **GitHub Issues**: Report bugs at `https://github.com/your-org/skills-subagents/issues`

---

**Next**: Read the full [Implementation Plan](./plan.md) for architectural details.

**Happy Building! 🤖**
