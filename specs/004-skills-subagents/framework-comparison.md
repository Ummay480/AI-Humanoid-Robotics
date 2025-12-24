# Quick Reference: Agent Orchestration Frameworks for Skills/Subagents

**Purpose**: Fast lookup guide for framework selection and pattern application
**Updated**: 2025-12-19

---

## 1. Framework Quick Selector

### Choose Based on Your Priority:

**Performance & Scalability** → Ray
**Ease of Use** → CrewAI
**Skill Composition** → LangGraph
**Batch Workflows** → Prefect
**Multi-Agent Coordination** → AutoGen

---

## 2. Requirements vs. Frameworks Matrix

| Requirement | LangGraph | CrewAI | AutoGen | Prefect | Ray | Custom asyncio |
|-------------|-----------|--------|---------|---------|-----|-----------------|
| Skill definitions | ✅ nodes | ✅ tools | ✅ functions | ✅ tasks | ⚠️ actors | ✅ decorators |
| Registry system | ✅ graph | ✅ agent tools | ✅ manifest | ✅ registry | ⚠️ manual | ✅ class-based |
| Concurrent execution | ✅ async | ✅ teams | ✅ group chat | ✅ workers | ✅✅ native | ✅ asyncio |
| Message passing | ⚠️ custom | ⚠️ internal | ✅ conversation | ⚠️ task results | ✅ object refs | ✅✅ native |
| Timeout/cancellation | ✅ configurable | ⚠️ limited | ⚠️ limited | ✅ native | ✅ native | ✅ native |
| Skill composition | ✅✅ subgraphs | ✅ task deps | ✅ chaining | ✅ DAG | ✅ task graph | ✅ manual |
| Error handling | ✅ good | ✅ good | ✅ good | ✅✅ excellent | ✅ good | ✅ custom |
| External dependencies | ⚠️ medium | ⚠️⚠️ high (LLM) | ⚠️⚠️ high (LLM) | ⚠️ medium | ✅ low | ✅✅ none |
| Learning curve | ⚠️ medium | ✅ low | ⚠️ high | ⚠️ medium | ⚠️⚠️ high | ✅ low |
| Production readiness | ✅ good | ✅✅ excellent | ✅✅ excellent | ✅✅ excellent | ✅✅ excellent | ✅ good |

**Legend**: ✅ Fully supported | ✅✅ Excellent fit | ⚠️ Partial/limited | ✅ Native | ⚠️⚠️ Significant overhead

---

## 3. Implementation Complexity vs. Capability Trade-off

```
Simplicity ───────────────────────────► Power/Features

Custom asyncio ━━ LangGraph ━━ CrewAI ━━ AutoGen ━━ Prefect ━━ Ray
   (MVP)           (Good)      (AI-heavy)  (Complex)   (Enterprise)  (Distributed)

Recommended path for robotics:
Start at Custom asyncio → Add LangGraph later if needed
```

---

## 4. Message Passing Pattern Selection

### When to Use Each Pattern:

| Pattern | Use Case | Latency | Scalability | Coupling |
|---------|----------|---------|-------------|----------|
| **Event Bus (Pub/Sub)** | Broadcast notifications, state changes | Low-medium | High | Loose |
| **Direct Messaging (Queue)** | Skill requests, responses, data passing | Low | Medium | Medium |
| **Hybrid (Both)** | Complex coordination, mixed patterns | Low | High | Flexible |
| **RPC (gRPC/HTTP)** | Distributed systems, external agents | Medium-high | High | Loose |

### Implementation Examples:

**Event Bus**: Skill completion triggers detection startup
```python
await bus.publish(Message("skill.complete", "nav-agent", {"result": success}))
```

**Message Queue**: Direct skill request with reply
```python
await broker.send_message("detector-1", SkillRequest(...))
response = await broker.receive_message("nav-agent-1")
```

**Hybrid**: Combine both for coordination
```python
# Direct request for critical path
response = await skill_request("detector-1", "detect", {...})

# Broadcast result for other interested agents
await notify_event("detection.complete", response)
```

---

## 5. Skill Registry Implementation Strategies

### Strategy 1: Decorator-Based (Fast MVP)
```python
@SkillRegistry.register(SkillMetadata(name="navigate", timeout=30))
async def navigate_skill(location: str):
    ...
```
**When**: Quick prototyping, all skills in main codebase
**Cost**: Low | **Flexibility**: Medium

### Strategy 2: Configuration-Based (Production)
```yaml
skills:
  - id: navigate
    module: robot.skills.navigation
    timeout: 30
```
**When**: Need hot-reloading, external skill packages
**Cost**: Medium | **Flexibility**: High

### Strategy 3: Plugin-Based (Enterprise)
```python
class SkillPlugin:
    def get_skills(self) -> list[SkillMetadata]: ...
```
**When**: Third-party skill extensions, marketplace
**Cost**: High | **Flexibility**: Very High

### Recommended Approach:
**Phase 1**: Decorator-based (decorators.py in each skill module)
**Phase 2**: Add YAML loader wrapping decorators
**Phase 3**: Plugin system if needed

---

## 6. Concurrent Execution Patterns

### Pattern A: asyncio Native (Recommended for Robotics)
```python
results = await asyncio.gather(
    agent1.execute_skill(...),
    agent2.execute_skill(...),
    return_exceptions=True
)
```
**Pros**: No overhead, lightweight, built-in
**Cons**: GIL for CPU work, single-machine only

### Pattern B: Thread Pool (for Blocking I/O)
```python
result = await executor.run_in_executor(pool, blocking_func, args)
```
**Pros**: Unblocks asyncio event loop
**Cons**: Inter-thread communication overhead

### Pattern C: Ray Actors (for Distributed)
```python
navigator = NavigatorActor.remote()
result = ray.get(navigator.execute_skill.remote(...))
```
**Pros**: Distributed, excellent performance
**Cons**: Learning curve, overkill for single-machine

### Pattern D: multiprocessing (CPU-Bound Skills)
```python
with ProcessPoolExecutor(max_workers=4) as executor:
    result = await loop.run_in_executor(executor, compute_heavy_skill)
```
**Pros**: True parallelism, escapes GIL
**Cons**: Process creation overhead, IPC complexity

**Recommended**: Use asyncio (Pattern A) + ThreadPoolExecutor (Pattern B) for robotics

---

## 7. Dependency Resolution Quick Guide

### Check for Circular Dependencies:
```python
# This fails - circular dependency
skills_config:
  skill_a: depends_on: [skill_b]
  skill_b: depends_on: [skill_a]  # CIRCULAR!
```

### Topological Sort Order:
```
Define: skill_fetch depends on [navigate, detect, grasp]
        skill_detect depends on [navigate]
        skill_navigate depends on []

Sort result: [navigate, detect, grasp, fetch]
Execute in order above
```

### Handle Dependencies:
```python
resolved = resolver.topological_sort(["fetch", "navigate"])
# Result: ["navigate", "detect", "grasp", "fetch"]

# Now safe to execute in order
for skill in resolved:
    await execute_skill(skill)
```

---

## 8. Error Handling & Recovery Patterns

### Timeout Handling:
```python
try:
    result = await asyncio.wait_for(execute_skill(...), timeout=30)
except asyncio.TimeoutError:
    # Trigger fallback or retry
    await handle_timeout(agent_id, skill_name)
```

### Retry Logic:
```python
@retry(max_attempts=3, backoff=exponential)
async def execute_skill_with_retry(skill_name, params):
    return await execute_skill(skill_name, params)
```

### Graceful Degradation:
```python
# Try primary, fallback to secondary
try:
    result = await primary_skill(...)
except SkillFailedError:
    logger.warning("Primary failed, using fallback")
    result = await fallback_skill(...)
```

### Subagent Crash Isolation:
```python
# Wrap subagent execution with isolation
async def isolated_execution(agent):
    try:
        await agent.execute_task()
    except Exception as e:
        logger.error(f"Agent {agent.id} crashed", exc_info=e)
        agent.state = "failed"
        # Other agents continue unaffected
```

---

## 9. Monitoring & Observability Quick Setup

### Minimal Logging:
```python
import logging

logger = logging.getLogger(__name__)

async def execute_skill(skill_name, params):
    logger.info(f"Starting skill: {skill_name}")
    start = time.time()
    try:
        result = await skill_func(**params)
        duration = time.time() - start
        logger.info(f"Skill completed: {skill_name} ({duration:.2f}s)")
        return result
    except Exception as e:
        logger.error(f"Skill failed: {skill_name}", exc_info=e)
        raise
```

### Metrics Collection:
```python
from collections import defaultdict

class ExecutionMetrics:
    def __init__(self):
        self.execution_times = defaultdict(list)
        self.success_count = defaultdict(int)
        self.failure_count = defaultdict(int)

    def record_execution(self, skill_name, duration, success):
        self.execution_times[skill_name].append(duration)
        if success:
            self.success_count[skill_name] += 1
        else:
            self.failure_count[skill_name] += 1

    def get_stats(self, skill_name):
        times = self.execution_times[skill_name]
        return {
            "avg_duration": sum(times) / len(times) if times else 0,
            "success_rate": self.success_count[skill_name] / 
                          (self.success_count[skill_name] + self.failure_count[skill_name])
        }
```

---

## 10. Integration Roadmap

### Phase 1 (Week 1-2): MVP Foundation
- [x] Custom asyncio SubagentManager
- [x] Decorator-based SkillRegistry
- [x] Basic message queue (asyncio.Queue)
- [x] Simple error handling
- [x] Execution logging

### Phase 2 (Week 3-4): Production Features
- [ ] YAML skill configuration loader
- [ ] Dependency resolver with cycle detection
- [ ] Event bus (pub/sub)
- [ ] Hybrid messaging (queue + event bus)
- [ ] Metrics and monitoring
- [ ] Lifecycle management (pause, resume, cancel)

### Phase 3 (Week 5+): Advanced Features
- [ ] LangGraph integration for complex workflows
- [ ] External message broker support (RabbitMQ/NATS)
- [ ] Web dashboard for monitoring
- [ ] Plugin system for skill extensions
- [ ] ROS 2 integration (optional)

---

## 11. Decision Tree: Which Framework?

```
Start: Building robotics skills/subagents system

├─ Need distributed execution?
│  ├─ Yes → Consider Ray (but start with asyncio MVP first)
│  └─ No → Continue
│
├─ Need AI agents that reason/decide?
│  ├─ Yes → CrewAI or AutoGen (but check cost/latency)
│  └─ No → Continue
│
├─ Need batch workflow orchestration?
│  ├─ Yes → Prefect or Airflow
│  └─ No → Continue
│
├─ Need complex state machine skill composition?
│  ├─ Yes → LangGraph (Phase 2+)
│  └─ No → Continue
│
└─ RECOMMENDATION: Start with Custom asyncio
   ├─ Minimal dependencies
   ├─ Full control over robotics-specific patterns
   ├─ Easy to integrate frameworks later
   └─ Can migrate incrementally as needs grow
```

---

## 12. Code Templates

### Minimal Skill Definition (Copy-Paste Ready):
```python
from enum import Enum
from dataclasses import dataclass
from typing import Callable

class SkillState(Enum):
    IDLE = "idle"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"

@dataclass
class SkillMetadata:
    name: str
    timeout: int = 30

class SkillRegistry:
    _registry = {}
    
    @classmethod
    def register(cls, meta: SkillMetadata):
        def decorator(func):
            cls._registry[meta.name] = (func, meta)
            return func
        return decorator
    
    @classmethod
    def get(cls, name: str):
        if name not in cls._registry:
            raise ValueError(f"Skill '{name}' not found")
        return cls._registry[name][0]

# Register a skill
@SkillRegistry.register(SkillMetadata(name="navigate", timeout=60))
async def navigate_skill(location: str):
    """Navigate to location"""
    return {"status": "success", "location": location}

# Execute skill
result = await SkillRegistry.get("navigate")("kitchen")
```

### Minimal Subagent Manager (Copy-Paste Ready):
```python
import asyncio
from uuid import uuid4

class SubagentManager:
    def __init__(self):
        self.agents = {}
    
    async def spawn(self, registry):
        agent_id = str(uuid4())
        self.agents[agent_id] = {"state": "idle"}
        return agent_id
    
    async def execute(self, agent_id: str, skill_name: str, params: dict):
        agent = self.agents[agent_id]
        agent["state"] = "running"
        try:
            skill = SkillRegistry.get(skill_name)
            result = await asyncio.wait_for(skill(**params), timeout=30)
            agent["state"] = "completed"
            return result
        except Exception as e:
            agent["state"] = "failed"
            raise
    
    async def execute_parallel(self, tasks: list):
        """Execute multiple (agent_id, skill, params) tuples in parallel"""
        coros = [self.execute(*task) for task in tasks]
        return await asyncio.gather(*coros, return_exceptions=True)
```

---

## 13. Common Pitfalls & Solutions

| Pitfall | Problem | Solution |
|---------|---------|----------|
| Circular skill dependencies | Infinite loops, deadlock | Use topological sort resolver |
| Subagent crashes killing system | Cascading failures | Isolate each agent in try/except |
| Message queue unbounded growth | Memory leak | Add max queue size + overflow handling |
| No timeout on long-running skills | Hung agents | Always use asyncio.wait_for with timeout |
| Tight coupling between agents | Hard to extend | Use event bus for notifications |
| No observability | Can't debug issues | Add structured logging + metrics |
| LLM API costs spiraling | Unexpected bills | Use deterministic skills + CrewAI carefully |

---

## 14. Links & Resources

### Frameworks:
- LangGraph: https://github.com/langchain-ai/langgraph
- CrewAI: https://github.com/joaomdmoura/crewai
- AutoGen: https://github.com/microsoft/autogen
- Prefect: https://github.com/PrefectHQ/prefect
- Ray: https://github.com/ray-project/ray

### Python Async:
- asyncio docs: https://docs.python.org/3/library/asyncio.html
- Real Python asyncio: https://realpython.com/async-io-python/

### Message Brokers:
- RabbitMQ: https://www.rabbitmq.com/
- NATS: https://nats.io/
- Redis: https://redis.io/

---

**Last Updated**: 2025-12-19
**Status**: Ready for implementation planning

