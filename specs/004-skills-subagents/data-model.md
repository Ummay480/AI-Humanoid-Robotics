# Data Model: Skills and Subagents Framework

**Feature**: 004-skills-subagents
**Date**: 2025-12-19
**Status**: Draft

## Overview

This document defines the core entities and their relationships for the Skills and Subagents Framework. The data model supports skill definition, registration, subagent management, inter-agent messaging, and execution tracking.

---

## Core Entities

### 1. Skill

**Purpose**: Represents a reusable capability that can be executed by subagents.

**Attributes**:

| Attribute | Type | Required | Description | Validation |
|-----------|------|----------|-------------|------------|
| `name` | `str` | Yes | Unique skill identifier | Alphanumeric + hyphens, 3-50 chars |
| `version` | `str` | Yes | Semantic version (semver) | Format: `X.Y.Z` |
| `description` | `str` | No | Human-readable description | Max 500 chars |
| `parameters_schema` | `dict` | Yes | JSON Schema for input parameters | Valid JSON Schema |
| `returns_schema` | `dict` | No | JSON Schema for return value | Valid JSON Schema |
| `timeout` | `int` | No | Max execution time (seconds) | Default: 30, Range: 1-3600 |
| `max_retries` | `int` | No | Retry attempts on failure | Default: 0, Range: 0-5 |
| `dependencies` | `list[str]` | No | List of prerequisite skill names | Must reference existing skills |
| `tags` | `list[str]` | No | Categorization labels | Max 10 tags, each max 30 chars |
| `created_at` | `datetime` | Auto | Skill registration timestamp | ISO 8601 format |
| `updated_at` | `datetime` | Auto | Last modification timestamp | ISO 8601 format |

**Example**:
```python
{
    "name": "navigate",
    "version": "1.0.0",
    "description": "Navigate robot to specified location",
    "parameters_schema": {
        "type": "object",
        "properties": {
            "location": {"type": "string"},
            "speed": {"type": "number", "default": 1.0}
        },
        "required": ["location"]
    },
    "returns_schema": {
        "type": "object",
        "properties": {
            "status": {"type": "string"},
            "final_position": {"type": "object"}
        }
    },
    "timeout": 60,
    "max_retries": 2,
    "dependencies": [],
    "tags": ["navigation", "motion"],
    "created_at": "2025-12-19T10:00:00Z",
    "updated_at": "2025-12-19T10:00:00Z"
}
```

**State Transitions**: N/A (skills are stateless definitions)

**Relationships**:
- Has many: `SkillExecution` (execution history)
- Referenced by: `Subagent` (assigned skills)
- Referenced by: `Skill` (dependency relationships)

---

### 2. Subagent

**Purpose**: Autonomous execution context that runs specific skills.

**Attributes**:

| Attribute | Type | Required | Description | Validation |
|-----------|------|----------|-------------|------------|
| `agent_id` | `str` | Auto | Unique agent identifier (UUID) | UUID v4 format |
| `name` | `str` | No | Human-readable agent name | Max 100 chars |
| `state` | `AgentState` | Auto | Current lifecycle state | Enum: idle, running, paused, completed, failed |
| `assigned_skill` | `str` | No | Current skill being executed | Must reference existing skill |
| `skill_params` | `dict` | No | Parameters for current skill | Must match skill's parameters_schema |
| `results` | `dict` | Auto | Execution results by skill name | Key: skill name, Value: result dict |
| `error` | `str` | No | Last error message | Max 1000 chars |
| `created_at` | `datetime` | Auto | Agent spawn timestamp | ISO 8601 format |
| `started_at` | `datetime` | Auto | Execution start timestamp | ISO 8601 format |
| `completed_at` | `datetime` | Auto | Execution end timestamp | ISO 8601 format |
| `metadata` | `dict` | No | Custom key-value metadata | Max 20 keys |

**State Enum**:
```python
class AgentState(Enum):
    IDLE = "idle"           # Agent created but not executing
    RUNNING = "running"     # Currently executing skill
    PAUSED = "paused"      # Execution paused (can resume)
    COMPLETED = "completed" # Skill execution finished successfully
    FAILED = "failed"       # Skill execution failed
```

**Example**:
```python
{
    "agent_id": "a7f3c2e1-8d4b-4a9c-9e2f-1b3c4d5e6f7a",
    "name": "Navigator-1",
    "state": "running",
    "assigned_skill": "navigate",
    "skill_params": {"location": "kitchen", "speed": 1.5},
    "results": {
        "previous_skill": {"status": "success", "data": "..."}
    },
    "error": null,
    "created_at": "2025-12-19T10:05:00Z",
    "started_at": "2025-12-19T10:05:10Z",
    "completed_at": null,
    "metadata": {"priority": "high", "user_id": "user123"}
}
```

**State Transitions**:
```
idle → running      # Start skill execution
running → paused    # Pause execution
paused → running    # Resume execution
running → completed # Skill succeeds
running → failed    # Skill fails
idle → failed       # Invalid skill assignment
```

**Relationships**:
- Executes: `Skill` (one skill at a time)
- Creates: `SkillExecution` (execution history)
- Sends/Receives: `Message` (inter-agent communication)

---

### 3. Message

**Purpose**: Communication payload between subagents or system components.

**Attributes**:

| Attribute | Type | Required | Description | Validation |
|-----------|------|----------|-------------|------------|
| `message_id` | `str` | Auto | Unique message identifier (UUID) | UUID v4 format |
| `topic` | `str` | Conditional | Pub/sub topic name | Required for broadcast messages |
| `sender_id` | `str` | Yes | Agent ID or system component | Must be valid agent_id or "system" |
| `recipient_id` | `str` | Conditional | Target agent ID | Required for direct messages |
| `message_type` | `MessageType` | Yes | Message classification | Enum: event, request, response, error |
| `payload` | `dict` | Yes | Message data | Valid JSON object |
| `timestamp` | `datetime` | Auto | Message creation time | ISO 8601 format |
| `ttl` | `int` | No | Time-to-live (seconds) | Default: 60, Range: 1-3600 |
| `priority` | `int` | No | Message priority | Default: 5, Range: 1-10 (1=highest) |
| `correlation_id` | `str` | No | Links request/response pairs | UUID v4 format |

**Message Type Enum**:
```python
class MessageType(Enum):
    EVENT = "event"         # State change notification (pub/sub)
    REQUEST = "request"     # Skill request or data query
    RESPONSE = "response"   # Reply to request
    ERROR = "error"         # Error notification
```

**Example - Event Message**:
```python
{
    "message_id": "m1a2b3c4-d5e6-f7a8-b9c0-d1e2f3a4b5c6",
    "topic": "navigation.complete",
    "sender_id": "a7f3c2e1-8d4b-4a9c-9e2f-1b3c4d5e6f7a",
    "recipient_id": null,  # Broadcast
    "message_type": "event",
    "payload": {
        "location": "kitchen",
        "final_position": {"x": 10.5, "y": 3.2}
    },
    "timestamp": "2025-12-19T10:10:00Z",
    "ttl": 60,
    "priority": 5,
    "correlation_id": null
}
```

**Example - Request/Response Pair**:
```python
# Request
{
    "message_id": "req-123",
    "topic": null,
    "sender_id": "agent-A",
    "recipient_id": "agent-B",
    "message_type": "request",
    "payload": {
        "skill": "detect",
        "params": {"area": "kitchen"}
    },
    "timestamp": "2025-12-19T10:11:00Z",
    "correlation_id": "corr-456"
}

# Response
{
    "message_id": "res-789",
    "topic": null,
    "sender_id": "agent-B",
    "recipient_id": "agent-A",
    "message_type": "response",
    "payload": {
        "status": "success",
        "objects": ["cup", "plate"]
    },
    "timestamp": "2025-12-19T10:11:02Z",
    "correlation_id": "corr-456"  # Same as request
}
```

**Relationships**:
- Sent by: `Subagent` or system
- Received by: `Subagent` (or all subscribers for events)

---

### 4. SkillExecution

**Purpose**: Records execution history for auditing and debugging.

**Attributes**:

| Attribute | Type | Required | Description | Validation |
|-----------|------|----------|-------------|------------|
| `execution_id` | `str` | Auto | Unique execution identifier (UUID) | UUID v4 format |
| `agent_id` | `str` | Yes | Executing agent ID | Must reference existing agent |
| `skill_name` | `str` | Yes | Executed skill name | Must reference existing skill |
| `skill_version` | `str` | Yes | Skill version at execution time | Semver format |
| `parameters` | `dict` | Yes | Actual parameters used | Must match skill schema |
| `result` | `dict` | No | Execution result | Populated on success |
| `error` | `str` | No | Error message | Populated on failure |
| `status` | `ExecutionStatus` | Auto | Execution outcome | Enum: pending, success, failed, timeout |
| `started_at` | `datetime` | Auto | Execution start time | ISO 8601 format |
| `completed_at` | `datetime` | Auto | Execution end time | ISO 8601 format |
| `duration_ms` | `int` | Auto | Execution time (milliseconds) | Calculated: completed_at - started_at |
| `retry_count` | `int` | Auto | Number of retry attempts | Incremented on each retry |

**Execution Status Enum**:
```python
class ExecutionStatus(Enum):
    PENDING = "pending"     # Execution queued
    SUCCESS = "success"     # Completed successfully
    FAILED = "failed"       # Failed with error
    TIMEOUT = "timeout"     # Exceeded timeout limit
```

**Example**:
```python
{
    "execution_id": "exec-a1b2c3d4",
    "agent_id": "a7f3c2e1-8d4b-4a9c-9e2f-1b3c4d5e6f7a",
    "skill_name": "navigate",
    "skill_version": "1.0.0",
    "parameters": {"location": "kitchen", "speed": 1.5},
    "result": {
        "status": "success",
        "final_position": {"x": 10.5, "y": 3.2}
    },
    "error": null,
    "status": "success",
    "started_at": "2025-12-19T10:05:10Z",
    "completed_at": "2025-12-19T10:06:20Z",
    "duration_ms": 70000,
    "retry_count": 0
}
```

**Relationships**:
- Belongs to: `Subagent` (one agent)
- References: `Skill` (one skill)

---

### 5. SkillRegistry

**Purpose**: Central repository tracking all registered skills.

**Attributes**:

| Attribute | Type | Required | Description | Validation |
|-----------|------|----------|-------------|------------|
| `registry_id` | `str` | Auto | Registry identifier | Singleton: "default" |
| `skills` | `dict[str, Skill]` | Auto | Map of skill name → Skill object | Key: skill name |
| `dependency_graph` | `dict[str, list[str]]` | Auto | Skill dependencies | Key: skill, Value: list of deps |
| `last_updated` | `datetime` | Auto | Last registry modification | ISO 8601 format |

**Methods**:
- `register_skill(skill: Skill) -> None`: Add skill to registry
- `get_skill(name: str, version: str = None) -> Skill`: Retrieve skill by name/version
- `list_skills(tags: list[str] = None) -> list[Skill]`: Query skills by tags
- `resolve_dependencies(skill_name: str) -> list[str]`: Get execution order
- `detect_cycles(skill_name: str) -> bool`: Check for circular dependencies

---

## Entity Relationships Diagram

```
┌─────────────────┐
│  SkillRegistry  │
│  (Singleton)    │
└────────┬────────┘
         │ contains
         ▼
    ┌─────────┐
    │  Skill  │◀──────────┐
    └────┬────┘           │
         │                │ depends on
         │ executed by    │
         ▼                │
  ┌──────────────┐        │
  │  Subagent    │        │
  └──┬───────┬───┘        │
     │       │            │
     │       └────────────┘
     │ creates
     ▼
┌──────────────────┐
│ SkillExecution   │
└──────────────────┘

      Subagent
         ├─── sends ───┐
         │             ▼
         │        ┌─────────┐
         │        │ Message │
         │        └─────────┘
         │             │
         └──── receives ──┘
```

---

## Data Validation Rules

### Skill Validation
1. **Unique Name**: No two skills can have the same name (version distinguishes variants)
2. **Dependency Existence**: All dependencies must reference existing skills
3. **No Circular Dependencies**: Dependency graph must be acyclic
4. **Schema Validity**: `parameters_schema` and `returns_schema` must be valid JSON Schema
5. **Timeout Range**: Must be between 1-3600 seconds

### Subagent Validation
1. **Skill Assignment**: `assigned_skill` must reference an existing skill in registry
2. **Parameter Matching**: `skill_params` must conform to skill's `parameters_schema`
3. **State Transitions**: Only valid state transitions allowed (see state machine above)
4. **Unique ID**: `agent_id` must be globally unique

### Message Validation
1. **Topic or Recipient**: Must specify either `topic` (broadcast) OR `recipient_id` (direct)
2. **Valid Sender**: `sender_id` must reference an existing agent or "system"
3. **Correlation Matching**: Response messages must have `correlation_id` matching request
4. **TTL**: Must be positive and <= 3600 seconds

### SkillExecution Validation
1. **Agent Existence**: `agent_id` must reference an existing subagent
2. **Skill Existence**: `skill_name` must reference an existing skill
3. **Duration Calculation**: `duration_ms` = (`completed_at` - `started_at`) in milliseconds
4. **Retry Limit**: `retry_count` must not exceed skill's `max_retries`

---

## Storage Strategy

### In-Memory (Development/Testing)
- **SkillRegistry**: Python dict in-memory singleton
- **Subagent**: Stored in `SubagentManager.agents` dict
- **Message**: Stored in asyncio queues or event bus subscribers
- **SkillExecution**: Stored in agent's `results` dict or separate execution log

### Persistent (Production)
- **SkillRegistry**: YAML/JSON files loaded at startup
- **Subagent**: Optional SQLite or PostgreSQL for persistence across restarts
- **Message**: In-memory queues with optional Redis for distributed systems
- **SkillExecution**: SQLite or PostgreSQL for execution history and analytics

---

## Indexing Strategy (for Production DB)

**Subagent Table**:
- Primary Key: `agent_id`
- Index: `state` (for querying active agents)
- Index: `created_at` (for time-based queries)

**SkillExecution Table**:
- Primary Key: `execution_id`
- Index: `agent_id` (for agent execution history)
- Index: `skill_name` (for skill performance analytics)
- Index: `status` (for filtering failures/successes)
- Index: `started_at` (for time-series analysis)

**Message Table** (if persisted):
- Primary Key: `message_id`
- Index: `recipient_id` (for mailbox queries)
- Index: `topic` (for pub/sub lookups)
- Index: `correlation_id` (for request/response matching)
- Index: `timestamp` (for TTL cleanup)

---

## Example Workflow Data Flow

### Scenario: Execute Composite Skill "Fetch Object"

**1. Skill Definitions**:
```yaml
# navigate skill
name: navigate
version: 1.0.0
dependencies: []

# detect skill
name: detect
version: 1.0.0
dependencies: [navigate]

# grasp skill
name: grasp
version: 1.0.0
dependencies: [detect]
```

**2. Spawn Subagent**:
```python
{
    "agent_id": "agent-123",
    "state": "idle",
    "assigned_skill": null
}
```

**3. Execute Skill Chain** (navigate → detect → grasp):

**Step 1 - Navigate**:
```python
# Update subagent
{
    "agent_id": "agent-123",
    "state": "running",
    "assigned_skill": "navigate",
    "skill_params": {"location": "kitchen"}
}

# Create execution record
{
    "execution_id": "exec-001",
    "agent_id": "agent-123",
    "skill_name": "navigate",
    "status": "pending"
}

# On completion
{
    "execution_id": "exec-001",
    "status": "success",
    "result": {"final_position": {"x": 10, "y": 5}},
    "duration_ms": 5000
}

# Publish event
{
    "topic": "navigation.complete",
    "sender_id": "agent-123",
    "payload": {"final_position": {"x": 10, "y": 5}}
}
```

**Step 2 - Detect** (triggered by event or sequentially):
```python
# Update subagent
{
    "agent_id": "agent-123",
    "assigned_skill": "detect",
    "skill_params": {"area": "kitchen"}
}

# Create execution record
{
    "execution_id": "exec-002",
    "skill_name": "detect",
    "parameters": {"area": "kitchen"}
}

# On completion
{
    "execution_id": "exec-002",
    "status": "success",
    "result": {"objects": ["cup"]},
    "duration_ms": 2000
}
```

**Step 3 - Grasp**:
```python
# Similar pattern...
{
    "execution_id": "exec-003",
    "skill_name": "grasp",
    "parameters": {"object_id": "cup"},
    "status": "success",
    "result": {"grasped": true}
}

# Final subagent state
{
    "agent_id": "agent-123",
    "state": "completed",
    "results": {
        "navigate": {"final_position": {...}},
        "detect": {"objects": [...]},
        "grasp": {"grasped": true}
    }
}
```

---

## Summary

This data model provides:
- **Clear entity boundaries** with well-defined attributes and validation rules
- **Flexible skill composition** through dependency graphs
- **Robust state management** for subagent lifecycles
- **Comprehensive messaging** supporting both pub/sub and direct communication
- **Execution tracking** for debugging and analytics
- **Scalable storage** with in-memory and persistent options

The model aligns with all functional requirements (FR-001 through FR-015) from the specification.
