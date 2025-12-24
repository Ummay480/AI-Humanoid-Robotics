# Skills and Subagents Framework - Examples

This directory contains example implementations demonstrating the framework's capabilities.

## Directory Structure

```
examples/
├── implementations/         # Skill implementation functions
│   ├── echo_message.py     # Echo skill implementation
│   ├── greet_user.py       # Greeting skill implementation
│   └── calculate_sum.py    # Calculator skill implementation
├── basic_skill.py          # Phase 3 demo: Load, register, execute skills
├── basic_skill.yaml        # Simple skill definition (echo)
├── composite_skill.yaml    # Skill composition example (Phase 6)
├── navigation_skill.yaml   # Navigation skill example
└── README.md              # This file
```

## Quick Start

### 1. Install Dependencies

```bash
# From project root
pip install -r requirements.txt
```

### 2. Run Phase 3 Example (Define and Execute Skills)

```bash
# From project root
python examples/basic_skill.py
```

**Expected Output:**
```
✓ Loaded skill: echo-message v1.0.0
✓ Registered: echo-message v1.0.0
✓ Executed successfully with results
✓ Error handling demonstrated
Phase 3 (User Story 1) Complete!
```

## Example Files

### Phase 3: Define and Execute Skills ✅

**File**: `basic_skill.py`

Demonstrates User Story 1 (P1) - Core skill execution:
- ✅ Load skill definition from YAML file
- ✅ Register skill in the registry
- ✅ Register implementation function
- ✅ Execute skill with parameter validation
- ✅ Handle errors gracefully
- ✅ Batch execution (concurrent skills)

**Skill Definitions**:
- `basic_skill.yaml` - Echo message with prefix
- `navigation_skill.yaml` - ROS 2 navigation skill example

**Implementations**:
- `implementations/echo_message.py` - Simple string processing
- `implementations/greet_user.py` - Greeting with async delay
- `implementations/calculate_sum.py` - Math operations

### Phase 4: Concurrent Subagents (Coming in Phase 4)

**Status**: Not yet implemented (requires Phase 4)

Will demonstrate:
- Spawning multiple subagents
- Concurrent skill execution
- Agent state management
- Result aggregation

### Phase 5: Inter-Agent Communication (Coming in Phase 5)

**Status**: Not yet implemented (requires Phase 5)

Will demonstrate:
- Pub/Sub messaging between agents
- Event-driven coordination
- Message routing

### Phase 6: Skill Composition (Coming in Phase 6)

**File**: `composite_skill.yaml`

Will demonstrate:
- Hierarchical skill composition
- Data flow between skills
- Dependency management

## How to Create Your Own Skill

### 1. Define Skill in YAML

Create `my_skill.yaml`:

```yaml
metadata:
  name: "my-skill"
  version: "1.0.0"
  description: "My custom skill"
  tags: ["custom"]

interface:
  parameters:
    - name: "input"
      type: "string"
      required: true
      description: "Input parameter"

  outputs:
    result:
      type: string
      description: "Output result"

  dependencies: []

config:
  timeout_seconds: 10.0
  retry_policy:
    max_attempts: 3
    backoff_multiplier: 2.0
    initial_delay_seconds: 1.0

implementation: "examples.implementations.my_skill"
```

### 2. Implement Skill Logic

Create `implementations/my_skill.py`:

```python
from typing import Dict, Any

async def execute(params: Dict[str, Any]) -> Dict[str, Any]:
    """
    Your skill implementation.

    Args:
        params: Input parameters (validated against schema)

    Returns:
        Results dictionary (must match outputs schema)
    """
    input_value = params["input"]

    # Your logic here
    result = f"Processed: {input_value}"

    return {"result": result}
```

### 3. Load and Execute

```python
from src.services import SkillLoader, SkillRegistry, SkillExecutor

# Load skill
skill = SkillLoader.load_from_file("my_skill.yaml")

# Register skill
registry = SkillRegistry()
registry.register(skill)

# Execute skill
executor = SkillExecutor(registry)
executor.register_implementation(skill.name, my_skill_impl.execute)

execution = await executor.execute(
    skill_name="my-skill",
    parameters={"input": "test"}
)

print(execution.results)  # {"result": "Processed: test"}
```

## Learn More

- **Specification**: `specs/004-skills-subagents/spec.md` - Feature requirements
- **Implementation Plan**: `specs/004-skills-subagents/plan.md` - Architecture
- **Task Breakdown**: `specs/004-skills-subagents/tasks.md` - Development roadmap
- **Data Models**: `specs/004-skills-subagents/data-model.md` - Entity definitions
- **ROS 2 Integration**: `ROS2_INTEGRATION_SUMMARY.md` - Production deployment patterns

## Prerequisites

- Python 3.11+
- pydantic >= 2.5.0
- pyyaml >= 6.0

### Optional (for ROS 2 integration)

- ROS 2 Humble
- rclpy >= 3.3.0

## Testing

Run unit tests:

```bash
# From project root
pytest tests/
```

Run specific test:

```bash
pytest tests/test_executor.py -v
```

## Support

For questions or issues, please create an issue in the project repository.
