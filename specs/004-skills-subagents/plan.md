# Implementation Plan: Skills and Subagents Framework

**Branch**: `004-skills-subagents` | **Date**: 2025-12-19 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-skills-subagents/spec.md`

## Summary

Build a modular framework enabling robotics developers to define reusable skills and spawn autonomous subagents for concurrent task execution. The system provides skill registry, lifecycle management, inter-agent communication, and skill composition capabilities. Primary use case: humanoid robotics behaviors composed from modular, testable skills.

**Technical Approach** (to be refined by Phase 0 research):
- Python 3.11+ with async/await for concurrent execution
- ROS 2 integration for robot control and sensing
- Action server pattern for skill execution
- Pub/sub messaging for inter-agent communication
- YAML-based skill definitions with validation

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: NEEDS CLARIFICATION (researching: agent frameworks, ROS 2 libraries, async libraries)
**Storage**: File-based (YAML skill definitions), optional in-memory registry cache
**Testing**: pytest with async support, ROS 2 testing tools
**Target Platform**: Linux (Ubuntu 22.04 LTS - ROS 2 Humble target)
**Project Type**: Single project (library/framework)
**Performance Goals**:
- Support 10+ concurrent subagents without degradation
- Message latency < 100ms between agents
- Skill execution startup < 500ms
**Constraints**:
- Memory per subagent < 50MB
- Total system overhead < 200MB for framework
- Graceful degradation on resource exhaustion
**Scale/Scope**:
- Support 50+ skill definitions in registry
- Handle 10+ concurrent subagents
- Framework codebase ~5,000 LOC

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Alignment with Core Principles

**I. Accuracy** ✅
- Will verify all ROS 2 integration patterns against official documentation
- Agent orchestration patterns sourced from established frameworks
- All code examples tested in target environment (ROS 2 Humble on Ubuntu 22.04)

**II. Clarity** ✅
- Framework API designed for developers with Python knowledge
- Clear documentation for skill definition and subagent management
- Examples provided for common patterns

**III. Reproducibility** ✅
- All skill examples executable as written
- Version constraints specified (Python 3.11+, ROS 2 Humble)
- Setup instructions include validation steps
- Sample skills provided for testing

**IV. Rigor** ✅
- Framework design based on peer-reviewed agent research
- ROS 2 patterns from official documentation
- Performance claims validated through testing

**V. Source Verification** ✅
- Code patterns cite source frameworks/papers
- ROS 2 integration references official tutorials
- Design decisions documented with rationale

**VI. Functional Accuracy** ✅
- All code samples compile and run as written
- Skill execution tested in simulation environment
- API calls use correct ROS 2 Humble versions
- Breaking changes documented with migration paths

### Feature Development Compliance

- ✅ Feature started with specification (completed in previous phase)
- ✅ Implementation proceeding with approved spec
- ✅ This plan traces to spec requirements (FR-001 through FR-015)
- ✅ Architecture decisions will be captured in ADRs if significant

### Testing & Validation Requirements

- ✅ Unit tests required for skill registry, lifecycle management, message passing
- ✅ Integration tests mandatory for ROS 2 interactions
- ✅ Code samples tested in ROS 2 Humble environment before merge
- ✅ Documentation validated against actual framework behavior

**GATE STATUS**: ✅ PASS - No constitution violations. Proceeding to Phase 0 research.

## Project Structure

### Documentation (this feature)

```text
specs/004-skills-subagents/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (pending)
├── data-model.md        # Phase 1 output (pending)
├── quickstart.md        # Phase 1 output (pending)
├── contracts/           # Phase 1 output (pending)
│   ├── skill-api.yaml   # Skill definition schema
│   └── agent-api.yaml   # Subagent management API
├── checklists/
│   └── requirements.md  # Quality validation (completed)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Single project structure (library/framework)
src/
├── models/
│   ├── skill.py          # Skill entity and validation
│   ├── subagent.py       # Subagent entity and state
│   └── message.py        # Inter-agent message types
├── services/
│   ├── skill_registry.py # Skill registration and lookup
│   ├── agent_manager.py  # Subagent lifecycle management
│   ├── executor.py       # Skill execution engine
│   └── messenger.py      # Inter-agent communication
├── integrations/
│   └── ros2/
│       ├── action_server.py  # ROS 2 action server wrapper
│       └── pub_sub.py        # ROS 2 pub/sub integration
├── cli/
│   └── skills_cli.py     # Command-line interface for testing
└── lib/
    ├── validators.py     # Skill definition validation
    └── utils.py          # Helper utilities

tests/
├── unit/
│   ├── test_skill_registry.py
│   ├── test_agent_manager.py
│   ├── test_executor.py
│   └── test_messenger.py
├── integration/
│   ├── test_ros2_integration.py
│   └── test_concurrent_agents.py
└── fixtures/
    ├── sample_skills/    # YAML skill definitions for testing
    └── mock_agents/      # Mock subagents for testing

examples/
├── basic_skill.yaml      # Simple skill definition example
├── composite_skill.yaml  # Skill composition example
└── demo_agents.py        # Demo showing multiple concurrent agents
```

**Structure Decision**: Single project library structure chosen because this is a reusable framework, not an application. All components (models, services, integrations) are part of one cohesive library that robotics developers will import into their projects.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations detected. Framework design aligns with constitution principles.

---

## Phase 0: Research & Technology Selection

**Status**: In Progress (research agents dispatched)

**Research Tasks**:
1. ✅ Agent orchestration frameworks evaluation (agent a9ea584)
2. ✅ ROS 2 integration patterns (agent aa81ff6)
3. Pending: Async execution libraries comparison
4. Pending: Message passing pattern selection
5. Pending: Skill composition architecture

**Deliverable**: `research.md` with technology selections and rationale

---

## Phase 1: Design & Contracts

**Status**: Pending (awaits Phase 0 completion)

**Tasks**:
1. Define data model for Skill, Subagent, Message entities
2. Create OpenAPI schema for skill registry API
3. Define YAML schema for skill definitions
4. Design agent lifecycle state machine
5. Create quickstart guide with examples

**Deliverables**:
- `data-model.md`
- `contracts/skill-api.yaml`
- `contracts/agent-api.yaml`
- `quickstart.md`

---

## Next Steps

1. ⏳ Await research agent findings
2. Generate `research.md` with technology decisions
3. Proceed to Phase 1 design
4. Update agent context
5. Re-validate constitution compliance
