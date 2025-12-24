# Implementation Tasks: Skills and Subagents Framework

**Feature**: 004-skills-subagents
**Branch**: `004-skills-subagents`
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)
**Generated**: 2025-12-19

## Overview

This task breakdown implements the Skills and Subagents Framework following the prioritized user stories from the specification. Each phase represents an independently testable increment that delivers complete functionality for one user story.

**Implementation Strategy**: MVP-first approach with incremental delivery. Start with P1 (core skill execution), then add P2 (concurrent subagents), P3 (communication), and P4 (composition).

---

## Task Summary

| Phase | User Story | Task Count | Can Run Independently? |
|-------|------------|------------|------------------------|
| Phase 1 | Setup | 8 | N/A (prerequisite) |
| Phase 2 | Foundational | 6 | N/A (prerequisite) |
| Phase 3 | US1 - Define and Execute Skills (P1) | 12 | ✅ Yes |
| Phase 4 | US2 - Spawn and Coordinate Subagents (P2) | 10 | ✅ Yes (requires US1) |
| Phase 5 | US3 - Inter-Agent Communication (P3) | 9 | ✅ Yes (requires US1, US2) |
| Phase 6 | US4 - Skill Composition (P4) | 8 | ✅ Yes (requires US1) |
| Phase 7 | Polish & Cross-Cutting | 6 | N/A |
| **Total** | **4 user stories** | **59 tasks** | **4 independent stories** |

---

## Dependencies Graph

```
Setup (Phase 1) → Foundational (Phase 2)
                         ↓
              ┌──────────┼──────────┬──────────┐
              ↓          ↓          ↓          ↓
            US1        US2        US3        US4
           (P1)       (P2)       (P3)       (P4)
                        ↓          ↓
                     US2+US3 → [US3 requires US1+US2]
                     US1+US4 → [US4 requires US1]
```

**Completion Order**:
1. **MVP**: Phase 1 → Phase 2 → Phase 3 (US1) ✅ Delivers core value
2. **Extended**: + Phase 4 (US2) ✅ Adds concurrency
3. **Advanced**: + Phase 5 (US3) ✅ Adds communication
4. **Complete**: + Phase 6 (US4) + Phase 7 ✅ Full feature set

---

## Phase 1: Setup & Project Initialization

**Goal**: Initialize project structure, dependencies, and development environment.

**Duration Estimate**: 1-2 hours

### Tasks

- [ ] T001 Create project directory structure per plan.md (src/, tests/, examples/, specs/)
- [ ] T002 [P] Initialize Python package with pyproject.toml and setup.py
- [ ] T003 [P] Create requirements.txt with Python 3.11+, pydantic, pytest, asyncio
- [ ] T004 [P] Set up .gitignore for Python project (__pycache__, .pytest_cache, *.pyc)
- [ ] T005 [P] Create src/__init__.py and mark as package
- [ ] T006 [P] Create tests/__init__.py and configure pytest.ini
- [ ] T007 [P] Add examples/__init__.py and README.md with usage instructions
- [ ] T008 Verify project structure matches plan.md and all imports work

**Acceptance**: Project structure created, Python environment configured, pytest runs successfully (even with 0 tests).

---

## Phase 2: Foundational Infrastructure

**Goal**: Build core models and validation infrastructure required by all user stories.

**Duration Estimate**: 2-3 hours

**Blocking**: This phase must complete before any user story can begin.

### Tasks

- [ ] T009 Create src/models/__init__.py package
- [ ] T010 [P] Implement AgentState enum in src/models/subagent.py (idle, running, paused, completed, failed)
- [ ] T011 [P] Implement ExecutionStatus enum in src/models/skill.py (pending, success, failed, timeout)
- [ ] T012 [P] Implement MessageType enum in src/models/message.py (event, request, response, error)
- [ ] T013 [P] Create src/lib/validators.py with JSON Schema validation utility functions
- [ ] T014 [P] Create src/lib/utils.py with helper functions (UUID generation, timestamp formatting)

**Acceptance**: All enums and utilities importable, basic validation functions work with sample schemas.

---

## Phase 3: User Story 1 - Define and Execute Skills (P1)

**Goal**: Enable developers to define reusable skills and execute them on demand.

**Priority**: P1 (MVP - Core Foundation)

**Independent Test**: Define a simple "greet" skill, register it, and execute it. Verify it returns success status and handles errors gracefully.

### Tasks

#### Models & Core Logic

- [ ] T015 [P] [US1] Implement SkillMetadata dataclass in src/models/skill.py with all attributes from data-model.md
- [ ] T016 [P] [US1] Implement Skill class in src/models/skill.py with validation methods
- [ ] T017 [P] [US1] Add parameter schema validation to Skill using JSON Schema (src/lib/validators.py)
- [ ] T018 [P] [US1] Implement SkillExecution dataclass in src/models/skill.py for tracking execution history

#### Skill Registry

- [ ] T019 [US1] Implement SkillRegistry class in src/services/skill_registry.py with decorator-based registration
- [ ] T020 [US1] Add SkillRegistry.register() decorator method accepting SkillMetadata
- [ ] T021 [US1] Add SkillRegistry.get_skill() method for retrieving skills by name/version
- [ ] T022 [US1] Add SkillRegistry.list_skills() method with optional tag filtering

#### Skill Execution

- [ ] T023 [US1] Implement basic skill executor in src/services/executor.py with async execute_skill()
- [ ] T024 [US1] Add timeout handling to executor using asyncio.wait_for()
- [ ] T025 [US1] Add error handling and cleanup in executor (try/except with resource cleanup)
- [ ] T026 [US1] Create example skill in examples/basic_skill.py demonstrating registration and execution

**Acceptance Criteria** (US1):
- ✅ Can define a skill using @SkillRegistry.register decorator
- ✅ Skill loads into registry and is retrievable by name
- ✅ Skill executes with valid parameters and returns result dict
- ✅ Skill fails gracefully on error with descriptive message
- ✅ Timeout mechanism prevents hung executions

**Test Command** (US1):
```bash
python examples/basic_skill.py  # Should print "Hello, World!" and success status
```

---

## Phase 4: User Story 2 - Spawn and Coordinate Subagents (P2)

**Goal**: Enable spawning specialized subagents for concurrent task execution.

**Priority**: P2

**Dependencies**: Requires US1 (skills must exist to assign to agents)

**Independent Test**: Spawn 2 subagents, assign different skills to each, execute concurrently. Verify both complete and execution time < sequential time.

### Tasks

#### Subagent Model & Lifecycle

- [ ] T027 [P] [US2] Implement Subagent class in src/models/subagent.py with all attributes from data-model.md
- [ ] T028 [P] [US2] Add state transition validation to Subagent (idle→running→completed/failed)
- [ ] T029 [P] [US2] Implement Subagent.execute_skill() async method calling executor

#### Agent Manager

- [ ] T030 [US2] Implement SubagentManager class in src/services/agent_manager.py
- [ ] T031 [US2] Add SubagentManager.spawn_subagent() method creating new agent with UUID
- [ ] T032 [US2] Add SubagentManager.get_agent_status() method returning agent state/results
- [ ] T033 [US2] Add SubagentManager.list_agents() method with optional state filtering
- [ ] T034 [US2] Add SubagentManager.execute_parallel() method running multiple agents concurrently with asyncio.gather()

#### Resource Management

- [ ] T035 [US2] Add max_agents limit to SubagentManager (default 10)
- [ ] T036 [US2] Create example in examples/concurrent_agents.py showing 2+ agents executing in parallel

**Acceptance Criteria** (US2):
- ✅ Can spawn multiple subagents (up to max limit)
- ✅ Each agent has unique ID and tracks its state
- ✅ Agents execute skills concurrently (verified by timing)
- ✅ Agent status queryable at any time
- ✅ Completed agents report results correctly

**Test Command** (US2):
```bash
python examples/concurrent_agents.py  # Should show 2 agents completing faster than sequential
```

---

## Phase 5: User Story 3 - Inter-Agent Communication (P3)

**Goal**: Enable subagents to communicate and coordinate via messaging.

**Priority**: P3

**Dependencies**: Requires US1 (skills) + US2 (subagents)

**Independent Test**: Create 2 agents where Agent A publishes event "task.complete" and Agent B subscribes to it. Verify B receives message and proceeds.

### Tasks

#### Message Models

- [ ] T037 [P] [US3] Implement Message dataclass in src/models/message.py with all attributes from data-model.md
- [ ] T038 [P] [US3] Add message validation (topic XOR recipient_id, TTL, priority ranges)

#### Event Bus (Pub/Sub)

- [ ] T039 [US3] Implement EventBus class in src/services/messenger.py with asyncio-based pub/sub
- [ ] T040 [US3] Add EventBus.subscribe(topic, handler) method storing callbacks in dict
- [ ] T041 [US3] Add EventBus.publish(message) method calling all topic subscribers with asyncio.gather()

#### Direct Messaging (Request/Response)

- [ ] T042 [US3] Implement MessageBroker class in src/services/messenger.py with agent mailboxes
- [ ] T043 [US3] Add MessageBroker.register_agent(agent_id) creating asyncio.Queue for mailbox
- [ ] T044 [US3] Add MessageBroker.send_message(to_agent_id, message) putting message in recipient queue
- [ ] T045 [US3] Add MessageBroker.receive_message(agent_id) getting message from queue with timeout

**Acceptance Criteria** (US3):
- ✅ Agents can publish events to topics
- ✅ Multiple agents can subscribe to same topic
- ✅ Event delivery is async and non-blocking
- ✅ Agents can send direct messages to specific agent IDs
- ✅ Request/response pairs work with correlation IDs

**Test Command** (US3):
```bash
python examples/message_example.py  # Should show pub/sub and direct messaging working
```

---

## Phase 6: User Story 4 - Skill Composition and Chaining (P4)

**Goal**: Enable composing complex skills from simpler ones with dependency resolution.

**Priority**: P4

**Dependencies**: Requires US1 (skills must be registered)

**Independent Test**: Define composite skill "navigate-square" = 4x(move-forward + turn). Execute and verify all sub-skills run in order.

### Tasks

#### Dependency Resolution

- [ ] T046 [P] [US4] Implement DependencyResolver class in src/services/skill_registry.py
- [ ] T047 [US4] Add DependencyResolver.build_graph() building adjacency list from skill dependencies
- [ ] T048 [US4] Add DependencyResolver.detect_cycles() using DFS to find circular dependencies
- [ ] T049 [US4] Add DependencyResolver.topological_sort() returning execution order for skill chain

#### Composite Skill Execution

- [ ] T050 [US4] Add Subagent.execute_composite_skill(skill_chain, params) method in src/models/subagent.py
- [ ] T051 [US4] Implement sequential execution in composite: await each sub-skill, pass results forward
- [ ] T052 [US4] Add error handling for composite: if sub-skill fails, handle per policy (abort/retry/fallback)
- [ ] T053 [US4] Create example in examples/composite_skill.py showing "navigate-square" skill chaining

**Acceptance Criteria** (US4):
- ✅ Can define skills with dependencies array
- ✅ System detects circular dependencies and raises error
- ✅ Topological sort returns correct execution order
- ✅ Composite skills execute sub-skills sequentially
- ✅ Sub-skill failures handled gracefully

**Test Command** (US4):
```bash
python examples/composite_skill.py  # Should execute navigate→detect→grasp in order
```

---

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Add logging, error handling, documentation, and production readiness.

**Duration Estimate**: 3-4 hours

### Tasks

#### Logging & Monitoring

- [ ] T054 [P] Add Python logging to all services (skill_registry, agent_manager, executor, messenger)
- [ ] T055 [P] Create execution logger in src/lib/utils.py tracking all skill executions with timestamps

#### Error Handling & Resilience

- [ ] T056 [P] Add retry logic to executor based on skill.max_retries
- [ ] T057 [P] Add graceful degradation when max_agents limit reached (queue or reject with error)

#### Documentation & Examples

- [ ] T058 [P] Create comprehensive README.md with quickstart guide from quickstart.md
- [ ] T059 [P] Add docstrings to all public methods following Google style

**Acceptance**: All modules have logging, errors handled gracefully, README guides user through all 4 user stories.

---

## Parallel Execution Opportunities

### Setup & Foundational (Can Run in Parallel)

**Group A** (Independent setup tasks):
- T002 (pyproject.toml)
- T003 (requirements.txt)
- T004 (.gitignore)
- T005 (src/__init__.py)
- T006 (tests/__init__.py)
- T007 (examples/README.md)

**Group B** (Independent foundational tasks):
- T010 (AgentState enum)
- T011 (ExecutionStatus enum)
- T012 (MessageType enum)
- T013 (validators.py)
- T014 (utils.py)

### Per User Story (Within Each Phase)

**US1 Parallel Groups**:
- Group 1: T015, T016, T017, T018 (models - can parallelize if different files)
- Group 2: T019, T020, T021, T022 (registry methods - sequential per class)

**US2 Parallel Groups**:
- Group 1: T027, T028, T029 (subagent model - sequential)
- Group 2: T030, T031, T032, T033, T034 (manager - sequential per class)

**US3 Parallel Groups**:
- Group 1: T037, T038 (message model)
- Group 2: T039, T040, T041 (event bus)
- Group 3: T042, T043, T044, T045 (message broker - can parallelize with event bus)

**US4 Parallel Groups**:
- Group 1: T046, T047, T048, T049 (dependency resolver methods)
- Group 2: T050, T051, T052 (composite execution)

---

## Testing Strategy (Optional - Not Required by Spec)

**Note**: The specification does not explicitly require tests. Tasks above focus on implementation. If tests are desired, add these optional tasks:

### Optional Test Tasks

**US1 Tests** (add after T026):
- [ ] T026a Write unit test for SkillRegistry in tests/unit/test_skill_registry.py
- [ ] T026b Write unit test for executor timeout handling in tests/unit/test_executor.py

**US2 Tests** (add after T036):
- [ ] T036a Write unit test for SubagentManager in tests/unit/test_agent_manager.py
- [ ] T036b Write integration test for concurrent execution in tests/integration/test_concurrent_agents.py

**US3 Tests** (add after T045):
- [ ] T045a Write unit test for EventBus in tests/unit/test_messenger.py
- [ ] T045b Write unit test for MessageBroker in tests/unit/test_messenger.py

**US4 Tests** (add after T053):
- [ ] T053a Write unit test for DependencyResolver in tests/unit/test_skill_registry.py
- [ ] T053b Write integration test for composite skill execution

---

## Implementation Notes

### MVP Scope (Recommended First Iteration)

**Phases to Complete**:
- Phase 1 (Setup)
- Phase 2 (Foundational)
- Phase 3 (US1 - Core Skills)

**Deliverable**: Developers can define and execute skills. This is the minimum viable product.

**Time Estimate**: 6-8 hours

### Extended Scope (Add Concurrency)

**Additional Phases**:
- Phase 4 (US2 - Subagents)

**Deliverable**: Parallel task execution with multiple agents.

**Time Estimate**: +4 hours

### Full Feature Set

**All Phases**: 1-7

**Deliverable**: Complete framework with communication, composition, and polish.

**Time Estimate**: 20-24 hours total

---

## Validation Checklist

Before marking feature complete, verify:

- [ ] All 4 user stories have acceptance criteria met
- [ ] Each user story can be tested independently
- [ ] Examples run successfully for all user stories
- [ ] No circular dependencies in skill composition
- [ ] Resource limits enforced (max agents)
- [ ] Error messages are descriptive and helpful
- [ ] README.md covers all user stories with examples
- [ ] Code follows Python PEP 8 style guide
- [ ] All public APIs have docstrings

---

## Task Format Validation

✅ **All tasks follow checklist format**: `- [ ] [TID] [P?] [Story?] Description with file path`

**Task Counts by Label**:
- Setup (no label): 8 tasks
- Foundational (no label): 6 tasks
- [US1]: 12 tasks
- [US2]: 10 tasks
- [US3]: 9 tasks
- [US4]: 8 tasks
- Polish (no label): 6 tasks

**Parallel Tasks**: 32 tasks marked [P]

**Total**: 59 tasks

---

**Generated**: 2025-12-19
**Ready for**: `/sp.implement` to execute tasks sequentially or in parallel per user story
