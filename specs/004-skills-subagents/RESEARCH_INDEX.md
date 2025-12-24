# Research Index: Skills & Subagents Framework

**Research Conducted**: 2025-12-19
**Framework Analysis**: 5 major Python orchestration frameworks
**Patterns Documented**: 10+ architectural patterns
**Lines of Documentation**: 1,566 lines across 2 research documents

---

## Research Documents

### 1. [research.md](./research.md) - Comprehensive Analysis
**Size**: 745 lines | **22 KB**
**Focus**: Deep dive into frameworks, patterns, and architectural decisions

**Contents**:
- Executive Summary
- **Part 1**: Framework Evaluation (5 frameworks)
  - LangGraph (State-machine orchestration)
  - CrewAI (Team-based agents)
  - AutoGen (Conversation-based coordination)
  - Prefect (Workflow DAGs)
  - Ray (Distributed computing)
- **Part 2**: Message Passing Patterns
  - Pub/Sub Event Bus
  - Direct Messaging / Request-Response
  - Hybrid Approach
- **Part 3**: Skill Registry & Capability Management
  - Decorator-based registration
  - Configuration-based loading
  - Dependency resolution
- **Part 4**: Concurrency Patterns for Subagents
  - asyncio-based execution
  - Thread pool execution
  - Ray actors
  - Multiprocessing
- **Part 5**: Recommended Architecture
  - Proposed tech stack
  - Architectural diagram
  - Implementation checklist
- **Part 6**: Comparison Matrix
- **Part 7**: Resources & References

**Best For**: Understanding trade-offs, deep technical decisions, detailed code examples

---

### 2. [framework-comparison.md](./framework-comparison.md) - Quick Reference
**Size**: 821 lines | **14 KB**
**Focus**: Fast lookup guide, decision trees, templates

**Contents**:
1. Framework Quick Selector (by priority)
2. Requirements vs. Frameworks Matrix
3. Implementation Complexity Trade-offs
4. Message Passing Pattern Selection
5. Skill Registry Implementation Strategies
6. Concurrent Execution Patterns
7. Dependency Resolution Quick Guide
8. Error Handling & Recovery Patterns
9. Monitoring & Observability Setup
10. Integration Roadmap (3 phases)
11. Decision Tree (Which framework?)
12. Code Templates (Copy-paste ready)
13. Common Pitfalls & Solutions
14. Links & Resources

**Best For**: Making quick decisions, finding templates, implementation roadmap

---

## Key Findings Summary

### Recommended Technical Stack for Robotics Skills/Subagents

```
PHASE 1 (MVP - Weeks 1-2)
├─ Core: asyncio + custom Python modules
├─ Skills: Decorator-based registry
├─ Subagents: Custom SubagentManager
├─ Messaging: asyncio.Queue (simple request/response)
├─ Error handling: Basic try/catch + timeouts
└─ Observability: Python logging

PHASE 2 (Production - Weeks 3-4)
├─ Add: YAML configuration loader for skills
├─ Add: Dependency resolver (topological sort)
├─ Add: Event bus (asyncio.Event + pub/sub)
├─ Add: Hybrid messaging (queue + event bus)
├─ Add: Metrics collection
└─ Add: Lifecycle management (pause/resume/cancel)

PHASE 3+ (Advanced)
├─ Optional: LangGraph integration (complex state machines)
├─ Optional: External message broker (RabbitMQ/NATS)
├─ Optional: Web dashboard
├─ Optional: Plugin system
└─ Optional: ROS 2 integration
```

---

## Framework Evaluation Summary

### 1. LangGraph ★★★★☆ (Best for Composition)
- **Fit**: 4/5 for robotics
- **Complexity**: Medium
- **Dependencies**: Medium
- **Sweet Spot**: Complex skill state machines
- **Recommendation**: Integrate in Phase 3 if needed

### 2. CrewAI ★★★☆☆ (AI-Heavy)
- **Fit**: 3/5 for robotics
- **Complexity**: Low (high abstraction)
- **Dependencies**: High (requires LLM)
- **Sweet Spot**: Team coordination with reasoning
- **Recommendation**: Skip unless AI agents needed

### 3. AutoGen ★★☆☆☆ (Conversation-Based)
- **Fit**: 2/5 for robotics
- **Complexity**: High
- **Dependencies**: High (requires LLM)
- **Sweet Spot**: Complex multi-agent dialogue
- **Recommendation**: Not recommended for deterministic robotics

### 4. Prefect ★★★★☆ (Enterprise Workflows)
- **Fit**: 3/5 for robotics
- **Complexity**: Medium
- **Dependencies**: Medium
- **Sweet Spot**: Batch workflows, enterprise scale
- **Recommendation**: Skip for real-time robotics; too heavyweight

### 5. Ray ★★★★★ (Distributed Computing)
- **Fit**: 4/5 for robotics
- **Complexity**: High
- **Dependencies**: Low (pure Python)
- **Sweet Spot**: Distributed systems, extreme scale
- **Recommendation**: Keep in mind for multi-robot future; start with asyncio

### Custom asyncio ★★★★★ (Recommended)
- **Fit**: 5/5 for robotics
- **Complexity**: Low
- **Dependencies**: None (built-in)
- **Sweet Spot**: Single-robot, deterministic skills
- **Recommendation**: Start here, integrate frameworks incrementally

---

## Message Passing Patterns Decision Tree

```
How should agents communicate?

├─ Broadcast to many agents?
│  └─ Use Event Bus (Pub/Sub)
│     ├─ Implementation: asyncio.Event or Redis
│     └─ Use case: "Navigation complete" → trigger detection
│
├─ One-to-one with response?
│  └─ Use Message Queue (Request/Response)
│     ├─ Implementation: asyncio.Queue or gRPC
│     └─ Use case: Detector response to nav request
│
└─ Mix of both?
   └─ Use Hybrid Approach (RECOMMENDED)
      ├─ Event bus for notifications
      ├─ Message queue for requests
      └─ Best for complex coordination
```

---

## Skill Registry Strategy Comparison

### Strategy 1: Decorators (MVP) ✅ RECOMMENDED
```python
@SkillRegistry.register(SkillMetadata(name="navigate"))
async def navigate_skill(location: str):
    ...
```
- When: Quick prototyping
- Cost: Low
- Flexibility: Medium

### Strategy 2: YAML Configuration (Production) ⭐ ADD IN PHASE 2
```yaml
skills:
  - id: navigate
    module: robot_skills.navigation
    timeout: 30
```
- When: Hot-reloading needed
- Cost: Medium
- Flexibility: High

### Strategy 3: Plugin System (Enterprise)
```python
class SkillPlugin:
    def get_skills(self): ...
```
- When: Third-party extensions
- Cost: High
- Flexibility: Very High

**Roadmap**: Phase 1 → Phase 2 → Phase 3 (optional)

---

## Implementation Roadmap

### Week 1-2: MVP Foundation
✓ Custom asyncio SubagentManager
✓ Decorator-based SkillRegistry
✓ Basic asyncio.Queue for messaging
✓ Simple try/catch error handling
✓ Python logging

### Week 3-4: Production Features
✓ YAML skill configuration loader
✓ Dependency resolver (cycle detection)
✓ Event bus (pub/sub)
✓ Hybrid messaging
✓ Execution metrics
✓ Lifecycle management

### Week 5+: Advanced (Optional)
✓ LangGraph integration
✓ External message brokers
✓ Web dashboard
✓ Plugin system
✓ ROS 2 bridge

---

## Code Templates Available

### Minimal Skill Registry
See `framework-comparison.md` Section 12 for copy-paste ready code

### Minimal Subagent Manager
See `framework-comparison.md` Section 12 for copy-paste ready code

### Both documents include:
- Event bus implementations
- Message broker patterns
- Dependency resolvers
- Error handling examples
- Monitoring setup

---

## Quick Decision Guide

### Use This Framework If...

**LangGraph**: You need complex, stateful skill composition and want visualization
**CrewAI**: You want AI agents that reason about tasks (accept LLM costs)
**AutoGen**: You need agents negotiating through conversation (research/experimental)
**Prefect**: You're orchestrating batch data workflows at enterprise scale
**Ray**: You need distributed execution across multiple machines
**Custom asyncio**: ✅ You're building robotics skills on a single machine (RECOMMENDED)

---

## Common Questions Answered

### Q: Should we use an external message broker?
**A**: No, not initially. Start with asyncio.Queue and Event. Migrate to RabbitMQ/NATS only if you need:
- Distributed systems (multi-robot)
- Message persistence
- Advanced routing

### Q: Will asyncio be a bottleneck?
**A**: No, for typical robotics:
- Skill execution is I/O-bound (sensors, actuators)
- asyncio is perfect for I/O coordination
- Use ThreadPoolExecutor if you have blocking calls
- Use Ray if CPU-bound skills across multiple machines

### Q: When should we integrate LangGraph?
**A**: In Phase 3 if you have:
- Complex state machines for skills
- Need to visualize execution flow
- Want persistence/checkpointing
Don't integrate early - build custom foundation first

### Q: How do we prevent circular dependencies?
**A**: Implement topological sort resolver (code in research.md, Part 3)
- Validates skill dependency graph
- Detects cycles at skill registration
- Orders skills for safe execution

### Q: What about ROS 2 integration?
**A**: Keep for Phase 3+:
- Custom framework works standalone
- Can wrap ROS 2 services as skills
- Add bridge layer when needed
- Don't force ROS 2 into initial design

---

## Comparison Tables Quick Reference

See `framework-comparison.md` for:
- Section 2: Requirements vs. Frameworks Matrix
- Section 3: Complexity vs. Capability Trade-off
- Section 12: Common Pitfalls & Solutions table

---

## Important Recommendations

### 🎯 DO:
1. Start with custom asyncio foundation
2. Use decorators for skill registration
3. Implement simple event bus + queue messaging
4. Add YAML config loading in Phase 2
5. Build dependency resolver early
6. Separate skill definitions from orchestration
7. Log everything from day 1

### ❌ DON'T:
1. Don't lock into a heavyweight framework too early
2. Don't require LLM for deterministic skills
3. Don't use Prefect/Airflow for real-time control
4. Don't skip error handling and timeouts
5. Don't couple agents tightly (no direct imports)
6. Don't defer observability/logging
7. Don't assume asyncio is the bottleneck

---

## Document Navigation

```
Start here ─────────┐
                    │
                    ▼
          RESEARCH_INDEX.md (YOU ARE HERE)
                    │
         ┌──────────┼──────────┐
         │          │          │
         ▼          ▼          ▼
    [Quick Q?] [Deep dive?] [Code?]
         │          │          │
         └─────────┬┘          │
                   ▼           │
        framework- │           │
        comparison ├──────────┬┘
        .md        │          │
                   ▼          ▼
              research.md (745 lines)
              - 5 frameworks analyzed
              - Code examples
              - Patterns with examples
              - Architecture recommendations
```

---

## Files in This Research

```
specs/004-skills-subagents/
├── spec.md                    [Requirement specification]
├── plan.md                    [Architecture plan]
├── research.md               [⭐ Main research document - 745 lines]
├── framework-comparison.md   [⭐ Quick reference - 821 lines]
├── RESEARCH_INDEX.md        [THIS FILE - Navigation guide]
└── checklists/
    └── requirements.md       [Quality checklist]
```

---

## Next Steps

1. **Read**: Start with `framework-comparison.md` Section 11 (Decision Tree)
2. **Choose**: Confirm custom asyncio + Phase 1-3 roadmap is right for your use case
3. **Code**: Use templates in `framework-comparison.md` Section 12 to start Phase 1
4. **Plan**: Create implementation plan based on Phase 1-3 roadmap
5. **Execute**: Build MVP in Week 1-2 with just asyncio + decorators

---

## Contact / Updates

**Status**: Research complete, ready for implementation planning
**Last Updated**: 2025-12-19
**Version**: 1.0

For questions on any research finding, refer to:
- Deep dives: `research.md`
- Quick answers: `framework-comparison.md`
- Spec requirements: `spec.md`

---

