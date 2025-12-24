# Skills & Subagents Framework - Research & Specifications

**Branch**: `004-skills-subagents`
**Status**: Specification + Research Complete
**Last Updated**: 2025-12-19

This directory contains the complete specification, research, and planning documents for the Skills & Subagents Framework - a system for defining reusable robot skills and coordinating concurrent subagent execution.

---

## Quick Start Navigation

### For Decision Makers / Executives
Start here: **[RESEARCH_INDEX.md](./RESEARCH_INDEX.md)** (15 min read)
- Overview of all findings
- Key recommendations
- FAQ section
- Next steps

### For Architects / Technical Leads
Start here: **[framework-comparison.md](./framework-comparison.md)** (30 min read)
- Decision trees for architectural choices
- Comparison matrices
- Integration roadmap
- Common pitfalls

### For Developers
Start here: **[research.md](./research.md)** (45 min read)
- Detailed framework analysis
- Complete code examples
- Pattern implementations
- Technical trade-offs

### For Requirements / Planning
Start here: **[spec.md](./spec.md)** (20 min read)
- Feature specification
- User stories (P1-P4)
- Acceptance criteria
- Success metrics

---

## Document Directory

| Document | Size | Lines | Purpose | Best For |
|----------|------|-------|---------|----------|
| **[RESEARCH_INDEX.md](./RESEARCH_INDEX.md)** | 12 KB | 398 | Navigation & summary | Quick overview |
| **[framework-comparison.md](./framework-comparison.md)** | 14 KB | 458 | Quick reference guide | Implementation decisions |
| **[research.md](./research.md)** | 19 KB | 560 | Deep technical analysis | Understanding trade-offs |
| **[spec.md](./spec.md)** | 10 KB | 160+ | Feature specification | Requirements & scope |
| **[plan.md](./plan.md)** | 8 KB | 200+ | Architecture plan | High-level design |
| **[checklists/requirements.md](./checklists/requirements.md)** | - | - | Quality validation | Requirement completeness |

**Total Research**: 1,416 lines (59 KB)

---

## Key Findings at a Glance

### Framework Selection
```
Recommendation: Custom asyncio foundation (NOT an existing framework)

Why? Robotics use cases require:
  - Deterministic execution (not LLM-dependent)
  - Lightweight footprint (asyncio = built-in)
  - Full control over patterns (custom = flexible)
  - No external service dependencies

Framework Comparison:
  ★★★★★ asyncio MVP (RECOMMENDED)
  ★★★★  LangGraph (Phase 3+ optional)
  ★★★★  Ray (Distributed future)
  ★★★   CrewAI, Prefect (Skip for robotics)
  ★★    AutoGen (Not recommended)
```

### Implementation Roadmap
```
Phase 1 (MVP - Weeks 1-2):
  - asyncio SubagentManager
  - Decorator-based SkillRegistry
  - Simple Queue messaging
  - Basic error handling & timeouts

Phase 2 (Production - Weeks 3-4):
  - YAML skill configuration
  - Event bus (pub/sub)
  - Dependency resolver
  - Hybrid messaging

Phase 3+ (Advanced - Optional):
  - LangGraph integration
  - External message brokers
  - Web dashboard
  - Plugin system
```

### Message Passing Strategy
```
Recommended: Hybrid Approach

Event Bus (Pub/Sub):
  - Broadcasts: "Navigation complete" → trigger detection
  - Notifications: System events, state changes
  - Decoupled communication

Message Queue (Request/Response):
  - Direct requests: Skill calls with parameters
  - Responses: Results back to requester
  - Point-to-point communication
```

### Skill Registry Strategy
```
Phase 1: Decorator-based (Simple, lightweight)
  @SkillRegistry.register(SkillMetadata(name="navigate"))
  async def navigate_skill(location: str): ...

Phase 2: YAML configuration (Production flexibility)
  skills:
    - id: navigate
      module: robot_skills.navigation
      timeout: 30

Phase 3: Plugin system (Extensibility)
  class SkillPlugin:
      def get_skills(self) -> list: ...
```

---

## Research Objectives Addressed

All 5 research objectives completed:

1. ✅ **Find existing Python frameworks for agent orchestration**
   - Analyzed: LangGraph, CrewAI, AutoGen, Prefect, Ray
   - Conclusion: None perfect; custom asyncio best fit
   - Details: research.md, Part 1

2. ✅ **Identify best practices for skill/capability management**
   - Patterns: Decorator registry, configuration-based loading, plugin system
   - Recommendation: Phase approach (decorator → YAML → plugins)
   - Details: research.md, Part 3

3. ✅ **Find patterns for concurrent agent execution**
   - Patterns: asyncio, threading, multiprocessing, Ray actors
   - Recommendation: asyncio + ThreadPoolExecutor for robotics
   - Details: research.md, Part 4

4. ✅ **Research message passing patterns**
   - Patterns: Pub/Sub, direct messaging, RPC, hybrid
   - Recommendation: Hybrid (event bus + message queue)
   - Details: research.md, Part 2

5. ✅ **Identify skill composition & dependency resolution**
   - Patterns: Topological sort, cycle detection, DAG-based
   - Recommendation: Custom resolver with cycle detection
   - Details: research.md, Part 3

---

## Code Examples Provided

### Copy-Paste Ready Templates
Available in **framework-comparison.md** Section 12:

1. **Minimal Skill Registry** (10 lines)
   ```python
   @SkillRegistry.register(SkillMetadata(name="navigate"))
   async def navigate_skill(location: str):
       return {"status": "success"}
   ```

2. **Minimal Subagent Manager** (20 lines)
   ```python
   agent_id = await manager.spawn_subagent(registry)
   result = await manager.execute(agent_id, "navigate", {"location": "kitchen"})
   ```

### Complete Implementations
Available in **research.md** with full working code:
- Event bus (pub/sub pattern)
- Message broker (request/response)
- Dependency resolver (topological sort)
- Error handling & recovery
- Concurrent execution patterns
- Monitoring & metrics

---

## Architectural Recommendations

### DO:
- ✅ Start with custom asyncio foundation
- ✅ Use decorators for skill registration
- ✅ Implement hybrid messaging (event bus + queue)
- ✅ Add YAML config in Phase 2
- ✅ Separate skill definitions from orchestration
- ✅ Log everything from day 1

### DON'T:
- ❌ Don't lock into heavyweight framework too early
- ❌ Don't require LLM for deterministic skills
- ❌ Don't use Prefect/Airflow for real-time control
- ❌ Don't skip error handling/timeouts
- ❌ Don't couple agents tightly
- ❌ Don't defer observability

---

## Implementation Checklist

### Phase 1 (MVP)
- [ ] Custom asyncio SubagentManager class
- [ ] Decorator-based SkillRegistry
- [ ] Basic subagent spawning and lifecycle
- [ ] Simple asyncio.Queue messaging
- [ ] Timeout management (asyncio.wait_for)
- [ ] Python logging framework
- [ ] Basic error handling

### Phase 2 (Production)
- [ ] YAML skill configuration loader
- [ ] Event bus (asyncio-based pub/sub)
- [ ] Hybrid messaging (queue + event bus)
- [ ] Dependency resolver (topological sort)
- [ ] Circular dependency detection
- [ ] Execution metrics & statistics
- [ ] Lifecycle management (pause/resume/cancel)

### Phase 3+ (Advanced)
- [ ] LangGraph integration (optional)
- [ ] External message broker support (RabbitMQ/NATS)
- [ ] Web dashboard for monitoring
- [ ] Plugin system for skill extensions
- [ ] ROS 2 integration bridge (optional)

---

## Common Questions

**Q: Should we use an existing framework like CrewAI?**
A: No, not for MVP. Start with custom asyncio (minimal overhead, full control). Add frameworks later if needs change.

**Q: Will asyncio be a bottleneck?**
A: No. Robotics skills are I/O-bound (sensors/actuators). asyncio is perfect. Use ThreadPoolExecutor for blocking calls.

**Q: When should we integrate LangGraph?**
A: Phase 3+, only if you have complex state machines or need workflow visualization.

**Q: How do we prevent circular dependencies in skills?**
A: Implement topological sort resolver in Phase 2 (code provided in research.md, Part 3).

**Q: What about ROS 2 integration?**
A: Keep for Phase 3+. Build custom framework independently first, then add ROS 2 bridge when needed.

---

## Resource Links

### Frameworks Evaluated
- [LangGraph](https://github.com/langchain-ai/langgraph)
- [CrewAI](https://github.com/joaomdmoura/crewai)
- [AutoGen](https://github.com/microsoft/autogen)
- [Prefect](https://github.com/PrefectHQ/prefect)
- [Ray](https://github.com/ray-project/ray)

### Python Concurrency
- [asyncio Documentation](https://docs.python.org/3/library/asyncio.html)
- [Real Python asyncio Guide](https://realpython.com/async-io-python/)

### Message Brokers (for Phase 2+)
- [RabbitMQ](https://www.rabbitmq.com/)
- [NATS](https://nats.io/)
- [Redis](https://redis.io/)

---

## Next Steps

### Immediate (Today)
1. Read [RESEARCH_INDEX.md](./RESEARCH_INDEX.md) for overview (15 min)
2. Review decision tree in [framework-comparison.md](./framework-comparison.md) Section 11 (10 min)
3. Confirm custom asyncio approach fits your constraints

### This Week
1. Review complete [research.md](./research.md) for deep understanding
2. Create implementation plan based on Phase 1-3 roadmap
3. Set up development environment and repository

### This Month
1. Implement Phase 1 MVP using provided templates
2. Test with real robotics skills
3. Plan Phase 2 features (YAML config, event bus)

---

## Document Maintenance

**Version**: 1.0
**Last Updated**: 2025-12-19
**Created By**: Agent research task
**Status**: Complete - Ready for implementation planning

### Updates When:
- New frameworks emerge
- Team refines architecture
- Phase 1-2 implementation reveals new patterns
- ROS 2 / distributed systems needs arise

---

## Support

For questions about:
- **Framework selection**: See framework-comparison.md Section 11
- **Code examples**: See research.md or framework-comparison.md Section 12
- **Implementation roadmap**: See framework-comparison.md Section 10
- **Architectural decisions**: See research.md Part 5
- **Quick answers**: See RESEARCH_INDEX.md FAQ section

---

**Status**: RESEARCH COMPLETE - Ready for planning and implementation phases
