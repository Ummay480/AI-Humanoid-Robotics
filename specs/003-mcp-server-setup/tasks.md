# Tasks: MCP Server Setup - Git & Context7

**Input**: Design documents from `/specs/003-mcp-server-setup/`
**Prerequisites**: plan.md (design complete), research.md (completed via agent)

**Organization**: Since this is an infrastructure setup feature without traditional user stories, tasks are organized by functional setup goals that represent independently testable increments:
- **Goal 1**: Environment verification (can verify prerequisites)
- **Goal 2**: Git MCP server setup (can test Git operations via MCP)
- **Goal 3**: Context7 verification (can test Context7 already configured)
- **Goal 4**: Documentation and quickstart guide

**Tests**: No automated tests for this infrastructure setup. Verification is manual via MCP tool calls and Claude Code integration testing.

## Format: `[ID] [P?] [Goal] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Goal]**: Which setup goal this task belongs to (e.g., G1, G2, G3, G4)
- Include exact file paths in descriptions

## Path Conventions

- **Configuration files**: At repository root (`/mnt/d/aidd/hackathon/`)
- **Documentation**: `specs/003-mcp-server-setup/`
- **No source code changes**: This is configuration-only infrastructure setup

---

## Phase 1: Setup (Environment Verification)

**Purpose**: Verify all prerequisites are met before MCP server configuration

- [ ] T001 Verify Node.js version (v18+ required) in WSL with `node --version`
- [ ] T002 [P] Verify npx is available in WSL with `which npx`
- [ ] T003 [P] Verify Git is installed and configured in WSL with `git --version` and `git config --list`
- [ ] T004 Verify `.env` file exists at `/mnt/d/aidd/hackathon/.env` and contains GITHUB_PAT
- [ ] T005 Check WSL version with `wsl --status` (WSL 2 required for mirrored networking)

---

## Phase 2: Foundational (WSL & Environment Configuration)

**Purpose**: Core infrastructure that MUST be complete before MCP server configuration

**⚠️ CRITICAL**: MCP servers will not communicate properly without these configurations

- [ ] T006 [G1] Verify or create `.wslconfig` file in Windows home directory with networkingMode=mirrored
- [ ] T007 [G1] If `.wslconfig` was modified, run `wsl --shutdown` from Windows to apply changes
- [ ] T008 [G1] Verify environment variable substitution works by checking Claude Code can read `${GITHUB_PAT}` from .env

**Checkpoint**: Foundation ready - MCP server configuration can now begin

---

## Phase 3: Goal 1 - Git MCP Server Configuration (Priority: P1) 🎯 MVP

**Goal**: Configure Git MCP server (@cyanheads/git-mcp-server) in .claude.json to enable local Git operations via MCP protocol

**Independent Test**: After configuration, ask Claude Code "What's the current Git status?" and verify it responds with repository status using MCP Git tools

### Implementation for Goal 1

- [ ] T009 [G1] Read current `.claude.json` configuration at `/mnt/d/aidd/hackathon/.claude.json`
- [ ] T010 [G1] Add Git MCP server entry to `.claude.json` mcpServers section with stdio transport, npx command, @cyanheads/git-mcp-server args, and GIT_BASE_DIR environment variable set to `/mnt/d/aidd/hackathon`
- [ ] T011 [G1] Verify JSON syntax is valid in updated `.claude.json` file
- [ ] T012 [G1] Restart Claude Code to reload MCP server configuration
- [ ] T013 [G1] Verify Git MCP server appears in Claude Code MCP server list
- [ ] T014 [G1] Test Git MCP server by asking "What is the current Git branch?" and verify response
- [ ] T015 [G1] Test Git MCP server by asking "Show me git status" and verify working tree status is returned
- [ ] T016 [G1] Test Git MCP server by asking "Show me the last 3 git commits" and verify commit history is returned

**Checkpoint**: Git MCP server fully functional - can perform all Git operations via MCP

---

## Phase 4: Goal 2 - Context7 MCP Server Verification (Priority: P1)

**Goal**: Verify Context7 MCP server (already configured) is working correctly and can fetch documentation

**Independent Test**: Ask Claude Code "Use context7 to get documentation for React" and verify it resolves library ID and fetches documentation

### Implementation for Goal 2

- [ ] T017 [G2] Verify Context7 MCP server exists in `.claude.json` at `/mnt/d/aidd/hackathon/.claude.json`
- [ ] T018 [G2] Verify Context7 MCP server appears in Claude Code MCP server list
- [ ] T019 [G2] Test Context7 by asking "Use context7 to resolve the library ID for React" and verify it returns matching libraries
- [ ] T020 [G2] Test Context7 by asking "Get documentation for React using context7" and verify formatted documentation is returned
- [ ] T021 [G2] Test Context7 by asking "Use context7 to get FastAPI documentation" and verify Python library documentation works

**Checkpoint**: Context7 MCP server verified functional - can fetch up-to-date documentation for any supported library

---

## Phase 5: Goal 3 - GitHub MCP Server Verification (Priority: P2)

**Goal**: Verify GitHub MCP server (already configured) is working correctly with GITHUB_PAT authentication

**Independent Test**: Ask Claude Code about GitHub repository information and verify it responds using GitHub API via MCP

### Implementation for Goal 3

- [ ] T022 [G3] Verify GitHub MCP server exists in `.claude.json` at `/mnt/d/aidd/hackathon/.claude.json`
- [ ] T023 [G3] Verify GitHub MCP server has GITHUB_PAT environment variable configured
- [ ] T024 [G3] Verify GitHub MCP server appears in Claude Code MCP server list
- [ ] T025 [G3] Test GitHub MCP server by asking about repository information and verify it responds with GitHub API data
- [ ] T026 [G3] Verify GITHUB_PAT authentication is working (no authentication errors in logs)

**Checkpoint**: GitHub MCP server verified functional - can access GitHub API via MCP

---

## Phase 6: Goal 4 - Documentation & Quickstart Guide (Priority: P2)

**Goal**: Create comprehensive documentation to enable setup completion in <15 minutes

**Independent Test**: Follow quickstart.md from scratch on a fresh WSL instance and verify setup completes successfully in under 15 minutes

### Implementation for Goal 4

- [ ] T027 [P] [G4] Create `research.md` at `/mnt/d/aidd/hackathon/specs/003-mcp-server-setup/research.md` documenting MCP architecture, Git/Context7 capabilities, WSL considerations
- [ ] T028 [P] [G4] Create `data-model.md` at `/mnt/d/aidd/hackathon/specs/003-mcp-server-setup/data-model.md` with MCP message schemas and configuration structure
- [ ] T029 [P] [G4] Create directory `contracts` at `/mnt/d/aidd/hackathon/specs/003-mcp-server-setup/contracts/`
- [ ] T030 [P] [G4] Create `git-tools.md` at `/mnt/d/aidd/hackathon/specs/003-mcp-server-setup/contracts/git-tools.md` documenting Git MCP tool definitions
- [ ] T031 [P] [G4] Create `context7-tools.md` at `/mnt/d/aidd/hackathon/specs/003-mcp-server-setup/contracts/context7-tools.md` documenting Context7 MCP tool definitions
- [ ] T032 [G4] Create `quickstart.md` at `/mnt/d/aidd/hackathon/specs/003-mcp-server-setup/quickstart.md` with prerequisites, installation steps, configuration steps, verification steps, and troubleshooting guide
- [ ] T033 [G4] Validate quickstart.md by following all steps and ensuring they work as documented
- [ ] T034 [G4] Add common troubleshooting scenarios to quickstart.md based on any issues encountered during validation

**Checkpoint**: Complete documentation enables anyone to set up MCP servers in <15 minutes

---

## Phase 7: Goal 5 - Health Monitoring & Logging Setup (Priority: P3)

**Goal**: Establish monitoring and logging practices for MCP server operations

**Independent Test**: Trigger MCP operations and verify diagnostic logs appear in stderr without polluting stdout

### Implementation for Goal 5

- [ ] T035 [G5] Verify MCP servers log to stderr by checking Claude Code console output during Git operations
- [ ] T036 [G5] Test error handling by triggering an invalid Git operation and verify structured error response
- [ ] T037 [G5] Document log locations and monitoring approach in quickstart.md troubleshooting section
- [ ] T038 [G5] Create example health check commands for verifying MCP server status
- [ ] T039 [G5] Test health check commands and document expected outputs

**Checkpoint**: Monitoring and logging fully configured - can diagnose MCP server issues quickly

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements and validation across all setup goals

- [ ] T040 [P] Review all `.claude.json` configuration entries for consistency and correctness
- [ ] T041 [P] Validate all file paths in configuration use absolute paths (no relative paths)
- [ ] T042 [P] Verify no secrets are committed to version control (GITHUB_PAT only in .env)
- [ ] T043 Test all three MCP servers together (Git, Context7, GitHub) in a single prompt to verify no conflicts
- [ ] T044 Verify Speckit commands (e.g., `/sp.git.commit_pr`) can interact with Git MCP server
- [ ] T045 [P] Add README section documenting MCP server setup for new developers
- [ ] T046 Measure and document response times for Git operations (<500ms target) and Context7 queries (<2s target)
- [ ] T047 Run complete quickstart.md validation from fresh WSL environment to ensure <15 minute setup time

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup (Phase 1) - BLOCKS all configuration goals
- **Goal 1 - Git MCP (Phase 3)**: Depends on Foundational (Phase 2) - Can proceed in parallel with other goals
- **Goal 2 - Context7 (Phase 4)**: Depends on Foundational (Phase 2) - Can proceed in parallel with other goals
- **Goal 3 - GitHub MCP (Phase 5)**: Depends on Foundational (Phase 2) - Can proceed in parallel with other goals
- **Goal 4 - Documentation (Phase 6)**: Can start after Goal 1, 2, 3 are complete (documents what works)
- **Goal 5 - Monitoring (Phase 7)**: Depends on Goal 1, 2, 3 (monitors configured servers)
- **Polish (Phase 8)**: Depends on all goals being complete

### Goal Dependencies

- **Goal 1 (Git MCP)**: Can start after Foundational (Phase 2) - No dependencies on other goals
- **Goal 2 (Context7)**: Can start after Foundational (Phase 2) - No dependencies on other goals
- **Goal 3 (GitHub MCP)**: Can start after Foundational (Phase 2) - No dependencies on other goals
- **Goal 4 (Documentation)**: Requires Goal 1, 2, 3 complete (documents actual working configuration)
- **Goal 5 (Monitoring)**: Requires Goal 1, 2, 3 complete (monitors actual servers)

### Within Each Goal

- Configuration before restart
- Restart before verification
- Basic verification before advanced testing
- All tests pass before moving to next goal

### Parallel Opportunities

- Phase 1: Tasks T002, T003 can run in parallel (different checks)
- Phase 2: Tasks T006, T008 can run in parallel (different configurations)
- Phase 3 (Goal 1): All tasks must run sequentially (each depends on previous)
- Phase 4 (Goal 2): All tasks must run sequentially (each depends on previous)
- Phase 5 (Goal 3): All tasks must run sequentially (each depends on previous)
- Phase 6 (Goal 4): Tasks T027, T028, T029, T030, T031 can run in parallel (different files)
- Phase 7 (Goal 5): Tasks T035, T037, T038 can run in parallel (different investigations)
- Phase 8: Tasks T040, T041, T042, T045 can run in parallel (different files/checks)

**Cross-Goal Parallelism**: After Foundational phase completes:
- Goal 1 (Git MCP), Goal 2 (Context7), and Goal 3 (GitHub MCP) can ALL proceed in parallel

---

## Parallel Example: Configuration Goals

```text
# After Foundational phase completes, launch all MCP server verifications in parallel:

Goal 1: "Configure and test Git MCP server"
Goal 2: "Verify Context7 MCP server"
Goal 3: "Verify GitHub MCP server"

# All three can proceed simultaneously since they:
# - Configure different MCP servers (no file conflicts)
# - Have no dependencies on each other
# - Are independently testable
```

---

## Parallel Example: Documentation Phase

```text
# Launch all documentation tasks together in Goal 4:

Task T027: "Create research.md"
Task T028: "Create data-model.md"
Task T029: "Create contracts directory"
Task T030: "Create git-tools.md"
Task T031: "Create context7-tools.md"

# All can proceed simultaneously since they:
# - Create different files (no conflicts)
# - Have no dependencies on each other
# - Are independently completable
```

---

## Implementation Strategy

### MVP First (Goal 1 Only - Git MCP)

1. Complete Phase 1: Setup (verify environment)
2. Complete Phase 2: Foundational (WSL config)
3. Complete Phase 3: Goal 1 (Git MCP server)
4. **STOP and VALIDATE**: Test Git operations via MCP
5. Ready for immediate use: AI can now perform Git operations

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add Goal 1 (Git MCP) → Test independently → **MVP Delivered! 🎯**
3. Add Goal 2 (Context7 verification) → Test independently → Enhanced docs capability
4. Add Goal 3 (GitHub verification) → Test independently → Full GitHub API access
5. Add Goal 4 (Documentation) → Complete reference materials
6. Add Goal 5 (Monitoring) → Production-ready observability
7. Each goal adds value without breaking previous goals

### Parallel Team Strategy

With multiple developers (or parallel agent execution):

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - **Developer/Agent A**: Goal 1 (Git MCP configuration)
   - **Developer/Agent B**: Goal 2 (Context7 verification)
   - **Developer/Agent C**: Goal 3 (GitHub verification)
3. Once all servers verified:
   - **Developer/Agent D**: Goal 4 (Documentation)
   - **Developer/Agent E**: Goal 5 (Monitoring)
4. Goals complete and integrate independently

### Sequential Strategy (Single Developer/Agent)

1. Phase 1 (Setup): T001 → T005 (5 tasks, ~5 minutes)
2. Phase 2 (Foundational): T006 → T008 (3 tasks, ~10 minutes including WSL restart)
3. **Phase 3 (Goal 1 - Git MCP)**: T009 → T016 (8 tasks, ~15 minutes) ← **MVP Checkpoint**
4. Phase 4 (Goal 2 - Context7): T017 → T021 (5 tasks, ~10 minutes)
5. Phase 5 (Goal 3 - GitHub): T022 → T026 (5 tasks, ~10 minutes)
6. Phase 6 (Goal 4 - Documentation): T027 → T034 (8 tasks, ~20 minutes if sequential, ~10 if parallel)
7. Phase 7 (Goal 5 - Monitoring): T035 → T039 (5 tasks, ~15 minutes)
8. Phase 8 (Polish): T040 → T047 (8 tasks, ~20 minutes)

**Total Estimated Time**: ~105 minutes sequential, ~70 minutes with parallelization

---

## Success Criteria Mapping

Tasks directly address these success criteria from plan.md:

- **SC-001** (Git MCP configured): T009, T010, T011
- **SC-002** (Git tool calls work): T014, T015, T016
- **SC-003** (Context7 works): T019, T020, T021
- **SC-004** (Stderr logging): T035, T036
- **SC-005** (Environment variables): T008, T023, T026
- **SC-006** (WSL networking): T006, T007
- **SC-007** (Claude Code recognizes servers): T013, T018, T024
- **SC-008** (Speckit integration): T044
- **SC-009** (No frontend modifications): Inherent (no frontend tasks)
- **SC-010** (Quickstart <15 minutes): T032, T033, T047

---

## Notes

- **[P]** tasks = different files, no dependencies, can run in parallel
- **[Goal]** label maps task to specific setup goal for traceability
- Each goal should be independently completable and testable
- Configuration-only: No source code changes to existing project
- All paths are absolute (no relative paths)
- Secrets stay in `.env` file (not version-controlled)
- Commit after each phase or logical group
- Stop at any checkpoint to validate goal independently
- MCP servers communicate via stdio transport (JSON-RPC 2.0)
- All diagnostic logs to stderr (MCP protocol requirement)

---

## Task Count Summary

- **Phase 1 (Setup)**: 5 tasks
- **Phase 2 (Foundational)**: 3 tasks
- **Phase 3 (Goal 1 - Git MCP)**: 8 tasks
- **Phase 4 (Goal 2 - Context7)**: 5 tasks
- **Phase 5 (Goal 3 - GitHub MCP)**: 5 tasks
- **Phase 6 (Goal 4 - Documentation)**: 8 tasks
- **Phase 7 (Goal 5 - Monitoring)**: 5 tasks
- **Phase 8 (Polish)**: 8 tasks

**Total Tasks**: 47 tasks

**Parallel Opportunities**:
- 11 tasks can run in parallel (marked with [P])
- 3 goals (Git, Context7, GitHub) can proceed in parallel after Foundational phase
- Estimated time savings: 35 minutes with full parallelization

**MVP Scope**: Phases 1-3 only (16 tasks, ~30 minutes) delivers functional Git MCP server
