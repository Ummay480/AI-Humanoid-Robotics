---
id: 002
title: MCP Server Setup Plan
stage: plan
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-docusaurus-chatkit-frontend
branch: 002-docusaurus-chatkit-frontend
user: ummay
command: /sp.plan
labels: ["mcp", "infrastructure", "git", "context7", "wsl", "planning"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-mcp-server-setup/plan.md
tests:
  - N/A (planning phase)
---

## Prompt

You are my coding assistant. I want to set up MCP servers for my project with full compatibility for Ubuntu / WSL and Speckit CLI.

1. Git MCP Server
- Initialize a Git MCP server to manage project commits, branching, and Git integration via MCP workflow.
- Connect it to my local Docusaurus project at 1-docusaurus-textbook/frontend.
- Ensure it works seamlessly with Ubuntu / WSL.
- Provide monitoring/logging for Git operations performed via MCP.
- Ensure Speckit commands can interact with this server.

2. Contex7 MCP Server
- Set up a Contex7 MCP server for context-aware project tasks and agentic workflows.
- Integrate it with the frontend project structure.
- Ensure compatibility with Ubuntu / WSL and Speckit CLI.
- Allow communication with other MCP servers (like Git MCP) if needed.
- Support AI-assisted development and future automation tasks.

Tasks:
- Generate a step-by-step implementation plan for setting up both servers.
- Include all necessary commands or scripts for launching the servers in Ubuntu / WSL.
- Highlight any environment variables, config files, or Speckit-specific setup required.
- Include verification steps to ensure both servers are running correctly.
- Provide suggestions for monitoring, logging, or troubleshooting.

Constraints:
- Do not modify existing frontend project files.
- Keep the folder structure intact.
- Ensure full compatibility with Ubuntu / WSL and Speckit CLI.

Workflow:
1. First, propose the complete plan for my review.
2. After approval, implement both MCP servers according to the plan.

Begin by analyzing the project structure and generating the detailed setup plan.

## Response snapshot

I've created a comprehensive implementation plan for setting up Git MCP Server and Context7 MCP Server with full Ubuntu/WSL compatibility. Here's what was delivered:

**Research Phase (Phase 0)**:
- Conducted extensive research on MCP (Model Context Protocol) architecture
- Analyzed stdio transport vs HTTP/SSE for local development
- Evaluated Git MCP server options: @upstash/github-mcp-server vs @cyanheads/git-mcp-server
- Identified WSL-specific configuration requirements (mirrored networking)
- Documented logging best practices (stderr-only for diagnostics)
- Established health monitoring patterns

**Design Phase (Phase 1)**:
- Created data model for MCP server configuration, JSON-RPC messages, and tool schemas
- Defined API contracts for Git operations (status, diff, log, branch, commit, push, pull)
- Defined API contracts for Context7 tools (resolve-library-id, get-library-docs)
- Designed .claude.json configuration update adding Git MCP server
- Created quickstart guide with prerequisites, installation, verification, and troubleshooting

**Key Decisions**:
1. **Dual Git Servers**: Use BOTH @upstash/github-mcp-server (GitHub API) and @cyanheads/git-mcp-server (local Git CLI)
2. **Stdio Transport**: Optimal for local WSL development; simpler than HTTP
3. **GIT_BASE_DIR**: Security constraint to prevent path traversal
4. **Environment Variables**: GITHUB_PAT sourced from .env file with ${VAR} substitution
5. **WSL Configuration**: Requires networkingMode=mirrored in .wslconfig

**Deliverables**:
- Implementation plan document: specs/003-mcp-server-setup/plan.md
- Constitution compliance verified (Accuracy, Reproducibility, Functional Accuracy, Rigor)
- Success criteria defined (10 measurable outcomes)
- Risk analysis with mitigation strategies

**Next Steps**:
1. User reviews this plan for approval
2. Run /sp.tasks to generate actionable implementation tasks
3. Execute configuration updates and verification steps

## Outcome

- ✅ Impact: Complete MCP server setup plan with research, design, and contracts; enables Git and Context7 integration for AI-assisted development
- 🧪 Tests: N/A (planning phase; verification steps defined in plan)
- 📁 Files: Created specs/003-mcp-server-setup/plan.md (comprehensive implementation plan)
- 🔁 Next prompts: User review and approval → /sp.tasks to generate implementation tasks → Execute tasks with verification
- 🧠 Reflection: Research agent provided excellent MCP architecture understanding; dual Git server strategy balances GitHub API access with local Git operations; WSL configuration critical for stdio transport reliability

## Evaluation notes (flywheel)

- Failure modes observed: None (planning successful)
- Graders run and results (PASS/FAIL): Constitution Check: PASS (Accuracy, Reproducibility, Functional Accuracy, Rigor)
- Prompt variant (if applicable): N/A (standard /sp.plan workflow)
- Next experiment (smallest change to try): After implementation, measure MCP server response times to validate <500ms Git operations and <2s Context7 queries
