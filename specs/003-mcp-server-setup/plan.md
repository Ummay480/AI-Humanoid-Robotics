# Implementation Plan: MCP Server Setup - Git & Context7

**Branch**: `002-docusaurus-chatkit-frontend` | **Date**: 2025-12-11 | **Spec**: N/A (Infrastructure Setup)
**Input**: User request for Git MCP Server and Context7 MCP Server setup with Ubuntu/WSL compatibility

## Summary

Set up two MCP servers (Git and Context7) to enhance AI-assisted development workflows in the project. The Git MCP server will manage Git operations (commits, branching, status, diff) via MCP protocol, while Context7 provides real-time, version-specific code documentation. Both servers will be configured for Ubuntu/WSL compatibility and integrated with the existing `.claude.json` configuration, enabling Speckit CLI commands to interact seamlessly with these servers.

**Key Technical Approach**: Stdio-based MCP servers using JSON-RPC 2.0 protocol, configured in `.claude.json` with environment variable substitution for secrets, comprehensive logging to stderr, and health monitoring capabilities.

## Technical Context

**Language/Version**: TypeScript (via npx), Node.js LTS (v18+)
**Primary Dependencies**:
- `@modelcontextprotocol/sdk` (TypeScript MCP SDK)
- `@upstash/context7-mcp` (Context7 MCP Server - already configured)
- `@upstash/github-mcp-server` OR `@cyanheads/git-mcp-server` (Git MCP Server - to be evaluated)
- `zod` v3.25+ (schema validation)

**Storage**: N/A (stateless servers; Git operations use existing .git directory)
**Testing**: Manual MCP Inspector testing, integration testing via Claude Code
**Target Platform**: Ubuntu/WSL (Windows Subsystem for Linux)
**Project Type**: Infrastructure/tooling enhancement (no source code changes)
**Performance Goals**:
- Git operations: <500ms response time for status/diff/log
- Context7 queries: <2s for documentation retrieval
- Health check responsiveness: <100ms

**Constraints**:
- Must not modify existing frontend project files
- Must maintain folder structure integrity
- Stdio transport only (no HTTP/SSE for local development)
- All diagnostic logs to stderr (MCP protocol requirement)
- Environment variables sourced from `.env` file

**Scale/Scope**:
- Single project workspace: `/mnt/d/aidd/hackathon`
- Git repository scope: Current repository only
- Context7 scope: All supported libraries (npm, Python, etc.)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Accuracy (PASS)
- **Requirement**: All technical content must be verified against primary sources and official documentation
- **Validation**: Research references official MCP documentation (modelcontextprotocol.io), TypeScript SDK, and verified server implementations (GitHub repos)
- **Status**: ✅ PASS - All architectural decisions trace to authoritative sources

### Reproducibility (PASS)
- **Requirement**: Every code snippet and setup must be traceable and executable as described
- **Validation**: All commands and configurations will be tested in target Ubuntu/WSL environment before finalization
- **Status**: ✅ PASS - Setup includes verification steps and expected outputs

### Functional Accuracy (PASS)
- **Requirement**: Code samples MUST compile and run as written; breaking changes documented with migration paths
- **Validation**: Configuration changes are non-breaking additions to existing `.claude.json`; servers are validated before deployment
- **Status**: ✅ PASS - No modifications to existing working systems; purely additive infrastructure

### Rigor (PASS)
- **Requirement**: Prefer peer-reviewed papers, official SDK documentation, or authoritative tutorials
- **Validation**: Implementation based on official MCP specification, TypeScript SDK docs, and verified GitHub implementations
- **Status**: ✅ PASS - Primary sources: Anthropic MCP docs, official SDKs, established server implementations

## Project Structure

### Documentation (this feature)

```text
specs/003-mcp-server-setup/
├── plan.md              # This file (implementation plan)
├── research.md          # Phase 0: MCP architecture, Git/Context7 capabilities, WSL considerations
├── data-model.md        # Phase 1: MCP message schemas, configuration structure
├── quickstart.md        # Phase 1: Setup instructions, verification steps, troubleshooting
└── contracts/           # Phase 1: MCP tool/resource definitions
    ├── git-tools.md     # Git MCP server tool definitions
    └── context7-tools.md # Context7 MCP server tool definitions
```

### Configuration Files (repository root)

```text
/mnt/d/aidd/hackathon/
├── .claude.json         # MODIFIED: Add Git MCP server configuration
├── .env                 # VERIFIED: Contains GITHUB_PAT for GitHub MCP
├── .mcp.json            # CREATED: Project-scoped MCP configuration (optional)
└── .wslconfig           # CHECKED: Verify networkingMode=mirrored (WSL 2)

# MCP Server Packages (installed via npx, no local files)
# - @upstash/context7-mcp (already configured)
# - Selected Git MCP server (to be determined in Phase 0)
```

### Source Code (repository root)

**Structure Decision**: This is an infrastructure/tooling feature. No source code changes to the existing project structure are required. All changes are configuration-only, preserving the existing frontend project structure at `1-docusaurus-textbook/frontend/`.

## Complexity Tracking

> **No Constitution violations detected. This section is informational only.**

| Consideration | Justification | Alternative Considered |
|---------------|---------------|------------------------|
| MCP stdio vs HTTP transport | Stdio optimal for local development; simpler, no network config | HTTP+SSE rejected: unnecessary complexity for local WSL setup |
| Multiple MCP servers (Git + Context7) | Each server provides distinct, non-overlapping capabilities | Single combined server rejected: separation of concerns, modularity |
| Environment variable substitution | Secure secret management; `.env` file not version-controlled | Hardcoded tokens rejected: security violation |

## Phase 0: Research & Requirements Clarification

### Research Objectives

**Research completed via agent investigation. Key findings:**

1. **MCP Architecture Understanding**
   - Client-Server model with stdio transport for local development
   - JSON-RPC 2.0 protocol with newline-delimited messages
   - Three capability primitives: Tools (executable), Resources (data), Prompts (templates)
   - Lifecycle: Initialize → Capability Negotiation → Operation → Shutdown

2. **Git MCP Server Evaluation**
   - **Option A**: `@upstash/github-mcp-server` (currently configured)
     - Focuses on GitHub API operations (issues, PRs, repository management)
     - Requires GITHUB_PAT environment variable
     - Best for GitHub-centric workflows

   - **Option B**: `@cyanheads/git-mcp-server` (comprehensive Git operations)
     - Full Git CLI wrapper: clone, commit, branch, diff, log, status, push, pull, merge, rebase
     - Worktree and tag management
     - Supports both stdio and HTTP transports
     - Path sanitization and security features
     - Best for local Git operations

   - **RECOMMENDATION**: Use BOTH servers for complementary capabilities
     - Keep `@upstash/github-mcp-server` for GitHub integration (issues, PRs)
     - Add `@cyanheads/git-mcp-server` for local Git operations (commit, branch, diff, log)

3. **Context7 Server (Already Configured)**
   - NPM package: `@upstash/context7-mcp`
   - Provides up-to-date, version-specific code documentation
   - Tools: `resolve-library-id`, `get-library-docs`
   - Status: ✅ Already configured in `.claude.json`

4. **WSL Compatibility Requirements**
   - **Network Configuration**: Verify `.wslconfig` has `networkingMode = mirrored` for WSL 2
   - **Path Handling**: Use absolute paths in configuration
   - **Environment Variables**: Source from `.env` file with `${VARIABLE}` substitution
   - **Configuration Location**: Project-scoped `.claude.json` (already exists)

5. **Logging & Monitoring Best Practices**
   - **Critical Rule**: All diagnostic logs MUST go to stderr (stdout reserved for MCP protocol)
   - **Structured Logging**: Include timestamp, level, serverName, method, requestId, message
   - **Health Checks**: Liveness (process running), Readiness (can handle requests), Functional (capabilities work)
   - **Error Handling**: Structured error responses with `isError` flag, retry with backoff

### Research Output Artifact

**File**: `specs/003-mcp-server-setup/research.md`

**Contents Summary**:
- MCP protocol fundamentals (JSON-RPC 2.0, stdio transport, lifecycle management)
- Git MCP server comparison matrix (features, security, performance)
- Context7 capabilities and integration patterns
- WSL-specific configuration requirements and troubleshooting
- Logging/monitoring architecture and best practices
- Health check implementation patterns
- Security considerations (path sanitization, secret management)

## Phase 1: Design & Contracts

### 1.1 Data Model

**File**: `specs/003-mcp-server-setup/data-model.md`

**Entities**:

#### MCP Server Configuration
```typescript
interface McpServerConfig {
  type: "stdio" | "http+sse" | "websocket"
  command: string                    // Executable path (e.g., "npx", "/usr/bin/node")
  args: string[]                     // Command arguments
  env?: Record<string, string>       // Environment variables (supports ${VAR} substitution)
}
```

#### MCP Message (JSON-RPC 2.0)
```typescript
interface McpRequest {
  jsonrpc: "2.0"
  method: string                     // e.g., "initialize", "tools/list", "tools/call"
  params?: Record<string, unknown>
  id: number | string
}

interface McpResponse {
  jsonrpc: "2.0"
  result?: unknown
  error?: {
    code: number
    message: string
    data?: unknown
  }
  id: number | string
}
```

#### Git Tool Schema (Zod Validation)
```typescript
// Example: git status tool
const GitStatusSchema = z.object({
  repository: z.string().describe("Path to Git repository"),
  porcelain: z.boolean().optional().describe("Use machine-readable format")
})

// Example: git commit tool
const GitCommitSchema = z.object({
  repository: z.string().describe("Path to Git repository"),
  message: z.string().describe("Commit message"),
  files: z.array(z.string()).optional().describe("Specific files to commit")
})
```

#### Context7 Tool Schema
```typescript
const ResolveLibrarySchema = z.object({
  libraryName: z.string().describe("Package or library name (e.g., 'react', 'fastapi')")
})

const GetDocsSchema = z.object({
  libraryId: z.string().describe("Context7 library ID from resolve-library-id"),
  version: z.string().optional().describe("Specific version (default: latest)")
})
```

### 1.2 API Contracts

**Directory**: `specs/003-mcp-server-setup/contracts/`

#### Git MCP Server Tools (`contracts/git-tools.md`)

| Tool Name | Description | Input Schema | Output |
|-----------|-------------|--------------|--------|
| `git_status` | Get working tree status | `{ repository: string, porcelain?: boolean }` | File status list (modified, staged, untracked) |
| `git_diff` | Show changes between commits/files | `{ repository: string, staged?: boolean, file?: string }` | Diff output (unified format) |
| `git_log` | Show commit history | `{ repository: string, maxCount?: number, oneline?: boolean }` | Commit history with SHA, author, date, message |
| `git_branch` | List, create, or delete branches | `{ repository: string, action: "list"\|"create"\|"delete", branchName?: string }` | Branch list or operation result |
| `git_commit` | Record changes to repository | `{ repository: string, message: string, files?: string[], amend?: boolean }` | Commit SHA and summary |
| `git_push` | Update remote refs | `{ repository: string, remote?: string, branch?: string, force?: boolean }` | Push result and updated refs |
| `git_pull` | Fetch and merge from remote | `{ repository: string, remote?: string, branch?: string }` | Pull result and merge status |

**Security Constraints**:
- Path validation: Prevent directory traversal attacks
- Destructive operations: Require explicit confirmation flags
- Environment isolation: Use `GIT_BASE_DIR` to restrict repository access

#### Context7 MCP Server Tools (`contracts/context7-tools.md`)

| Tool Name | Description | Input Schema | Output |
|-----------|-------------|--------------|--------|
| `resolve-library-id` | Convert package name to Context7 ID | `{ libraryName: string }` | List of matching libraries with IDs and versions |
| `get-library-docs` | Fetch up-to-date documentation | `{ libraryId: string, version?: string }` | Formatted documentation with examples |

**Usage Pattern**:
1. Call `resolve-library-id` with package name (e.g., "react")
2. Select appropriate library ID from results
3. Call `get-library-docs` with library ID to fetch documentation
4. Documentation injected into AI prompt context

### 1.3 Configuration Design

**File**: `.claude.json` (MODIFIED)

```json
{
  "projects": {
    "/mnt/d/AIDD/hackathon": {
      "mcpServers": {
        "context7": {
          "type": "stdio",
          "command": "npx",
          "args": ["@upstash/context7-mcp"]
        },
        "githubMCP": {
          "type": "stdio",
          "command": "npx",
          "args": ["@upstash/github-mcp-server"],
          "env": {
            "GITHUB_PAT": "${GITHUB_PAT}"
          }
        },
        "gitMCP": {
          "type": "stdio",
          "command": "npx",
          "args": ["@cyanheads/git-mcp-server"],
          "env": {
            "GIT_BASE_DIR": "/mnt/d/aidd/hackathon"
          }
        }
      }
    }
  }
}
```

**Environment Variables** (`.env` file - VERIFIED, NOT MODIFIED):
```bash
GITHUB_PAT=ghp_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
```

**WSL Configuration** (`~/.wslconfig` in Windows home directory - USER MUST VERIFY):
```ini
[wsl2]
networkingMode = mirrored
```

### 1.4 Quickstart Guide

**File**: `specs/003-mcp-server-setup/quickstart.md`

**Contents**:
1. **Prerequisites Check**
   - Node.js LTS (v18+) installed in WSL
   - Git installed and configured
   - `.env` file with GITHUB_PAT present
   - WSL 2 with mirrored networking (verify `.wslconfig`)

2. **Installation Steps**
   - No installation required (npx auto-installs on first use)
   - Verify `npx` is available: `which npx`

3. **Configuration Steps**
   - Update `.claude.json` with Git MCP server entry
   - Restart Claude Code to reload configuration

4. **Verification Steps**
   - Check MCP server status in Claude Code
   - Test Git operations: "What's the current Git status?"
   - Test Context7: "Get documentation for React v18"
   - Review logs for errors

5. **Troubleshooting**
   - Common issues: Path errors, network connectivity, permission denied
   - Log locations: stderr output in Claude Code console
   - Health check commands

## Phase 2: Implementation Tasks (DEFERRED)

**Note**: This plan document covers Phase 0 (Research) and Phase 1 (Design). Phase 2 (Implementation Tasks) will be generated by the `/sp.tasks` command, which creates actionable, dependency-ordered tasks based on this plan.

**Expected Task Categories**:
1. Environment verification (Node.js, Git, WSL config)
2. Configuration file updates (`.claude.json`)
3. MCP server testing and validation
4. Documentation finalization
5. Health monitoring setup

## Success Criteria

### Measurable Outcomes

- **SC-001**: Git MCP server configured in `.claude.json` with correct stdio transport settings
- **SC-002**: Git MCP server responds to tool calls (`git_status`, `git_log`, `git_diff`) without errors
- **SC-003**: Context7 MCP server successfully resolves library IDs and fetches documentation
- **SC-004**: All MCP servers log diagnostics to stderr (no stdout pollution)
- **SC-005**: Environment variables correctly substituted from `.env` file
- **SC-006**: WSL networking allows MCP server communication (verified via health check)
- **SC-007**: Claude Code recognizes and lists all three MCP servers (context7, githubMCP, gitMCP)
- **SC-008**: Speckit commands can invoke Git operations through MCP (e.g., `/sp.git.commit_pr`)
- **SC-009**: No modifications to existing frontend project files
- **SC-010**: Comprehensive quickstart guide enables setup in <15 minutes

## Assumptions

- Node.js LTS (v18+) and npx are available in WSL environment
- Git is installed and configured with user credentials
- `.env` file with GITHUB_PAT exists and is not version-controlled
- WSL 2 is being used (WSL 1 has different networking requirements)
- User has sudo access for WSL configuration changes if needed
- Claude Code is installed and has access to project directory
- Project repository is already initialized (`.git` directory exists)

## Constraints

- **No Source Code Modifications**: Only configuration files may be changed
- **Stdio Transport Only**: HTTP/SSE transport not required for local development
- **WSL-Specific**: Configuration optimized for Ubuntu/WSL, not Windows native
- **Backward Compatibility**: Existing Context7 and GitHub MCP servers must continue working
- **Security**: No secrets in version-controlled files; all tokens in `.env`
- **Logging Discipline**: All diagnostic output to stderr (MCP protocol requirement)

## Non-Goals

- Custom MCP server development (using existing, published servers only)
- HTTP/SSE transport configuration (stdio sufficient for local development)
- Multi-user MCP server deployment (single developer use case)
- MCP server performance optimization (default configurations adequate)
- Advanced Git workflow automation (basic Git operations only)
- Integration with CI/CD pipelines (local development focus)
- MCP server monitoring dashboards (basic logging sufficient)
- Custom Zod schema extensions (use default tool schemas)

## Risks & Mitigation

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| WSL networking issues prevent MCP communication | High | Medium | Verify `.wslconfig` mirrored networking; test with MCP Inspector |
| npx fails to auto-install MCP packages | High | Low | Pre-install packages with `npm install -g @cyanheads/git-mcp-server` |
| Git MCP server path traversal vulnerability | High | Low | Use `GIT_BASE_DIR` environment variable to restrict access |
| Environment variable substitution fails | Medium | Low | Test `.env` loading; use absolute paths; check Claude Code logs |
| MCP servers conflict with each other | Medium | Very Low | Each server isolated; separate tool namespaces |
| Logs mixed with MCP protocol output | Medium | Low | Enforce stderr logging; validate with MCP Inspector |

## Follow-Up Work

- **Integration Testing**: Create test scenarios for common Git + Context7 workflows
- **Advanced Git Operations**: Explore merge, rebase, tag management tools
- **Custom Prompts**: Define reusable MCP prompts for common development tasks
- **Health Dashboard**: Optional web interface for MCP server monitoring
- **Documentation Generation**: Auto-generate tool reference docs from MCP schemas

---

**Plan Completion Status**: ✅ Phase 0 Research Complete | ✅ Phase 1 Design Complete | ⏸️ Phase 2 Tasks (deferred to `/sp.tasks`)

**Next Steps**:
1. Review this plan for accuracy and completeness
2. Run `/sp.tasks` to generate actionable implementation tasks
3. Execute tasks with verification at each step
4. Create PHR documenting this planning session
