/**
 * Agent API Types
 *
 * CRITICAL: This is a relay-only proxy on the frontend.
 * All agent execution, tool calling, and MCP invocation happens on the backend.
 * Frontend ONLY: Sends requests to /api/agent, displays responses.
 */

export interface AgentRequest {
  message: string;
  attachments?: AgentAttachment[];
  context_files: string[]; // ≤7 files (enforced by Context-7 system)
  user_settings: {
    theme: string;
    language: string;
  };
}

export interface AgentAttachment {
  id: string;
  name: string;
  type: string;
  size: number;
  data?: string; // base64 if needed
}

export interface AgentResponse {
  message: string;
  references?: AgentReference[];
  status: 'pending' | 'completed' | 'error';
  error?: AgentError;
  metadata?: {
    processingTime?: number;
    tokensUsed?: number;
    toolsCalled?: string[];
  };
}

export interface AgentReference {
  type: 'lesson' | 'chapter' | 'section' | 'external';
  title: string;
  url?: string;
  source?: string;
}

export interface AgentError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

export interface AgentStreamMessage {
  type: 'start' | 'chunk' | 'reference' | 'end' | 'error';
  data: unknown;
  timestamp: number;
}

/**
 * Frontend responsibility: ONLY
 * - Send AgentRequest to /api/agent POST endpoint
 * - Stream responses as AgentStreamMessage[]
 * - Display AgentResponse to user
 * - Manage UI state (loading, error)
 *
 * Backend responsibility (NOT on frontend):
 * - Execute OpenAI agents
 * - Call tools
 * - Invoke MCP servers
 * - Process context files
 * - Handle authentication and secrets
 */
