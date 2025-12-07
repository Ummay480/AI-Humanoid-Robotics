export interface ChatMessage {
  id: string;
  timestamp: number;
  sender: 'user' | 'bot';
  content: string;
  attachments?: ChatAttachment[];
  references?: string[];
}

export interface ChatAttachment {
  id: string;
  name: string;
  type: string;
  size: number;
  url?: string;
}

export interface ChatState {
  messages: ChatMessage[];
  isLoading: boolean;
  error?: string;
  selectedContextFiles: ContextFile[];
}

export interface ChatRequest {
  message: string;
  contextFiles: ContextFile[];
  userSettings: {
    theme: string;
    language: string;
  };
}

export interface ContextFile {
  id: string;
  name: string;
  path: string;
  size: number;
  type: string;
  uploadedAt: number;
}
