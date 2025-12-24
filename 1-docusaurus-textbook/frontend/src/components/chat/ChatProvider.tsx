import React, { createContext, useState, useCallback, useMemo, ReactNode } from 'react';
import { useAuth } from '../../contexts/AuthContext';

/**
 * Message interface representing a single chat message
 */
export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

/**
 * Chat context type definition
 */
export interface ChatContextType {
  messages: Message[];
  addMessage: (msg: Omit<Message, 'id' | 'timestamp'>) => void;
  updateLastMessage: (content: string) => void;
  clearMessages: () => void;
  isOpen: boolean;
  setIsOpen: (open: boolean) => void;
  isLoading: boolean;
  setIsLoading: (loading: boolean) => void;
  isAuthenticated: boolean;
}

/**
 * Chat Context for global chat state management
 */
export const ChatContext = createContext<ChatContextType | null>(null);

/**
 * ChatProvider Props
 */
interface ChatProviderProps {
  children: ReactNode;
}

/**
 * ChatProvider component - Manages global chat state
 *
 * @example
 * ```tsx
 * <ChatProvider>
 *   <App />
 * </ChatProvider>
 * ```
 */
export function ChatProvider({ children }: ChatProviderProps) {
  const { token, isLoading: authLoading } = useAuth();
  const [messages, setMessages] = useState<Message[]>([]);
  const [isOpen, setIsOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);

  // Check if user is authenticated
  const isAuthenticated = !!token && !authLoading;

  /**
   * Add a new message to the chat history
   * Automatically generates ID and timestamp
   */
  const addMessage = useCallback((msg: Omit<Message, 'id' | 'timestamp'>) => {
    const newMessage: Message = {
      ...msg,
      id: `msg-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, newMessage]);
  }, []);

  /**
   * Update the content of the last message (for streaming)
   * Used to append chunks to the assistant's response
   */
  const updateLastMessage = useCallback((content: string) => {
    setMessages((prev) => {
      if (prev.length === 0) return prev;
      const lastMessage = prev[prev.length - 1];
      if (lastMessage.role !== 'assistant') return prev;

      return [
        ...prev.slice(0, -1),
        { ...lastMessage, content }
      ];
    });
  }, []);

  /**
   * Clear all messages from chat history
   */
  const clearMessages = useCallback(() => {
    setMessages([]);
  }, []);

  // Memoize context value to prevent unnecessary re-renders
  const value = useMemo(
    () => ({
      messages,
      addMessage,
      updateLastMessage,
      clearMessages,
      isOpen,
      setIsOpen,
      isLoading,
      setIsLoading,
      isAuthenticated,
    }),
    [messages, addMessage, updateLastMessage, clearMessages, isOpen, isLoading, isAuthenticated]
  );

  return (
    <ChatContext.Provider value={value}>
      {children}
    </ChatContext.Provider>
  );
}
