import React, { useEffect } from 'react';
import { createPortal } from 'react-dom';
import { useChat } from '../../hooks/useChat';
import { useStreamResponse } from '../../hooks/useStreamResponse';
import { ChatMessages } from './ChatMessages';
import { ChatInput } from './ChatInput';
import { sendMessage } from '../../utils/chatHelpers';

/**
 * ChatPanel Props
 */
interface ChatPanelProps {
  isOpen: boolean;
  onClose: () => void;
}

/**
 * ChatPanel Component - Main chat interface panel
 *
 * Features:
 * - Renders as a portal (outside main DOM tree)
 * - Slide-in animation from right
 * - Header with title and close button
 * - Scrollable message area
 * - Fixed input at bottom
 * - Click overlay to close
 * - Handles message sending and streaming responses
 *
 * @param props - Component props
 * @param props.isOpen - Whether panel is visible
 * @param props.onClose - Callback to close the panel
 *
 * @example
 * ```tsx
 * <ChatPanel
 *   isOpen={isChatOpen}
 *   onClose={() => setIsChatOpen(false)}
 * />
 * ```
 */
export function ChatPanel({ isOpen, onClose }: ChatPanelProps) {
  const { messages, addMessage, updateLastMessage, clearMessages, isLoading, setIsLoading } = useChat();
  const streamResponse = useStreamResponse();

  // Handle sending a message
  const handleSend = async (text: string) => {
    // Validate and log message
    const isValid = sendMessage(text);
    if (!isValid) return;

    // Add user message to chat
    addMessage({ role: 'user', content: text });

    // Set loading state
    setIsLoading(true);

    // Add empty assistant message to update during streaming
    addMessage({ role: 'assistant', content: '' });

    try {
      // Stream the response
      await streamResponse(
        text,
        (chunk, fullText) => {
          // Update the last message with accumulated text
          updateLastMessage(fullText);
        },
        () => {
          // Complete - turn off loading
          setIsLoading(false);
        },
        (error) => {
          // Error handling
          console.error('Stream error:', error);
          updateLastMessage('Sorry, I encountered an error processing your question. Please try again.');
          setIsLoading(false);
        }
      );
    } catch (error) {
      console.error('Failed to stream response:', error);
      updateLastMessage('Sorry, something went wrong. Please try again.');
      setIsLoading(false);
    }
  };

  // Handle clear chat confirmation
  const handleClearChat = () => {
    if (messages.length === 0) return;

    const confirmed = window.confirm('Clear all messages? This cannot be undone.');
    if (confirmed) {
      clearMessages();
    }
  };

  // Close panel on Escape key
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        onClose();
      }
    };

    document.addEventListener('keydown', handleEscape);
    return () => document.removeEventListener('keydown', handleEscape);
  }, [isOpen, onClose]);

  if (!isOpen) return null;

  const panel = (
    <div className="chat-overlay" onClick={onClose}>
      <div
        className="chat-panel"
        onClick={(e) => e.stopPropagation()} // Prevent closing when clicking inside panel
      >
        {/* Header */}
        <div className="chat-header">
          <h2 className="chat-header-title">AI Assistant</h2>
          <div className="chat-header-actions">
            {messages.length > 0 && (
              <button
                className="chat-clear-button"
                onClick={handleClearChat}
                aria-label="Clear chat"
                title="Clear all messages"
              >
                🗑️
              </button>
            )}
            <button
              className="chat-close-button"
              onClick={onClose}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>
        </div>

        {/* Messages */}
        <ChatMessages messages={messages} isLoading={isLoading} />

        {/* Input */}
        <ChatInput onSend={handleSend} disabled={isLoading} />
      </div>
    </div>
  );

  // Render as portal to avoid z-index issues
  return createPortal(panel, document.body);
}
