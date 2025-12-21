import React, { useRef, useEffect, useState } from 'react';
import { Message } from './ChatProvider';
import { ChatBubble } from './ChatBubble';
import { TypingIndicator } from './TypingIndicator';

/**
 * ChatMessages Props
 */
interface ChatMessagesProps {
  messages: Message[];
  isLoading: boolean;
}

/**
 * ChatMessages Component - Displays scrollable message history
 *
 * Features:
 * - Auto-scrolls to bottom when new messages arrive
 * - Detects if user has scrolled up manually
 * - Only auto-scrolls if user is near the bottom
 * - Shows empty state when no messages
 * - Displays typing indicator when loading
 *
 * @param props - Component props
 * @param props.messages - Array of messages to display
 * @param props.isLoading - Whether AI is currently responding
 *
 * @example
 * ```tsx
 * <ChatMessages
 *   messages={chatMessages}
 *   isLoading={isAIThinking}
 * />
 * ```
 */
export function ChatMessages({ messages, isLoading }: ChatMessagesProps) {
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [isAtBottom, setIsAtBottom] = useState(true);

  // Auto-scroll to bottom when messages change, but only if user is near bottom
  useEffect(() => {
    if (isAtBottom && messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages, isLoading, isAtBottom]);

  // Detect if user is scrolled near the bottom
  const handleScroll = () => {
    if (!containerRef.current) return;

    const { scrollTop, scrollHeight, clientHeight } = containerRef.current;
    const distanceFromBottom = scrollHeight - scrollTop - clientHeight;

    // Consider "at bottom" if within 100px of the bottom
    setIsAtBottom(distanceFromBottom < 100);
  };

  // Empty state when no messages
  if (messages.length === 0 && !isLoading) {
    return (
      <div className="chat-messages">
        <div className="chat-messages-empty">
          <div className="chat-messages-empty-icon">🤖</div>
          <div className="chat-messages-empty-title">
            Ask me anything about Physical AI!
          </div>
          <div className="chat-messages-empty-text">
            I can help you understand concepts from the textbook including ROS 2,
            humanoid robots, sensors, kinematics, and more.
          </div>
        </div>
      </div>
    );
  }

  return (
    <div
      ref={containerRef}
      className="chat-messages"
      onScroll={handleScroll}
    >
      {messages.map((message) => (
        <ChatBubble key={message.id} message={message} />
      ))}

      {isLoading && <TypingIndicator />}

      {/* Invisible anchor for auto-scroll */}
      <div ref={messagesEndRef} />
    </div>
  );
}
