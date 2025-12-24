import React from 'react';
import { Message } from './ChatProvider';
import { formatTimestamp } from '../../utils/chatHelpers';

/**
 * ChatBubble Props
 */
interface ChatBubbleProps {
  message: Message;
}

/**
 * ChatBubble Component - Displays a single chat message
 *
 * Renders messages differently based on role (user vs assistant):
 * - User messages: Right-aligned, blue background
 * - Assistant messages: Left-aligned, gray background
 *
 * @param props - Component props
 * @param props.message - The message to display
 *
 * @example
 * ```tsx
 * <ChatBubble message={{
 *   id: '1',
 *   role: 'user',
 *   content: 'Hello!',
 *   timestamp: new Date()
 * }} />
 * ```
 */
export function ChatBubble({ message }: ChatBubbleProps) {
  const isUser = message.role === 'user';
  const bubbleClass = isUser ? 'chat-bubble chat-bubble-user' : 'chat-bubble chat-bubble-bot';

  return (
    <div className={bubbleClass}>
      <div className="chat-bubble-content">
        {/* Render message content - supports simple markdown formatting */}
        {message.content.split('\n').map((line, index) => (
          <React.Fragment key={index}>
            {line}
            {index < message.content.split('\n').length - 1 && <br />}
          </React.Fragment>
        ))}
      </div>
      <div className="chat-bubble-timestamp">
        {formatTimestamp(message.timestamp)}
      </div>
    </div>
  );
}
