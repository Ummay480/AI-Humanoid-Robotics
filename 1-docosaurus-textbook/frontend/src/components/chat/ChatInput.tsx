import React, { useState, KeyboardEvent, ChangeEvent } from 'react';

/**
 * ChatInput Props
 */
interface ChatInputProps {
  onSend: (text: string) => void;
  disabled?: boolean;
}

/**
 * ChatInput Component - Message input field with send button
 *
 * Features:
 * - Multi-line textarea support
 * - Enter to send, Shift+Enter for new line
 * - Character counter (placeholder)
 * - Disabled state during loading
 * - Auto-clears after sending
 * - Validates non-empty input
 *
 * @param props - Component props
 * @param props.onSend - Callback function when message is sent
 * @param props.disabled - Whether input is disabled (during loading)
 *
 * @example
 * ```tsx
 * <ChatInput
 *   onSend={(text) => handleSendMessage(text)}
 *   disabled={isLoading}
 * />
 * ```
 */
export function ChatInput({ onSend, disabled = false }: ChatInputProps) {
  const [inputValue, setInputValue] = useState('');

  const handleInputChange = (e: ChangeEvent<HTMLTextAreaElement>) => {
    setInputValue(e.target.value);
  };

  const handleSend = () => {
    const trimmed = inputValue.trim();

    if (trimmed.length === 0) {
      return; // Don't send empty messages
    }

    onSend(trimmed);
    setInputValue(''); // Clear input after sending
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    // Send on Enter (without Shift)
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault(); // Prevent new line
      handleSend();
    }
    // Allow Shift+Enter for new line (default behavior)
  };

  return (
    <div className="chat-input">
      <textarea
        className="chat-input-field"
        value={inputValue}
        onChange={handleInputChange}
        onKeyDown={handleKeyDown}
        placeholder="Ask a question about Physical AI..."
        disabled={disabled}
        rows={1}
        aria-label="Message input"
      />
      <button
        className="chat-input-send"
        onClick={handleSend}
        disabled={disabled || inputValue.trim().length === 0}
        aria-label="Send message"
      >
        Send
      </button>
    </div>
  );
}
