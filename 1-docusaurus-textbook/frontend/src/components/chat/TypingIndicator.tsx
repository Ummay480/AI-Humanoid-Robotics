import React from 'react';

/**
 * TypingIndicator Component - Shows animated dots while AI is responding
 *
 * Displays a simple three-dot animation to indicate that the assistant
 * is "typing" or generating a response.
 *
 * @example
 * ```tsx
 * {isLoading && <TypingIndicator />}
 * ```
 */
export function TypingIndicator() {
  return (
    <div className="typing-indicator">
      <div className="typing-indicator-dot" />
      <div className="typing-indicator-dot" />
      <div className="typing-indicator-dot" />
    </div>
  );
}
