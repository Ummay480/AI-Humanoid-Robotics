import React, { useCallback, useRef } from 'react';

/**
 * ChatButton Props
 */
interface ChatButtonProps {
  onClick: () => void;
}

/**
 * ChatButton Component - Floating action button to open chat
 *
 * Features:
 * - Fixed position at bottom-right corner
 * - Hover animation (scale effect)
 * - Debounced click to prevent rapid toggling
 * - Accessible with ARIA labels
 * - Robot emoji icon
 *
 * @param props - Component props
 * @param props.onClick - Callback when button is clicked
 *
 * @example
 * ```tsx
 * <ChatButton onClick={() => setIsChatOpen(true)} />
 * ```
 */
export function ChatButton({ onClick }: ChatButtonProps) {
  const lastClickRef = useRef<number>(0);

  // Debounce clicks to prevent rapid toggling
  const handleClick = useCallback(() => {
    const now = Date.now();
    const timeSinceLastClick = now - lastClickRef.current;

    // Require at least 300ms between clicks
    if (timeSinceLastClick < 300) {
      console.log('[ChatButton] Click debounced');
      return;
    }

    lastClickRef.current = now;
    onClick();
  }, [onClick]);

  return (
    <button
      className="chat-button"
      onClick={handleClick}
      aria-label="Open AI chat assistant"
      title="Chat with AI Assistant"
    >
      <span role="img" aria-label="Robot">
        🤖
      </span>
    </button>
  );
}
