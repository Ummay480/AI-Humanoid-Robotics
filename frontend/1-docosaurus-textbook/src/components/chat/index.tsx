import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { useChat } from '../../hooks/useChat';
import { ChatButton } from './ChatButton';
import { ChatPanel } from './ChatPanel';
import { ErrorBoundary } from './ErrorBoundary';

/**
 * ChatWidget Component - Main entry point for chat functionality
 *
 * This component wraps the chat button and panel in BrowserOnly to prevent
 * SSR (Server-Side Rendering) issues since chat is client-side only.
 *
 * Features:
 * - Client-side only rendering (no SSR)
 * - Integrates ChatButton and ChatPanel
 * - Uses ChatContext for state management
 * - Automatically handles open/close state
 * - Error boundary for graceful error handling
 *
 * @example
 * ```tsx
 * // In your layout:
 * <ChatProvider>
 *   <ChatWidget />
 * </ChatProvider>
 * ```
 */
function ChatWidgetInternal() {
  const { isOpen, setIsOpen } = useChat();

  return (
    <>
      <ChatButton onClick={() => setIsOpen(true)} />
      <ChatPanel isOpen={isOpen} onClose={() => setIsOpen(false)} />
    </>
  );
}

/**
 * ChatWidget - Exported with BrowserOnly and ErrorBoundary wrappers
 *
 * This is the component you should import and use in your layout.
 * It ensures the chat only renders on the client side, preventing
 * hydration errors, and catches any runtime errors gracefully.
 */
export default function ChatWidget() {
  return (
    <BrowserOnly fallback={<div />}>
      {() => (
        <ErrorBoundary>
          <ChatWidgetInternal />
        </ErrorBoundary>
      )}
    </BrowserOnly>
  );
}
