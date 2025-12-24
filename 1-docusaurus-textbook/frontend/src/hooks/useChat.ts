import { useContext } from 'react';
import { ChatContext, ChatContextType } from '../components/chat/ChatProvider';

/**
 * Custom hook to access the ChatContext
 *
 * @throws Error if used outside of ChatProvider
 * @returns ChatContextType with all chat state and methods
 *
 * @example
 * ```tsx
 * function MyComponent() {
 *   const { messages, addMessage, isOpen, setIsOpen } = useChat();
 *
 *   const handleSend = (text: string) => {
 *     addMessage({ role: 'user', content: text });
 *   };
 *
 *   return <div>...</div>;
 * }
 * ```
 */
export function useChat(): ChatContextType {
  const context = useContext(ChatContext);

  if (!context) {
    throw new Error(
      'useChat must be used within a ChatProvider. ' +
      'Wrap your component tree with <ChatProvider>...</ChatProvider>'
    );
  }

  return context;
}
