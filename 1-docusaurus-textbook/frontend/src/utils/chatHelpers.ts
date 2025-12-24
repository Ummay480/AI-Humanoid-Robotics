/**
 * Chat Helper Functions
 * Placeholder implementations for AI chat functionality
 */

/**
 * Validate and send a user message
 * Placeholder function that logs to console
 *
 * @param text - The message text to send
 * @returns true if message is valid and sent, false otherwise
 *
 * @example
 * ```ts
 * if (sendMessage('Hello AI!')) {
 *   console.log('Message sent successfully');
 * }
 * ```
 */
export function sendMessage(text: string): boolean {
  // Trim whitespace
  const trimmedText = text.trim();

  // Validate: must have at least 1 character
  if (trimmedText.length === 0) {
    console.warn('[sendMessage] Empty message, not sending');
    return false;
  }

  // Validate: check for reasonable length (max 5000 characters)
  if (trimmedText.length > 5000) {
    console.warn('[sendMessage] Message too long (>5000 chars), truncating');
    // In production, you might want to handle this differently
  }

  // Log to console (placeholder for actual API call)
  console.log('[sendMessage] Sending message:', trimmedText);
  console.log('[sendMessage] Message length:', trimmedText.length);
  console.log('[sendMessage] Timestamp:', new Date().toISOString());

  // Placeholder: In production, this would make an API call
  // Example:
  // try {
  //   await fetch('/api/chat', {
  //     method: 'POST',
  //     headers: { 'Content-Type': 'application/json' },
  //     body: JSON.stringify({ message: trimmedText })
  //   });
  //   return true;
  // } catch (error) {
  //   console.error('Failed to send message:', error);
  //   return false;
  // }

  return true;
}

/**
 * Async generator that yields streaming response chunks
 * Placeholder implementation that simulates streaming with delays
 *
 * @param message - The user's message to respond to
 * @param delayMs - Delay between chunks in milliseconds (default: 50ms)
 * @yields Individual chunks of the response text
 *
 * @example
 * ```ts
 * for await (const chunk of streamResponse('What is ROS?')) {
 *   console.log('Chunk:', chunk);
 * }
 * ```
 */
export async function* streamResponse(
  message: string,
  delayMs: number = 50
): AsyncGenerator<string, void, unknown> {
  console.log('[streamResponse] Starting stream for:', message);
  console.log('[streamResponse] Delay per chunk:', delayMs, 'ms');

  // Placeholder: Generate a mock response
  const mockResponse = `Thank you for your question: "${message}". This is a placeholder streaming response that simulates how a real AI would respond character by character. In production, this would connect to an actual LLM API.`;

  console.log('[streamResponse] Total response length:', mockResponse.length);

  // Yield chunks character by character
  for (let i = 0; i < mockResponse.length; i++) {
    await new Promise((resolve) => setTimeout(resolve, delayMs));
    yield mockResponse[i];
  }

  console.log('[streamResponse] Stream complete');
}

/**
 * Format a timestamp for display in chat bubbles
 *
 * @param date - The date to format
 * @returns Formatted time string (HH:MM)
 *
 * @example
 * ```ts
 * formatTimestamp(new Date()); // "14:23"
 * ```
 */
export function formatTimestamp(date: Date): string {
  const hours = date.getHours().toString().padStart(2, '0');
  const minutes = date.getMinutes().toString().padStart(2, '0');
  return `${hours}:${minutes}`;
}

/**
 * Sanitize user input to prevent XSS attacks
 * Basic implementation - in production, use a proper sanitization library
 *
 * @param text - The text to sanitize
 * @returns Sanitized text
 */
export function sanitizeInput(text: string): string {
  return text
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#x27;')
    .replace(/\//g, '&#x2F;');
}

/**
 * Truncate text to a maximum length with ellipsis
 *
 * @param text - The text to truncate
 * @param maxLength - Maximum length before truncation
 * @returns Truncated text with ellipsis if needed
 */
export function truncateText(text: string, maxLength: number): string {
  if (text.length <= maxLength) {
    return text;
  }
  return text.substring(0, maxLength - 3) + '...';
}

/**
 * Extract keywords from user message for context-aware responses
 *
 * @param message - The user's message
 * @returns Array of detected keywords
 */
export function extractKeywords(message: string): string[] {
  const keywords: string[] = [];
  const lowerMsg = message.toLowerCase();

  const keywordMap = {
    ros: ['ros', 'robot operating system', 'ros2'],
    sensors: ['sensor', 'camera', 'lidar', 'imu'],
    control: ['control', 'pid', 'feedback'],
    kinematics: ['kinematics', 'forward', 'inverse', 'ik'],
    humanoid: ['humanoid', 'bipedal', 'walking', 'balance'],
  };

  for (const [category, terms] of Object.entries(keywordMap)) {
    if (terms.some((term) => lowerMsg.includes(term))) {
      keywords.push(category);
    }
  }

  return keywords;
}
