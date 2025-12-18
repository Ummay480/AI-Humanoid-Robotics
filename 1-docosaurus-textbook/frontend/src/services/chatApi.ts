/**
 * Chat API client for backend communication
 */

// Get API URL from environment or use default
const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

/**
 * Stream chat response from backend using Server-Sent Events
 *
 * @param message - User message
 * @param currentPage - Current page for context-aware responses
 * @param onChunk - Callback for each text chunk
 * @param onComplete - Callback when streaming is complete
 * @param onError - Callback for errors
 */
export async function streamChatResponse(
  message: string,
  currentPage: string | undefined,
  onChunk: (chunk: string, fullText: string) => void,
  onComplete?: (sources?: any[]) => void,
  onError?: (error: Error) => void
): Promise<void> {
  try {
    const response = await fetch(`${API_URL}/api/chat/stream`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        message,
        page: currentPage || null,
        session_id: null, // Can implement session management later
      }),
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    if (!response.body) {
      throw new Error('Response body is null');
    }

    // Read stream using ReadableStream API
    const reader = response.body
      .pipeThrough(new TextDecoderStream())
      .getReader();

    let fullText = '';
    let sources: any[] | undefined;

    while (true) {
      const { done, value } = await reader.read();

      if (done) {
        break;
      }

      // Parse SSE data
      const lines = value.split('\n');

      for (const line of lines) {
        if (line.startsWith('data: ')) {
          try {
            const data = JSON.parse(line.slice(6)); // Remove "data: " prefix

            if (data.chunk) {
              // Text chunk received
              fullText += data.chunk;
              onChunk(data.chunk, fullText);
            }

            if (data.done) {
              // Streaming complete
              if (data.sources) {
                sources = data.sources;
              }
            }
          } catch (parseError) {
            console.error('Error parsing SSE data:', parseError);
          }
        }
      }
    }

    // Call completion callback
    if (onComplete) {
      onComplete(sources);
    }
  } catch (error) {
    console.error('Error in streamChatResponse:', error);
    if (onError) {
      onError(error as Error);
    }
  }
}

/**
 * Send chat message (non-streaming) - for testing
 *
 * @param message - User message
 * @param currentPage - Current page for context
 * @returns Promise with response text and sources
 */
export async function sendChatMessage(
  message: string,
  currentPage?: string
): Promise<{ response: string; sources: any[] }> {
  try {
    const response = await fetch(`${API_URL}/api/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        message,
        page: currentPage || null,
        session_id: null,
      }),
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    const data = await response.json();
    return {
      response: data.response,
      sources: data.sources || [],
    };
  } catch (error) {
    console.error('Error in sendChatMessage:', error);
    throw error;
  }
}

/**
 * Check chat service health
 *
 * @returns Promise with health status
 */
export async function checkChatHealth(): Promise<any> {
  try {
    const response = await fetch(`${API_URL}/api/chat/health`);

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error checking chat health:', error);
    throw error;
  }
}
