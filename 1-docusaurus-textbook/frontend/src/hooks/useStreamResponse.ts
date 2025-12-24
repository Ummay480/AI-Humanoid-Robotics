import { useCallback } from 'react';
import { streamChatResponse } from '../services/chatApi';

/**
 * Generate a context-aware mock response based on user message
 */
function getMockResponse(userMessage: string, currentPage?: string): string {
  const msg = userMessage.toLowerCase();

  // Context-aware responses based on keywords
  if (msg.includes('ros') || msg.includes('robot operating system')) {
    return "ROS 2 is the Robot Operating System, a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.\n\nKey features include:\n- **Nodes**: Independent processes that communicate via topics\n- **Topics**: Named buses for asynchronous message passing\n- **Services**: Synchronous request/reply communication\n- **Actions**: Long-running tasks with feedback\n\nWould you like to learn more about any specific aspect of ROS 2?";
  }

  if (msg.includes('physical ai') || msg.includes('embodied')) {
    return "Physical AI, also known as Embodied AI, refers to artificial intelligence systems that interact with the physical world through sensors and actuators. Unlike traditional digital AI that processes data in virtual environments, Physical AI must deal with:\n\n- Real-world physics (gravity, friction, momentum)\n- Sensor noise and uncertainty\n- Real-time decision-making constraints\n- Physical consequences of actions\n\nThis makes Physical AI significantly more challenging than digital AI, but also more impactful for real-world applications like humanoid robots, autonomous vehicles, and robotic manipulation.";
  }

  if (msg.includes('humanoid') || msg.includes('robot')) {
    return "Humanoid robots are designed with human-like form factors, typically including a torso, head, two arms, and two legs. This morphology offers unique advantages:\n\n- **Human Environment Compatibility**: Can navigate spaces designed for humans (stairs, doorways)\n- **Tool Use**: Can operate human tools without modification\n- **Social Interaction**: Human-like appearance facilitates natural communication\n- **Intuitive Telepresence**: Easier for humans to understand robot perspectives\n\nExamples include Atlas (Boston Dynamics), Optimus (Tesla), and Digit (Agility Robotics). Each has different design priorities based on their intended applications.";
  }

  if (msg.includes('sensor') || msg.includes('perception')) {
    return "Robot sensors can be categorized into two main types:\n\n**Proprioceptive Sensors** (internal state):\n- Encoders: Measure joint positions\n- IMU: Measures orientation and acceleration\n- Force/Torque sensors: Detect contact forces\n\n**Exteroceptive Sensors** (external environment):\n- Cameras: Visual perception (RGB, depth, stereo)\n- LIDAR: 3D point clouds for mapping\n- Tactile sensors: Detect touch and slip\n\nSensor fusion combines multiple sensor types to get robust state estimates despite individual sensor limitations.";
  }

  if (msg.includes('control') || msg.includes('kinematics')) {
    return "Robot control involves several key concepts:\n\n**Forward Kinematics**: Given joint angles, compute end-effector position in space. This is straightforward to calculate using transformation matrices.\n\n**Inverse Kinematics (IK)**: Given desired end-effector position, compute required joint angles. This can have no solution, one solution, or infinite solutions (redundancy).\n\n**Control Loops**: Typically structured in layers:\n- Low-level: Motor PID controllers (1kHz+)\n- Mid-level: Whole-body control, balance (100-500Hz)\n- High-level: Planning and AI decision-making (1-10Hz)\n\nWould you like to explore any of these topics in more depth?";
  }

  if (msg.includes('hello') || msg.includes('hi') || msg.includes('hey')) {
    return "Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook. I'm here to help you understand concepts related to robotics, ROS 2, sensors, control systems, and more.\n\nFeel free to ask me questions about any topic in the book, or ask for clarification on specific concepts. What would you like to learn about today?";
  }

  if (msg.includes('help') || msg.includes('what can you')) {
    return "I can help you with topics covered in this Physical AI & Humanoid Robotics textbook, including:\n\n- **ROS 2**: Nodes, topics, services, URDF\n- **Robotics Fundamentals**: Kinematics, dynamics, control\n- **Sensors**: Cameras, LIDAR, IMU, encoders\n- **Humanoid Robots**: Locomotion, balance, whole-body control\n- **Perception**: Computer vision, sensor fusion\n- **Physical AI**: Embodied intelligence, sim-to-real transfer\n\nJust ask a question, and I'll do my best to explain! You can also ask for examples, clarifications, or deeper explanations of any concept.";
  }

  // Default response for other questions
  return `That's a great question about ${currentPage || 'Physical AI'}! \n\nWhile I'm currently a placeholder implementation (simulating streaming responses), in the full version I would provide detailed answers based on the textbook content and your current chapter.\n\nFor now, I can help with general questions about:\n- ROS 2 and robotic software\n- Humanoid robot design\n- Sensors and perception\n- Kinematics and control\n- Physical AI concepts\n\nTry asking about any of these topics, and I'll provide a more detailed response!`;
}

/**
 * Custom hook for simulating streaming AI responses
 *
 * @param currentPage - Optional current page context for tailored responses
 * @returns Function to stream a response character-by-character
 *
 * @example
 * ```tsx
 * const streamResponse = useStreamResponse('/docs/chapter1');
 *
 * await streamResponse('What is ROS 2?', (chunk) => {
 *   console.log('Received chunk:', chunk);
 * });
 * ```
 */
export function useStreamResponse(currentPage?: string) {
  return useCallback(
    async (
      userMessage: string,
      onChunk: (chunk: string, fullText: string) => void,
      onComplete?: () => void,
      onError?: (error: Error) => void
    ) => {
      console.log('[useStreamResponse] Starting stream for message:', userMessage);
      console.log('[useStreamResponse] Current page:', currentPage || 'unknown');

      try {
        // Use real backend API
        await streamChatResponse(
          userMessage,
          currentPage,
          onChunk,
          onComplete,
          onError
        );

        console.log('[useStreamResponse] Stream complete');
      } catch (error) {
        console.error('[useStreamResponse] Error during streaming:', error);

        if (onError) {
          onError(error as Error);
        }
      }
    },
    [currentPage]
  );
}
