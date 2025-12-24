# Physical AI & Humanoid Robotics - Interactive Textbook

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

## Features

### 🤖 AI Chat Assistant

This textbook includes an **interactive AI chat assistant** that helps you understand concepts as you read!

- **Floating Chat Button**: Click the robot icon (🤖) in the bottom-right corner on any page
- **Context-Aware Responses**: The AI provides explanations tailored to Physical AI and Humanoid Robotics
- **Streaming Responses**: Watch answers appear character-by-character like a real conversation
- **Persistent Chat**: Your conversation stays active as you navigate between chapters
- **Dark Mode Support**: Works seamlessly in both light and dark themes
- **Mobile Responsive**: Optimized for smartphones and tablets

**Note**: Currently uses simulated responses for demonstration. In production, this would connect to a real LLM API (OpenAI, Anthropic, etc.).

#### Chat Topics Covered

Ask questions about:
- ROS 2 (Nodes, Topics, Services, URDF)
- Humanoid Robot Design
- Sensors and Perception (Cameras, LIDAR, IMU)
- Kinematics and Control
- Physical AI Concepts
- And more!

See `src/components/chat/README.md` for technical documentation.

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
