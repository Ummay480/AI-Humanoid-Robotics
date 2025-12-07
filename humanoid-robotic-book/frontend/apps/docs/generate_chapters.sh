#!/bin/bash

# Script to generate all chapter and lesson files for the Humanoid Robotics Textbook
# Creates 10 chapters, each with 15 lessons

# Chapter definitions: chapter_number:chapter_name:chapter_description
declare -a CHAPTERS=(
  "1:Fundamentals:Introduction to robotics, hardware, and software basics"
  "2:Kinematics:Motion analysis, trajectory planning, and forward/inverse kinematics"
  "3:Dynamics:Physics-based simulation, forces, and dynamics modeling"
  "4:Control:Feedback control, PID, LQR, and AI-based control systems"
  "5:Vision:Computer vision, image processing, and perception"
  "6:Motion Planning:Path planning, RRT, trajectory optimization"
  "7:Manipulation:Grasping, manipulation, and dexterous control"
  "8:Locomotion:Walking, running, balance, and gaits"
  "9:HRI:Human-robot interaction and learning from humans"
  "10:AI:Machine learning, deep learning, and AI for robotics"
)

# Lesson titles for each chapter
declare -a CHAPTER_1_LESSONS=(
  "Introduction to Robotics"
  "Robot Anatomy and Components"
  "Actuators and Motors"
  "Sensors Overview"
  "Control Systems Basics"
  "Programming Paradigms"
  "Real-Time Systems"
  "Safety Considerations"
  "Hardware Interfaces"
  "Software Architectures"
  "Communication Protocols"
  "Power Systems"
  "Mechanical Design Basics"
  "Simulation Environments"
  "Getting Started with Projects"
)

declare -a CHAPTER_2_LESSONS=(
  "Coordinate Systems"
  "Forward Kinematics"
  "Inverse Kinematics"
  "Denavit-Hartenberg Convention"
  "Jacobian Matrix"
  "Singularities"
  "Workspace Analysis"
  "Trajectory Planning"
  "Motion Primitives"
  "Interpolation Methods"
  "Redundancy Resolution"
  "Kinematic Chains"
  "Parallel Mechanisms"
  "Gait Analysis"
  "Kinematic Examples"
)

declare -a CHAPTER_3_LESSONS=(
  "Newton's Laws"
  "Lagrangian Mechanics"
  "Hamiltonian Formulation"
  "Equation of Motion"
  "Inertia Matrices"
  "Centripetal Forces"
  "Friction Models"
  "Contact Dynamics"
  "Collision Response"
  "Impact Analysis"
  "Energy Conservation"
  "Stability Analysis"
  "Passivity Concepts"
  "Numerical Integration"
  "Dynamic Simulation"
)

declare -a CHAPTER_4_LESSONS=(
  "Feedback Control"
  "PID Controllers"
  "State Space Representation"
  "Linear Systems"
  "Nonlinear Control"
  "Stability (Lyapunov)"
  "Pole Placement"
  "LQR Control"
  "Adaptive Control"
  "Robust Control"
  "Model Predictive Control"
  "Reinforcement Learning Basics"
  "Neural Network Control"
  "Control System Design"
  "Real-World Implementation"
)

declare -a CHAPTER_5_LESSONS=(
  "Image Processing"
  "Feature Detection"
  "Object Recognition"
  "Semantic Segmentation"
  "Pose Estimation"
  "Depth Estimation"
  "Visual Odometry"
  "SLAM Basics"
  "Camera Calibration"
  "Stereo Vision"
  "3D Reconstruction"
  "Gesture Recognition"
  "Hand-Eye Coordination"
  "Deep Learning for Vision"
  "Vision Applications"
)

declare -a CHAPTER_6_LESSONS=(
  "Configuration Space"
  "Roadmap Methods"
  "Rapidly-Exploring Trees (RRT)"
  "Potential Field Methods"
  "Sampling-Based Methods"
  "Graph Search Algorithms"
  "Trajectory Optimization"
  "Collision Avoidance"
  "Dynamic Constraints"
  "Multi-Robot Planning"
  "Manipulation Planning"
  "Grasp Planning"
  "Whole-Body Planning"
  "Learning-Based Planning"
  "Planning Applications"
)

declare -a CHAPTER_7_LESSONS=(
  "Gripper Design"
  "Grasp Mechanics"
  "Force Closure"
  "Grasp Quality Metrics"
  "Dexterous Manipulation"
  "In-Hand Manipulation"
  "Object Tracking"
  "Compliance Control"
  "Impedance Control"
  "Contact State Estimation"
  "Manipulation Primitives"
  "Tool Use"
  "Assembly Tasks"
  "Learning Manipulation"
  "Manipulation Applications"
)

declare -a CHAPTER_8_LESSONS=(
  "Walking Mechanics"
  "Gait Types"
  "Balance Control"
  "Center of Mass Dynamics"
  "Zero Moment Point"
  "Step Planning"
  "Terrain Adaptation"
  "Climbing and Jumping"
  "Bipedal Gait Design"
  "Quadrupedal Locomotion"
  "Passive Walking"
  "Energy Efficiency"
  "Learning Gaits"
  "Sim-to-Real Transfer"
  "Locomotion Examples"
)

declare -a CHAPTER_9_LESSONS=(
  "Interaction Models"
  "Natural Language Processing"
  "Gesture Recognition"
  "Facial Expression Analysis"
  "Emotion Recognition"
  "Safety and Compliance"
  "Collaborative Manipulation"
  "Learning from Demonstration"
  "Imitation Learning"
  "Inverse Reinforcement Learning"
  "Social Navigation"
  "User Acceptance Factors"
  "Ethical Considerations"
  "Communication Strategies"
  "HRI Applications"
)

declare -a CHAPTER_10_LESSONS=(
  "Machine Learning Basics"
  "Supervised Learning"
  "Unsupervised Learning"
  "Reinforcement Learning"
  "Deep Neural Networks"
  "Convolutional Neural Networks"
  "Recurrent Neural Networks"
  "Attention Mechanisms"
  "Transformer Models"
  "Large Language Models"
  "Meta-Learning"
  "Transfer Learning"
  "Sim-to-Real Learning"
  "Safety in AI"
  "Future Directions"
)

# Create docs directory
DOCS_DIR="docs"
mkdir -p "$DOCS_DIR"

# Function to create chapter directory and files
create_chapter() {
  local chapter_num=$1
  local chapter_name=$2
  local chapter_desc=$3
  local lessons_array=$4

  # Create chapter directory with zero-padded number
  local chapter_dir="${DOCS_DIR}/chapter-${chapter_num}-$(echo "$chapter_name" | tr '[:upper:]' '[:lower:]')"
  mkdir -p "$chapter_dir"

  # Create _category_.json for Docusaurus
  cat > "${chapter_dir}/_category_.json" << EOF
{
  "label": "Chapter ${chapter_num}: ${chapter_name}",
  "position": ${chapter_num},
  "description": "${chapter_desc}",
  "link": {
    "type": "generated-index",
    "description": "Chapter ${chapter_num} - ${chapter_name}: ${chapter_desc}"
  }
}
EOF

  echo "Created chapter ${chapter_num}: ${chapter_name}"

  # Create 15 lessons for this chapter
  for lesson_num in {1..15}; do
    # Zero-pad lesson number
    local lesson_padded=$(printf "%02d" $lesson_num)

    # Get lesson title from appropriate array
    local lessons_var="CHAPTER_${chapter_num}_LESSONS[@]"
    local lesson_title="${!lessons_var:$(($lesson_num-1)):1}"

    # If no custom title, use generic one
    if [ -z "$lesson_title" ]; then
      lesson_title="Lesson ${lesson_padded}"
    fi

    # Convert lesson title to kebab-case for filename
    local lesson_slug=$(echo "$lesson_title" | tr '[:upper:]' '[:lower:]' | sed 's/[^a-z0-9]/-/g' | sed 's/-\+/-/g' | sed 's/-$//')

    # Create lesson markdown file
    local lesson_file="${chapter_dir}/lesson-${lesson_padded}-${lesson_slug}.md"

    cat > "$lesson_file" << EOF
---
id: lesson-${lesson_padded}-${lesson_slug}
title: "Lesson ${lesson_padded}: ${lesson_title}"
sidebar_position: ${lesson_num}
---

# Lesson ${lesson_padded}: ${lesson_title}

## Chapter Context

This lesson is part of **Chapter ${chapter_num}: ${chapter_name}**

${chapter_desc}

## Learning Objectives

After completing this lesson, you will be able to:
- Understand the core concepts of ${lesson_title}
- Apply these concepts to real-world robotics problems
- Implement solutions using provided code examples

## Introduction

${lesson_title} is a fundamental concept in robotics and humanoid systems. This lesson introduces the key ideas and provides practical examples.

### Motivation

Understanding ${lesson_title} is essential for:
- Building effective robotic systems
- Solving complex robotics problems
- Creating intelligent autonomous agents
- Advancing the field of robotics

## Key Concepts

### Concept 1: Core Theory

This section covers the fundamental theory behind ${lesson_title}.

\`\`\`python
# Example code
import numpy as np

# Placeholder for implementation
def example_function():
    pass
\`\`\`

### Concept 2: Implementation Details

Practical implementation considerations for ${lesson_title}.

\`\`\`cpp
// C++ Example
#include <iostream>

// Placeholder implementation
void exampleFunction() {
    // Implementation here
}
\`\`\`

### Concept 3: Applications

Real-world applications of ${lesson_title} in robotics.

## Worked Examples

### Example 1: Basic Application

A simple example demonstrating ${lesson_title}.

### Example 2: Advanced Scenario

A more complex example showing advanced usage.

## Labs and Exercises

### Lab 1: Implementation

**Objective**: Implement a basic version of ${lesson_title}

**Instructions**:
1. Start with the template code provided
2. Implement the required functionality
3. Test your implementation
4. Compare results with expected outputs

**Expected Output**: [Description of expected results]

### Lab 2: Analysis

**Objective**: Analyze and compare different approaches to ${lesson_title}

**Instructions**:
1. Implement multiple approaches
2. Measure performance characteristics
3. Create a comparison table
4. Document your findings

## Review Questions

1. What is the definition of ${lesson_title}?
2. How is ${lesson_title} applied in practice?
3. What are the limitations of ${lesson_title}?
4. How does ${lesson_title} relate to other concepts in robotics?

## Further Reading

- **Related Lesson**: [Link to related lesson]
- **Advanced Topic**: [Link to advanced topic]
- **Research Paper**: [Link to research paper]

## Code Repository

Full code examples and lab solutions are available at:
- [GitHub Repository](https://github.com/humanoid-robotics/examples)
- Lab folder: \`chapter-${chapter_num}/lesson-${lesson_padded}/\`

## FAQ

**Q: How does ${lesson_title} differ from related concepts?**

A: [Answer explaining differences]

**Q: What are common pitfalls in implementing ${lesson_title}?**

A: [Answer describing common mistakes]

**Q: How can I extend ${lesson_title} for advanced applications?**

A: [Answer describing extensions]

## Glossary

- **Term 1**: Definition
- **Term 2**: Definition
- **Term 3**: Definition

## Next Steps

After completing this lesson:
1. Try the interactive lab exercises
2. Ask the ChatKit chatbot for clarification
3. Move to Lesson $((lesson_num + 1)): [Next Lesson Title]
4. Check out related chapters for deeper understanding

---

**Last Updated**: 2025-12-06 | **Status**: Draft | **Feedback**: [Send feedback](https://github.com/humanoid-robotics/textbook/issues)
EOF

    echo "  Created lesson ${lesson_num}: ${lesson_title}"
  done
}

# Create all chapters
create_chapter 1 "Fundamentals" "Introduction to robotics, hardware, and software basics" "CHAPTER_1_LESSONS"
create_chapter 2 "Kinematics" "Motion analysis, trajectory planning, and forward/inverse kinematics" "CHAPTER_2_LESSONS"
create_chapter 3 "Dynamics" "Physics-based simulation, forces, and dynamics modeling" "CHAPTER_3_LESSONS"
create_chapter 4 "Control" "Feedback control, PID, LQR, and AI-based control systems" "CHAPTER_4_LESSONS"
create_chapter 5 "Vision" "Computer vision, image processing, and perception" "CHAPTER_5_LESSONS"
create_chapter 6 "Motion Planning" "Path planning, RRT, trajectory optimization" "CHAPTER_6_LESSONS"
create_chapter 7 "Manipulation" "Grasping, manipulation, and dexterous control" "CHAPTER_7_LESSONS"
create_chapter 8 "Locomotion" "Walking, running, balance, and gaits" "CHAPTER_8_LESSONS"
create_chapter 9 "HRI" "Human-robot interaction and learning from humans" "CHAPTER_9_LESSONS"
create_chapter 10 "AI" "Machine learning, deep learning, and AI for robotics" "CHAPTER_10_LESSONS"

echo ""
echo "✅ Chapter generation complete!"
echo "Created 10 chapters with 15 lessons each = 150 total lessons"
echo ""
echo "Directory structure:"
echo "docs/"
echo "├── intro.md"
echo "└── chapter-*/"
echo "    ├── _category_.json"
echo "    └── lesson-*.md"
