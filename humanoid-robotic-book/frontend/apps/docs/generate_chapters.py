#!/usr/bin/env python3
"""
Generate all chapter and lesson markdown files for the Humanoid Robotics Textbook.
Creates 10 chapters with 15 lessons each = 150 total lessons.
"""

import os
import json
import re
from pathlib import Path

# Chapter configurations
CHAPTERS = [
    {
        "number": 1,
        "name": "Fundamentals",
        "description": "Introduction to robotics, hardware, and software basics",
        "lessons": [
            "Introduction to Robotics",
            "Robot Anatomy and Components",
            "Actuators and Motors",
            "Sensors Overview",
            "Control Systems Basics",
            "Programming Paradigms",
            "Real-Time Systems",
            "Safety Considerations",
            "Hardware Interfaces",
            "Software Architectures",
            "Communication Protocols",
            "Power Systems",
            "Mechanical Design Basics",
            "Simulation Environments",
            "Getting Started with Projects",
        ]
    },
    {
        "number": 2,
        "name": "Kinematics",
        "description": "Motion analysis, trajectory planning, and forward/inverse kinematics",
        "lessons": [
            "Coordinate Systems",
            "Forward Kinematics",
            "Inverse Kinematics",
            "Denavit-Hartenberg Convention",
            "Jacobian Matrix",
            "Singularities",
            "Workspace Analysis",
            "Trajectory Planning",
            "Motion Primitives",
            "Interpolation Methods",
            "Redundancy Resolution",
            "Kinematic Chains",
            "Parallel Mechanisms",
            "Gait Analysis",
            "Kinematic Examples",
        ]
    },
    {
        "number": 3,
        "name": "Dynamics",
        "description": "Physics-based simulation, forces, and dynamics modeling",
        "lessons": [
            "Newton's Laws",
            "Lagrangian Mechanics",
            "Hamiltonian Formulation",
            "Equation of Motion",
            "Inertia Matrices",
            "Centripetal Forces",
            "Friction Models",
            "Contact Dynamics",
            "Collision Response",
            "Impact Analysis",
            "Energy Conservation",
            "Stability Analysis",
            "Passivity Concepts",
            "Numerical Integration",
            "Dynamic Simulation",
        ]
    },
    {
        "number": 4,
        "name": "Control",
        "description": "Feedback control, PID, LQR, and AI-based control systems",
        "lessons": [
            "Feedback Control",
            "PID Controllers",
            "State Space Representation",
            "Linear Systems",
            "Nonlinear Control",
            "Stability (Lyapunov)",
            "Pole Placement",
            "LQR Control",
            "Adaptive Control",
            "Robust Control",
            "Model Predictive Control",
            "Reinforcement Learning Basics",
            "Neural Network Control",
            "Control System Design",
            "Real-World Implementation",
        ]
    },
    {
        "number": 5,
        "name": "Vision",
        "description": "Computer vision, image processing, and perception",
        "lessons": [
            "Image Processing",
            "Feature Detection",
            "Object Recognition",
            "Semantic Segmentation",
            "Pose Estimation",
            "Depth Estimation",
            "Visual Odometry",
            "SLAM Basics",
            "Camera Calibration",
            "Stereo Vision",
            "3D Reconstruction",
            "Gesture Recognition",
            "Hand-Eye Coordination",
            "Deep Learning for Vision",
            "Vision Applications",
        ]
    },
    {
        "number": 6,
        "name": "Motion Planning",
        "description": "Path planning, RRT, trajectory optimization",
        "lessons": [
            "Configuration Space",
            "Roadmap Methods",
            "Rapidly-Exploring Trees (RRT)",
            "Potential Field Methods",
            "Sampling-Based Methods",
            "Graph Search Algorithms",
            "Trajectory Optimization",
            "Collision Avoidance",
            "Dynamic Constraints",
            "Multi-Robot Planning",
            "Manipulation Planning",
            "Grasp Planning",
            "Whole-Body Planning",
            "Learning-Based Planning",
            "Planning Applications",
        ]
    },
    {
        "number": 7,
        "name": "Manipulation",
        "description": "Grasping, manipulation, and dexterous control",
        "lessons": [
            "Gripper Design",
            "Grasp Mechanics",
            "Force Closure",
            "Grasp Quality Metrics",
            "Dexterous Manipulation",
            "In-Hand Manipulation",
            "Object Tracking",
            "Compliance Control",
            "Impedance Control",
            "Contact State Estimation",
            "Manipulation Primitives",
            "Tool Use",
            "Assembly Tasks",
            "Learning Manipulation",
            "Manipulation Applications",
        ]
    },
    {
        "number": 8,
        "name": "Locomotion",
        "description": "Walking, running, balance, and gaits",
        "lessons": [
            "Walking Mechanics",
            "Gait Types",
            "Balance Control",
            "Center of Mass Dynamics",
            "Zero Moment Point",
            "Step Planning",
            "Terrain Adaptation",
            "Climbing and Jumping",
            "Bipedal Gait Design",
            "Quadrupedal Locomotion",
            "Passive Walking",
            "Energy Efficiency",
            "Learning Gaits",
            "Sim-to-Real Transfer",
            "Locomotion Examples",
        ]
    },
    {
        "number": 9,
        "name": "HRI",
        "description": "Human-robot interaction and learning from humans",
        "lessons": [
            "Interaction Models",
            "Natural Language Processing",
            "Gesture Recognition",
            "Facial Expression Analysis",
            "Emotion Recognition",
            "Safety and Compliance",
            "Collaborative Manipulation",
            "Learning from Demonstration",
            "Imitation Learning",
            "Inverse Reinforcement Learning",
            "Social Navigation",
            "User Acceptance Factors",
            "Ethical Considerations",
            "Communication Strategies",
            "HRI Applications",
        ]
    },
    {
        "number": 10,
        "name": "AI",
        "description": "Machine learning, deep learning, and AI for robotics",
        "lessons": [
            "Machine Learning Basics",
            "Supervised Learning",
            "Unsupervised Learning",
            "Reinforcement Learning",
            "Deep Neural Networks",
            "Convolutional Neural Networks",
            "Recurrent Neural Networks",
            "Attention Mechanisms",
            "Transformer Models",
            "Large Language Models",
            "Meta-Learning",
            "Transfer Learning",
            "Sim-to-Real Learning",
            "Safety in AI",
            "Future Directions",
        ]
    },
]

def slugify(text):
    """Convert text to URL-safe slug."""
    slug = re.sub(r'[^\w\s-]', '', text.lower())
    slug = re.sub(r'[-\s]+', '-', slug)
    return slug.strip('-')

def create_category_json(chapter_number, chapter_name, chapter_description):
    """Create _category_.json for a chapter."""
    return {
        "label": f"Chapter {chapter_number}: {chapter_name}",
        "position": chapter_number,
        "description": chapter_description,
        "link": {
            "type": "generated-index",
            "description": f"Chapter {chapter_number} - {chapter_name}: {chapter_description}"
        }
    }

def create_lesson_markdown(chapter_number, chapter_name, lesson_number, lesson_title):
    """Create markdown content for a lesson."""
    lesson_slug = slugify(lesson_title)
    lesson_padded = f"{lesson_number:02d}"

    return f"""---
id: lesson-{lesson_padded}-{lesson_slug}
title: "Lesson {lesson_padded}: {lesson_title}"
sidebar_position: {lesson_number}
---

# Lesson {lesson_padded}: {lesson_title}

## Chapter Context

This lesson is part of **Chapter {chapter_number}: {chapter_name}**

## Learning Objectives

After completing this lesson, you will be able to:
- Understand the core concepts of {lesson_title}
- Apply these concepts to real-world robotics problems
- Implement solutions using provided code examples

## Introduction

{lesson_title} is a fundamental concept in robotics and humanoid systems. This lesson introduces the key ideas and provides practical examples.

## Key Concepts

### Concept 1: Core Theory

This section covers the fundamental theory behind {lesson_title}.

```python
# Example code
import numpy as np

# Placeholder for implementation
def example_function():
    \"\"\"Example function for {lesson_title}.\"\"\"
    pass
```

### Concept 2: Implementation Details

Practical implementation considerations for {lesson_title}.

```cpp
// C++ Example
#include <iostream>

// Placeholder implementation
void exampleFunction() {{
    // Implementation here
}}
```

## Labs and Exercises

### Lab 1: Implementation

**Objective**: Implement a basic version of {lesson_title}

**Instructions**:
1. Start with the template code provided
2. Implement the required functionality
3. Test your implementation
4. Compare results with expected outputs

## Review Questions

1. What is the definition of {lesson_title}?
2. How is {lesson_title} applied in practice?
3. What are the limitations of {lesson_title}?

## Next Steps

After completing this lesson:
1. Try the interactive lab exercises
2. Ask the ChatKit chatbot for clarification
3. Move to the next lesson
4. Check out related chapters

---

**Last Updated**: 2025-12-06 | **Status**: Draft
"""

def main():
    """Generate all chapter and lesson files."""
    docs_dir = Path("docs")

    # Create docs directory if it doesn't exist
    docs_dir.mkdir(exist_ok=True)

    total_chapters = 0
    total_lessons = 0

    for chapter in CHAPTERS:
        chapter_num = chapter["number"]
        chapter_name = chapter["name"]
        chapter_desc = chapter["description"]
        lessons = chapter["lessons"]

        # Create chapter directory
        chapter_slug = slugify(chapter_name)
        chapter_dir = docs_dir / f"chapter-{chapter_num}-{chapter_slug}"
        chapter_dir.mkdir(exist_ok=True)

        # Create _category_.json
        category_data = create_category_json(chapter_num, chapter_name, chapter_desc)
        category_file = chapter_dir / "_category_.json"
        with open(category_file, 'w') as f:
            json.dump(category_data, f, indent=2)

        print(f"[OK] Created Chapter {chapter_num}: {chapter_name}")
        total_chapters += 1

        # Create lesson files
        for lesson_num, lesson_title in enumerate(lessons, 1):
            lesson_slug = slugify(lesson_title)
            lesson_content = create_lesson_markdown(
                chapter_num, chapter_name, lesson_num, lesson_title
            )

            lesson_file = chapter_dir / f"lesson-{lesson_num:02d}-{lesson_slug}.md"
            with open(lesson_file, 'w') as f:
                f.write(lesson_content)

            total_lessons += 1
            print(f"  Lesson {lesson_num:02d}: {lesson_title}")

    print(f"\n[OK] Generated {total_chapters} chapters with {total_lessons} lessons total!")
    print(f"\nDirectory structure created in: {docs_dir}/")

if __name__ == "__main__":
    main()
