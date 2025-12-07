---
id: 1
title: Physical AI & Humanoid Robotics Constitution
stage: constitution
date: 2025-12-07
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: 002-book-index
user: aidd
command: /sp.constitution
labels: ["constitution", "accuracy", "reproducibility", "robotics", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - Manual verification of all placeholders replaced
  - Version format compliance check
---

## Prompt

Project: Physical AI & Humanoid Robotics — AI Systems in the Physical World

Core Principles:

Accuracy: All technical content, robotics simulations, AI algorithms, and agent behaviors must be verified against primary sources, official documentation (ROS 2, Gazebo, NVIDIA Isaac), and peer-reviewed research.

Clarity: Writing must be clear and accessible to students with computer science backgrounds, including proper explanations of AI, robotics, and simulation concepts.

Reproducibility: Every code snippet, simulation setup, and AI/robotic example must be traceable and executable as described.

Rigor: Prefer peer-reviewed papers, official SDK docs, or authoritative tutorials. Avoid unverified internet sources unless necessary.

Key Standards:

Source Verification: Minimum 50% of sources must be peer-reviewed; others must be official documentation or authoritative tutorials.

Citation Format: APA style for all references.

Plagiarism: 0% tolerance; all content must be original or properly cited.

Writing Clarity: Flesch-Kincaid grade 10–12 readability.

Code Accuracy: All code, YAML, URDF, ROS 2 nodes, Unity/Gazebo setups must work as described.

Bot Accuracy: RAG chatbot must answer questions correctly, based only on the book content selected by the user.

Constraints:

Book Format: Written in Docusaurus and deployable on GitHub Pages.

Word Count: Each chapter 2,000–5,000 words; total book 25,000–35,000 words.

Source Count: Minimum 50 sources for entire book.

Functionality: Chapters must support personalization (based on signup info), language translation (Urdu), and interactive RAG chatbot integration.

PDF Export: Optional chapter PDF download with embedded citations.

Success Criteria:

Content Verification: All factual and technical claims are verified.

Code Reproducibility: All examples, agents, and simulations run as described.

Interactive Features:

RAG chatbot answers chapter-specific questions accurately.

User personalization button updates content dynamically.

Urdu translation button func

## Response snapshot

Created constitution v1.0.0 for Physical AI & Humanoid Robotics project with 6 core principles:
- Accuracy (verification against primary sources)
- Clarity (CS student accessibility, grade 10-12 readability)
- Reproducibility (executable code/simulations)
- Rigor (peer-reviewed sources minimum 50%)
- Source Verification & Citation (APA, zero plagiarism)
- Functional Accuracy (RAG chatbot, working code/simulations)

Added three constraint sections (Content Standards, Development & Quality Workflow, Interactive Features)
defining book format (Docusaurus, 25k-35k words, 50+ sources), code requirements, and feature functionality.
Governance section specifies amendment procedures with version semver and dependent template updates.

## Outcome

- ✅ Impact: Project-level governance established; all 6 principles converted from template placeholders to specific, measurable requirements tailored to robotics textbook + AI systems context
- 🧪 Tests: Manual verification passed—no bracket tokens remaining; version format (1.0.0) correct; dates ISO format (2025-12-07); governance section complete
- 📁 Files: .specify/memory/constitution.md updated; history/prompts/constitution/ created
- 🔁 Next prompts: Verify dependent templates (spec-template, plan-template, tasks-template) align with constitution; create feature spec for book index
- 🧠 Reflection: Constitution articulates high standards for technical accuracy and reproducibility—critical for robotics/AI textbook context where executable examples and verified claims are non-negotiable

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results: N/A (governance document; manual verification sufficient)
- Prompt variant: initial ratification
- Next experiment: Apply constitution to feature specifications; verify spec-template constraints match (peer-reviewed sources, code accuracy, etc.)
