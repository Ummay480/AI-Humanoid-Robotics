# Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Accuracy
All technical content, robotics simulations, AI algorithms, and agent behaviors MUST be verified against
primary sources, official documentation (ROS 2, Gazebo, NVIDIA Isaac), and peer-reviewed research.
No unverified claims permitted; every statement must trace to authoritative references.

### II. Clarity
Writing must be clear and accessible to students with computer science backgrounds, including proper
explanations of AI, robotics, and simulation concepts. Flesch-Kincaid grade 10–12 readability required.
All acronyms explained on first use; analogies to familiar concepts when introducing novel topics.

### III. Reproducibility
Every code snippet, simulation setup, and AI/robotic example MUST be traceable and executable as
described. Code tested in target environments before publication. Configuration files and setup steps
fully specified with version constraints. Instructions include expected outputs for validation.

### IV. Rigor
Prefer peer-reviewed papers, official SDK documentation, or authoritative tutorials. Unverified internet
sources strictly avoided unless explicitly justified. All claims about algorithms, frameworks, or
robotics capabilities must cite supporting evidence. Minimum 50% of all sources must be peer-reviewed.

### V. Source Verification & Citation
All references in APA style. Minimum 50 sources for entire book. All borrowed content properly cited;
zero tolerance for plagiarism. When paraphrasing external work, attribution must be explicit.
Every figure, diagram, and code example includes source attribution or authorship statement.

### VI. Functional Accuracy
RAG chatbot must answer questions correctly based only on book content selected by the user. Code
samples MUST compile and run as written. Simulations MUST execute without modification. API calls and
library imports MUST use correct, tested versions. Breaking API changes must be documented with migration paths.

## Content Standards

### Book Format & Structure
- Written in Docusaurus; deployable on GitHub Pages
- Each chapter: 2,000–5,000 words; total book: 25,000–35,000 words
- Chapters support personalization (user signup context), Urdu translation, and interactive RAG chatbot
- Optional PDF export with embedded citations

### Code & Simulation Requirements
- All code (YAML, URDF, ROS 2 nodes, Unity/Gazebo setups) MUST execute without error
- Tested in documented environments; version constraints explicit
- Simulations include expected outputs and validation steps
- Breaking changes to code or simulations require changelog and migration guidance

### Interactive Features
- RAG chatbot: answers chapter-specific questions correctly, limited to selected content
- Personalization: user signup info dynamically updates content relevance
- Urdu translation: functional for all chapters; preserves technical accuracy in translation
- PDF export: includes citations and maintains layout quality

## Development & Quality Workflow

### Feature Development
1. All features start with specification (user stories, acceptance criteria)
2. Implementation blocked until spec is approved
3. Code changes reference spec; traceability to requirements maintained
4. Breaking changes require architectural decision review

### Testing & Validation
- Unit tests required for all utility functions and robotics helpers
- Integration tests mandatory for simulator interactions and API calls
- Content validation: each chapter fact-checked against cited sources
- Code samples: tested in actual environments before merge

### Documentation & Knowledge
- Commit messages include spec/ticket references
- ADRs created for architecturally significant decisions (framework choice, data model, API design)
- Deprecations announced with migration timeline; no silent removals

## Governance

This constitution supersedes all other practices and policies. Amendments require:
1. Clear rationale documented
2. Impact assessment on dependent templates (spec, plan, tasks)
3. Version bump with reason (MAJOR/MINOR/PATCH)
4. Update to dependent files (spec-template.md, plan-template.md, tasks-template.md)

All PRs must verify compliance with these principles. When complexity violations occur, justify in PR
description with reference to specific principle and rationale. Use runtime guidance (CLAUDE.md) for
agent-specific execution flows; this constitution defines project-level guarantees.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
