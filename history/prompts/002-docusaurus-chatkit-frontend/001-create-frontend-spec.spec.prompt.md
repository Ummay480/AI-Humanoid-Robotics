---
id: 001
title: Create Docusaurus ChatKit Frontend Spec
stage: spec
date: 2025-12-08
surface: agent
model: claude-haiku-4-5-20251001
feature: 002-docusaurus-chatkit-frontend
branch: 002-docusaurus-chatkit-frontend
user: user
command: /sp.specify
labels: ["frontend", "docusaurus", "chatkit", "ui-components", "specification"]
links:
  spec: specs/002-docusaurus-chatkit-frontend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-docusaurus-chatkit-frontend/spec.md
  - specs/002-docusaurus-chatkit-frontend/checklists/requirements.md
tests:
  - Specification validation checklist passed (all items checked)
  - No [NEEDS CLARIFICATION] markers present
  - All user stories marked with priority levels (P1-P3)
  - Success criteria include both functional and responsive requirements
---

## Prompt

You are my Frontend Architect.
Your job is to generate the **complete frontend** for my documentation book
**"Physical AI & Humanoid Robotics"** using Docusaurus + ChatKit UI.

IMPORTANT:
- Only frontend code and file structure
- No backend, no RAG logic, no vector DB logic
- Just components, pages, layout, UI, structure, placeholders
- Must be fully compatible with Docusaurus Classic template

-----------------------------------
📌 PROJECT REQUIREMENTS
-----------------------------------
Build the frontend of a "Book + Interactive AI Assistant" website using:

1. **Docusaurus**
   - `/docs` → book chapters
   - `/blog` optional
   - `/src/components` → custom React components
   - `/src/pages` → UI pages (Home, Book, Chat)

2. **ChatKit-style UI**
   - Floating Chat Assistant
   - Chat messages list
   - User input bar
   - Placeholder functions for:
     - `sendMessage()`
     - `streamResponse()`
   (No backend implementation required.)

3. **Book Layout**
   - Beautiful Hero section
   - Sidebar-enabled documentation
   - Chapter pages
   - Lesson pages
   - Auto-generated docs structure

4. **UI components to include**
   - ChatBox.jsx
   - ChatBubble.jsx
   - SidebarBookMenu.jsx
   - AIButton.jsx
   - HeroBanner.jsx
   - Layout.jsx
   - Theme toggle button
   - Search (placeholder)

5. **Frontend only**
   - No database
   - No RAG embedding logic
   - No SDK usage
   - Only placeholder functions/hooks
   - Only UI code

## Response snapshot

Created comprehensive feature specification for Docusaurus + ChatKit frontend including:

**User Stories (P1-P3)**:
- P1: Reader explores book chapters with sidebar navigation
- P1: Reader interacts with floating AI chat assistant
- P2: Reader searches content across the book
- P3: Reader toggles dark/light theme mode

**16 Functional Requirements** covering:
- Responsive Docusaurus navbar with logo, search, theme toggle, chat button
- Hero landing page with "Start Reading" CTA
- Auto-generated sidebar from `/docs` folder structure
- Floating chat panel with message bubbles (user/bot distinction)
- Loading indicators for chat responses
- Session-persisted chat history
- Markdown content rendering with proper formatting
- Theme toggle for light/dark modes
- Mobile-responsive design (hamburger menu on mobile)
- Custom React components as JSX files
- Placeholder functions for sendMessage() and streamResponse()

**Key Entities**:
- Chapter (container for lessons with metadata)
- Lesson (individual page with markdown content)
- ChatMessage (message in chat history with user/bot indicator)
- SiteConfig (Docusaurus config and theme setup)

**10 Success Criteria** including:
- All 6 core React components render without errors
- Website loads with functional navbar, hero, sidebar, chat button
- Users navigate through complete chapters with breadcrumbs
- Chat opens/closes smoothly with mock responses
- Responsive design on mobile/tablet/desktop (768px breakpoint)
- Theme toggle switches modes while maintaining readability
- Zero console errors in component rendering
- Auto-generated sidebar hierarchy from docs structure
- Placeholder functions callable and log to console
- Project compiles with `npm start` without core warnings

**Constraints & Assumptions**:
- Frontend-only (no backend, database, or auth)
- Based on Docusaurus Classic template
- Functional components only (no class-based)
- Docusaurus CSS/Infima + optional Tailwind
- Chat history persists session-only (no localStorage)
- Search and chat responses are mocked/placeholder

**Non-Goals**: Auth, persistent storage, real AI/LLM integration, i18n, advanced analytics

Specification validated with quality checklist (all items passed).

## Outcome

- ✅ Impact: Comprehensive spec establishes clear requirements for P1 (book navigation + chat) and P2-P3 (search + theme) features. Frontend-only scope clearly delimited from backend work.
- 🧪 Tests: Specification validation checklist completed with all items passing. No clarification questions required.
- 📁 Files: 2 files created (spec.md + requirements.md checklist). Committed to branch 002-docusaurus-chatkit-frontend.
- 🔁 Next prompts: `/sp.plan` to create architectural design and implementation strategy
- 🧠 Reflection: Specification captures full user journey from book discovery through AI interaction. Clear MVP boundary (P1 features) vs nice-to-have (P2-P3). Ready for planning phase.

## Evaluation notes (flywheel)

- Failure modes observed: None - specification completed successfully without ambiguities
- Graders run and results (PASS/FAIL): Specification Quality Checklist - PASS (all 13 validation items checked)
- Prompt variant (if applicable): None (single pass)
- Next experiment: Planning phase will validate whether component architecture decisions require ADR documentation
