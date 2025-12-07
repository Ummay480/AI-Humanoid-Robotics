# Frontend - Humanoid Robotics Interactive Textbook

A comprehensive, modern frontend for the Physical AI & Humanoid Robotics Interactive Textbook.

## Project Structure

```
frontend/
├── apps/
│   ├── docs/              # Docosaurus static site (150 lessons, 10 chapters)
│   └── web/               # Next.js 14 interactive web application
├── packages/
│   ├── config/            # Shared Tailwind, TypeScript, design tokens
│   ├── ui/                # Shared React components library
│   └── types/             # Shared TypeScript type definitions
└── package.json           # Monorepo root configuration
```

## Technology Stack

- **Docosaurus 3.0** - Static documentation site (lessons, chapters)
- **Next.js 14** (App Router) - Interactive web application
- **React 18+** - UI library
- **TypeScript 5+** - Type safety
- **Tailwind CSS 3+** - Utility-first styling
- **Zustand** - State management
- **Jest + Playwright** - Testing
- **pnpm** - Package manager

## Quick Start

### Prerequisites

- Node.js 18+
- pnpm 8.0+

### Installation

```bash
cd frontend

# Install dependencies
pnpm install

# Run development servers
pnpm dev

# Build for production
pnpm build
```

### Available Scripts

```bash
# Development
pnpm dev                    # Start all dev servers
pnpm docs:serve             # Start Docosaurus dev server

# Building
pnpm build                  # Build all apps
pnpm docs:build             # Build Docosaurus site

# Testing
pnpm test                   # Run all tests
pnpm test:unit              # Run unit tests only
pnpm test:integration       # Run integration tests
pnpm test:e2e               # Run end-to-end tests
pnpm test:coverage          # Generate coverage report

# Code Quality
pnpm lint                   # Lint all code
pnpm lint:fix               # Fix linting issues
pnpm type-check             # Check TypeScript types
pnpm format                 # Format code with Prettier

# Validation
pnpm validate               # Run lint + type-check + tests

# Cleanup
pnpm clean                  # Remove build artifacts
```

## Key Concepts

### Monorepo Structure

The frontend is organized as a **pnpm monorepo** with separate workspaces:

- **`/apps/docs`** - Static documentation built with Docosaurus
  - Contains 150 markdown lessons across 10 chapters
  - Fully indexed and searchable
  - Generated from `/specs/1-docosaurus-textbook/`

- **`/apps/web`** - Interactive Next.js application
  - Chapter browsing with rich UI
  - Full-text search functionality
  - ChatKit chatbot integration (with Context-7)
  - Responsive design (mobile, tablet, desktop)
  - WCAG 2.1 AA accessible

- **`/packages/config`** - Shared configuration
  - Tailwind CSS configuration
  - Design tokens (colors, typography, spacing)
  - TypeScript configuration

- **`/packages/ui`** - Component library
  - Reusable React components
  - Consistent styling via Tailwind
  - Accessibility-first approach

- **`/packages/types`** - Shared TypeScript types
  - Chapter, Lesson, Section types
  - Search types
  - Chat and Agent types
  - Context-7 file selection types

### Architecture Principles

#### Zero Backend Logic on Frontend (MANDATORY)

The frontend is **UI-only**. All backend operations happen on the server:

❌ **NO**: Agent execution on frontend
❌ **NO**: Tool calling on frontend
❌ **NO**: MCP invocation on frontend

✅ **YES**: REST API proxy to `/api/agent` (relay-only, no execution)
✅ **YES**: ChatKit UI for message display
✅ **YES**: Context-7 file selection

#### Context-7 System

Students can select up to 7 context files per chatbot query:

- **Cognitive Load**: Miller's Law (7 ± 2 items)
- **Token Efficiency**: 7 files × ~1000 tokens each = ~7K tokens
- **Latency Control**: <5 seconds response time target
- **Cost Predictability**: Fixed context size per query

Implementation in `/packages/types/context.ts`.

#### Design System

All components use a **consistent design system**:

- **Colors**: Primary, secondary, success, warning, error
- **Typography**: Sans, mono families with defined scales
- **Spacing**: Consistent margin/padding scale
- **Responsive**: Mobile-first approach with breakpoints at 320px, 768px, 1024px, 1440px

## Development Workflow

### Creating a New Component

1. **Create component in `/apps/web/components/` or `/packages/ui/components/`**
   ```tsx
   // apps/web/components/MyComponent.tsx
   export function MyComponent() {
     return <div className="p-4">Hello</div>;
   }
   ```

2. **Export from appropriate index**
   ```ts
   // packages/ui/index.ts
   export { MyComponent } from './components/MyComponent';
   ```

3. **Use in Next.js**
   ```tsx
   import { MyComponent } from '@humanoid-robotics/ui';
   ```

### Testing

All tests use **Jest** with **@testing-library/react**:

```tsx
// MyComponent.test.tsx
import { render, screen } from '@testing-library/react';
import { MyComponent } from './MyComponent';

test('renders component', () => {
  render(<MyComponent />);
  expect(screen.getByText('Hello')).toBeInTheDocument();
});
```

E2E tests use **Playwright**:

```ts
// e2e/navigation.spec.ts
import { test, expect } from '@playwright/test';

test('navigates to chapter', async ({ page }) => {
  await page.goto('/');
  await page.click('text=Chapter 1');
  await expect(page).toHaveURL('/chapter/fundamentals');
});
```

### Accessibility

All components must be **WCAG 2.1 Level AA** compliant:

- ✅ Keyboard navigable
- ✅ Screen reader compatible
- ✅ 4.5:1 contrast ratio
- ✅ No motion without user interaction

Use `jest-axe` for automated testing:

```tsx
import { axe, toHaveNoViolations } from 'jest-axe';

test('has no accessibility violations', async () => {
  const { container } = render(<MyComponent />);
  expect(await axe(container)).toHaveNoViolations();
});
```

## Deployment

### To Vercel

```bash
# Connect GitHub repo to Vercel
vercel link

# Deploy
vercel
```

**Environment Variables** (set in Vercel dashboard):
```
NEXT_PUBLIC_API_URL=https://api.example.com
NEXT_PUBLIC_DOCOSAURUS_URL=https://docs.example.com
```

### Building for Production

```bash
pnpm build         # Builds all apps
pnpm start         # Start production server (Next.js)
```

## Constitutional Compliance Checklist

Before committing, ensure:

- ✅ Zero backend logic on frontend
- ✅ ChatKit used for UI layer only
- ✅ Context-7 enforces 7-file maximum
- ✅ Monorepo structure maintained
- ✅ Design consistency verified
- ✅ Accessibility (WCAG 2.1 AA) checked
- ✅ TypeScript strict mode enabled
- ✅ No hardcoded secrets
- ✅ Bundle size <300KB gzipped
- ✅ Lighthouse 90+ on all metrics

See `/specs/1-docosaurus-textbook/` for complete requirements.

## Documentation

- **Architecture**: `/specs/1-docosaurus-textbook/plan.md`
- **Specification**: `/specs/1-docosaurus-textbook/spec.md`
- **Data Model**: `/specs/1-docosaurus-textbook/data-model.md`
- **Implementation Tasks**: `/specs/1-docosaurus-textbook/tasks.md`
- **Architecture Decisions**: `history/adr/ADR-*.md`

## Contributing

1. Read the **Frontend Constitution**: `/.specify/memory/constitution.md`
2. Review the **Architecture Plan**: `/specs/1-docosaurus-textbook/plan.md`
3. Check the **Implementation Tasks**: `/specs/1-docosaurus-textbook/tasks.md`
4. Follow the **Constitutional Compliance Checklist** above
5. Write tests for all new features
6. Ensure accessibility compliance before PR
7. Update documentation as needed

## Support

For questions or issues:

1. Check `/specs/1-docosaurus-textbook/` documentation
2. Review existing ADRs in `history/adr/`
3. Open an issue on GitHub

---

**Project**: Physical AI & Humanoid Robotics Interactive Textbook
**Status**: Phase 1 Complete (Monorepo Setup)
**Next Phase**: Phase 2 (Foundational Components & Design System)
