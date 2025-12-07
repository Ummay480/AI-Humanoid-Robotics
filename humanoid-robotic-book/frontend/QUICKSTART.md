# Frontend Quick Start Guide

## 🚀 Getting Started

### Step 1: Navigate to Frontend Directory
```bash
cd frontend
```

### Step 2: Install Dependencies
```bash
npm install
# or if you have pnpm installed:
pnpm install
```

This will install all dependencies for:
- Root monorepo configuration
- `/apps/docs` - Docosaurus static site
- `/apps/web` - Next.js web application
- `/packages/config` - Design tokens
- `/packages/ui` - Component library
- `/packages/types` - Type definitions

**First time? This may take 3-5 minutes to download all packages.**

### Step 3: Start Development Server
```bash
npm run dev
```

This starts:
- **Next.js app**: http://localhost:3000
- **Docosaurus site**: http://localhost:3001 (or auto-selected port)

### Step 4: View the Application

Open your browser:
- **Home**: http://localhost:3000
- **Chapters**: Listed on home page with links
- **Docs**: http://localhost:3001 (Docosaurus with 150 lessons)

## 📁 Project Structure

```
frontend/
├── apps/
│   ├── docs/          ← Docosaurus (static lessons)
│   └── web/           ← Next.js app (interactive)
├── packages/
│   ├── config/        ← Design tokens
│   ├── ui/            ← Components (Phase 2)
│   └── types/         ← TypeScript definitions
└── package.json       ← Monorepo root
```

## 🔧 Common Commands

```bash
# Development
npm run dev              # Start both apps
npm run build            # Build for production
npm run test             # Run all tests
npm run lint             # Check code quality
npm run type-check       # Check TypeScript

# Just Next.js
cd apps/web
npm run dev              # Start Next.js only
npm run build            # Build Next.js

# Just Docosaurus
cd apps/docs
npm run dev              # Start Docosaurus only
npm run build            # Build Docosaurus
```

## 📚 What to Expect

### Home Page (http://localhost:3000)
- Welcome message with project description
- List of all 10 chapters with descriptions
- Buttons to start learning and search content

### Docosaurus Site (http://localhost:3001)
- All 150 lessons with proper navigation
- Sidebar showing chapter structure
- Search functionality
- Proper markdown rendering

## 🛠️ Next Steps

1. **Review Architecture** - Read `/specs/1-docosaurus-textbook/plan.md`
2. **Check Tasks** - See `/specs/1-docosaurus-textbook/tasks.md` for Phase 2+
3. **Read Constitution** - Understand `/specs/1-docosaurus-textbook/spec.md`
4. **Start Phase 2** - Build UI components and design system

## ❓ Troubleshooting

### `npm install` hangs or takes too long
- This is normal for large monorepos
- Wait 5-10 minutes for first install
- Check your internet connection
- Try clearing npm cache: `npm cache clean --force`

### Port already in use
- Next.js will auto-select port 3001+ if 3000 is taken
- Check which app is using which port in terminal output

### Dependencies not found
- Make sure you're in the `frontend` directory
- Delete `node_modules` and `package-lock.json`
- Run `npm install` again

### TypeScript errors
- This is expected during Phase 1
- Run `npm run type-check` to see all errors
- They'll be fixed during Phase 2+ implementation

## 📖 Documentation

- **README.md** - Full project documentation
- **PHASE_1_COMPLETE.md** - What was built in Phase 1
- **IMPLEMENTATION_STATUS.md** - Overall project status

## 🎯 Current Phase

**Phase 1 Complete** - Monorepo initialized and configured

**Next**: Phase 2 - Design system and UI component library

---

**Last Updated**: 2025-12-06
**Status**: Ready to develop
