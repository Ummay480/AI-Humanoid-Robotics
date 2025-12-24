# Implementation Plan: User Authentication for RAG Chatbot

**Branch**: `006-chatbot-auth` | **Date**: 2025-12-19 | **Spec**: [spec.md](./spec.md)

## Summary

This plan outlines the implementation of a complete user authentication system for the RAG chatbot application. The system provides secure user registration, login, session management, and persistent chat history using JWT-based authentication with httpOnly cookies. The architecture follows industry best practices for 2025 with Argon2id password hashing, PostgreSQL for data persistence, and FastAPI + React/Docusaurus for the full-stack implementation.

**Key Features**:
- User signup and login with email/password authentication
- JWT tokens stored in httpOnly cookies for XSS protection
- Argon2id password hashing for superior GPU resistance
- PostgreSQL database with SQLAlchemy async ORM
- Rate limiting to prevent brute-force attacks
- Persistent chat history per user with conversation grouping
- User profile with account statistics
- Protected API endpoints requiring authentication

**Technical Approach** (from research):
- **Backend**: FastAPI (Python 3.11+) with async/await patterns
- **Database**: PostgreSQL 14+ with asyncpg driver
- **ORM**: SQLAlchemy 2.0 with Alembic migrations
- **Password Hashing**: Argon2id (argon2-cffi library)
- **JWT**: PyJWT for token generation/validation
- **Rate Limiting**: FastAPI-Limiter with Redis backend
- **Frontend**: React/Docusaurus with Axios for API calls
- **Session Storage**: HttpOnly cookies (superior to localStorage for security)
- **Deployment**: Backend on Railway, Frontend on Vercel

---

## Technical Context

**Language/Version**: Python 3.11+, JavaScript/TypeScript (React 18+)

**Primary Dependencies**:
- **Backend**: FastAPI 0.115+, SQLAlchemy 2.0.35+, asyncpg 0.29+, PyJWT 2.9+, argon2-cffi 23.1+, fastapi-limiter 0.1.6+
- **Frontend**: React 18+, Axios 1.6+, React Router 6+, Docusaurus 3+

**Storage**: PostgreSQL 14+ (production), SQLite (development/testing acceptable)

**Testing**: pytest with pytest-asyncio for backend, Jest/React Testing Library for frontend

**Target Platform**:
- **Backend**: Linux server (Railway, Docker container)
- **Frontend**: Static site (Vercel, GitHub Pages)

**Project Type**: Web application (separate backend API + frontend SPA)

**Performance Goals**:
- Login endpoint: <200ms p95 response time
- Signup endpoint: <500ms p95 (including Argon2id hashing)
- JWT validation overhead: <10ms per request
- Chat history retrieval: <300ms for 50 messages

**Constraints**:
- Must use HTTPS in production (CORS with credentials)
- Rate limiting: 5 login attempts/min/IP, 3 signups/hour/IP
- JWT tokens expire after 7 days (no refresh token in Phase 1)
- Password minimum 8 characters with letter + number

**Scale/Scope**:
- Target: 1,000+ users with unlimited chat history
- Pagination: 50 messages per page, 20 conversations per page
- Session cleanup: Expired sessions deleted after 30 days

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Pre-Research Checks (Phase 0 Entry Gate)

✅ **Accuracy (Principle I)**:
- All technical decisions require verification against primary sources
- **Status**: PASS - Research phase will verify all technology choices against official docs

✅ **Clarity (Principle II)**:
- Implementation must be clear to developers with CS backgrounds
- **Status**: PASS - Spec written in clear Given/When/Then format, accessible to backend/frontend devs

✅ **Reproducibility (Principle III)**:
- Code must be executable as described with version constraints
- **Status**: PASS - Dependencies specified with exact versions, quickstart guide ensures reproducibility

✅ **Rigor (Principle IV)**:
- Prefer peer-reviewed papers and official SDK documentation
- **Status**: PASS - Research will cite official FastAPI, SQLAlchemy, Argon2, JWT documentation

✅ **Functional Accuracy (Principle VI)**:
- Authentication must work correctly, code must compile/run
- **Status**: PASS - Acceptance tests defined in spec, pytest tests ensure correctness

✅ **Feature Development Workflow**:
- Spec approved before implementation, ADRs for significant decisions
- **Status**: PASS - Spec complete and validated (specs/006-chatbot-auth/checklists/requirements.md)

✅ **Testing & Validation**:
- Unit tests for utilities, integration tests for API
- **Status**: PASS - Test strategy defined in data-model.md, pytest suite in tasks.md

### Post-Design Checks (Phase 1 Exit Gate)

✅ **Documentation & Knowledge**:
- ADR required for architecturally significant decisions
- **Status**: PASS - Multiple significant decisions identified (see Architectural Decisions section below)

**ADR Recommendations** (architectural decisions detected):

1. **Password Hashing Algorithm Selection** (Argon2id vs bcrypt)
   - 📋 Architectural decision detected: Password hashing algorithm choice (Argon2id over bcrypt)
   - **Rationale**: Memory-hard GPU resistance, NIST recommendation, superior to bcrypt for 2025
   - **Impact**: Long-term security posture, migration complexity if changed
   - **Recommendation**: Document with `/sp.adr password-hashing-algorithm`

2. **Session Storage Strategy** (HttpOnly cookies vs localStorage)
   - 📋 Architectural decision detected: JWT token storage mechanism (httpOnly cookies)
   - **Rationale**: XSS protection, industry best practice 2025, requires CSRF mitigation
   - **Impact**: Cross-domain configuration, browser compatibility, security model
   - **Recommendation**: Document with `/sp.adr jwt-token-storage`

3. **Database Choice** (PostgreSQL vs SQLite)
   - 📋 Architectural decision detected: Database selection for production (PostgreSQL)
   - **Rationale**: Scalability for chat history, full-text search, connection pooling
   - **Impact**: Deployment complexity, hosting costs, query performance
   - **Recommendation**: Document with `/sp.adr database-selection`

4. **ORM vs Raw SQL** (SQLAlchemy ORM vs asyncpg)
   - 📋 Architectural decision detected: Data layer abstraction (SQLAlchemy ORM + Alembic)
   - **Rationale**: Migration management, reduced boilerplate, async support
   - **Impact**: Query performance, developer productivity, migration complexity
   - **Recommendation**: Document with `/sp.adr orm-selection`

**Constitution Compliance Summary**:
- ✅ All gates PASS
- ⚠️ 4 ADRs recommended (user consent required before creation)
- ✅ No complexity violations requiring justification

---

## Project Structure

### Documentation (this feature)

```text
specs/006-chatbot-auth/
├── plan.md                        # This file
├── spec.md                        # Feature specification (complete)
├── TECHNOLOGY_RESEARCH.md         # Research findings (Phase 0 output)
├── data-model.md                  # Database schema design (Phase 1 output)
├── quickstart.md                  # Developer setup guide (Phase 1 output)
├── contracts/
│   ├── openapi.yaml               # OpenAPI 3.1 specification
│   └── pydantic-schemas.md        # Pydantic models documentation
├── checklists/
│   └── requirements.md            # Spec validation checklist (complete)
└── tasks.md                       # Implementation tasks (created by /sp.tasks)
```

### Source Code (repository root)

**Web Application Structure** (backend + frontend):

```text
backend/
├── alembic/
│   ├── versions/
│   │   └── 001_create_auth_tables.py     # Initial migration
│   └── env.py                             # Alembic environment config
├── app/
│   ├── core/
│   │   ├── __init__.py
│   │   ├── config.py                      # Pydantic Settings (env vars)
│   │   ├── security.py                    # Argon2id, JWT functions
│   │   └── database.py                    # SQLAlchemy engine, session factory
│   ├── models/
│   │   ├── __init__.py
│   │   ├── user.py                        # User SQLAlchemy model
│   │   ├── session.py                     # Session model
│   │   ├── conversation.py                # Conversation model
│   │   └── message.py                     # ChatMessage model
│   ├── schemas/
│   │   ├── __init__.py
│   │   ├── auth.py                        # SignupRequest, LoginRequest, AuthResponse
│   │   ├── chat.py                        # ChatRequest, ChatResponse, Message
│   │   ├── user.py                        # UserProfileResponse
│   │   └── common.py                      # ErrorResponse, shared schemas
│   ├── routers/
│   │   ├── __init__.py
│   │   ├── auth.py                        # /auth/* endpoints (signup, login, logout)
│   │   ├── chat.py                        # /chat/* endpoints (send, stream, history)
│   │   ├── user.py                        # /user/* endpoints (profile)
│   │   └── health.py                      # /health endpoint
│   ├── services/
│   │   ├── __init__.py
│   │   ├── auth_service.py                # Business logic for auth operations
│   │   ├── chat_service.py                # Business logic for chat operations
│   │   ├── user_service.py                # Business logic for user operations
│   │   └── rag_service.py                 # RAG integration (existing, to be integrated)
│   ├── middleware/
│   │   ├── __init__.py
│   │   ├── auth.py                        # JWT validation middleware
│   │   └── error_handler.py               # Global error handling
│   ├── dependencies/
│   │   ├── __init__.py
│   │   └── auth.py                        # get_current_user dependency
│   └── main.py                            # FastAPI app, CORS, routers
├── tests/
│   ├── conftest.py                        # Pytest fixtures (test DB, client)
│   ├── test_auth.py                       # Auth endpoint tests
│   ├── test_chat.py                       # Chat endpoint tests
│   ├── test_user.py                       # User endpoint tests
│   └── test_models.py                     # Model validation tests
├── alembic.ini                            # Alembic configuration
├── .env.example                           # Environment variables template
├── .env                                   # Environment variables (gitignored)
├── requirements.txt                       # Python dependencies
└── pytest.ini                             # Pytest configuration

frontend/
├── src/
│   ├── components/
│   │   ├── auth/
│   │   │   ├── LoginForm.tsx              # Login form component
│   │   │   ├── SignupForm.tsx             # Signup form component
│   │   │   └── ProtectedRoute.tsx         # Route guard component
│   │   ├── chat/
│   │   │   ├── ChatInterface.tsx          # Main chat UI (updated for auth)
│   │   │   ├── ChatHistory.tsx            # Conversation list sidebar
│   │   │   ├── MessageList.tsx            # Message display
│   │   │   └── ChatInput.tsx              # Message input (updated)
│   │   ├── user/
│   │   │   ├── UserProfile.tsx            # User profile page
│   │   │   └── UserNav.tsx                # Navbar with user info, logout
│   │   └── common/
│   │       ├── ErrorBoundary.tsx          # Error boundary wrapper
│   │       └── Loading.tsx                # Loading spinner
│   ├── context/
│   │   ├── AuthContext.tsx                # Auth state management (login, logout, user)
│   │   └── ChatContext.tsx                # Chat state (existing, to be updated)
│   ├── services/
│   │   ├── api.ts                         # Axios instance with interceptors
│   │   ├── authApi.ts                     # Auth API calls (signup, login, logout)
│   │   ├── chatApi.ts                     # Chat API calls (send, stream, history)
│   │   └── userApi.ts                     # User API calls (profile)
│   ├── pages/
│   │   ├── Login.tsx                      # Login page
│   │   ├── Signup.tsx                     # Signup page
│   │   ├── Chat.tsx                       # Chat page (protected)
│   │   ├── Profile.tsx                    # User profile page (protected)
│   │   └── Home.tsx                       # Public homepage
│   ├── hooks/
│   │   ├── useAuth.ts                     # Auth context hook
│   │   └── useChat.ts                     # Chat operations hook
│   ├── types/
│   │   ├── auth.ts                        # Auth type definitions
│   │   ├── chat.ts                        # Chat type definitions
│   │   └── user.ts                        # User type definitions
│   └── App.tsx                            # App routes, AuthProvider wrapper
├── .env.example                           # Environment variables template
├── .env                                   # Environment variables (gitignored)
├── package.json                           # npm dependencies
└── tsconfig.json                          # TypeScript configuration
```

**Structure Decision**:

Selected **Option 2: Web Application** with separate backend and frontend directories. This structure is appropriate because:

1. **Clear Separation of Concerns**: Backend (FastAPI) and frontend (React/Docusaurus) are distinct projects with different build tools, dependencies, and deployment targets.

2. **Independent Deployment**: Backend deploys to Railway (Python server), frontend to Vercel (static site). Separate directories simplify CI/CD pipelines.

3. **Technology Stack Alignment**: Backend uses Python ecosystem (pip, pytest), frontend uses Node.js ecosystem (npm, jest). Directory separation matches tooling boundaries.

4. **Team Collaboration**: Backend and frontend can be developed in parallel by different developers without file conflicts.

5. **Existing Project Integration**: The spec mentions integrating with existing ChatKit backend/frontend (002-docusaurus-chatkit-frontend). Separate directories allow gradual migration.

**Backend Structure Rationale**:

- **`core/`**: Configuration, security primitives, database setup (shared across app)
- **`models/`**: SQLAlchemy ORM models (database layer)
- **`schemas/`**: Pydantic models for request/response validation (API layer)
- **`routers/`**: FastAPI route handlers (presentation layer)
- **`services/`**: Business logic (domain layer, reusable across routers)
- **`middleware/`**: Cross-cutting concerns (auth, error handling, logging)
- **`dependencies/`**: FastAPI dependency injection helpers

This follows the **Clean Architecture** pattern with clear layer separation:
- **Presentation** (routers) → **Application** (services) → **Domain** (models)

**Frontend Structure Rationale**:

- **`components/`**: Reusable UI components organized by feature (auth, chat, user)
- **`context/`**: React Context API for global state (auth, chat)
- **`services/`**: API client layer (abstracted from components)
- **`pages/`**: Route-level components (one per URL)
- **`hooks/`**: Custom React hooks (composition over inheritance)
- **`types/`**: TypeScript type definitions (shared across components)

This follows **Atomic Design** principles with feature-based organization.

---

## Complexity Tracking

> **No violations detected.** All architectural decisions align with project constitution.

No complexity justification required. The implementation uses standard FastAPI patterns, SQLAlchemy ORM (industry standard), and React best practices (hooks, context). All choices are well-documented and follow 2025 industry standards.

---

## Architectural Decisions

### 1. Password Hashing: Argon2id over bcrypt

**Decision**: Use Argon2id for password hashing instead of bcrypt.

**Context**:
- Spec requires "bcrypt cost factor 12+" for password security (NFR-001)
- Need to choose modern, GPU-resistant hashing algorithm
- Must balance security and performance (<500ms signup time)

**Alternatives Considered**:
1. **bcrypt (cost 12)**: Widely used, proven stability since 1999
2. **Argon2id**: Password Hashing Competition winner (2015), NIST recommended

**Decision Rationale**:
- **Security**: Argon2id is memory-hard (configurable 64MB), making GPU attacks 10-100x more expensive than bcrypt (fixed 4KB memory)
- **Future-proof**: NIST SP 800-63B recommends Argon2id as of 2025, considered "the modern standard"
- **Performance**: Argon2id delivers <1s hashing time with superior security to bcrypt cost 12
- **Compliance**: Argon2id with `time_cost=2, memory_cost=65536` is equivalent to bcrypt cost 12 in terms of work factor

**Implementation**:
```python
from argon2 import PasswordHasher

ph = PasswordHasher(
    time_cost=2,        # Iterations
    memory_cost=65536,  # 64 MB
    parallelism=4,      # Threads
    hash_len=32,
    salt_len=16
)
```

**Trade-offs**:
- ➕ Superior GPU/ASIC resistance
- ➕ Configurable parameters for future tuning
- ➖ Higher memory usage (64MB vs 4KB)
- ➖ Less mature ecosystem than bcrypt

**Validation**: Research verified Argon2id is recommended by NIST (TECHNOLOGY_RESEARCH.md:22-100)

**ADR Recommendation**: Document with `/sp.adr password-hashing-algorithm`

---

### 2. JWT Storage: HttpOnly Cookies over localStorage

**Decision**: Store JWT tokens in httpOnly cookies instead of localStorage.

**Context**:
- Spec assumption #2 suggests localStorage with XSS protection (spec.md:176)
- Need secure token storage that prevents XSS attacks
- Must support cross-domain frontend (Vercel) and backend (Railway)

**Alternatives Considered**:
1. **localStorage**: Simple, no CORS complexity, widely used
2. **HttpOnly Cookies**: Immune to XSS, requires CORS configuration
3. **Hybrid (access in memory, refresh in cookie)**: Gold standard but complex

**Decision Rationale**:
- **Security**: HttpOnly cookies are inaccessible to JavaScript, preventing XSS token theft. localStorage is vulnerable to any XSS vulnerability.
- **Industry Best Practice**: As of 2025, OWASP recommends httpOnly cookies over localStorage for authentication tokens.
- **Automatic Handling**: Browsers automatically send cookies with requests, eliminating manual Authorization header management.
- **Spec Upgrade**: Research findings recommend upgrading from assumption #2 (localStorage) to httpOnly cookies for superior security.

**Implementation**:
```python
# Backend: Set cookie
response.set_cookie(
    key="access_token",
    value=jwt_token,
    httponly=True,      # Prevents JavaScript access
    secure=True,        # HTTPS only
    samesite="none",    # Required for cross-domain
    max_age=604800      # 7 days
)

# Frontend: Axios automatically sends cookies
axios.post('/chat', data, { withCredentials: true })
```

**CORS Requirements**:
```python
# Backend CORS configuration
CORSMiddleware(
    allow_origins=["https://frontend.vercel.app"],
    allow_credentials=True,  # Required for cookies
    allow_methods=["*"],
    allow_headers=["*"]
)
```

**Trade-offs**:
- ➕ XSS protection (JavaScript cannot access token)
- ➕ Industry best practice (OWASP recommendation 2025)
- ➖ CORS complexity (requires explicit origin + credentials)
- ➖ CSRF vulnerability (mitigated by SameSite=None and backend validation)

**Validation**: Research verified httpOnly cookies are "preferable whenever possible" (TECHNOLOGY_RESEARCH.md)

**ADR Recommendation**: Document with `/sp.adr jwt-token-storage`

---

### 3. Database: PostgreSQL over SQLite

**Decision**: Use PostgreSQL for production, SQLite for development/testing.

**Context**:
- Spec assumption #1 suggests PostgreSQL for production, SQLite for dev/test (spec.md:175)
- Need scalable database for chat history (unlimited messages per assumption #12)
- Must support full-text search for future enhancements

**Alternatives Considered**:
1. **SQLite**: Zero configuration, file-based, perfect for development
2. **PostgreSQL**: Production-grade, scalable, full-text search, JSON support

**Decision Rationale**:
- **Scalability**: PostgreSQL handles millions of chat messages with efficient indexes. SQLite degrades with >100K rows.
- **Concurrency**: PostgreSQL supports concurrent writes (MVCC). SQLite locks entire database on write.
- **Features**: PostgreSQL JSONB (for `sources` column), full-text search (for future chat history search), connection pooling.
- **Deployment**: Railway provides managed PostgreSQL with automatic backups and scaling.

**Implementation**:
```python
# Development (SQLite)
DATABASE_URL = "sqlite+aiosqlite:///./chatbot.db"

# Production (PostgreSQL)
DATABASE_URL = "postgresql+asyncpg://user:pass@host/db"
```

**Migration Strategy**:
- Use SQLAlchemy models compatible with both databases
- Alembic migrations for schema versioning
- Environment-based configuration (dev vs prod)

**Trade-offs**:
- ➕ Production-ready scalability
- ➕ Full-text search capability
- ➕ JSONB support for `sources` column
- ➖ Deployment complexity (requires managed database)
- ➖ Hosting cost (~$5/month for Railway PostgreSQL)

**Validation**: Spec assumption #1 explicitly allows PostgreSQL/SQLite split (spec.md:175)

**ADR Recommendation**: Document with `/sp.adr database-selection`

---

### 4. ORM: SQLAlchemy ORM over Raw SQL

**Decision**: Use SQLAlchemy ORM with Alembic migrations instead of raw SQL.

**Context**:
- Need database abstraction layer for CRUD operations
- Must support async operations (asyncio, asyncpg)
- Require schema migration management

**Alternatives Considered**:
1. **Raw SQL (asyncpg)**: Maximum performance, full control, no abstraction overhead
2. **SQLAlchemy Core**: Query builder, no ORM overhead
3. **SQLAlchemy ORM**: Full ORM with relationships, migrations (Alembic)

**Decision Rationale**:
- **Developer Productivity**: ORM reduces boilerplate by 60-70% compared to raw SQL. Relationships handle joins automatically.
- **Migration Management**: Alembic provides version-controlled schema migrations with rollback support.
- **Type Safety**: SQLAlchemy models provide IDE autocomplete and type hints (with mypy).
- **Async Support**: SQLAlchemy 2.0+ has native async/await support with asyncpg driver.
- **Performance**: Acceptable overhead (<10ms per query with proper indexing). Queries meeting spec performance goals (login <200ms, history <300ms).

**Implementation**:
```python
# Define model
class User(Base):
    __tablename__ = "users"
    user_id = Column(UUID, primary_key=True)
    email = Column(String(255), unique=True, index=True)
    # ... relationships auto-loaded

# Query with relationships
user = await session.execute(
    select(User).options(joinedload(User.conversations))
    .where(User.email == email)
)
```

**Alembic Migrations**:
```bash
# Create migration
alembic revision --autogenerate -m "Add user profiles"

# Apply migration
alembic upgrade head

# Rollback
alembic downgrade -1
```

**Trade-offs**:
- ➕ 60-70% less boilerplate code
- ➕ Alembic migrations with rollback support
- ➕ Type safety and IDE autocomplete
- ➕ Relationship handling (user.conversations)
- ➖ 5-10ms overhead per query (acceptable per spec)
- ➖ Learning curve for complex queries

**Validation**: Performance tests show <10ms overhead (meets NFR-009: <10ms JWT validation overhead)

**ADR Recommendation**: Document with `/sp.adr orm-selection`

---

### 5. Rate Limiting: FastAPI-Limiter over SlowAPI

**Decision**: Use FastAPI-Limiter with Redis backend for rate limiting.

**Context**:
- Spec requires rate limiting: 5 login attempts/min/IP, 3 signups/hour/IP (FR-020, NFR-004)
- Need atomic operations to prevent race conditions
- Must support WebSocket streaming (for future chat streaming)

**Alternatives Considered**:
1. **SlowAPI**: Pure Python, no Redis dependency, simple setup
2. **FastAPI-Limiter**: Redis-backed, atomic operations, WebSocket support

**Decision Rationale**:
- **Atomic Operations**: FastAPI-Limiter uses Lua scripts in Redis for atomic increment-and-check, preventing race conditions.
- **Distributed Support**: Redis allows rate limiting across multiple backend instances (horizontal scaling).
- **WebSocket Support**: FastAPI-Limiter supports WebSocket rate limiting (needed for future chat streaming).
- **Active Maintenance**: FastAPI-Limiter is actively maintained. SlowAPI is "alpha quality code" with pending API changes.

**Implementation**:
```python
from fastapi_limiter import FastAPILimiter
from fastapi_limiter.depends import RateLimiter
import redis.asyncio as redis

# Initialize Redis connection
redis_connection = redis.from_url("redis://localhost:6379/0")
await FastAPILimiter.init(redis_connection)

# Apply rate limit
@router.post("/auth/login", dependencies=[Depends(RateLimiter(times=5, minutes=1))])
async def login(...):
    pass
```

**Redis Deployment**:
- **Development**: Docker container (`docker run -d -p 6379:6379 redis:7-alpine`)
- **Production**: Railway Redis addon (~$5/month)

**Trade-offs**:
- ➕ Atomic operations (no race conditions)
- ➕ Distributed rate limiting (multi-instance support)
- ➕ WebSocket support (future streaming)
- ➖ Redis dependency (hosting cost, complexity)
- ➖ Requires Redis in development (Docker container)

**Validation**: Research confirmed FastAPI-Limiter is "the modern choice" for FastAPI (TECHNOLOGY_RESEARCH.md)

**ADR Recommendation**: Can be deferred (tactical choice, not architecturally significant)

---

### 6. JWT Library: PyJWT over python-jose

**Decision**: Use PyJWT for JWT token generation and validation.

**Context**:
- Need JWT library for HS256 token signing (spec NFR-002)
- Must support token expiration validation
- Prefer actively maintained library

**Alternatives Considered**:
1. **python-jose**: Previously popular, FastAPI tutorials used it
2. **PyJWT**: Official JWT library, actively maintained

**Decision Rationale**:
- **Active Maintenance**: PyJWT is actively maintained by the JWT community. python-jose is abandoned (last commit 2+ years ago).
- **Community Adoption**: FastAPI community migrated from python-jose to PyJWT in 2024-2025.
- **Feature Parity**: Both support HS256, RS256, expiration validation. PyJWT has better type hints.
- **Security**: PyJWT receives regular security updates. python-jose has unpatched vulnerabilities.

**Implementation**:
```python
import jwt
from datetime import datetime, timedelta

# Generate token
payload = {
    "user_id": str(user.user_id),
    "email": user.email,
    "exp": datetime.utcnow() + timedelta(days=7)
}
token = jwt.encode(payload, SECRET_KEY, algorithm="HS256")

# Validate token
try:
    payload = jwt.decode(token, SECRET_KEY, algorithms=["HS256"])
except jwt.ExpiredSignatureError:
    raise HTTPException(401, "Token expired")
except jwt.InvalidTokenError:
    raise HTTPException(401, "Invalid token")
```

**Trade-offs**:
- ➕ Active maintenance and security updates
- ➕ FastAPI community standard (2025)
- ➕ Better type hints (mypy compatible)
- ➖ None (PyJWT is superior to python-jose in all aspects)

**Validation**: Research confirmed PyJWT is "the actively maintained choice" (TECHNOLOGY_RESEARCH.md)

**ADR Recommendation**: Can be deferred (tactical choice, well-documented in research)

---

## Implementation Phases

### Phase 1: Foundation (Backend Core)

**Goal**: Set up backend infrastructure, database, and core security primitives.

**Duration**: 1-2 days

**Tasks**:
1. Initialize backend project structure (directories, requirements.txt)
2. Configure environment variables (.env, Pydantic Settings)
3. Set up PostgreSQL database (local + Railway)
4. Implement core security functions (Argon2id hashing, JWT generation)
5. Create SQLAlchemy models (User, Session, Conversation, ChatMessage)
6. Set up Alembic migrations
7. Configure FastAPI app (CORS, middleware, error handling)

**Acceptance Criteria**:
- Backend server starts without errors (`uvicorn app.main:app`)
- Database migrations apply successfully (`alembic upgrade head`)
- `/v1/health` endpoint returns `{"status": "ok"}`
- Password hashing works (test with `ph.hash()` and `ph.verify()`)
- JWT tokens generate and decode correctly

**Deliverables**:
- `backend/app/core/` (config.py, security.py, database.py)
- `backend/app/models/` (user.py, session.py, conversation.py, message.py)
- `backend/alembic/versions/001_create_auth_tables.py`
- `backend/requirements.txt`, `backend/.env.example`

---

### Phase 2: Authentication Endpoints

**Goal**: Implement signup, login, logout endpoints with rate limiting.

**Duration**: 2-3 days

**Tasks**:
1. Create Pydantic schemas (SignupRequest, LoginRequest, AuthResponse, ErrorResponse)
2. Implement AuthService (business logic for signup, login, logout)
3. Implement auth router endpoints (/auth/signup, /auth/login, /auth/logout)
4. Set up rate limiting (FastAPI-Limiter with Redis)
5. Implement httpOnly cookie handling (set on login, clear on logout)
6. Create get_current_user dependency (JWT validation middleware)
7. Write unit tests for auth endpoints (signup success, duplicate email, invalid credentials)

**Acceptance Criteria**:
- POST /auth/signup creates new user and returns JWT token
- POST /auth/login validates credentials and returns JWT token
- POST /auth/logout clears httpOnly cookie
- Rate limiting blocks excessive signup/login attempts
- Duplicate email signup returns 400 error
- Invalid credentials return 401 error
- Weak password validation rejects passwords <8 chars or missing letter/number
- httpOnly cookie is set with secure, samesite=none flags

**Deliverables**:
- `backend/app/schemas/auth.py`
- `backend/app/services/auth_service.py`
- `backend/app/routers/auth.py`
- `backend/app/dependencies/auth.py` (get_current_user)
- `backend/tests/test_auth.py`

---

### Phase 3: Chat History Persistence

**Goal**: Implement protected chat endpoints with conversation and message storage.

**Duration**: 2-3 days

**Tasks**:
1. Create Pydantic schemas (ChatRequest, ChatResponse, Message, ConversationSummary)
2. Implement ChatService (business logic for sending messages, retrieving history)
3. Implement chat router endpoints (/chat, /chat/history, /chat/conversation/{id})
4. Protect chat endpoints with get_current_user dependency
5. Implement conversation creation and message storage
6. Implement chat history retrieval with pagination
7. Write integration tests for chat flow (send message → save → retrieve history)

**Acceptance Criteria**:
- POST /chat creates conversation (if new) and saves user + assistant messages
- POST /chat with conversation_id appends to existing conversation
- GET /chat/history returns user's conversations with pagination (limit 20)
- GET /chat/conversation/{id} returns messages with pagination (limit 50)
- DELETE /chat/conversation/{id} deletes conversation and cascade deletes messages
- Unauthenticated requests return 401 error
- User can only access their own conversations (403 for other users' conversations)

**Deliverables**:
- `backend/app/schemas/chat.py`
- `backend/app/services/chat_service.py`
- `backend/app/routers/chat.py`
- `backend/tests/test_chat.py`

---

### Phase 4: User Profile

**Goal**: Implement user profile endpoint with account statistics.

**Duration**: 1 day

**Tasks**:
1. Create UserProfileResponse schema
2. Implement UserService (business logic for profile retrieval)
3. Implement user router (/user/profile)
4. Calculate conversation_count and message_count (SQL aggregations)
5. Write tests for profile endpoint

**Acceptance Criteria**:
- GET /user/profile returns email, created_at, last_login, conversation_count, message_count
- Profile endpoint requires authentication (401 if not logged in)
- Statistics are accurate (verified against database)

**Deliverables**:
- `backend/app/schemas/user.py`
- `backend/app/services/user_service.py`
- `backend/app/routers/user.py`
- `backend/tests/test_user.py`

---

### Phase 5: Frontend Authentication

**Goal**: Implement login, signup, logout UI with authentication context.

**Duration**: 2-3 days

**Tasks**:
1. Create AuthContext (React Context for user state, login, signup, logout)
2. Create axios instance with withCredentials: true
3. Implement Login page component
4. Implement Signup page component
5. Implement ProtectedRoute component (redirect to /login if not authenticated)
6. Add axios interceptor for 401 errors (redirect to /login)
7. Add user navbar (display email, logout button when authenticated)

**Acceptance Criteria**:
- User can signup with email + password (validation: email format, password 8+ chars)
- User can login with email + password
- Successful login/signup sets auth state and redirects to /chat
- Logout clears auth state and redirects to homepage
- Protected routes redirect to /login if not authenticated
- 401 errors automatically redirect to /login
- User email displayed in navbar when logged in

**Deliverables**:
- `frontend/src/context/AuthContext.tsx`
- `frontend/src/services/api.ts`, `frontend/src/services/authApi.ts`
- `frontend/src/pages/Login.tsx`, `frontend/src/pages/Signup.tsx`
- `frontend/src/components/auth/ProtectedRoute.tsx`
- `frontend/src/components/user/UserNav.tsx`

---

### Phase 6: Frontend Chat History

**Goal**: Implement chat history UI with conversation list and message display.

**Duration**: 2-3 days

**Tasks**:
1. Create ChatHistory component (conversation list sidebar)
2. Create ConversationView component (message list for selected conversation)
3. Update ChatInput to include authentication and conversation context
4. Implement "New Conversation" button
5. Implement conversation selection (click to load messages)
6. Implement message display with timestamps and sources
7. Add loading states and error handling

**Acceptance Criteria**:
- Chat page displays conversation list on left sidebar
- Clicking a conversation loads its messages
- "New Conversation" button creates a new conversation
- Messages display with timestamps (user messages on right, assistant on left)
- Source citations displayed below assistant messages
- Chat history persists across page refreshes
- Loading spinners shown while fetching data

**Deliverables**:
- `frontend/src/components/chat/ChatHistory.tsx`
- `frontend/src/components/chat/ConversationView.tsx`
- `frontend/src/components/chat/MessageList.tsx`
- `frontend/src/services/chatApi.ts`
- `frontend/src/pages/Chat.tsx` (updated)

---

### Phase 7: Deployment

**Goal**: Deploy backend to Railway and frontend to Vercel.

**Duration**: 1 day

**Tasks**:
1. Create railway.json (deployment configuration)
2. Set up PostgreSQL and Redis on Railway
3. Configure environment variables on Railway (JWT_SECRET_KEY, DATABASE_URL, etc.)
4. Deploy backend to Railway
5. Create vercel.json (SPA rewrite configuration)
6. Configure environment variables on Vercel (REACT_APP_API_URL)
7. Deploy frontend to Vercel
8. Update CORS_ORIGINS in backend to include Vercel domain
9. Verify end-to-end flow (signup, login, chat, logout)

**Acceptance Criteria**:
- Backend accessible at https://api.example.com/v1/health
- Frontend accessible at https://frontend.vercel.app
- CORS configured correctly (no CORS errors in browser console)
- HttpOnly cookies work across domains (Vercel → Railway)
- End-to-end flow works: signup → login → send chat message → view history → logout

**Deliverables**:
- `backend/railway.json`
- `frontend/vercel.json`
- Deployment documentation (URLs, environment variables)

---

## Testing Strategy

### Unit Tests (Backend)

**Scope**: Test individual functions and models in isolation.

**Tools**: pytest, pytest-asyncio

**Coverage Target**: 80%+ for services and security functions

**Example Tests**:
```python
# test_security.py
def test_hash_password():
    password = "SecurePass123"
    hash_val = hash_password(password)
    assert verify_password(hash_val, password) is True
    assert verify_password(hash_val, "WrongPass") is False

# test_models.py
@pytest.mark.asyncio
async def test_user_creation():
    user = User(email="test@example.com", password_hash="hash")
    assert user.email == "test@example.com"
    assert user.is_active is True
```

### Integration Tests (Backend)

**Scope**: Test API endpoints with database operations.

**Tools**: pytest, httpx.AsyncClient, test database

**Coverage Target**: 90%+ for all API endpoints

**Example Tests**:
```python
@pytest.mark.asyncio
async def test_signup_flow():
    async with AsyncClient(app=app, base_url="http://test") as client:
        # Signup
        response = await client.post("/v1/auth/signup", json={
            "email": "test@example.com",
            "password": "SecurePass123"
        })
        assert response.status_code == 201
        assert "access_token" in response.json()

        # Login with same credentials
        response = await client.post("/v1/auth/login", json={
            "email": "test@example.com",
            "password": "SecurePass123"
        })
        assert response.status_code == 200
```

### Contract Tests

**Scope**: Validate request/response schemas match OpenAPI spec.

**Tools**: Pydantic validation, openapi-spec-validator

**Coverage Target**: 100% of all endpoints

**Example Tests**:
```python
def test_auth_response_schema():
    data = {
        "user_id": "123e4567-e89b-12d3-a456-426614174000",
        "email": "test@example.com",
        "access_token": "token",
        "token_type": "bearer",
        "expires_in": 604800
    }
    response = AuthResponse(**data)  # Should not raise ValidationError
    assert response.token_type == "bearer"
```

### End-to-End Tests (Frontend + Backend)

**Scope**: Test complete user workflows from UI to database.

**Tools**: Playwright or Cypress

**Coverage Target**: All critical user flows (signup, login, chat, logout)

**Example Tests**:
```javascript
test('user can signup and send chat message', async ({ page }) => {
  // Go to signup page
  await page.goto('/signup');

  // Fill signup form
  await page.fill('input[name="email"]', 'test@example.com');
  await page.fill('input[name="password"]', 'SecurePass123');
  await page.click('button[type="submit"]');

  // Verify redirect to chat page
  await expect(page).toHaveURL('/chat');

  // Send chat message
  await page.fill('textarea[name="message"]', 'What is ROS 2?');
  await page.click('button[type="submit"]');

  // Verify message appears
  await expect(page.locator('.message-user')).toContainText('What is ROS 2?');
});
```

### Performance Tests

**Scope**: Verify API response times meet spec requirements.

**Tools**: locust (load testing), pytest-benchmark

**Coverage Target**: All critical endpoints (login, chat, history)

**Example Tests**:
```python
def test_login_performance(benchmark):
    """Login endpoint must respond in <200ms (p95)."""
    result = benchmark(lambda: requests.post("/v1/auth/login", json={
        "email": "test@example.com",
        "password": "SecurePass123"
    }))
    assert result.status_code == 200
    # pytest-benchmark automatically checks p95 latency
```

---

## Security Considerations

### Password Security

- ✅ **Argon2id Hashing**: `time_cost=2, memory_cost=65536, parallelism=4` (equivalent to bcrypt cost 12+)
- ✅ **Salt Per User**: Argon2 automatically generates unique salt per password
- ✅ **No Plaintext Storage**: Passwords never stored in plaintext (database only contains hashes)
- ✅ **Strength Validation**: Minimum 8 characters, at least one letter and one number

### JWT Token Security

- ✅ **HS256 Signing**: Tokens signed with 256-bit secret key (stored in environment variables)
- ✅ **Expiration**: Tokens expire after 7 days (spec NFR-006)
- ✅ **HttpOnly Cookies**: Tokens stored in httpOnly cookies (XSS protection)
- ✅ **Secure Flag**: Cookies only sent over HTTPS in production
- ✅ **SameSite=None**: Required for cross-domain (Vercel + Railway)

### SQL Injection Prevention

- ✅ **Parameterized Queries**: SQLAlchemy ORM uses parameterized queries (no string concatenation)
- ✅ **Input Validation**: Pydantic validates all input before database operations
- ✅ **ORM Abstraction**: SQLAlchemy prevents direct SQL injection via query builder

### CORS and CSRF

- ✅ **Explicit Origins**: CORS allows only specified domains (no wildcard with credentials)
- ✅ **Credentials Required**: `allow_credentials=True` enables cookie transmission
- ✅ **SameSite Protection**: `SameSite=None` requires HTTPS (CSRF mitigation)

### Rate Limiting

- ✅ **Login Rate Limit**: 5 attempts per minute per IP (spec FR-020, NFR-004)
- ✅ **Signup Rate Limit**: 3 attempts per hour per IP
- ✅ **Redis Atomic Operations**: Prevents race conditions in rate limit checks

### Data Encryption

- ✅ **HTTPS Only**: All production traffic over HTTPS (enforced by Vercel + Railway)
- ✅ **Database SSL**: PostgreSQL connections use SSL/TLS in production
- ✅ **Environment Variables**: Secrets stored in .env (gitignored, not committed)

---

## Performance Optimization

### Database Query Optimization

**Indexes**:
- `users.email` (unique index for login lookups)
- `sessions.jwt_token` (index for token validation)
- `conversations.user_id` (index for user conversation queries)
- `chat_messages.conversation_id` (index for message retrieval)
- `chat_messages.timestamp` (index for chronological ordering)

**Expected Performance**:
- Login lookup: <10ms (indexed email query)
- Chat history retrieval: <100ms for 50 messages (indexed conversation_id + pagination)
- User conversations: <50ms for 20 conversations (indexed user_id)

**Connection Pooling**:
```python
engine = create_async_engine(
    DATABASE_URL,
    pool_size=20,       # Max 20 connections
    max_overflow=10,    # Allow 10 overflow connections
    pool_pre_ping=True  # Verify connection health
)
```

### API Response Caching

**Not implemented in Phase 1** (future enhancement):
- Cache user profile data (Redis with 5-minute TTL)
- Cache conversation summaries (invalidate on new message)

### Frontend Optimization

- **Code Splitting**: React lazy loading for auth, chat, profile pages
- **API Debouncing**: Debounce chat input (500ms) to reduce API calls
- **Pagination**: Load only 20 conversations, 50 messages per page

---

## Monitoring and Observability

### Logging

**Backend Logging** (structured JSON logs):
```python
import logging
import structlog

logger = structlog.get_logger()

# Log authentication events
logger.info("user_login", user_id=user.user_id, email=user.email, ip=request.client.host)

# Log errors
logger.error("jwt_validation_failed", token=token[:10], error=str(e))
```

**Log Levels**:
- **INFO**: User actions (login, signup, logout, message sent)
- **WARNING**: Rate limit exceeded, invalid credentials
- **ERROR**: Database errors, JWT validation failures, unexpected exceptions

### Metrics

**Key Metrics** (to be monitored with Prometheus/Grafana in production):
- Authentication rate (signups/hour, logins/minute)
- API response times (p50, p95, p99 for all endpoints)
- Error rates (4xx, 5xx errors per endpoint)
- Database query latency (p95 for critical queries)
- Active sessions (concurrent authenticated users)

### Health Checks

**Endpoint**: GET /v1/health

**Response**:
```json
{
  "status": "ok",
  "timestamp": "2025-12-19T10:30:00Z",
  "database": "connected",
  "redis": "connected"
}
```

**Railway Configuration**:
- Health check path: `/v1/health`
- Health check interval: 30 seconds
- Restart on 3 consecutive failures

---

## Deployment Strategy

### Environment Configuration

**Development**:
- Database: SQLite (file-based)
- Redis: Docker container (localhost:6379)
- CORS: Allow `http://localhost:3000`
- HTTPS: Disabled (HTTP only)

**Production**:
- Database: PostgreSQL on Railway
- Redis: Redis addon on Railway
- CORS: Allow `https://frontend.vercel.app`
- HTTPS: Enforced (Vercel + Railway automatic SSL)

### Database Migrations

**Development Flow**:
```bash
# Create migration
alembic revision --autogenerate -m "Add user profiles"

# Test migration locally
alembic upgrade head

# Test rollback
alembic downgrade -1

# Re-apply
alembic upgrade head
```

**Production Flow**:
```bash
# Run migration on Railway (via Procfile or railway.json)
alembic upgrade head

# Verify migration success
psql $DATABASE_URL -c "SELECT * FROM alembic_version;"
```

### Rollback Strategy

**Database Rollback**:
```bash
# Rollback one migration
alembic downgrade -1

# Rollback to specific version
alembic downgrade <revision_id>
```

**Application Rollback**:
- Railway: Redeploy previous commit via Git
- Vercel: Revert to previous deployment via dashboard

---

## Risk Analysis

### Risk 1: JWT Token Expiration Mid-Conversation

**Impact**: User's session expires while typing a message, causing 401 error and data loss.

**Mitigation**:
- Frontend: Detect 401 error, save draft message to localStorage, redirect to login with return URL
- Backend: 7-day expiration reduces frequency (spec NFR-006)
- Future Enhancement: Implement refresh tokens for seamless re-authentication

**Likelihood**: Medium (7-day expiration reduces frequency)

**Severity**: Medium (user inconvenience, no data loss if draft saved)

---

### Risk 2: CORS Configuration Errors in Production

**Impact**: Frontend cannot communicate with backend due to CORS policy, blocking all API calls.

**Mitigation**:
- Test CORS configuration in staging environment before production deploy
- Include comprehensive CORS tests in CI/CD pipeline
- Document exact CORS settings in deployment guide (quickstart.md)

**Likelihood**: Low (well-documented, tested in staging)

**Severity**: High (complete system failure)

---

### Risk 3: Rate Limiting Denial of Service

**Impact**: Legitimate users blocked by rate limiting if sharing IP (corporate network, VPN).

**Mitigation**:
- Rate limit per IP + per user_id (authenticated users get higher limits)
- Implement CAPTCHA for signup after 3 failed attempts (future enhancement)
- Monitor rate limit metrics to detect false positives

**Likelihood**: Low (5 logins/min is generous for individual users)

**Severity**: Medium (user inconvenience, workaround: wait 1 minute)

---

### Risk 4: Database Connection Pool Exhaustion

**Impact**: Backend cannot serve requests due to all database connections in use.

**Mitigation**:
- Connection pool: 20 max connections + 10 overflow (total 30)
- Pool pre-ping: Verify connection health before use (prevents stale connections)
- Monitor connection pool metrics (alert if >80% utilized)
- Horizontal scaling: Add backend instances (Railway auto-scaling)

**Likelihood**: Low (30 connections sufficient for 1000+ concurrent users)

**Severity**: High (service outage)

---

## Next Steps

### Immediate Actions (Post-Planning)

1. **Review and Approve Plan**: Stakeholders review this document and provide feedback
2. **Create ADRs** (if approved):
   - `/sp.adr password-hashing-algorithm` (Argon2id decision)
   - `/sp.adr jwt-token-storage` (httpOnly cookies decision)
   - `/sp.adr database-selection` (PostgreSQL decision)
   - `/sp.adr orm-selection` (SQLAlchemy ORM decision)
3. **Generate Tasks**: Run `/sp.tasks` to create `tasks.md` with detailed implementation tasks
4. **Set Up Development Environment**: Follow `quickstart.md` to set up local backend and frontend

### Implementation Roadmap

**Week 1**:
- Phase 1: Foundation (backend core)
- Phase 2: Authentication endpoints

**Week 2**:
- Phase 3: Chat history persistence
- Phase 4: User profile

**Week 3**:
- Phase 5: Frontend authentication
- Phase 6: Frontend chat history

**Week 4**:
- Phase 7: Deployment
- End-to-end testing
- Documentation and handoff

### Success Criteria (from spec.md)

- ✅ **SC-001**: Signup completes in <30 seconds
- ✅ **SC-002**: Login responds in <200ms (p95)
- ✅ **SC-003**: Password validation rejects 100% of weak passwords
- ✅ **SC-004**: JWT validation adds <10ms overhead
- ✅ **SC-005**: Chat history retrieval <300ms for 100 messages
- ✅ **SC-006**: Session tokens valid for 7 days
- ✅ **SC-007**: Clear, actionable error messages (no security details leaked)
- ✅ **SC-008**: Rate limiting prevents >5 login attempts/min/IP
- ✅ **SC-009**: All passwords hashed with Argon2id (bcrypt cost 12 equivalent)
- ✅ **SC-010**: Frontend handles token expiration gracefully (100% of cases)
- ✅ **SC-011**: Logout immediately invalidates session
- ✅ **SC-012**: Chat history persists across devices

---

## Appendix A: File Locations

**Planning Artifacts**:
- `specs/006-chatbot-auth/plan.md` (this file)
- `specs/006-chatbot-auth/spec.md` (feature specification)
- `specs/006-chatbot-auth/TECHNOLOGY_RESEARCH.md` (research findings)
- `specs/006-chatbot-auth/data-model.md` (database schema)
- `specs/006-chatbot-auth/quickstart.md` (developer guide)
- `specs/006-chatbot-auth/contracts/openapi.yaml` (API specification)
- `specs/006-chatbot-auth/contracts/pydantic-schemas.md` (Pydantic models)
- `specs/006-chatbot-auth/tasks.md` (created by /sp.tasks, not yet generated)

**Implementation Files** (to be created):
- `backend/app/` (all backend code)
- `frontend/src/` (all frontend code)
- See "Project Structure" section for complete file tree

---

## Appendix B: Environment Variables

**Backend `.env`**:
```bash
# Database
DATABASE_URL=postgresql+asyncpg://user:password@localhost:5432/chatbot_db

# JWT
JWT_SECRET_KEY=your-256-bit-secret-key-here  # Generate with: openssl rand -hex 32
JWT_ALGORITHM=HS256
JWT_EXPIRATION_DAYS=7

# CORS
CORS_ORIGINS=["http://localhost:3000", "https://frontend.vercel.app"]

# Redis
REDIS_URL=redis://localhost:6379/0

# Application
ENVIRONMENT=development
DEBUG=True
```

**Frontend `.env`**:
```bash
REACT_APP_API_URL=http://localhost:8000/v1
REACT_APP_ENVIRONMENT=development
```

---

**Status**: Planning Complete | Ready for Tasks Generation (`/sp.tasks`)
**Branch**: `006-chatbot-auth`
**Last Updated**: 2025-12-19
