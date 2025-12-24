# Tasks: User Authentication for RAG Chatbot

**Input**: Design documents from `/specs/006-chatbot-auth/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/pydantic-schemas.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This project follows **web app structure** per plan.md:
- Backend: `backend/app/`, `backend/tests/`
- Frontend: `frontend/src/`
- Database migrations: `backend/alembic/versions/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create backend project structure with directories: app/core/, app/models/, app/schemas/, app/routers/, app/services/, app/middleware/, app/dependencies/
- [ ] T002 Create backend/requirements.txt with dependencies: fastapi>=0.115.0, sqlalchemy>=2.0.35, asyncpg>=0.29.0, pyjwt>=2.9.0, argon2-cffi>=23.1.0, fastapi-limiter>=0.1.6, pydantic>=2.0.0, pydantic[email], alembic>=1.13.0, uvicorn[standard]>=0.30.0, redis>=5.0.0
- [ ] T003 [P] Create backend/.env.example with template variables: DATABASE_URL, JWT_SECRET_KEY, JWT_ALGORITHM, JWT_EXPIRATION_DAYS, CORS_ORIGINS, REDIS_URL, ENVIRONMENT
- [ ] T004 [P] Create backend/pytest.ini with pytest configuration for async tests
- [ ] T005 [P] Create backend/alembic.ini for database migrations
- [ ] T006 Create frontend project structure with directories: src/components/auth/, src/components/chat/, src/components/user/, src/context/, src/services/, src/pages/, src/hooks/, src/types/
- [ ] T007 [P] Update frontend/package.json with dependencies: axios>=1.6.0, react-router-dom>=6.0.0
- [ ] T008 [P] Create frontend/.env.example with template: REACT_APP_API_URL, REACT_APP_ENVIRONMENT

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Backend Foundation

- [ ] T009 [P] Implement configuration management in backend/app/core/config.py using Pydantic BaseSettings for environment variables (DATABASE_URL, JWT_SECRET_KEY, CORS_ORIGINS, REDIS_URL)
- [ ] T010 [P] Implement database connection in backend/app/core/database.py with SQLAlchemy async engine, session factory, connection pooling (pool_size=20, max_overflow=10)
- [ ] T011 [P] Implement security primitives in backend/app/core/security.py: hash_password() using Argon2id (time_cost=2, memory_cost=65536, parallelism=4), verify_password(), create_jwt_token(), verify_jwt_token()
- [ ] T012 [P] Create SQLAlchemy Base model in backend/app/models/__init__.py with declarative base and async session utilities
- [ ] T013 [P] Create error response schema in backend/app/schemas/common.py with ErrorResponse Pydantic model (error, detail, timestamp fields)
- [ ] T014 Implement global error handler middleware in backend/app/middleware/error_handler.py to catch HTTPException and return ErrorResponse format
- [ ] T015 [P] Initialize FastAPI app in backend/app/main.py with CORS middleware (allow_credentials=True, explicit origins from config), error handler middleware, health check endpoint /v1/health
- [ ] T016 [P] Setup Redis connection in backend/app/core/database.py for rate limiting with connection pooling
- [ ] T017 [P] Initialize FastAPI-Limiter in backend/app/main.py startup event with Redis connection

### Frontend Foundation

- [ ] T018 [P] Create Axios instance in frontend/src/services/api.ts with baseURL from env, withCredentials: true, request/response interceptors for 401 handling
- [ ] T019 [P] Create TypeScript types in frontend/src/types/auth.ts: User, AuthResponse, SignupRequest, LoginRequest
- [ ] T020 [P] Create TypeScript types in frontend/src/types/chat.ts: ChatRequest, ChatResponse, Message, Conversation, SourceReference
- [ ] T021 [P] Create TypeScript types in frontend/src/types/user.ts: UserProfile
- [ ] T022 [P] Create common components in frontend/src/components/common/ErrorBoundary.tsx and Loading.tsx

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - User Registration and Account Creation (Priority: P1) 🎯 MVP

**Goal**: New users can signup with email/password, creating an account with hashed password, and receive JWT token for immediate login access

**Independent Test**: Navigate to /signup, enter valid email and password (8+ chars with letter+number), submit form. Verify: (1) User record created in database with Argon2id hash, (2) JWT token returned in httpOnly cookie, (3) Redirect to /chat with user email displayed in navbar

**User Story Mapping**:
- **Functional Requirements**: FR-001, FR-002, FR-003, FR-014
- **Success Criteria**: SC-001, SC-003, SC-009
- **Entities**: User
- **Endpoints**: POST /v1/auth/signup
- **UI**: SignupForm, Signup page

### Backend Implementation for User Story 1

- [ ] T023 [P] [US1] Create User SQLAlchemy model in backend/app/models/user.py with columns: user_id (UUID, PK), email (String 255, unique, indexed), password_hash (String 255), created_at (DateTime with timezone), last_login (DateTime nullable), is_active (Boolean default True)
- [ ] T024 [P] [US1] Create SignupRequest Pydantic schema in backend/app/schemas/auth.py with email (EmailStr), password (str, min 8 chars) and password validator (regex for letter+number)
- [ ] T025 [P] [US1] Create AuthResponse Pydantic schema in backend/app/schemas/auth.py with user_id (UUID4), email (str), access_token (str), token_type (str default "bearer"), expires_in (int default 604800)
- [ ] T026 [US1] Implement signup business logic in backend/app/services/auth_service.py: create_user(email, password) function that checks email uniqueness, hashes password with Argon2id, creates User record, generates JWT token, creates Session record, returns AuthResponse
- [ ] T027 [US1] Create auth router in backend/app/routers/auth.py with POST /v1/auth/signup endpoint, rate limit 3 requests/hour/IP (RateLimiter(times=3, hours=1)), calls auth_service.create_user(), sets httpOnly cookie (secure=True, samesite="none", max_age=604800), returns AuthResponse with 201 status
- [ ] T028 [US1] Create Alembic migration in backend/alembic/versions/001_create_users_table.py to create users table with all columns and indexes (idx_users_email, idx_users_created_at)
- [ ] T029 [US1] Register auth router in backend/app/main.py with app.include_router(auth_router)
- [ ] T030 [US1] Test signup endpoint: Create backend/tests/test_auth.py with test_signup_success (valid credentials), test_signup_duplicate_email (400 error), test_signup_weak_password (422 validation error), test_signup_invalid_email (422 error)

### Frontend Implementation for User Story 1

- [ ] T031 [P] [US1] Create signup API function in frontend/src/services/authApi.ts: signup(email, password) that POSTs to /v1/auth/signup with credentials, returns AuthResponse
- [ ] T032 [P] [US1] Create SignupForm component in frontend/src/components/auth/SignupForm.tsx with controlled inputs for email/password, client-side validation (email format, password 8+ chars with letter+number), submit handler calling authApi.signup()
- [ ] T033 [US1] Create Signup page in frontend/src/pages/Signup.tsx that renders SignupForm, handles success (redirect to /chat) and errors (display ErrorResponse.detail)
- [ ] T034 [US1] Add signup route to frontend/src/App.tsx: <Route path="/signup" element={<Signup />} />
- [ ] T035 [US1] Test signup flow: Navigate to /signup, fill form with "test@example.com" and "SecurePass123", submit, verify redirect to /chat (implementation in User Story 3)

**Checkpoint**: User Story 1 complete - users can signup and receive JWT tokens. Independent validation: signup endpoint returns 201, token stored in httpOnly cookie, user record in database.

---

## Phase 4: User Story 2 - User Login and Session Management (Priority: P1)

**Goal**: Returning users can login with email/password, receive JWT token, and logout to invalidate session

**Independent Test**: (1) Login: Navigate to /login, enter registered email/password, submit. Verify JWT token in httpOnly cookie, redirect to /chat, email displayed in navbar. (2) Logout: Click logout button, verify cookie cleared, redirect to homepage, navbar shows login/signup links.

**User Story Mapping**:
- **Functional Requirements**: FR-004, FR-005, FR-006, FR-007, FR-015, FR-016, FR-017, FR-018
- **Success Criteria**: SC-002, SC-006, SC-007, SC-010, SC-011
- **Entities**: Session
- **Endpoints**: POST /v1/auth/login, POST /v1/auth/logout
- **UI**: LoginForm, Login page, AuthContext, UserNav (logout button)

### Backend Implementation for User Story 2

- [ ] T036 [P] [US2] Create Session SQLAlchemy model in backend/app/models/session.py with columns: session_id (UUID, PK), user_id (UUID, FK to users), jwt_token (Text, indexed), issued_at (DateTime with timezone), expires_at (DateTime with timezone, indexed), ip_address (String 45 nullable), user_agent (Text nullable)
- [ ] T037 [P] [US2] Create LoginRequest Pydantic schema in backend/app/schemas/auth.py with email (EmailStr), password (str, min 1 char)
- [ ] T038 [US2] Implement login business logic in backend/app/services/auth_service.py: authenticate_user(email, password) function that queries User by email and is_active=True, verifies password with Argon2id, updates last_login, creates Session record with 7-day expiration, generates JWT token, returns AuthResponse
- [ ] T039 [US2] Add POST /v1/auth/login endpoint to backend/app/routers/auth.py with rate limit 5 requests/minute/IP (RateLimiter(times=5, minutes=1)), calls auth_service.authenticate_user(), sets httpOnly cookie, returns AuthResponse with 200 status
- [ ] T040 [US2] Implement logout business logic in backend/app/services/auth_service.py: revoke_session(user_id) function that deletes Session record(s) for user
- [ ] T041 [US2] Add POST /v1/auth/logout endpoint to backend/app/routers/auth.py that requires authentication (Depends(get_current_user)), calls auth_service.revoke_session(), clears httpOnly cookie (max_age=0), returns 204 No Content
- [ ] T042 [US2] Implement get_current_user dependency in backend/app/dependencies/auth.py that extracts JWT from cookie, validates with verify_jwt_token(), queries User by user_id from payload, raises 401 if token invalid/expired/user not found
- [ ] T043 [US2] Create authentication middleware in backend/app/middleware/auth.py to optionally extract user from JWT cookie and attach to request.state (for endpoints using Depends(get_current_user))
- [ ] T044 [US2] Create Alembic migration in backend/alembic/versions/002_create_sessions_table.py to create sessions table with foreign key to users and cascade delete
- [ ] T045 [US2] Test login/logout endpoints: Add to backend/tests/test_auth.py: test_login_success (valid credentials return token), test_login_invalid_credentials (401 error), test_login_rate_limit (429 after 5 attempts), test_logout_success (204 status, cookie cleared), test_logout_unauthenticated (401 error)

### Frontend Implementation for User Story 2

- [ ] T046 [P] [US2] Create login/logout API functions in frontend/src/services/authApi.ts: login(email, password) POSTs to /v1/auth/login, logout() POSTs to /v1/auth/logout
- [ ] T047 [US2] Create AuthContext in frontend/src/context/AuthContext.tsx with state (user: User | null, isAuthenticated: boolean), actions (login, logout, checkAuth), provider wrapping axios calls to authApi
- [ ] T048 [P] [US2] Create LoginForm component in frontend/src/components/auth/LoginForm.tsx with email/password inputs, submit handler calling login from AuthContext, error display
- [ ] T049 [US2] Create Login page in frontend/src/pages/Login.tsx rendering LoginForm, handling success (redirect to /chat) and errors (display ErrorResponse.detail)
- [ ] T050 [P] [US2] Create UserNav component in frontend/src/components/user/UserNav.tsx displaying user email when authenticated and logout button calling logout from AuthContext
- [ ] T051 [US2] Wrap frontend/src/App.tsx with AuthProvider and add UserNav to layout
- [ ] T052 [US2] Add login route to frontend/src/App.tsx: <Route path="/login" element={<Login />} />
- [ ] T053 [US2] Add 401 interceptor to frontend/src/services/api.ts that catches 401 responses, clears auth state, redirects to /login with return URL preserved
- [ ] T054 [US2] Test login/logout flow: (1) Login with registered user, verify cookie set, redirect to /chat, email in navbar. (2) Click logout, verify cookie cleared, redirect to /, navbar shows login/signup links

**Checkpoint**: User Story 2 complete - users can login, sessions are tracked, logout invalidates tokens. Independent validation: login returns 200 with token, logout returns 204 and clears cookie.

---

## Phase 5: User Story 3 - Authenticated Chat Endpoints (Priority: P1)

**Goal**: Authenticated users can send chat messages, receive RAG responses, and view persistent chat history grouped by conversation

**Independent Test**: (1) Login as user. (2) Send message "What is ROS 2?" to /chat endpoint. Verify: conversation_id created, user message saved, assistant response with sources returned. (3) Call /chat/history endpoint. Verify: conversation appears in list with message_count=2. (4) Call /chat/conversation/{id} endpoint. Verify: both messages returned with timestamps and sources.

**User Story Mapping**:
- **Functional Requirements**: FR-008, FR-009, FR-010, FR-011, FR-012, FR-013, FR-019
- **Success Criteria**: SC-004, SC-005, SC-012
- **Entities**: Conversation, ChatMessage
- **Endpoints**: POST /v1/chat, GET /v1/chat/history, GET /v1/chat/conversation/{id}, DELETE /v1/chat/conversation/{id}
- **UI**: ChatInterface (updated for auth), ChatHistory sidebar, MessageList

### Backend Implementation for User Story 3

- [ ] T055 [P] [US3] Create Conversation SQLAlchemy model in backend/app/models/conversation.py with columns: conversation_id (UUID, PK), user_id (UUID, FK to users, indexed), started_at (DateTime with timezone), last_message_at (DateTime with timezone, indexed), title (String 255 nullable), message_count (Integer default 0)
- [ ] T056 [P] [US3] Create ChatMessage SQLAlchemy model in backend/app/models/message.py with columns: message_id (UUID, PK), conversation_id (UUID, FK to conversations, indexed), user_id (UUID, FK to users, indexed), role (String 20 with CHECK constraint 'user' or 'assistant'), content (Text), timestamp (DateTime with timezone, indexed), page_context (String 500 nullable), sources (JSONB nullable)
- [ ] T057 [P] [US3] Create ChatRequest Pydantic schema in backend/app/schemas/chat.py with message (str, 1-5000 chars), conversation_id (UUID4 optional), page_context (str optional, max 500 chars)
- [ ] T058 [P] [US3] Create ChatResponse Pydantic schema in backend/app/schemas/chat.py with conversation_id (UUID4), message_id (UUID4), role (str default "assistant"), content (str), timestamp (datetime), sources (List[SourceReference] optional)
- [ ] T059 [P] [US3] Create SourceReference Pydantic schema in backend/app/schemas/chat.py with title (str), url (HttpUrl), relevance_score (float 0.0-1.0 optional)
- [ ] T060 [P] [US3] Create ConversationSummary Pydantic schema in backend/app/schemas/chat.py with conversation_id (UUID4), title (str optional), started_at (datetime), last_message_at (datetime), message_count (int >= 0)
- [ ] T061 [P] [US3] Create Message Pydantic schema in backend/app/schemas/chat.py with message_id (UUID4), role (Literal["user", "assistant"]), content (str), timestamp (datetime), page_context (str optional), sources (List[SourceReference] optional)
- [ ] T062 [P] [US3] Create ChatHistoryResponse Pydantic schema in backend/app/schemas/chat.py with conversations (List[ConversationSummary]), total (int >= 0), limit (int), offset (int >= 0)
- [ ] T063 [P] [US3] Create ConversationMessagesResponse Pydantic schema in backend/app/schemas/chat.py with conversation_id (UUID4), messages (List[Message]), total (int >= 0), limit (int), offset (int >= 0)
- [ ] T064 [US3] Implement chat business logic in backend/app/services/chat_service.py: send_message(user_id, message, conversation_id, page_context) function that creates/retrieves Conversation, saves user ChatMessage, calls RAG service (existing rag_service.py), saves assistant ChatMessage with sources, increments message_count, updates last_message_at, returns ChatResponse
- [ ] T065 [US3] Implement get_chat_history(user_id, limit=20, offset=0) in backend/app/services/chat_service.py that queries user's Conversations ordered by last_message_at DESC with pagination, returns ChatHistoryResponse
- [ ] T066 [US3] Implement get_conversation_messages(user_id, conversation_id, limit=50, offset=0) in backend/app/services/chat_service.py that verifies user owns conversation (403 if not), queries ChatMessages ordered by timestamp ASC with pagination, returns ConversationMessagesResponse
- [ ] T067 [US3] Implement delete_conversation(user_id, conversation_id) in backend/app/services/chat_service.py that verifies ownership, deletes Conversation (cascade deletes messages), returns None
- [ ] T068 [US3] Create chat router in backend/app/routers/chat.py with POST /v1/chat endpoint requiring authentication (Depends(get_current_user)), calls chat_service.send_message(), returns ChatResponse with 200 status
- [ ] T069 [P] [US3] Add GET /v1/chat/history endpoint to backend/app/routers/chat.py with query params (limit default 20, offset default 0), requires authentication, calls chat_service.get_chat_history(), returns ChatHistoryResponse
- [ ] T070 [P] [US3] Add GET /v1/chat/conversation/{conversation_id} endpoint to backend/app/routers/chat.py with query params (limit default 50, offset default 0), requires authentication, calls chat_service.get_conversation_messages(), returns ConversationMessagesResponse, handles 403 if user doesn't own conversation
- [ ] T071 [P] [US3] Add DELETE /v1/chat/conversation/{conversation_id} endpoint to backend/app/routers/chat.py requiring authentication, calls chat_service.delete_conversation(), returns 204 No Content
- [ ] T072 [US3] Register chat router in backend/app/main.py with app.include_router(chat_router)
- [ ] T073 [US3] Create Alembic migration in backend/alembic/versions/003_create_conversations_and_messages_tables.py to create conversations and chat_messages tables with foreign keys, indexes, and CHECK constraints
- [ ] T074 [US3] Test chat endpoints: Create backend/tests/test_chat.py with test_send_message_new_conversation (creates conversation), test_send_message_existing_conversation (appends to conversation), test_chat_requires_auth (401 without token), test_get_history (returns paginated conversations), test_get_conversation_messages (returns messages), test_delete_conversation (204 and cascade delete), test_access_other_user_conversation (403 error)

### Frontend Implementation for User Story 3

- [ ] T075 [P] [US3] Create chat API functions in frontend/src/services/chatApi.ts: sendMessage(message, conversationId, pageContext) POSTs to /v1/chat, getChatHistory(limit, offset) GETs from /v1/chat/history, getConversationMessages(conversationId, limit, offset) GETs from /v1/chat/conversation/{id}, deleteConversation(conversationId) DELETEs from /v1/chat/conversation/{id}
- [ ] T076 [US3] Create ChatContext in frontend/src/context/ChatContext.tsx with state (conversations: Conversation[], currentConversation: Conversation | null, messages: Message[]), actions (sendMessage, loadHistory, loadMessages, deleteConversation, createNewConversation)
- [ ] T077 [P] [US3] Update ChatInput component in frontend/src/components/chat/ChatInput.tsx to use ChatContext.sendMessage() with current page context, handle authentication errors
- [ ] T078 [P] [US3] Update MessageList component in frontend/src/components/chat/MessageList.tsx to display messages from ChatContext with timestamps, role-based styling (user messages right, assistant left), source citations for assistant messages
- [ ] T079 [P] [US3] Create ChatHistory component in frontend/src/components/chat/ChatHistory.tsx rendering conversation list sidebar from ChatContext, "New Conversation" button, conversation click handler to load messages
- [ ] T080 [US3] Update ChatInterface in frontend/src/components/chat/ChatInterface.tsx to wrap ChatProvider, render ChatHistory sidebar, MessageList, and ChatInput, load chat history on mount if authenticated
- [ ] T081 [US3] Create ProtectedRoute component in frontend/src/components/auth/ProtectedRoute.tsx that checks AuthContext.isAuthenticated, redirects to /login with return URL if not authenticated
- [ ] T082 [US3] Create Chat page in frontend/src/pages/Chat.tsx rendering ChatInterface
- [ ] T083 [US3] Update frontend/src/App.tsx to wrap Chat route with ProtectedRoute: <Route path="/chat" element={<ProtectedRoute><Chat /></ProtectedRoute>} />
- [ ] T084 [US3] Test chat flow: (1) Login, navigate to /chat. (2) Send message, verify response appears. (3) Refresh page, verify conversation persists. (4) Click conversation in sidebar, verify messages load. (5) Click "New Conversation", verify new conversation created

**Checkpoint**: User Story 3 complete - authenticated users can send messages, view history, and manage conversations. All chat data persists across sessions and devices. Independent validation: /chat returns ChatResponse, /chat/history returns conversations, /chat/conversation/{id} returns messages.

---

## Phase 6: User Profile (Priority: P2)

**Goal**: Authenticated users can view their account profile with email, registration date, and usage statistics (conversation count, message count)

**Independent Test**: Login as user, navigate to /profile. Verify: email displayed, created_at timestamp, last_login timestamp, conversation_count matches database, message_count matches database.

**User Story Mapping**:
- **Functional Requirements**: FR-013
- **Success Criteria**: None specific (enhancement)
- **Entities**: None (aggregates from existing tables)
- **Endpoints**: GET /v1/user/profile
- **UI**: Profile page

### Backend Implementation for User Profile

- [ ] T085 [P] [US4] Create UserProfileResponse Pydantic schema in backend/app/schemas/user.py with user_id (UUID4), email (str), created_at (datetime), last_login (datetime optional), conversation_count (int >= 0), message_count (int >= 0)
- [ ] T086 [US4] Implement get_user_profile(user_id) in backend/app/services/user_service.py that queries User, counts Conversations and ChatMessages for user (SQL aggregation), returns UserProfileResponse
- [ ] T087 [US4] Create user router in backend/app/routers/user.py with GET /v1/user/profile endpoint requiring authentication (Depends(get_current_user)), calls user_service.get_user_profile(), returns UserProfileResponse
- [ ] T088 [US4] Register user router in backend/app/main.py
- [ ] T089 [US4] Test profile endpoint: Create backend/tests/test_user.py with test_get_profile_success (returns UserProfileResponse with accurate counts), test_get_profile_unauthenticated (401 error)

### Frontend Implementation for User Profile

- [ ] T090 [P] [US4] Create user API function in frontend/src/services/userApi.ts: getUserProfile() GETs from /v1/user/profile
- [ ] T091 [P] [US4] Create UserProfile component in frontend/src/components/user/UserProfile.tsx displaying email, created_at (formatted), last_login (formatted), conversation_count, message_count
- [ ] T092 [US4] Create Profile page in frontend/src/pages/Profile.tsx that calls userApi.getUserProfile() on mount, renders UserProfile component
- [ ] T093 [US4] Add profile route to frontend/src/App.tsx: <Route path="/profile" element={<ProtectedRoute><Profile /></ProtectedRoute>} />
- [ ] T094 [US4] Add "Profile" link to frontend/src/components/user/UserNav.tsx navigation menu
- [ ] T095 [US4] Test profile page: Login, click Profile link, verify all fields display correctly, verify counts match database

**Checkpoint**: User Profile complete - users can view account information and usage statistics.

---

## Phase 7: Deployment & Production Configuration

**Goal**: Deploy backend to Railway with PostgreSQL and Redis, deploy frontend to Vercel, configure CORS for cross-domain authentication

**Independent Test**: (1) Backend health check accessible at https://api.example.com/v1/health returns 200. (2) Frontend accessible at https://frontend.vercel.app. (3) Complete signup → login → send chat → logout flow works across production domains without CORS errors.

### Backend Deployment

- [ ] T096 [P] Create backend/railway.json with build command: pip install -r requirements.txt, start command: alembic upgrade head && uvicorn app.main:app --host 0.0.0.0 --port $PORT
- [ ] T097 [P] Create backend/Dockerfile (optional, for Docker deployment): FROM python:3.11-slim, COPY requirements.txt, RUN pip install, COPY app/, CMD alembic upgrade head && uvicorn app.main:app
- [ ] T098 Setup PostgreSQL database on Railway: Create PostgreSQL addon, copy DATABASE_URL to environment variables
- [ ] T099 Setup Redis on Railway: Create Redis addon, copy REDIS_URL to environment variables
- [ ] T100 Configure backend environment variables on Railway: JWT_SECRET_KEY (generate with openssl rand -hex 32), CORS_ORIGINS=["https://frontend.vercel.app"], DATABASE_URL, REDIS_URL, ENVIRONMENT=production
- [ ] T101 Deploy backend to Railway: Connect GitHub repo, select backend directory, set build/start commands, deploy
- [ ] T102 Run database migrations on Railway: Execute alembic upgrade head via Railway shell or deployment script
- [ ] T103 Test backend deployment: Verify /v1/health returns {"status": "ok", "database": "connected", "redis": "connected"} at production URL

### Frontend Deployment

- [ ] T104 [P] Create frontend/vercel.json with SPA rewrites: {"rewrites": [{"source": "/(.*)", "destination": "/index.html"}]}
- [ ] T105 Configure frontend environment variables on Vercel: REACT_APP_API_URL=https://api.example.com/v1
- [ ] T106 Deploy frontend to Vercel: Connect GitHub repo, select frontend directory, set build command: npm run build, deploy
- [ ] T107 Update CORS_ORIGINS in backend Railway config: Add production Vercel domain to allowed origins list
- [ ] T108 Test cross-domain authentication: Signup from Vercel frontend → verify httpOnly cookie set → verify subsequent API calls include credentials → verify logout clears cookie
- [ ] T109 Test end-to-end production flow: Signup → Login → Send chat message → View history → View profile → Logout → verify all features work without CORS errors

**Checkpoint**: Deployment complete - production application fully functional across Railway (backend) and Vercel (frontend) with cross-domain authentication working.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T110 [P] Add loading states to frontend forms: SignupForm, LoginForm, ChatInput (disable submit button, show spinner while request pending)
- [ ] T111 [P] Add error toast notifications in frontend using react-toastify or similar for user-friendly error display across all forms
- [ ] T112 [P] Add form reset on success: Clear signup/login forms after successful submission and redirect
- [ ] T113 [P] Add input validation indicators in frontend: Show green checkmark for valid email, red X for invalid password strength in real-time
- [ ] T114 [P] Add password visibility toggle in signup/login forms
- [ ] T115 Add conversation title auto-generation in backend/app/services/chat_service.py: Set title to first 50 chars of first user message if title is NULL
- [ ] T116 [P] Add pagination controls to frontend/src/components/chat/ChatHistory.tsx for conversation list (load more button)
- [ ] T117 [P] Add pagination controls to frontend/src/components/chat/MessageList.tsx for messages (load more button for older messages)
- [ ] T118 Add cleanup job for expired sessions: Create backend/scripts/cleanup_sessions.py to delete sessions where expires_at < NOW(), schedule with cron or Railway scheduler
- [ ] T119 [P] Add logging for authentication events in backend: Log signup (user_id, email), login (user_id, ip_address), logout, failed login attempts (email, ip_address)
- [ ] T120 [P] Add logging for chat events in backend: Log message sent (user_id, conversation_id, message_id), conversation created, conversation deleted
- [ ] T121 [P] Update backend/.env.example and frontend/.env.example with production values and deployment instructions
- [ ] T122 [P] Create backend/README.md with setup instructions: Install dependencies, configure .env, run migrations, start server, run tests
- [ ] T123 [P] Create frontend/README.md with setup instructions: Install dependencies, configure .env, start dev server, build for production
- [ ] T124 Run quickstart.md validation: Follow quickstart guide from scratch on clean environment, verify all steps work, update any outdated instructions
- [ ] T125 Run full test suite: backend pytest (verify all tests pass), frontend tests if added, manual smoke tests for all user stories
- [ ] T126 Security audit: Review CORS config (no wildcard with credentials), verify httpOnly cookies enabled in production, verify JWT secret is 256-bit, verify Argon2id parameters match spec, verify rate limiting active
- [ ] T127 Performance testing: Measure login response time (target <200ms p95), signup response time (target <500ms p95), chat history retrieval (target <300ms for 50 messages), JWT validation overhead (target <10ms)
- [ ] T128 Create deployment documentation in specs/006-chatbot-auth/deployment.md: Railway setup, Vercel setup, environment variables, database migrations, monitoring, rollback procedures

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup (Phase 1) - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational (Phase 2) - signup functionality
- **User Story 2 (Phase 4)**: Depends on Foundational (Phase 2) and User Story 1 (Phase 3) - login requires users to exist
- **User Story 3 (Phase 5)**: Depends on Foundational (Phase 2) and User Story 2 (Phase 4) - chat requires authentication
- **User Profile (Phase 6)**: Depends on User Story 3 (Phase 5) - profile displays conversation/message counts
- **Deployment (Phase 7)**: Depends on all desired user stories being complete
- **Polish (Phase 8)**: Depends on core features being deployed

### User Story Dependencies

- **User Story 1 (Signup)**: Independent after Foundational phase
- **User Story 2 (Login/Logout)**: Depends on User Story 1 (requires users table and User model)
- **User Story 3 (Authenticated Chat)**: Depends on User Story 2 (requires authentication system)
- **User Profile**: Depends on User Story 3 (requires chat data for statistics)

### Within Each User Story

- Backend tasks before frontend tasks (API must exist before UI calls it)
- Models before services (services depend on models)
- Schemas before endpoints (endpoints use schemas for validation)
- Services before endpoints (endpoints call services)
- Database migrations after models (migrations create tables for models)
- Tests after implementation (tests verify implementation works)

### Parallel Opportunities

**Setup Phase**:
- T003, T004, T005 (backend config files) can run in parallel
- T007, T008 (frontend config files) can run in parallel
- Backend setup and frontend setup can run in parallel

**Foundational Phase**:
- T009, T010, T011, T012, T013, T016 (all backend foundation) can run in parallel
- T018, T019, T020, T021, T022 (all frontend foundation) can run in parallel
- Backend and frontend foundation can run in parallel

**User Story 1 (Signup)**:
- Backend: T023, T024, T025 (models and schemas) can run in parallel
- Frontend: T031, T032 (API client and form component) can run in parallel
- Backend and frontend work can proceed in parallel after backend endpoints are complete

**User Story 2 (Login/Logout)**:
- Backend: T036, T037 (Session model and LoginRequest schema) can run in parallel
- Frontend: T048 (LoginForm), T050 (UserNav) can run in parallel after API endpoints exist

**User Story 3 (Authenticated Chat)**:
- Backend: T055, T056 (Conversation and ChatMessage models) can run in parallel
- Backend: T057-T063 (all Pydantic schemas) can run in parallel
- Backend: T069, T070, T071 (GET /chat/history, GET /chat/conversation/{id}, DELETE /chat/conversation/{id}) can run in parallel after service layer complete
- Frontend: T077, T078, T079 (ChatInput, MessageList, ChatHistory components) can run in parallel after ChatContext created

**User Profile**:
- Backend: T085 (schema) can run in parallel with T086 (service)
- Frontend: T090, T091 (API client and component) can run in parallel

**Deployment**:
- T096, T097 (Railway and Docker config) can run in parallel
- T104 (Vercel config) can run independently
- Backend deployment and frontend deployment can proceed in parallel

**Polish Phase**:
- T110-T114 (all frontend UI improvements) can run in parallel
- T116, T117 (pagination controls) can run in parallel
- T119, T120 (all logging additions) can run in parallel
- T121, T122, T123 (all documentation) can run in parallel

---

## Parallel Example: User Story 1 (Signup)

### Backend Tasks (Sequential groups with internal parallelism)

```bash
# Group 1: Models and schemas (parallel)
Task T023: "Create User SQLAlchemy model in backend/app/models/user.py"
Task T024: "Create SignupRequest Pydantic schema in backend/app/schemas/auth.py"
Task T025: "Create AuthResponse Pydantic schema in backend/app/schemas/auth.py"

# Group 2: Service layer (sequential, depends on Group 1)
Task T026: "Implement signup business logic in backend/app/services/auth_service.py"

# Group 3: Router and migration (parallel, depends on Group 2)
Task T027: "Create auth router with POST /v1/auth/signup endpoint"
Task T028: "Create Alembic migration 001_create_users_table.py"

# Group 4: Registration and tests (sequential, depends on Group 3)
Task T029: "Register auth router in backend/app/main.py"
Task T030: "Test signup endpoint in backend/tests/test_auth.py"
```

### Frontend Tasks (Sequential groups with internal parallelism)

```bash
# Group 1: API client and form component (parallel)
Task T031: "Create signup API function in frontend/src/services/authApi.ts"
Task T032: "Create SignupForm component in frontend/src/components/auth/SignupForm.tsx"

# Group 2: Page and routing (sequential, depends on Group 1)
Task T033: "Create Signup page in frontend/src/pages/Signup.tsx"
Task T034: "Add signup route to frontend/src/App.tsx"

# Group 3: Testing (sequential, depends on Group 2)
Task T035: "Test signup flow end-to-end"
```

---

## Implementation Strategy

### MVP First (User Stories 1-3 Only)

1. **Phase 1**: Setup (T001-T008) → Project structure ready
2. **Phase 2**: Foundational (T009-T022) → Foundation ready for all stories
3. **Phase 3**: User Story 1 - Signup (T023-T035) → Users can register
4. **Phase 4**: User Story 2 - Login/Logout (T036-T054) → Users can authenticate
5. **Phase 5**: User Story 3 - Authenticated Chat (T055-T084) → Complete MVP!
6. **STOP and VALIDATE**: Test all three user stories independently and together
7. **Phase 7**: Deploy MVP (T096-T109) → Production ready

**MVP Scope**: 84 tasks (T001-T084) deliver complete authentication + chat system

### Incremental Delivery

1. **Week 1**: Setup + Foundational + User Story 1 → Signup working (T001-T035)
   - **Demo**: Show user signup, JWT token issuance, database record creation
2. **Week 2**: User Story 2 → Login/Logout working (T036-T054)
   - **Demo**: Show login, session management, logout, httpOnly cookies
3. **Week 3**: User Story 3 → Authenticated chat working (T055-T084)
   - **Demo**: Show chat messages, conversation history, persistence across sessions
4. **Week 4**: User Profile + Deployment + Polish (T085-T128)
   - **Demo**: Show profile page, production deployment, complete end-to-end flow

### Parallel Team Strategy

With 3 developers after Foundational phase completes:

- **Developer A**: User Story 1 (Signup) - Backend + Frontend
- **Developer B**: User Story 2 (Login/Logout) - Backend + Frontend (starts after US1 backend models exist)
- **Developer C**: User Story 3 (Authenticated Chat) - Backend + Frontend (starts after US2 auth system exists)

Alternatively, with 2 developers:
- **Backend Developer**: All backend tasks sequentially (US1 → US2 → US3)
- **Frontend Developer**: All frontend tasks sequentially (US1 → US2 → US3), starting after each backend endpoint is ready

---

## Task Summary

**Total Tasks**: 128

**By Phase**:
- Phase 1 (Setup): 8 tasks
- Phase 2 (Foundational): 14 tasks
- Phase 3 (User Story 1 - Signup): 13 tasks
- Phase 4 (User Story 2 - Login/Logout): 19 tasks
- Phase 5 (User Story 3 - Authenticated Chat): 30 tasks
- Phase 6 (User Profile): 11 tasks
- Phase 7 (Deployment): 14 tasks
- Phase 8 (Polish): 19 tasks

**By Type**:
- Backend tasks: ~70 (models, services, endpoints, migrations, tests)
- Frontend tasks: ~45 (components, pages, API clients, context, routing)
- Deployment tasks: ~13 (Railway, Vercel, config, testing)

**Parallel Tasks**: 48 tasks marked with [P] can run in parallel within their phase

**MVP Scope**: Tasks T001-T084 (84 tasks) deliver complete authentication + chat system ready for deployment

**Independent Test Points**: Each user story has clear independent test criteria to validate completion before moving to next story

---

## Notes

- All tasks follow strict checklist format: `- [ ] [ID] [P?] [Story] Description with file path`
- [P] marker indicates parallelizable tasks (different files, no dependencies)
- [Story] label (US1, US2, US3, US4) maps tasks to user stories for traceability
- Each user story is independently testable and deliverable
- MVP = User Stories 1-3 (signup, login, authenticated chat)
- User Story 4 (profile) is enhancement (Priority P2)
- All file paths follow web app structure: backend/ and frontend/ directories
- Database migrations use Alembic with sequential versioning (001, 002, 003)
- Rate limiting uses FastAPI-Limiter with Redis
- Authentication uses JWT tokens in httpOnly cookies
- Password hashing uses Argon2id (superior to bcrypt per plan.md)
- Tests are included but optional (can be skipped for faster MVP delivery)
