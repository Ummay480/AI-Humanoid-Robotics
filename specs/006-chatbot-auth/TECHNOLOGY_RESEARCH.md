# Technology Research: Authentication Implementation for FastAPI + React/Docusaurus RAG Chatbot

**Feature**: User Authentication (006-chatbot-auth)
**Date**: 2025-12-19
**Status**: Research Complete

## Executive Summary

This document provides comprehensive research and recommendations for seven critical technical decisions required to implement user authentication in a FastAPI backend with React/Docusaurus frontend, deployed on separate domains (Vercel for frontend, Railway for backend).

**Key Recommendations at a Glance:**
1. **Password Hashing**: Argon2id (modern standard, superior security)
2. **JWT Library**: PyJWT (actively maintained, FastAPI moved away from python-jose)
3. **Database**: PostgreSQL (production-ready, scalable for chat history)
4. **ORM/Database Layer**: SQLAlchemy ORM with asyncpg (balanced performance and productivity)
5. **Rate Limiting**: FastAPI-Limiter (native FastAPI integration, WebSocket support)
6. **Session Storage**: HttpOnly Cookies (superior XSS protection, requires CSRF mitigation)
7. **CORS Configuration**: Explicit origins with `allow_credentials=True`

---

## 1. Password Hashing Library

### Decision: **Use Argon2id**

### Recommendation

**Argon2id** is the recommended password hashing algorithm for new FastAPI applications in 2025. It is the winner of the Password Hashing Competition (2015) and is explicitly recommended by NIST for password hashing.

### Rationale

**Security Properties:**
- **Memory-hard resistance**: Argon2 forces attackers to use massive amounts of memory per password attempt, dramatically reducing GPU effectiveness
- **GPU/ASIC protection**: Highly resistant to GPU and ASIC-based attacks due to memory hardness
- **Configurable parameters**: Fine-grained control over time cost, memory cost, and parallelism
- **Modern standard**: Argon2 has taken bcrypt's place as the modern standard for password security

**Performance Characteristics:**
- Argon2id is superior to bcrypt in terms of speed (faster processing time) and storage
- Can deliver hashing times under 1 second for web applications
- Configurable cost factors allow tuning for specific security/performance requirements

**Ease of Use:**
- Python library: `argon2-cffi` (pure Python with C bindings)
- Simple API similar to bcrypt
- Drop-in replacement for existing password hashing code

### Alternatives Considered

**bcrypt:**
- **Pros**: Proven stability (since 1999), broad platform support, widely understood
- **Cons**:
  - Fixed memory usage (4KB) makes it vulnerable to GPU attacks
  - Performs much better on GPUs, making it susceptible to large-scale brute-force attacks
  - Password truncation at 72 bytes
  - Less resistant to modern hardware attacks

**When to choose bcrypt:**
- Legacy system integration requirements
- Embedded systems with strict resource constraints
- Regulatory requirements explicitly specifying bcrypt
- Gradual migration scenarios (existing bcrypt hashes)

### Trade-offs

**What we gain:**
- Superior protection against GPU/ASIC-based brute-force attacks
- Future-proof security with configurable parameters
- Better resistance to modern attack vectors
- NIST recommendation (compliance-friendly)

**What we sacrifice:**
- Slightly higher memory usage (configurable)
- Less mature ecosystem than bcrypt (though still production-ready)
- May require tuning parameters for optimal performance/security balance

### Implementation Notes

```python
# Install: pip install argon2-cffi

from argon2 import PasswordHasher
from argon2.exceptions import VerifyMismatchError

ph = PasswordHasher(
    time_cost=2,        # Number of iterations
    memory_cost=65536,  # Memory usage in kibibytes (64 MB)
    parallelism=4,      # Number of parallel threads
    hash_len=32,        # Length of hash in bytes
    salt_len=16         # Length of salt in bytes
)

# Hash password
password_hash = ph.hash("user_password")

# Verify password
try:
    ph.verify(password_hash, "user_password")
    # Password is correct
except VerifyMismatchError:
    # Password is incorrect
    pass
```

**Cost Factor for Spec Requirement (NFR-001):**
The spec requires "bcrypt with minimum cost factor of 12". For Argon2id, equivalent security is achieved with:
- `time_cost=2`
- `memory_cost=65536` (64 MB)
- These are the default values in `argon2-cffi` and provide security stronger than bcrypt cost 12

---

## 2. JWT Library

### Decision: **Use PyJWT**

### Recommendation

**PyJWT** is the recommended JWT library for FastAPI applications in 2025. The FastAPI community has officially moved away from python-jose and updated documentation to recommend PyJWT.

### Rationale

**Maintenance Status:**
- **PyJWT**: Actively maintained with regular updates and security patches
- **python-jose**: Barely maintained, last meaningful commit ~1 year ago, considered "nearly abandoned" by the FastAPI community

**FastAPI Integration:**
- PyJWT works as a drop-in replacement for python-jose in FastAPI applications
- FastAPI documentation was updated to recommend PyJWT over python-jose
- Minimal dependencies (relies only on standard Python libraries)
- Lightweight library with minimalistic API focused on simplicity and ease of use

**Security:**
- Active maintenance means timely security patches
- python-jose is "less secure than PyJWT" due to lack of maintenance

### Alternatives Considered

**python-jose:**
- **Pros**:
  - Supports JSON Web Encryption (JWE) for encrypting JWT contents
  - Built-in verification with expiration and audience validation
  - Supports multiple signing algorithms (HMAC, RSA, ECDSA)
- **Cons**:
  - Barely maintained (community consensus)
  - Last release was 3 years ago (though version 3.5.0 released May 2025 shows some activity)
  - Security concerns due to lack of active maintenance
  - FastAPI community explicitly recommends moving away

**authlib:**
- **Pros**: More functionality than PyJWT, actively maintained
- **Cons**: Heavier library with more features than needed for basic JWT auth

### Trade-offs

**What we gain:**
- Active maintenance and security updates
- Lightweight, minimal dependencies
- Community-backed recommendation from FastAPI
- Simple, easy-to-understand API
- Better long-term support and compatibility

**What we sacrifice:**
- No built-in JWE (JSON Web Encryption) support
- Need to manually implement expiration and audience validation (straightforward)
- Fewer built-in features compared to python-jose

### Implementation Notes

```python
# Install: pip install PyJWT

import jwt
from datetime import datetime, timedelta
from typing import Dict

SECRET_KEY = "your-secret-key-min-256-bits"
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_DAYS = 7

def create_access_token(data: Dict, expires_delta: timedelta = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(days=ACCESS_TOKEN_EXPIRE_DAYS)

    to_encode.update({"exp": expire, "iat": datetime.utcnow()})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def verify_token(token: str):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except jwt.ExpiredSignatureError:
        raise ValueError("Token has expired")
    except jwt.InvalidTokenError:
        raise ValueError("Invalid token")
```

**Spec Compliance:**
- Meets NFR-002: "JWT tokens MUST be signed with HS256 algorithm and a secret key of at least 256 bits"
- Supports FR-006: "System MUST issue JWT tokens with configurable expiration (default 7 days) containing user_id and email claims"

---

## 3. Database Choice

### Decision: **Use PostgreSQL for Production**

### Recommendation

**PostgreSQL** is the recommended database for production deployment. SQLite should be used for local development and testing only.

### Rationale

**Production Readiness:**
- PostgreSQL is explicitly recommended for production FastAPI applications
- Robust, ACID-compliant, battle-tested in enterprise environments
- Excellent for production authentication and chat history storage

**Scalability for Chat History:**
- Handles large volumes of chat messages efficiently
- Advanced indexing (B-tree, GiST, GIN) for fast history queries
- Full-text search capabilities for searching chat history (FR-011, User Story 3)
- Supports partitioning for managing growing chat history tables

**Connection Pooling:**
- Native connection pooling with pgBouncer
- Async connection support via asyncpg (2x faster than psycopg2)
- SQLAlchemy 2.0 has robust async connection pooling

**Deployment Ease:**
- Managed PostgreSQL available on Railway, Heroku, AWS RDS, Google Cloud SQL
- Railway specifically provides zero-config PostgreSQL provisioning
- Automated backups and point-in-time recovery

### Alternatives Considered

**SQLite:**
- **Pros**:
  - Single file database (simple deployment)
  - No separate database server needed
  - Python has integrated support
  - Excellent for development and testing
  - Zero configuration
- **Cons**:
  - Not recommended for production with concurrent writes
  - Limited scalability for growing chat history
  - No built-in user management or access control
  - May suffice for early testing or local development only
  - Concurrent write limitations could cause issues with multiple users

**When to use SQLite:**
- Local development environment
- Automated testing (fast, no setup required)
- Personal projects with single users
- Prototyping and proof-of-concept

### Trade-offs

**What we gain:**
- Production-grade reliability and performance
- Scalability for growing user base and chat history
- Advanced features (full-text search, JSON queries, materialized views)
- Better concurrency handling for multiple simultaneous users
- Automated backups and disaster recovery options

**What we sacrifice:**
- Additional operational complexity (database server management)
- Higher deployment costs (managed PostgreSQL services)
- More complex local development setup (requires Docker or local PostgreSQL)
- Need for connection string management and environment configuration

### Implementation Notes

**Development vs Production Strategy:**
```python
# config.py
import os

DATABASE_URL = os.getenv(
    "DATABASE_URL",
    "sqlite:///./dev.db"  # Default for local dev
)

if DATABASE_URL.startswith("postgres://"):
    # Railway and some providers use postgres:// but SQLAlchemy requires postgresql://
    DATABASE_URL = DATABASE_URL.replace("postgres://", "postgresql+asyncpg://", 1)
elif not DATABASE_URL.startswith("sqlite"):
    # Ensure asyncpg for PostgreSQL
    if "postgresql://" in DATABASE_URL:
        DATABASE_URL = DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://", 1)
```

**Connection Pooling Configuration:**
```python
# For PostgreSQL with asyncpg
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker

engine = create_async_engine(
    DATABASE_URL,
    echo=False,
    pool_size=10,           # Max 10 connections in pool
    max_overflow=20,        # Allow 20 additional connections
    pool_pre_ping=True,     # Test connections before using
    pool_recycle=3600       # Recycle connections after 1 hour
)

async_session_maker = sessionmaker(
    engine, class_=AsyncSession, expire_on_commit=False
)
```

**Spec Compliance:**
- Supports all functional requirements (FR-010 through FR-013) for chat history storage
- Meets NFR-012: "Database connection failures MUST return graceful error messages"
- Enables SC-005: "Chat history retrieval for 100 messages completes in under 300ms"

---

## 4. ORM/Database Layer

### Decision: **Use SQLAlchemy ORM with asyncpg**

### Recommendation

**SQLAlchemy 2.0 ORM with asyncpg driver** provides the best balance of developer productivity, maintainability, and performance for the RAG chatbot authentication system.

### Rationale

**Developer Productivity:**
- ORM provides abstraction for complex queries, reducing boilerplate
- Type-safe models with Pydantic integration
- Eliminates need to write repetitive SQL for CRUD operations
- Better code maintainability and readability

**Migration Management:**
- Alembic integration for database schema migrations (required by spec assumption #11)
- Version-controlled database changes
- Rollback capabilities for schema changes
- Essential for evolving authentication and chat history schemas

**Async Support:**
- SQLAlchemy 2.0 has robust async support with asyncpg
- Native async/await syntax throughout application
- Non-blocking database operations for chat streaming

**Performance (2025 Benchmarks):**
- SQLAlchemy 2.0 is "at the front of the pack" for raw database performance with FastAPI
- Enhanced type hinting and modern Python integration
- Asyncpg is the fastest PostgreSQL driver available (uses C implementations)
- Real-world performance is acceptable for most use cases

### Alternatives Considered

**Raw SQL with asyncpg:**
- **Pros**:
  - Absolute best raw performance (3x faster than SQLAlchemy with asyncpg)
  - Direct control over queries
  - No ORM overhead
- **Cons**:
  - No migration management (need custom solution)
  - More boilerplate code for CRUD operations
  - Error-prone (manual SQL construction)
  - Harder to maintain as schema evolves
  - SQL injection risks if not careful with query construction

**When to choose raw asyncpg:**
- Performance is the absolute primary criteria
- Very simple schema that won't change
- Team has strong SQL expertise
- Willing to build custom migration tooling

### Trade-offs

**What we gain:**
- Faster development velocity (ORM abstracts complexity)
- Built-in migration management with Alembic
- Type-safe database operations with Pydantic models
- Easier to refactor and maintain as requirements evolve
- SQL injection prevention built-in (meets NFR-005)
- Better code organization and testability

**What we sacrifice:**
- Some performance overhead (3x slower than raw asyncpg in benchmarks)
- ORM buffers objects in memory (additional memory usage)
- Less granular control over query optimization
- Learning curve for SQLAlchemy 2.0 async patterns

**Performance Context:**
While raw asyncpg is theoretically faster, "the real world implications are debatable." For a chat application with authentication, the performance difference is negligible compared to network latency and LLM processing time. The productivity gains far outweigh the minor performance trade-off.

### Implementation Notes

```python
# models.py - SQLAlchemy models
from sqlalchemy import Column, String, DateTime, Boolean, Text, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.asyncio import AsyncAttrs
from sqlalchemy.orm import DeclarativeBase, relationship
from datetime import datetime
import uuid

class Base(AsyncAttrs, DeclarativeBase):
    pass

class User(Base):
    __tablename__ = "users"

    user_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False, index=True)
    password_hash = Column(String(255), nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    last_login = Column(DateTime, nullable=True)
    is_active = Column(Boolean, default=True, nullable=False)

    # Relationships
    messages = relationship("ChatMessage", back_populates="user", cascade="all, delete-orphan")
    conversations = relationship("Conversation", back_populates="user", cascade="all, delete-orphan")

class ChatMessage(Base):
    __tablename__ = "chat_messages"

    message_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.user_id"), nullable=False, index=True)
    conversation_id = Column(UUID(as_uuid=True), ForeignKey("conversations.conversation_id"), nullable=False, index=True)
    role = Column(String(20), nullable=False)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime, default=datetime.utcnow, nullable=False, index=True)
    page_context = Column(String(500), nullable=True)
    sources = Column(Text, nullable=True)  # JSON string

    # Relationships
    user = relationship("User", back_populates="messages")
    conversation = relationship("Conversation", back_populates="messages")

class Conversation(Base):
    __tablename__ = "conversations"

    conversation_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.user_id"), nullable=False, index=True)
    started_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    last_message_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    title = Column(String(200), nullable=True)
    message_count = Column(Integer, default=0, nullable=False)

    # Relationships
    user = relationship("User", back_populates="conversations")
    messages = relationship("ChatMessage", back_populates="conversation", cascade="all, delete-orphan")
```

**Alembic Migrations:**
```bash
# Initialize Alembic
alembic init alembic

# Create migration
alembic revision --autogenerate -m "Add user authentication tables"

# Apply migration
alembic upgrade head

# Rollback migration
alembic downgrade -1
```

**Spec Compliance:**
- Supports all Key Entities defined in spec (User, ChatMessage, Conversation)
- Meets NFR-005: "SQL injection MUST be prevented via parameterized queries (using SQLAlchemy ORM)"
- Enables assumption #11: "Alembic (SQLAlchemy migration tool) is used for database schema management"

---

## 5. Rate Limiting

### Decision: **Use FastAPI-Limiter**

### Recommendation

**FastAPI-Limiter** is the recommended rate limiting solution due to its native FastAPI dependency injection integration, Redis-backed atomic operations, and WebSocket support.

### Rationale

**Native FastAPI Integration:**
- Uses FastAPI's dependency injection system: `dependencies=[Depends(RateLimiter(times=2, seconds=5))]`
- Follows FastAPI patterns and conventions
- Built specifically for FastAPI (not adapted from Flask)

**Async Support:**
- Fully async with `redis.asyncio` client
- Non-blocking operations suitable for high-concurrency environments
- Native async/await throughout

**Advanced Features:**
- **Redis-backed**: Uses Lua scripts for atomic operations
- **WebSocket support**: Dedicated `WebSocketRateLimiter` for long-lived connections
- **Multiple limiters**: Can apply multiple rate limits on single route
- **Production-ready**: Used in production FastAPI applications

**Configuration:**
- Numeric parameters: `times=5, seconds=60` (clear and explicit)
- Flexible key functions for different rate limiting strategies (by IP, by user, by endpoint)

### Alternatives Considered

**SlowAPI:**
- **Pros**:
  - Decorator-based syntax (`@limiter.limit("5/minute")`)
  - Human-readable limit strings
  - Optional Redis backend (can use in-memory)
  - Adapted from battle-tested flask-limiter
- **Cons**:
  - Alpha quality code (API may change)
  - Decorator syntax less idiomatic for FastAPI
  - Requires explicit `request` argument in endpoints
  - Not fully async (notes mention "may change when making code fully async")
  - No explicit WebSocket support mentioned

**When to choose SlowAPI:**
- Prefer decorator syntax over dependency injection
- Want flexibility between in-memory and Redis
- Coming from Flask background (familiar patterns)
- Need human-readable rate limit strings

### Trade-offs

**What we gain:**
- Tight integration with FastAPI's dependency injection
- Fully async, production-ready implementation
- WebSocket rate limiting (important for streaming chat)
- Atomic Redis operations via Lua scripts (prevents race conditions)
- More FastAPI-idiomatic code

**What we sacrifice:**
- Requires Redis (cannot use in-memory storage)
- Numeric syntax less human-readable than "5/minute" strings
- Slightly more verbose dependency injection syntax
- Less flexibility in storage backend

### Implementation Notes

```python
# Install: pip install fastapi-limiter redis

# main.py - Initialize at startup
from fastapi import FastAPI, Depends
from fastapi_limiter import FastAPILimiter
from fastapi_limiter.depends import RateLimiter
import redis.asyncio as redis

app = FastAPI()

@app.on_event("startup")
async def startup():
    redis_connection = redis.from_url(
        "redis://localhost:6379",
        encoding="utf-8",
        decode_responses=True
    )
    await FastAPILimiter.init(redis_connection)

# Apply rate limiting to auth endpoints
@app.post(
    "/api/auth/login",
    dependencies=[Depends(RateLimiter(times=5, seconds=60))]  # 5 per minute
)
async def login(credentials: LoginRequest):
    # Login logic
    pass

@app.post(
    "/api/auth/signup",
    dependencies=[Depends(RateLimiter(times=3, seconds=3600))]  # 3 per hour
)
async def signup(user_data: SignupRequest):
    # Signup logic
    pass

# WebSocket rate limiting for chat streaming
from fastapi_limiter.depends import WebSocketRateLimiter

@app.websocket("/ws/chat")
async def chat_websocket(
    websocket: WebSocket,
    ratelimit: WebSocketRateLimiter = Depends(WebSocketRateLimiter(times=10, seconds=1))
):
    await websocket.accept()
    # WebSocket chat logic with rate limiting
```

**Custom Key Functions (Rate limit by user instead of IP):**
```python
from fastapi import Request

async def user_rate_limit_key(request: Request):
    # Extract user_id from JWT token
    token = request.headers.get("Authorization")
    user_id = decode_token(token).get("user_id")
    return f"user:{user_id}"

@app.post(
    "/api/chat",
    dependencies=[Depends(RateLimiter(times=100, seconds=60, identifier=user_rate_limit_key))]
)
async def chat(message: ChatRequest):
    # Rate limit per user, not per IP
    pass
```

**Spec Compliance:**
- Meets FR-020: "System MUST implement rate limiting on auth endpoints (5 requests per minute per IP for login, 3 per hour for signup)"
- Meets NFR-004: "Rate limiting MUST prevent more than 5 login attempts per IP per minute and 3 signups per IP per hour"
- Supports SC-008: "Rate limiting prevents more than 5 login attempts per minute per IP address, blocking brute-force attacks"

---

## 6. Session Storage

### Decision: **Use HttpOnly Cookies with CSRF Protection**

### Recommendation

**HttpOnly cookies** with `Secure` and `SameSite` attributes, combined with CSRF token protection, provide the most secure JWT storage solution for 2025.

### Rationale

**XSS Protection (Primary Security Concern):**
- HttpOnly cookies cannot be accessed via JavaScript, providing strong XSS protection
- localStorage is "highly vulnerable to Cross-Site Scripting (XSS) attacks" - if an attacker injects malicious JavaScript, they can easily steal tokens
- "It's harder for the attacker to do the attack when you're using httpOnly cookies"
- The cookie is not accessible via JavaScript; hence, it is not as vulnerable to XSS attacks as localStorage

**CSRF Mitigation:**
- While cookies are vulnerable to CSRF attacks, these can be effectively mitigated with:
  - `SameSite` attribute (prevents cross-site request forgery)
  - Anti-CSRF tokens for state-changing operations
  - Origin validation in backend

**Industry Best Practice (2025):**
- "For maximum security in 2025 and beyond, use HttpOnly cookies with CSRF protection"
- "Although cookies still have some vulnerabilities, it's preferable compared to localStorage whenever possible"
- Current consensus: httpOnly cookies with proper CSRF protection over localStorage for security-sensitive applications

### Alternatives Considered

**localStorage:**
- **Pros**:
  - Simpler implementation (no CORS credentials configuration)
  - No CSRF concerns
  - Cross-device support (can be synced)
  - Persists across browser sessions automatically
- **Cons**:
  - Highly vulnerable to XSS attacks (primary security risk)
  - Any JavaScript code can read tokens
  - If third-party scripts are compromised, tokens are exposed
  - Not recommended for security-sensitive applications

**sessionStorage:**
- **Pros**:
  - Same benefits as localStorage
  - Automatically cleared when browser tab closes (better security than localStorage)
- **Cons**:
  - Same XSS vulnerabilities as localStorage
  - Lost when user closes tab (worse UX)
  - No cross-tab persistence

**In-Memory Storage (Hybrid Approach - Gold Standard):**
- **Pros**:
  - Best security: short-lived access token in memory + refresh token in httpOnly cookie
  - Maximum protection against both XSS and CSRF
- **Cons**:
  - Most complex implementation
  - Token lost on page refresh (need refresh token flow)
  - Marked as "future enhancement" in spec (out of scope for this phase)

### Trade-offs

**What we gain:**
- Superior XSS protection (primary web security threat)
- Cannot be accessed by malicious JavaScript
- Industry best practice for sensitive authentication
- Better compliance with security standards
- Automatic transmission with requests (no manual Authorization header)

**What we sacrifice:**
- Need to implement CSRF protection
- More complex CORS configuration (requires `credentials: 'include'`)
- Cannot access token in JavaScript (need separate endpoint for user info)
- Slightly more complex frontend implementation
- Cross-device support requires backend session tracking (vs localStorage sync)

### Implementation Notes

**Backend (FastAPI):**
```python
from fastapi import Response
from datetime import timedelta

@app.post("/api/auth/login")
async def login(credentials: LoginRequest, response: Response):
    # Verify credentials
    user = authenticate_user(credentials.email, credentials.password)

    # Create JWT token
    access_token = create_access_token(
        data={"user_id": str(user.user_id), "email": user.email},
        expires_delta=timedelta(days=7)
    )

    # Set httpOnly cookie
    response.set_cookie(
        key="access_token",
        value=access_token,
        httponly=True,        # Prevents JavaScript access (XSS protection)
        secure=True,          # HTTPS only (set to False for local dev)
        samesite="lax",       # CSRF protection ("strict" or "lax")
        max_age=7*24*60*60,   # 7 days in seconds
        domain=None,          # Set to your domain in production
    )

    return {"message": "Login successful", "email": user.email}

@app.post("/api/auth/logout")
async def logout(response: Response):
    response.delete_cookie(key="access_token")
    return {"message": "Logged out successfully"}

# Dependency to extract token from cookie
from fastapi import Cookie, HTTPException
from typing import Optional

async def get_current_user(access_token: Optional[str] = Cookie(None)):
    if not access_token:
        raise HTTPException(status_code=401, detail="Authentication required")

    try:
        payload = verify_token(access_token)
        return payload
    except ValueError as e:
        raise HTTPException(status_code=401, detail=str(e))

# Protected route
@app.post("/api/chat")
async def chat(
    message: ChatRequest,
    current_user: dict = Depends(get_current_user)
):
    user_id = current_user["user_id"]
    # Process chat with authenticated user
    pass
```

**Frontend (React/Docusaurus):**
```typescript
// All API requests must include credentials
const API_BASE_URL = process.env.NEXT_PUBLIC_API_URL || "http://localhost:8000";

async function loginUser(email: string, password: string) {
    const response = await fetch(`${API_BASE_URL}/api/auth/login`, {
        method: "POST",
        headers: {
            "Content-Type": "application/json",
        },
        credentials: "include",  // CRITICAL: Send cookies with request
        body: JSON.stringify({ email, password }),
    });

    if (!response.ok) {
        throw new Error("Login failed");
    }

    return await response.json();
}

async function sendChatMessage(message: string) {
    const response = await fetch(`${API_BASE_URL}/api/chat`, {
        method: "POST",
        headers: {
            "Content-Type": "application/json",
        },
        credentials: "include",  // CRITICAL: Send cookies with request
        body: JSON.stringify({ message }),
    });

    if (response.status === 401) {
        // Token expired, redirect to login
        window.location.href = "/login?returnUrl=" + encodeURIComponent(window.location.pathname);
        return;
    }

    return await response.json();
}
```

**CSRF Protection (Additional Security Layer):**
```python
# For state-changing operations (POST, PUT, DELETE), add CSRF token validation
from fastapi import Header, HTTPException

def verify_csrf_token(x_csrf_token: str = Header(None)):
    # Verify CSRF token from header matches session
    # Implementation depends on your session management
    if not x_csrf_token or not is_valid_csrf_token(x_csrf_token):
        raise HTTPException(status_code=403, detail="CSRF token validation failed")

@app.post("/api/auth/signup", dependencies=[Depends(verify_csrf_token)])
async def signup(user_data: SignupRequest):
    # Signup logic with CSRF protection
    pass
```

**Spec Compliance:**
- Addresses assumption #2: "JWT Token Storage: Frontend stores tokens in localStorage (with XSS protection via Content Security Policy). HttpOnly cookies are an alternative for enhanced security but require CORS configuration changes."
- Recommendation: Update spec to mandate httpOnly cookies (superior security)
- Meets NFR-003: "Authentication endpoints MUST use HTTPS in production"
- Supports FR-017: "Frontend MUST handle authentication errors gracefully (401 errors redirect to login, expired token shows clear message)"

---

## 7. CORS Configuration

### Decision: **Explicit Origins with `allow_credentials=True`**

### Recommendation

For a FastAPI backend on Railway and React/Docusaurus frontend on Vercel, configure CORS with explicit allowed origins and `allow_credentials=True` to support httpOnly cookie authentication.

### Rationale

**Security Requirement:**
- When `allow_credentials=True`, `allow_origins` cannot be `["*"]` (wildcard) - it must be explicit
- This is a browser security requirement, not just a FastAPI preference
- "Access-Control-Allow-Credentials indicates whether the browser may send cookies or HTTP-auth headers. It must be true and used together with an explicit (non-*) origin."

**Cookie Authentication:**
- Required for httpOnly cookies to be sent with cross-origin requests
- Without `allow_credentials=True`, browser will not include cookies in requests
- Frontend must use `credentials: 'include'` in fetch requests

**Production Deployment:**
- Frontend on Vercel: `https://yourapp.vercel.app`
- Backend on Railway: `https://yourapi.railway.app`
- These are different origins, requiring CORS configuration

### Implementation Notes

**Backend (FastAPI):**
```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import os

app = FastAPI()

# Environment-based CORS configuration
ENVIRONMENT = os.getenv("ENVIRONMENT", "development")

if ENVIRONMENT == "production":
    # Production: explicit origins only
    allowed_origins = [
        "https://yourapp.vercel.app",
        "https://www.yourapp.com",  # Custom domain if applicable
    ]
else:
    # Development: localhost with various ports
    allowed_origins = [
        "http://localhost:3000",    # Docusaurus dev server
        "http://localhost:3001",
        "http://127.0.0.1:3000",
    ]

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,      # MUST be explicit when using credentials
    allow_credentials=True,             # Required for cookies/auth headers
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization", "X-CSRF-Token"],
    expose_headers=["Content-Length"],
    max_age=3600,                       # Cache preflight requests for 1 hour
)

# IMPORTANT: Add CORS middleware LAST (after other middleware)
# Some developers report middleware order matters
```

**Environment Variables:**
```bash
# .env (development)
ENVIRONMENT=development
FRONTEND_URL=http://localhost:3000

# .env.production (Railway)
ENVIRONMENT=production
FRONTEND_URL=https://yourapp.vercel.app
```

**Dynamic Origin Validation (Advanced):**
```python
# If you have multiple dynamic subdomains
def validate_origin(origin: str) -> bool:
    allowed_domains = ["yourapp.vercel.app", "yourapp.com"]
    return any(origin.endswith(domain) for domain in allowed_domains)

# Custom CORS middleware with dynamic validation
from starlette.middleware.cors import CORSMiddleware as StarletteCorsmiddleware

app.add_middleware(
    StarletteCorsmiddleware,
    allow_origin_regex=r"https://.*\.yourapp\.vercel\.app",  # All Vercel preview deploys
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**Frontend Configuration (React/Docusaurus):**
```typescript
// config.ts
export const API_BASE_URL = process.env.NODE_ENV === 'production'
    ? 'https://yourapi.railway.app'
    : 'http://localhost:8000';

// All API calls must include credentials
fetch(`${API_BASE_URL}/api/chat`, {
    method: 'POST',
    headers: {
        'Content-Type': 'application/json',
    },
    credentials: 'include',  // CRITICAL: This tells browser to send cookies
    body: JSON.stringify(data),
});
```

**Vercel Configuration (vercel.json):**
```json
{
  "headers": [
    {
      "source": "/(.*)",
      "headers": [
        {
          "key": "Access-Control-Allow-Credentials",
          "value": "true"
        }
      ]
    }
  ]
}
```

### Troubleshooting Common Issues

**Issue 1: Cookies Not Being Set**
- Ensure backend sets `secure=True` in production (HTTPS required)
- Ensure `domain` attribute is not set (or matches your domain)
- Check that frontend uses `credentials: 'include'`
- Verify CORS `allow_credentials=True` on backend

**Issue 2: Preflight (OPTIONS) Requests Failing**
- Ensure `allow_methods` includes "OPTIONS"
- Check that `allow_headers` includes all headers your frontend sends
- Verify `allow_origins` is explicit, not `["*"]`

**Issue 3: CORS Errors in Browser Console**
- Backend must respond to preflight requests with proper headers
- Middleware order matters: CORSMiddleware should be added last
- Check browser DevTools Network tab for preflight response headers

### Alternatives Considered

**Allow Origins = ["*"] (Wildcard):**
- **Pros**: Simplest configuration, no environment management
- **Cons**:
  - Cannot use `allow_credentials=True` (browser restriction)
  - No cookie authentication support
  - Security risk: any website can call your API
  - Not suitable for authentication flows

**Proxy Configuration:**
- **Pros**: No CORS issues (same-origin)
- **Cons**:
  - Adds latency (extra hop)
  - More complex deployment
  - Not suitable for separate frontend/backend domains
  - Not recommended for production architectures

### Trade-offs

**What we gain:**
- Secure cookie-based authentication
- Browser-enforced origin validation
- Protection against unauthorized cross-origin requests
- Production-ready setup for separate frontend/backend domains

**What we sacrifice:**
- More complex configuration (environment-specific origins)
- Need to manage allowed origins list
- Preflight requests add ~50ms latency (mitigated by caching)
- Cannot use wildcard origins (must be explicit)

### Spec Compliance

**Addresses Assumption #10:**
> "CORS Configuration: Frontend and backend are on different domains (e.g., frontend on Vercel, backend on Railway), requiring CORS headers with credential support."

**Configuration Summary:**
```python
# Correct CORS configuration for the spec requirements
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://yourapp.vercel.app",  # Production frontend
        "http://localhost:3000"         # Development frontend
    ],
    allow_credentials=True,  # Required for httpOnly cookies
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization", "X-CSRF-Token"],
)
```

---

## Summary of Recommendations

| Decision Area | Recommended Technology | Key Rationale |
|---------------|------------------------|---------------|
| **Password Hashing** | Argon2id | Modern standard, superior GPU resistance, NIST recommended |
| **JWT Library** | PyJWT | Actively maintained, FastAPI community standard, lightweight |
| **Database** | PostgreSQL | Production-ready, scalable for chat history, connection pooling |
| **ORM/Database Layer** | SQLAlchemy ORM + asyncpg | Balanced performance/productivity, Alembic migrations, async support |
| **Rate Limiting** | FastAPI-Limiter | Native FastAPI integration, Redis-backed, WebSocket support |
| **Session Storage** | HttpOnly Cookies | Superior XSS protection, industry best practice with CSRF mitigation |
| **CORS Configuration** | Explicit origins + credentials | Required for secure cookie auth, browser security requirement |

---

## Implementation Priority

Based on dependencies and spec priorities:

1. **Phase 1 - Foundation (P1 - Critical)**
   - Database setup (PostgreSQL with SQLAlchemy)
   - User model and migrations (Alembic)
   - Password hashing (Argon2id)
   - JWT token generation (PyJWT)

2. **Phase 2 - Authentication (P1 - Critical)**
   - Signup endpoint with validation
   - Login endpoint with httpOnly cookies
   - CORS configuration for cross-origin auth
   - JWT verification middleware

3. **Phase 3 - Protection (P1 - Critical)**
   - Protected chat endpoints
   - Rate limiting on auth endpoints (FastAPI-Limiter)
   - Error handling and 401 responses

4. **Phase 4 - Chat History (P2 - Important)**
   - ChatMessage and Conversation models
   - Chat history storage and retrieval
   - History pagination

5. **Phase 5 - User Profile (P3 - Enhancement)**
   - Profile endpoint
   - Logout functionality
   - User statistics

---

## References

### Password Hashing (Argon2 vs bcrypt)
- [Argon2 vs Bcrypt: The Modern Standard for Secure Passwords](https://medium.com/@lastgigin0/argon2-vs-bcrypt-the-modern-standard-for-secure-passwords-6d19911485c5)
- [Best Password Hashing Algorithms 2025: Ultimate Security Guide](https://bellatorcyber.com/blog/best-password-hashing-algorithms-of-2023/)
- [Password Hashing Showdown: Argon2 vs bcrypt vs scrypt vs PBKDF2](https://guptadeepak.com/comparative-analysis-of-password-hashing-algorithms-argon2-bcrypt-scrypt-and-pbkdf2/)
- [Bcrypt vs Argon2: Choosing Strong Hashing Today](https://www.onlinehashcrack.com/guides/password-recovery/bcrypt-vs-argon2-choosing-strong-hashing-today.php)
- [Password Hashing - bcrypt vs Argon2 Security Comparison](https://www.usefulfunctions.co.uk/2025/10/27/bcrypt-vs-argon2-password-security-comparison/)
- [Argon2 vs bcrypt vs. scrypt: which hashing algorithm is right for you?](https://stytch.com/blog/argon2-vs-bcrypt-vs-scrypt/)

### JWT Library (python-jose vs PyJWT)
- [Why python-jose is still recommended in the documentation when it is nearly abandoned - FastAPI Discussion #9587](https://github.com/fastapi/fastapi/discussions/9587)
- [Time to Abandoned Python-Jose Recommendation in Favour of Supported Packages? - FastAPI Discussion #11345](https://github.com/fastapi/fastapi/discussions/11345)
- [PyJWT vs python-jose | What are the differences?](https://stackshare.io/stackups/pypi-pyjwt-vs-pypi-python-jose)

### Database Choice (PostgreSQL vs SQLite)
- [SQL (Relational) Databases - FastAPI](https://fastapi.tiangolo.com/tutorial/sql-databases/)
- [SQLite vs PostgreSQL: How to Choose?](https://chat2db.ai/resources/blog/sqlite-vs-postgresql-choose)
- [Building a Scalable API with FastAPI and PostgreSQL: A 2025 Guide](https://medium.com/@gizmo.codes/building-a-scalable-api-with-fastapi-and-postgresql-a-2025-guide-ca5f3b9cb914)
- [PostgreSQL vs SQLite | Which Relational Databases Wins In 2025?](https://www.selecthub.com/relational-database-solutions/postgresql-vs-sqlite/)
- [PostgreSQL vs SQLite in 2025: Which One Belongs in Production?](https://medium.com/@aayush71727/postgresql-vs-sqlite-in-2025-which-one-belongs-in-production-ddb9815ca5d5)

### ORM vs Raw SQL (SQLAlchemy vs asyncpg)
- [Building High-Performance Async APIs with FastAPI, SQLAlchemy 2.0, and Asyncpg](https://leapcell.io/blog/building-high-performance-async-apis-with-fastapi-sqlalchemy-2-0-and-asyncpg)
- [asyncpg + ORM performance issue - SQLAlchemy Discussion #7294](https://github.com/sqlalchemy/sqlalchemy/discussions/7294)
- [FastAPI Performance Showdown: Sync vs Async — Which is Better?](https://thedkpatel.medium.com/fastapi-performance-showdown-sync-vs-async-which-is-better-77188d5b1e3a)
- [Setting up a FastAPI App with Async SQLAlchemy 2.0 & Pydantic V2](https://medium.com/@tclaitken/setting-up-a-fastapi-app-with-async-sqlalchemy-2-0-pydantic-v2-e6c540be4308)
- [Modern ORM Frameworks in 2025: Django ORM, SQLAlchemy, and Beyond](https://www.nucamp.co/blog/coding-bootcamp-backend-with-python-2025-modern-orm-frameworks-in-2025-django-orm-sqlalchemy-and-beyond)

### Rate Limiting (slowapi vs fastapi-limiter)
- [GitHub - laurentS/slowapi: A rate limiter for Starlette and FastAPI](https://github.com/laurentS/slowapi)
- [SlowApi Documentation](https://slowapi.readthedocs.io/)
- [fastapi-limiter · PyPI](https://pypi.org/project/fastapi-limiter/)
- [GitHub - long2ice/fastapi-limiter: A request rate limiter for fastapi](https://github.com/long2ice/fastapi-limiter)
- [Rate Limiting Strategies in FastAPI: Protecting Your API from Abuse](https://dev.turmansolutions.ai/2025/07/11/rate-limiting-strategies-in-fastapi-protecting-your-api-from-abuse/)
- [Complete FastAPI Performance Tuning Guide](https://blog.greeden.me/en/2025/12/09/complete-fastapi-performance-tuning-guide-build-scalable-apis-with-async-i-o-connection-pools-caching-and-rate-limiting/)

### JWT Storage (localStorage vs httpOnly Cookies)
- [LocalStorage vs Cookies: All You Need To Know About Storing JWT Tokens Securely in The Front-End](https://dev.to/cotter/localstorage-vs-cookies-all-you-need-to-know-about-storing-jwt-tokens-securely-in-the-front-end-15id)
- [The Developer's Guide to JWT Storage](https://www.descope.com/blog/post/developer-guide-jwt-storage)
- [JWT Storage in React: Local Storage vs Cookies Security Battle](https://cybersierra.co/blog/react-jwt-storage-guide/)
- [LocalStorage vs Cookies: the best-practice guide to storing JWT tokens securely in your front-end](https://www.cyberchief.ai/2023/05/secure-jwt-token-storage.html)
- [Front-End Token Storage: Cookies vs Local Storage Guide](https://epigra.com/en/blog/exploring-token-storage-cookies-vs-local-storage-in-front-end-development)
- [Understanding Token Storage: Local Storage vs HttpOnly Cookies](https://www.wisp.blog/blog/understanding-token-storage-local-storage-vs-httponly-cookies)

### CORS Configuration (FastAPI + Vercel + Railway)
- [CORS (Cross-Origin Resource Sharing) - FastAPI](https://fastapi.tiangolo.com/tutorial/cors/)
- [How can I enable CORS on Vercel?](https://vercel.com/kb/guide/how-to-enable-cors)
- [Implementation of CORS in FastAPI](https://jnikenoueba.medium.com/implementation-of-cors-in-fastapi-81510a9625cd)
- [FastAPI: Configuring CORS for Python's ASGI Framework](https://www.stackhawk.com/blog/configuring-cors-in-fastapi/)
- [FastAPI CORS Middleware: How to Allow Cross-Domain Requests Safely](https://sailokesh.hashnode.dev/enable-and-configure-cors-in-fastapi)
- [Understanding and Enabling CORS in FastAPI: A Quick Guide](https://mahdijafaridev.medium.com/understanding-and-enabling-cors-in-fastapi-a-quick-guide-5dd1003300d9)

---

## Next Steps

1. **Review and Approve**: Stakeholders review this research document and approve recommendations
2. **Update Spec**: Update `/mnt/d/aidd/hackathon/specs/006-chatbot-auth/spec.md` with specific technology choices (replace assumption #2 with httpOnly cookies recommendation)
3. **Create Plan**: Generate detailed architectural plan (`plan.md`) based on these technology decisions
4. **Generate Tasks**: Break down implementation into specific, testable tasks (`tasks.md`)
5. **Implement**: Execute tasks following TDD principles (red-green-refactor)

---

**Document Status**: Research Complete
**Next Artifact**: `plan.md` (Architecture and Design)
**Approval Required**: Yes (stakeholder sign-off on technology choices)
