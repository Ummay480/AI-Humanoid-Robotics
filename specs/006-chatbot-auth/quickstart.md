# Quickstart Guide: User Authentication Implementation

**Feature**: 006-chatbot-auth
**Date**: 2025-12-19
**Audience**: Developers implementing the authentication system

## Overview

This guide provides step-by-step instructions to implement user authentication in the RAG chatbot application. Follow this guide to set up the backend (FastAPI + PostgreSQL) and frontend (React/Docusaurus) with JWT-based authentication.

**Estimated Time**: 4-6 hours for complete implementation

---

## Prerequisites

### Required Software

- **Python**: 3.11+ (recommended 3.12)
- **PostgreSQL**: 14+ (or Docker container)
- **Node.js**: 18+ (for frontend)
- **npm or yarn**: Latest version
- **Git**: For version control

### Required Knowledge

- Python async/await patterns
- FastAPI framework basics
- React hooks (useState, useEffect, useContext)
- HTTP authentication concepts (JWT, cookies)
- PostgreSQL basics (tables, foreign keys, indexes)

### Environment Setup

**Backend `.env` file**:

```bash
# Database
DATABASE_URL=postgresql+asyncpg://user:password@localhost:5432/chatbot_db

# JWT Secret (generate with: openssl rand -hex 32)
JWT_SECRET_KEY=your-256-bit-secret-key-here
JWT_ALGORITHM=HS256
JWT_EXPIRATION_DAYS=7

# CORS
CORS_ORIGINS=["http://localhost:3000", "https://your-frontend.vercel.app"]

# Rate Limiting (Redis)
REDIS_URL=redis://localhost:6379/0

# Application
ENVIRONMENT=development
DEBUG=True
```

**Frontend `.env` file**:

```bash
REACT_APP_API_URL=http://localhost:8000/v1
REACT_APP_ENVIRONMENT=development
```

---

## Phase 1: Database Setup (30 minutes)

### Step 1.1: Install PostgreSQL

**Option A: Local Installation**

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install postgresql postgresql-contrib

# macOS
brew install postgresql@14
brew services start postgresql@14

# Windows
# Download installer from https://www.postgresql.org/download/windows/
```

**Option B: Docker Container**

```bash
docker run --name chatbot-postgres \
  -e POSTGRES_USER=chatbot_user \
  -e POSTGRES_PASSWORD=secure_password \
  -e POSTGRES_DB=chatbot_db \
  -p 5432:5432 \
  -d postgres:14-alpine
```

### Step 1.2: Create Database

```bash
# Connect to PostgreSQL
psql -U postgres

# Create database and user
CREATE DATABASE chatbot_db;
CREATE USER chatbot_user WITH ENCRYPTED PASSWORD 'secure_password';
GRANT ALL PRIVILEGES ON DATABASE chatbot_db TO chatbot_user;

# Exit psql
\q
```

### Step 1.3: Verify Connection

```bash
psql -U chatbot_user -d chatbot_db -h localhost

# You should see:
# chatbot_db=>
```

---

## Phase 2: Backend Implementation (2-3 hours)

### Step 2.1: Install Dependencies

Create `requirements.txt`:

```text
fastapi==0.115.0
uvicorn[standard]==0.31.0
sqlalchemy[asyncio]==2.0.35
asyncpg==0.29.0
alembic==1.13.3
pydantic[email]==2.9.2
pydantic-settings==2.5.2
argon2-cffi==23.1.0
pyjwt==2.9.0
python-multipart==0.0.17
fastapi-limiter==0.1.6
redis==5.1.1
python-dotenv==1.0.1
pytest==8.3.3
pytest-asyncio==0.24.0
httpx==0.27.2
```

Install packages:

```bash
cd backend
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

### Step 2.2: Project Structure

```bash
mkdir -p backend/app/{models,schemas,routers,services,middleware,core}
mkdir -p backend/alembic/versions
mkdir -p backend/tests
```

**Final structure**:

```
backend/
├── alembic/
│   ├── versions/
│   └── env.py
├── app/
│   ├── core/
│   │   ├── __init__.py
│   │   ├── config.py          # Settings (Pydantic BaseSettings)
│   │   ├── security.py        # Password hashing, JWT functions
│   │   └── database.py        # SQLAlchemy engine, session
│   ├── models/
│   │   ├── __init__.py
│   │   ├── user.py            # User SQLAlchemy model
│   │   ├── session.py         # Session model
│   │   ├── conversation.py    # Conversation model
│   │   └── message.py         # ChatMessage model
│   ├── schemas/
│   │   ├── __init__.py
│   │   ├── auth.py            # SignupRequest, LoginRequest, AuthResponse
│   │   ├── chat.py            # ChatRequest, ChatResponse, Message
│   │   └── user.py            # UserProfileResponse
│   ├── routers/
│   │   ├── __init__.py
│   │   ├── auth.py            # /auth/* endpoints
│   │   ├── chat.py            # /chat/* endpoints
│   │   └── user.py            # /user/* endpoints
│   ├── services/
│   │   ├── __init__.py
│   │   ├── auth_service.py    # Business logic for auth
│   │   ├── chat_service.py    # Business logic for chat
│   │   └── rag_service.py     # RAG integration (existing)
│   ├── middleware/
│   │   ├── __init__.py
│   │   └── auth.py            # JWT validation middleware
│   └── main.py                # FastAPI app initialization
├── tests/
│   ├── test_auth.py
│   ├── test_chat.py
│   └── conftest.py
├── alembic.ini
├── .env
└── requirements.txt
```

### Step 2.3: Core Configuration

**`app/core/config.py`**:

```python
from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import List

class Settings(BaseSettings):
    # Database
    database_url: str

    # JWT
    jwt_secret_key: str
    jwt_algorithm: str = "HS256"
    jwt_expiration_days: int = 7

    # CORS
    cors_origins: List[str] = ["http://localhost:3000"]

    # Redis
    redis_url: str = "redis://localhost:6379/0"

    # Application
    environment: str = "development"
    debug: bool = True

    model_config = SettingsConfigDict(env_file=".env", case_sensitive=False)

settings = Settings()
```

**`app/core/security.py`**:

```python
from argon2 import PasswordHasher
from argon2.exceptions import VerifyMismatchError
from datetime import datetime, timedelta
from typing import Optional
import jwt
from app.core.config import settings

ph = PasswordHasher(
    time_cost=2,
    memory_cost=65536,
    parallelism=4,
    hash_len=32,
    salt_len=16
)

def hash_password(password: str) -> str:
    """Hash a password using Argon2id."""
    return ph.hash(password)

def verify_password(password_hash: str, password: str) -> bool:
    """Verify a password against its hash."""
    try:
        ph.verify(password_hash, password)
        return True
    except VerifyMismatchError:
        return False

def create_access_token(user_id: str, email: str) -> str:
    """Create a JWT access token."""
    expires_at = datetime.utcnow() + timedelta(days=settings.jwt_expiration_days)
    payload = {
        "user_id": user_id,
        "email": email,
        "iat": datetime.utcnow(),
        "exp": expires_at
    }
    return jwt.encode(payload, settings.jwt_secret_key, algorithm=settings.jwt_algorithm)

def decode_access_token(token: str) -> Optional[dict]:
    """Decode and validate a JWT token."""
    try:
        payload = jwt.decode(token, settings.jwt_secret_key, algorithms=[settings.jwt_algorithm])
        return payload
    except jwt.ExpiredSignatureError:
        return None  # Token expired
    except jwt.InvalidTokenError:
        return None  # Invalid token
```

**`app/core/database.py`**:

```python
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.orm import declarative_base
from app.core.config import settings

# Create async engine
engine = create_async_engine(
    settings.database_url,
    echo=settings.debug,
    pool_size=20,
    max_overflow=10,
    pool_pre_ping=True
)

# Session factory
AsyncSessionLocal = async_sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False
)

# Base class for models
Base = declarative_base()

async def get_db():
    """Dependency for getting database session."""
    async with AsyncSessionLocal() as session:
        yield session
```

### Step 2.4: Database Models

**`app/models/user.py`**:

```python
from sqlalchemy import Column, String, DateTime, Boolean
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
import uuid
from app.core.database import Base

class User(Base):
    __tablename__ = "users"

    user_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False, index=True)
    password_hash = Column(String(255), nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    last_login = Column(DateTime(timezone=True), nullable=True)
    is_active = Column(Boolean, default=True, nullable=False)

    # Relationships
    conversations = relationship("Conversation", back_populates="user", cascade="all, delete-orphan")
    sessions = relationship("Session", back_populates="user", cascade="all, delete-orphan")
    messages = relationship("ChatMessage", back_populates="user", cascade="all, delete-orphan")
```

**(Create similar models for Session, Conversation, ChatMessage following data-model.md)**

### Step 2.5: Database Migrations

**Initialize Alembic**:

```bash
alembic init alembic
```

**Edit `alembic.ini`**:

```ini
# Replace this line:
sqlalchemy.url = driver://user:pass@localhost/dbname

# With:
sqlalchemy.url = postgresql+asyncpg://user:password@localhost:5432/chatbot_db
```

**Edit `alembic/env.py`**:

```python
from app.core.database import Base
from app.models.user import User
from app.models.session import Session
from app.models.conversation import Conversation
from app.models.message import ChatMessage

target_metadata = Base.metadata
```

**Create migration**:

```bash
alembic revision --autogenerate -m "Create users, sessions, conversations, and chat_messages tables"
```

**Apply migration**:

```bash
alembic upgrade head
```

### Step 2.6: Authentication Router

**`app/routers/auth.py`**:

```python
from fastapi import APIRouter, Depends, HTTPException, status, Response
from fastapi_limiter.depends import RateLimiter
from sqlalchemy.ext.asyncio import AsyncSession
from app.core.database import get_db
from app.schemas.auth import SignupRequest, LoginRequest, AuthResponse
from app.services.auth_service import AuthService

router = APIRouter(prefix="/auth", tags=["Authentication"])

@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED, dependencies=[Depends(RateLimiter(times=3, hours=1))])
async def signup(request: SignupRequest, response: Response, db: AsyncSession = Depends(get_db)):
    """Register a new user account."""
    service = AuthService(db)
    auth_response = await service.signup(request.email, request.password)

    # Set httpOnly cookie
    response.set_cookie(
        key="access_token",
        value=auth_response.access_token,
        httponly=True,
        secure=True,  # HTTPS only in production
        samesite="none",  # Required for cross-domain
        max_age=604800  # 7 days
    )

    return auth_response

@router.post("/login", response_model=AuthResponse, dependencies=[Depends(RateLimiter(times=5, minutes=1))])
async def login(request: LoginRequest, response: Response, db: AsyncSession = Depends(get_db)):
    """Login with email and password."""
    service = AuthService(db)
    auth_response = await service.login(request.email, request.password)

    # Set httpOnly cookie
    response.set_cookie(
        key="access_token",
        value=auth_response.access_token,
        httponly=True,
        secure=True,
        samesite="none",
        max_age=604800
    )

    return auth_response

@router.post("/logout")
async def logout(response: Response):
    """Logout and clear session cookie."""
    response.delete_cookie(key="access_token", samesite="none", secure=True)
    return {"message": "Logged out successfully"}
```

### Step 2.7: Main App

**`app/main.py`**:

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi_limiter import FastAPILimiter
from contextlib import asynccontextmanager
import redis.asyncio as redis
from app.core.config import settings
from app.routers import auth, chat, user

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize Redis for rate limiting
    redis_connection = redis.from_url(settings.redis_url, encoding="utf-8", decode_responses=True)
    await FastAPILimiter.init(redis_connection)
    yield
    # Shutdown: Close Redis connection
    await redis_connection.close()

app = FastAPI(title="RAG Chatbot API", version="1.0.0", lifespan=lifespan)

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"]
)

# Routers
app.include_router(auth.router, prefix="/v1")
app.include_router(chat.router, prefix="/v1")
app.include_router(user.router, prefix="/v1")

@app.get("/v1/health")
async def health_check():
    return {"status": "ok"}
```

**Run the server**:

```bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

---

## Phase 3: Frontend Implementation (1-2 hours)

### Step 3.1: Install Dependencies

```bash
cd frontend
npm install axios react-router-dom
```

### Step 3.2: Create Auth Context

**`src/context/AuthContext.tsx`**:

```tsx
import React, { createContext, useState, useContext, useEffect } from 'react';
import axios from 'axios';

interface User {
  user_id: string;
  email: string;
}

interface AuthContextType {
  user: User | null;
  login: (email: string, password: string) => Promise<void>;
  signup: (email: string, password: string) => Promise<void>;
  logout: () => Promise<void>;
  isAuthenticated: boolean;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);

  const login = async (email: string, password: string) => {
    const response = await axios.post(
      `${process.env.REACT_APP_API_URL}/auth/login`,
      { email, password },
      { withCredentials: true }
    );
    setUser({ user_id: response.data.user_id, email: response.data.email });
  };

  const signup = async (email: string, password: string) => {
    const response = await axios.post(
      `${process.env.REACT_APP_API_URL}/auth/signup`,
      { email, password },
      { withCredentials: true }
    );
    setUser({ user_id: response.data.user_id, email: response.data.email });
  };

  const logout = async () => {
    await axios.post(
      `${process.env.REACT_APP_API_URL}/auth/logout`,
      {},
      { withCredentials: true }
    );
    setUser(null);
  };

  return (
    <AuthContext.Provider value={{ user, login, signup, logout, isAuthenticated: !!user }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) throw new Error('useAuth must be used within AuthProvider');
  return context;
};
```

### Step 3.3: Login Page

**`src/pages/Login.tsx`**:

```tsx
import React, { useState } from 'react';
import { useAuth } from '../context/AuthContext';
import { useNavigate } from 'react-router-dom';

export const Login: React.FC = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const { login } = useAuth();
  const navigate = useNavigate();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    try {
      await login(email, password);
      navigate('/chat');
    } catch (err: any) {
      setError(err.response?.data?.error || 'Login failed');
    }
  };

  return (
    <div className="login-container">
      <h1>Login</h1>
      {error && <div className="error">{error}</div>}
      <form onSubmit={handleSubmit}>
        <input
          type="email"
          placeholder="Email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          required
        />
        <input
          type="password"
          placeholder="Password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          required
        />
        <button type="submit">Login</button>
      </form>
    </div>
  );
};
```

### Step 3.4: Protected Routes

**`src/components/ProtectedRoute.tsx`**:

```tsx
import React from 'react';
import { Navigate } from 'react-router-dom';
import { useAuth } from '../context/AuthContext';

export const ProtectedRoute: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const { isAuthenticated } = useAuth();

  if (!isAuthenticated) {
    return <Navigate to="/login" replace />;
  }

  return <>{children}</>;
};
```

### Step 3.5: Axios Interceptor for Auth

**`src/services/api.ts`**:

```typescript
import axios from 'axios';

const api = axios.create({
  baseURL: process.env.REACT_APP_API_URL,
  withCredentials: true  // Send cookies with requests
});

// Intercept 401 errors and redirect to login
api.interceptors.response.use(
  (response) => response,
  (error) => {
    if (error.response?.status === 401) {
      window.location.href = '/login';
    }
    return Promise.reject(error);
  }
);

export default api;
```

---

## Phase 4: Testing (1 hour)

### Step 4.1: Backend Tests

**`tests/test_auth.py`**:

```python
import pytest
from httpx import AsyncClient
from app.main import app

@pytest.mark.asyncio
async def test_signup_success():
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post("/v1/auth/signup", json={
            "email": "test@example.com",
            "password": "SecurePass123"
        })
        assert response.status_code == 201
        assert "access_token" in response.json()

@pytest.mark.asyncio
async def test_login_success():
    async with AsyncClient(app=app, base_url="http://test") as client:
        # First signup
        await client.post("/v1/auth/signup", json={
            "email": "test2@example.com",
            "password": "SecurePass123"
        })
        # Then login
        response = await client.post("/v1/auth/login", json={
            "email": "test2@example.com",
            "password": "SecurePass123"
        })
        assert response.status_code == 200
        assert "access_token" in response.json()
```

**Run tests**:

```bash
pytest tests/ -v
```

---

## Deployment

### Backend (Railway)

1. Create `railway.json`:

```json
{
  "$schema": "https://railway.app/railway.schema.json",
  "build": {
    "builder": "NIXPACKS"
  },
  "deploy": {
    "startCommand": "uvicorn app.main:app --host 0.0.0.0 --port $PORT",
    "restartPolicyType": "ON_FAILURE",
    "restartPolicyMaxRetries": 10
  }
}
```

2. Add PostgreSQL service in Railway dashboard
3. Set environment variables (JWT_SECRET_KEY, etc.)
4. Deploy from GitHub

### Frontend (Vercel)

1. Create `vercel.json`:

```json
{
  "rewrites": [{ "source": "/(.*)", "destination": "/" }]
}
```

2. Set environment variables (REACT_APP_API_URL)
3. Deploy from GitHub

---

## Verification Checklist

- [ ] Database migrations applied successfully
- [ ] Backend server runs without errors
- [ ] `/v1/health` endpoint returns `{"status": "ok"}`
- [ ] Signup creates new user in database
- [ ] Login returns JWT token and sets httpOnly cookie
- [ ] Protected chat endpoint requires authentication
- [ ] Logout clears cookie and invalidates session
- [ ] Frontend can signup, login, logout
- [ ] Chat history persists across sessions
- [ ] Rate limiting works (test with multiple requests)

---

## Troubleshooting

**Issue**: `CORS error: No 'Access-Control-Allow-Credentials' header`
**Solution**: Add `allow_credentials=True` in CORS middleware and include frontend domain in `cors_origins`

**Issue**: `401 Unauthorized on protected routes`
**Solution**: Ensure `withCredentials: true` in axios requests and cookies are being sent

**Issue**: `Database connection failed`
**Solution**: Verify `DATABASE_URL` in `.env` and PostgreSQL is running

**Issue**: `Rate limit not working`
**Solution**: Ensure Redis is running and `REDIS_URL` is correct

---

## Next Steps

After completing this quickstart:

1. Review `tasks.md` for detailed implementation tasks
2. Implement RAG integration with authenticated chat endpoints
3. Add frontend chat history UI
4. Implement user profile page
5. Add comprehensive error handling
6. Write integration tests for all endpoints

---

**Status**: Quickstart Complete | Ready for Implementation
