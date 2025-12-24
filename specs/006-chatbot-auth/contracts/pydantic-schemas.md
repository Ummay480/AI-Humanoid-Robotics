# Pydantic Schemas: Request/Response Models

**Feature**: 006-chatbot-auth
**Date**: 2025-12-19
**Status**: Design Complete

## Overview

This document defines Pydantic models for request validation and response serialization in the FastAPI backend. All models use Pydantic V2 syntax with strict validation.

## Authentication Schemas

### SignupRequest

**Purpose**: Validate user signup input

```python
from pydantic import BaseModel, EmailStr, Field, field_validator
import re

class SignupRequest(BaseModel):
    """Request model for user registration."""

    email: EmailStr = Field(
        ...,
        description="User's email address (RFC 5322 compliant)",
        min_length=5,
        max_length=255,
        examples=["user@example.com"]
    )

    password: str = Field(
        ...,
        description="Password (min 8 chars, at least one letter and one number)",
        min_length=8,
        max_length=128,
        examples=["SecurePass123"]
    )

    @field_validator('password')
    @classmethod
    def validate_password_strength(cls, v: str) -> str:
        """
        Validate password strength:
        - Minimum 8 characters
        - At least one letter (a-z or A-Z)
        - At least one digit (0-9)
        """
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters long')

        if not re.search(r'[a-zA-Z]', v):
            raise ValueError('Password must contain at least one letter')

        if not re.search(r'\d', v):
            raise ValueError('Password must contain at least one number')

        return v

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "email": "alice@example.com",
                    "password": "SecurePass123"
                }
            ]
        }
    }
```

**Validation Rules**:
- Email: RFC 5322 format (handled by `EmailStr`)
- Password: 8-128 characters, at least one letter and one number

**Error Messages**:
- "Password must be at least 8 characters long"
- "Password must contain at least one letter"
- "Password must contain at least one number"
- "value is not a valid email address" (from `EmailStr`)

---

### LoginRequest

**Purpose**: Validate login credentials

```python
from pydantic import BaseModel, EmailStr, Field

class LoginRequest(BaseModel):
    """Request model for user login."""

    email: EmailStr = Field(
        ...,
        description="User's email address",
        examples=["user@example.com"]
    )

    password: str = Field(
        ...,
        description="User's password",
        min_length=1,  # No max length for login (accept any password attempt)
        examples=["SecurePass123"]
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "email": "alice@example.com",
                    "password": "SecurePass123"
                }
            ]
        }
    }
```

**Validation Rules**:
- Email: Valid email format
- Password: Non-empty (no strength validation on login, only signup)

---

### AuthResponse

**Purpose**: Return authentication success with JWT token

```python
from pydantic import BaseModel, Field, UUID4
from datetime import datetime

class AuthResponse(BaseModel):
    """Response model for successful authentication."""

    user_id: UUID4 = Field(
        ...,
        description="Unique user identifier",
        examples=["123e4567-e89b-12d3-a456-426614174000"]
    )

    email: str = Field(
        ...,
        description="User's email address",
        examples=["user@example.com"]
    )

    access_token: str = Field(
        ...,
        description="JWT access token",
        examples=["eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."]
    )

    token_type: str = Field(
        default="bearer",
        description="Token type (always 'bearer')",
        examples=["bearer"]
    )

    expires_in: int = Field(
        default=604800,  # 7 days in seconds
        description="Token expiration time in seconds",
        examples=[604800]
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "user_id": "123e4567-e89b-12d3-a456-426614174000",
                    "email": "alice@example.com",
                    "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJ1c2VyX2lkIjoiMTIzZTQ1NjctZTg5Yi0xMmQzLWE0NTYtNDI2NjE0MTc0MDAwIiwiZW1haWwiOiJhbGljZUBleGFtcGxlLmNvbSJ9.xyz",
                    "token_type": "bearer",
                    "expires_in": 604800
                }
            ]
        }
    }
```

---

## Chat Schemas

### ChatRequest

**Purpose**: Validate chat message input

```python
from pydantic import BaseModel, Field, UUID4
from typing import Optional

class ChatRequest(BaseModel):
    """Request model for sending a chat message."""

    message: str = Field(
        ...,
        description="User's message to the chatbot",
        min_length=1,
        max_length=5000,
        examples=["What is ROS 2 navigation?"]
    )

    conversation_id: Optional[UUID4] = Field(
        default=None,
        description="Optional conversation ID to continue an existing conversation",
        examples=["550e8400-e29b-41d4-a716-446655440000"]
    )

    page_context: Optional[str] = Field(
        default=None,
        description="Current page URL or context where message was sent",
        max_length=500,
        examples=["https://example.com/docs/module1/ros2-nodes"]
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "message": "What is ROS 2 navigation?",
                    "page_context": "https://example.com/docs/module1/ros2-nodes"
                },
                {
                    "message": "Can you explain that in more detail?",
                    "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
                    "page_context": "https://example.com/docs/module1/ros2-nodes"
                }
            ]
        }
    }
```

**Validation Rules**:
- Message: 1-5000 characters
- Conversation ID: Valid UUID4 if provided
- Page context: Max 500 characters if provided

---

### SourceReference

**Purpose**: Represent a RAG source citation

```python
from pydantic import BaseModel, Field, HttpUrl
from typing import Optional

class SourceReference(BaseModel):
    """Model for a source reference from RAG retrieval."""

    title: str = Field(
        ...,
        description="Source document title",
        examples=["ROS 2 Navigation Concepts"]
    )

    url: HttpUrl = Field(
        ...,
        description="Source document URL",
        examples=["https://example.com/docs/module1/ros2-nodes#navigation"]
    )

    relevance_score: Optional[float] = Field(
        default=None,
        description="Relevance score from vector search (0.0 to 1.0)",
        ge=0.0,
        le=1.0,
        examples=[0.85]
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "title": "ROS 2 Navigation Concepts",
                    "url": "https://example.com/docs/module1/ros2-nodes#navigation",
                    "relevance_score": 0.85
                }
            ]
        }
    }
```

---

### ChatResponse

**Purpose**: Return chatbot response with metadata

```python
from pydantic import BaseModel, Field, UUID4
from datetime import datetime
from typing import Optional, List

class ChatResponse(BaseModel):
    """Response model for chat message."""

    conversation_id: UUID4 = Field(
        ...,
        description="Conversation ID (created if new conversation)",
        examples=["550e8400-e29b-41d4-a716-446655440000"]
    )

    message_id: UUID4 = Field(
        ...,
        description="Unique ID for the assistant's response message",
        examples=["660f9511-f39c-52e5-b827-557766551111"]
    )

    role: str = Field(
        default="assistant",
        description="Message author (always 'assistant' for responses)",
        examples=["assistant"]
    )

    content: str = Field(
        ...,
        description="Chatbot's response message",
        examples=["ROS 2 navigation is a framework for autonomous robot movement."]
    )

    timestamp: datetime = Field(
        ...,
        description="Message creation timestamp",
        examples=["2025-12-19T10:30:00Z"]
    )

    sources: Optional[List[SourceReference]] = Field(
        default=None,
        description="Source references from RAG retrieval"
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
                    "message_id": "660f9511-f39c-52e5-b827-557766551111",
                    "role": "assistant",
                    "content": "ROS 2 navigation is a framework for autonomous robot movement.",
                    "timestamp": "2025-12-19T10:30:00Z",
                    "sources": [
                        {
                            "title": "ROS 2 Navigation Concepts",
                            "url": "https://example.com/docs/module1/ros2-nodes",
                            "relevance_score": 0.85
                        }
                    ]
                }
            ]
        }
    }
```

---

## History Schemas

### ConversationSummary

**Purpose**: Represent a conversation in history list

```python
from pydantic import BaseModel, Field, UUID4
from datetime import datetime
from typing import Optional

class ConversationSummary(BaseModel):
    """Model for conversation summary in history list."""

    conversation_id: UUID4 = Field(
        ...,
        description="Conversation unique identifier",
        examples=["550e8400-e29b-41d4-a716-446655440000"]
    )

    title: Optional[str] = Field(
        default=None,
        description="Conversation title (auto-generated or user-defined)",
        examples=["ROS 2 Navigation Discussion"]
    )

    started_at: datetime = Field(
        ...,
        description="Conversation start timestamp",
        examples=["2025-12-19T09:00:00Z"]
    )

    last_message_at: datetime = Field(
        ...,
        description="Timestamp of last message in conversation",
        examples=["2025-12-19T10:30:00Z"]
    )

    message_count: int = Field(
        ...,
        description="Total number of messages in conversation",
        ge=0,
        examples=[12]
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
                    "title": "ROS 2 Navigation Discussion",
                    "started_at": "2025-12-19T09:00:00Z",
                    "last_message_at": "2025-12-19T10:30:00Z",
                    "message_count": 12
                }
            ]
        }
    }
```

---

### Message

**Purpose**: Represent a single message in conversation

```python
from pydantic import BaseModel, Field, UUID4
from datetime import datetime
from typing import Optional, List, Literal

class Message(BaseModel):
    """Model for a single chat message."""

    message_id: UUID4 = Field(
        ...,
        description="Unique message identifier",
        examples=["660f9511-f39c-52e5-b827-557766551111"]
    )

    role: Literal["user", "assistant"] = Field(
        ...,
        description="Message author (user or assistant)",
        examples=["user"]
    )

    content: str = Field(
        ...,
        description="Message content",
        examples=["What is ROS 2 navigation?"]
    )

    timestamp: datetime = Field(
        ...,
        description="Message creation timestamp",
        examples=["2025-12-19T10:30:00Z"]
    )

    page_context: Optional[str] = Field(
        default=None,
        description="Page URL where message was sent",
        examples=["https://example.com/docs/module1/ros2-nodes"]
    )

    sources: Optional[List[SourceReference]] = Field(
        default=None,
        description="Source references (only for assistant messages)"
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "message_id": "660f9511-f39c-52e5-b827-557766551111",
                    "role": "user",
                    "content": "What is ROS 2 navigation?",
                    "timestamp": "2025-12-19T10:30:00Z",
                    "page_context": "https://example.com/docs/module1/ros2-nodes",
                    "sources": None
                },
                {
                    "message_id": "770f9622-f49d-63f6-c938-668877662222",
                    "role": "assistant",
                    "content": "ROS 2 navigation is a framework...",
                    "timestamp": "2025-12-19T10:30:05Z",
                    "page_context": None,
                    "sources": [
                        {
                            "title": "ROS 2 Navigation",
                            "url": "https://example.com/docs/module1/ros2-nodes",
                            "relevance_score": 0.85
                        }
                    ]
                }
            ]
        }
    }
```

---

### ChatHistoryResponse

**Purpose**: Return paginated conversation history

```python
from pydantic import BaseModel, Field
from typing import List

class ChatHistoryResponse(BaseModel):
    """Response model for chat history endpoint."""

    conversations: List[ConversationSummary] = Field(
        ...,
        description="List of user's conversations"
    )

    total: int = Field(
        ...,
        description="Total number of conversations for this user",
        ge=0,
        examples=[15]
    )

    limit: int = Field(
        ...,
        description="Maximum conversations returned in this response",
        examples=[20]
    )

    offset: int = Field(
        ...,
        description="Number of conversations skipped",
        ge=0,
        examples=[0]
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "conversations": [
                        {
                            "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
                            "title": "ROS 2 Navigation Discussion",
                            "started_at": "2025-12-19T09:00:00Z",
                            "last_message_at": "2025-12-19T10:30:00Z",
                            "message_count": 12
                        }
                    ],
                    "total": 15,
                    "limit": 20,
                    "offset": 0
                }
            ]
        }
    }
```

---

### ConversationMessagesResponse

**Purpose**: Return paginated messages for a conversation

```python
from pydantic import BaseModel, Field, UUID4
from typing import List

class ConversationMessagesResponse(BaseModel):
    """Response model for conversation messages endpoint."""

    conversation_id: UUID4 = Field(
        ...,
        description="Conversation ID",
        examples=["550e8400-e29b-41d4-a716-446655440000"]
    )

    messages: List[Message] = Field(
        ...,
        description="Messages in this conversation (ordered by timestamp)"
    )

    total: int = Field(
        ...,
        description="Total number of messages in this conversation",
        ge=0,
        examples=[12]
    )

    limit: int = Field(
        ...,
        description="Maximum messages returned",
        examples=[50]
    )

    offset: int = Field(
        ...,
        description="Number of messages skipped",
        ge=0,
        examples=[0]
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
                    "messages": [
                        {
                            "message_id": "660f9511-f39c-52e5-b827-557766551111",
                            "role": "user",
                            "content": "What is ROS 2 navigation?",
                            "timestamp": "2025-12-19T10:30:00Z",
                            "page_context": "https://example.com/docs/module1/ros2-nodes",
                            "sources": None
                        }
                    ],
                    "total": 12,
                    "limit": 50,
                    "offset": 0
                }
            ]
        }
    }
```

---

## User Schemas

### UserProfileResponse

**Purpose**: Return user profile information

```python
from pydantic import BaseModel, Field, UUID4
from datetime import datetime
from typing import Optional

class UserProfileResponse(BaseModel):
    """Response model for user profile endpoint."""

    user_id: UUID4 = Field(
        ...,
        description="User's unique identifier",
        examples=["123e4567-e89b-12d3-a456-426614174000"]
    )

    email: str = Field(
        ...,
        description="User's email address",
        examples=["user@example.com"]
    )

    created_at: datetime = Field(
        ...,
        description="Account creation timestamp",
        examples=["2025-12-01T08:00:00Z"]
    )

    last_login: Optional[datetime] = Field(
        default=None,
        description="Last successful login timestamp",
        examples=["2025-12-19T10:00:00Z"]
    )

    conversation_count: int = Field(
        ...,
        description="Total number of conversations",
        ge=0,
        examples=[15]
    )

    message_count: int = Field(
        ...,
        description="Total number of messages sent",
        ge=0,
        examples=[87]
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "user_id": "123e4567-e89b-12d3-a456-426614174000",
                    "email": "alice@example.com",
                    "created_at": "2025-12-01T08:00:00Z",
                    "last_login": "2025-12-19T10:00:00Z",
                    "conversation_count": 15,
                    "message_count": 87
                }
            ]
        }
    }
```

---

## Error Schemas

### ErrorResponse

**Purpose**: Standard error response format

```python
from pydantic import BaseModel, Field
from datetime import datetime
from typing import Optional

class ErrorResponse(BaseModel):
    """Standard error response model."""

    error: str = Field(
        ...,
        description="Short error message",
        examples=["Invalid credentials"]
    )

    detail: Optional[str] = Field(
        default=None,
        description="Detailed error explanation",
        examples=["Email or password is incorrect."]
    )

    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Error timestamp",
        examples=["2025-12-19T10:30:00Z"]
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "error": "Invalid credentials",
                    "detail": "Email or password is incorrect.",
                    "timestamp": "2025-12-19T10:30:00Z"
                },
                {
                    "error": "Rate limit exceeded",
                    "detail": "Maximum 5 login attempts per minute. Please try again later.",
                    "timestamp": "2025-12-19T10:30:00Z"
                }
            ]
        }
    }
```

---

## Internal Models (Database ORM)

These models are not exposed in the API but are used internally for database operations.

### User (SQLAlchemy Model)

```python
from sqlalchemy import Column, String, DateTime, Boolean
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
import uuid

class User(Base):
    """SQLAlchemy model for users table."""
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

---

## Validation Summary

| Schema              | Key Validations                                                                 |
|---------------------|---------------------------------------------------------------------------------|
| SignupRequest       | Email (RFC 5322), Password (8+ chars, letter + number)                         |
| LoginRequest        | Email (RFC 5322), Password (non-empty)                                         |
| ChatRequest         | Message (1-5000 chars), Conversation ID (UUID4), Page context (max 500)        |
| SourceReference     | URL (HttpUrl), Relevance score (0.0-1.0)                                       |
| Message             | Role (literal "user" or "assistant"), Content (non-empty)                      |
| ConversationSummary | Message count (>= 0)                                                           |
| UserProfileResponse | Conversation count (>= 0), Message count (>= 0)                                |

---

## Usage in FastAPI Endpoints

```python
from fastapi import APIRouter, Depends, HTTPException, status
from app.schemas import SignupRequest, AuthResponse, ErrorResponse

router = APIRouter(prefix="/auth", tags=["Authentication"])

@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED, responses={
    400: {"model": ErrorResponse, "description": "Invalid request"},
    429: {"model": ErrorResponse, "description": "Rate limit exceeded"}
})
async def signup(request: SignupRequest):
    """
    Register a new user account.

    Validates email format and password strength, then creates a new user
    with hashed password and returns a JWT token.
    """
    # Implementation here
    pass
```

---

**Status**: Design Complete | Ready for Implementation
