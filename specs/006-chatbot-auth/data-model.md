# Data Model: User Authentication for RAG Chatbot

**Feature**: 006-chatbot-auth
**Date**: 2025-12-19
**Status**: Design Complete

## Overview

This document defines the database schema, entities, relationships, and validation rules for user authentication and chat history persistence. The design uses PostgreSQL as the primary database with SQLAlchemy ORM for async operations.

## Entity Relationship Diagram

```
┌──────────────┐         ┌──────────────────┐         ┌──────────────────┐
│     User     │1      n│   Conversation   │1      n│   ChatMessage    │
│──────────────│◄────────│──────────────────│◄────────│──────────────────│
│ user_id (PK) │         │conversation_id(PK)│        │ message_id (PK)  │
│ email        │         │ user_id (FK)     │         │ conversation_id  │
│ password_hash│         │ started_at       │         │ user_id (FK)     │
│ created_at   │         │ last_message_at  │         │ role             │
│ last_login   │         │ title            │         │ content          │
│ is_active    │         │ message_count    │         │ timestamp        │
└──────────────┘         └──────────────────┘         │ page_context     │
       │1                                              │ sources (JSON)   │
       │                                               └──────────────────┘
       │n
       ▼
┌──────────────┐
│   Session    │
│──────────────│
│session_id(PK)│
│ user_id (FK) │
│ jwt_token    │
│ issued_at    │
│ expires_at   │
│ ip_address   │
│ user_agent   │
└──────────────┘
```

## Entities

### 1. User

Represents a registered user account with authentication credentials.

**Table Name**: `users`

**Columns**:

| Column         | Type         | Constraints                  | Description                                    |
|----------------|--------------|------------------------------|------------------------------------------------|
| user_id        | UUID         | PRIMARY KEY, DEFAULT uuid_v4 | Unique identifier for the user                 |
| email          | VARCHAR(255) | UNIQUE, NOT NULL             | User's email address (used for login)          |
| password_hash  | VARCHAR(255) | NOT NULL                     | Argon2id hashed password                       |
| created_at     | TIMESTAMP    | NOT NULL, DEFAULT NOW()      | Account creation timestamp                     |
| last_login     | TIMESTAMP    | NULL                         | Last successful login timestamp                |
| is_active      | BOOLEAN      | NOT NULL, DEFAULT TRUE       | Account active status (for soft deletion)      |

**Indexes**:
- `idx_users_email` on `email` (for login lookups)
- `idx_users_created_at` on `created_at` (for analytics)

**Validation Rules**:
- Email MUST match RFC 5322 format (validated by Pydantic EmailStr)
- Password MUST be at least 8 characters, contain at least one letter and one number
- Password hash MUST use Argon2id with time_cost=2, memory_cost=65536, parallelism=4

**State Transitions**:
- **Created** → `is_active=True`, `last_login=NULL`
- **First Login** → `last_login` set to current timestamp
- **Subsequent Logins** → `last_login` updated
- **Deactivated** → `is_active=False` (soft delete, future enhancement)

**Example SQLAlchemy Model**:

```python
from sqlalchemy import Column, String, DateTime, Boolean
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid

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

---

### 2. Session

Represents an active user session with JWT token tracking.

**Table Name**: `sessions`

**Columns**:

| Column       | Type         | Constraints                  | Description                                    |
|--------------|--------------|------------------------------|------------------------------------------------|
| session_id   | UUID         | PRIMARY KEY, DEFAULT uuid_v4 | Unique identifier for the session              |
| user_id      | UUID         | FOREIGN KEY (users), NOT NULL| Reference to the user                          |
| jwt_token    | TEXT         | NOT NULL                     | Issued JWT token (for revocation tracking)     |
| issued_at    | TIMESTAMP    | NOT NULL, DEFAULT NOW()      | Token issuance timestamp                       |
| expires_at   | TIMESTAMP    | NOT NULL                     | Token expiration timestamp (issued_at + 7 days)|
| ip_address   | VARCHAR(45)  | NULL                         | Client IP address (IPv4/IPv6)                  |
| user_agent   | TEXT         | NULL                         | Client user agent string                       |

**Indexes**:
- `idx_sessions_user_id` on `user_id` (for user session lookups)
- `idx_sessions_jwt_token` on `jwt_token` (for token validation)
- `idx_sessions_expires_at` on `expires_at` (for cleanup queries)

**Validation Rules**:
- `expires_at` MUST be `issued_at + 7 days` (per spec NFR-006)
- `jwt_token` MUST be a valid HS256-signed JWT
- `ip_address` MUST be a valid IPv4 or IPv6 address if provided

**State Transitions**:
- **Created** → Session starts with `issued_at` and `expires_at`
- **Expired** → When `NOW() > expires_at`, session is invalid
- **Revoked** → Session deleted from database (logout)

**Example SQLAlchemy Model**:

```python
from datetime import datetime, timedelta

class Session(Base):
    __tablename__ = "sessions"

    session_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.user_id"), nullable=False, index=True)
    jwt_token = Column(Text, nullable=False, index=True)
    issued_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    expires_at = Column(DateTime(timezone=True), nullable=False, index=True)
    ip_address = Column(String(45), nullable=True)
    user_agent = Column(Text, nullable=True)

    # Relationships
    user = relationship("User", back_populates="sessions")

    @staticmethod
    def calculate_expiration():
        return datetime.utcnow() + timedelta(days=7)
```

---

### 3. Conversation

Represents a chat session grouping multiple messages.

**Table Name**: `conversations`

**Columns**:

| Column            | Type         | Constraints                  | Description                                    |
|-------------------|--------------|------------------------------|------------------------------------------------|
| conversation_id   | UUID         | PRIMARY KEY, DEFAULT uuid_v4 | Unique identifier for the conversation         |
| user_id           | UUID         | FOREIGN KEY (users), NOT NULL| Reference to the user                          |
| started_at        | TIMESTAMP    | NOT NULL, DEFAULT NOW()      | Conversation start timestamp                   |
| last_message_at   | TIMESTAMP    | NOT NULL, DEFAULT NOW()      | Timestamp of last message in conversation      |
| title             | VARCHAR(255) | NULL                         | Optional conversation title (auto or manual)   |
| message_count     | INTEGER      | NOT NULL, DEFAULT 0          | Total number of messages in conversation       |

**Indexes**:
- `idx_conversations_user_id` on `user_id` (for user conversation lookups)
- `idx_conversations_last_message_at` on `last_message_at` (for sorting recent conversations)

**Validation Rules**:
- `message_count` MUST be >= 0
- `last_message_at` MUST be >= `started_at`
- `title` is optional; if NULL, display as "Conversation on [started_at date]"

**State Transitions**:
- **Created** → New conversation with `message_count=0`
- **Message Added** → `message_count` incremented, `last_message_at` updated
- **Deleted** → All related messages cascade deleted

**Example SQLAlchemy Model**:

```python
class Conversation(Base):
    __tablename__ = "conversations"

    conversation_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.user_id"), nullable=False, index=True)
    started_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    last_message_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False, index=True)
    title = Column(String(255), nullable=True)
    message_count = Column(Integer, default=0, nullable=False)

    # Relationships
    user = relationship("User", back_populates="conversations")
    messages = relationship("ChatMessage", back_populates="conversation", cascade="all, delete-orphan", order_by="ChatMessage.timestamp")
```

---

### 4. ChatMessage

Represents a single message in a conversation (user or assistant).

**Table Name**: `chat_messages`

**Columns**:

| Column           | Type         | Constraints                  | Description                                    |
|------------------|--------------|------------------------------|------------------------------------------------|
| message_id       | UUID         | PRIMARY KEY, DEFAULT uuid_v4 | Unique identifier for the message              |
| conversation_id  | UUID         | FOREIGN KEY (conversations), NOT NULL | Reference to parent conversation   |
| user_id          | UUID         | FOREIGN KEY (users), NOT NULL| Reference to the user                          |
| role             | VARCHAR(20)  | NOT NULL, CHECK IN ('user','assistant') | Message author (user or AI)       |
| content          | TEXT         | NOT NULL                     | Message content                                |
| timestamp        | TIMESTAMP    | NOT NULL, DEFAULT NOW()      | Message creation timestamp                     |
| page_context     | VARCHAR(500) | NULL                         | Current page URL/context when message sent     |
| sources          | JSONB        | NULL                         | Array of source references for RAG responses   |

**Indexes**:
- `idx_chat_messages_conversation_id` on `conversation_id` (for loading conversation messages)
- `idx_chat_messages_user_id` on `user_id` (for user message lookups)
- `idx_chat_messages_timestamp` on `timestamp` (for chronological ordering)
- GIN index on `sources` (for JSON querying, if needed)

**Validation Rules**:
- `role` MUST be either 'user' or 'assistant'
- `content` MUST NOT be empty (minimum 1 character)
- `sources` MUST be a valid JSON array if provided (e.g., `[{"title": "...", "url": "..."}]`)

**State Transitions**:
- **Created** → Message added to conversation, `conversation.message_count` incremented
- **Deleted** → Message removed (future enhancement: soft delete with `deleted_at` column)

**Example SQLAlchemy Model**:

```python
from sqlalchemy.dialects.postgresql import JSONB

class ChatMessage(Base):
    __tablename__ = "chat_messages"

    message_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    conversation_id = Column(UUID(as_uuid=True), ForeignKey("conversations.conversation_id"), nullable=False, index=True)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.user_id"), nullable=False, index=True)
    role = Column(String(20), nullable=False)
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime(timezone=True), server_default=func.now(), nullable=False, index=True)
    page_context = Column(String(500), nullable=True)
    sources = Column(JSONB, nullable=True)

    # Relationships
    conversation = relationship("Conversation", back_populates="messages")
    user = relationship("User", back_populates="messages")

    __table_args__ = (
        CheckConstraint("role IN ('user', 'assistant')", name="check_role"),
        CheckConstraint("LENGTH(content) > 0", name="check_content_not_empty"),
    )
```

---

## Relationships

### User ↔ Conversation (One-to-Many)
- One user can have many conversations
- Each conversation belongs to exactly one user
- Cascade: Deleting a user deletes all their conversations

### User ↔ Session (One-to-Many)
- One user can have many sessions (multi-device support in future)
- Each session belongs to exactly one user
- Cascade: Deleting a user deletes all their sessions

### User ↔ ChatMessage (One-to-Many)
- One user can have many messages
- Each message belongs to exactly one user
- Cascade: Deleting a user deletes all their messages

### Conversation ↔ ChatMessage (One-to-Many)
- One conversation contains many messages
- Each message belongs to exactly one conversation
- Cascade: Deleting a conversation deletes all its messages
- Order: Messages ordered by `timestamp` ASC

---

## Database Migrations

**Migration Tool**: Alembic (SQLAlchemy migration tool)

**Initial Migration** (Version 001):

```bash
alembic revision --autogenerate -m "Create users, sessions, conversations, and chat_messages tables"
```

**Migration Script Structure**:

```python
# alembic/versions/001_create_auth_tables.py

def upgrade():
    # Create users table
    op.create_table(
        'users',
        sa.Column('user_id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('email', sa.String(255), unique=True, nullable=False),
        sa.Column('password_hash', sa.String(255), nullable=False),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now()),
        sa.Column('last_login', sa.DateTime(timezone=True)),
        sa.Column('is_active', sa.Boolean, default=True)
    )
    op.create_index('idx_users_email', 'users', ['email'])

    # Create sessions table
    op.create_table(
        'sessions',
        sa.Column('session_id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('users.user_id')),
        sa.Column('jwt_token', sa.Text, nullable=False),
        sa.Column('issued_at', sa.DateTime(timezone=True), server_default=sa.func.now()),
        sa.Column('expires_at', sa.DateTime(timezone=True), nullable=False),
        sa.Column('ip_address', sa.String(45)),
        sa.Column('user_agent', sa.Text)
    )
    op.create_index('idx_sessions_user_id', 'sessions', ['user_id'])
    op.create_index('idx_sessions_jwt_token', 'sessions', ['jwt_token'])

    # Create conversations table
    op.create_table(
        'conversations',
        sa.Column('conversation_id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('users.user_id')),
        sa.Column('started_at', sa.DateTime(timezone=True), server_default=sa.func.now()),
        sa.Column('last_message_at', sa.DateTime(timezone=True), server_default=sa.func.now()),
        sa.Column('title', sa.String(255)),
        sa.Column('message_count', sa.Integer, default=0)
    )
    op.create_index('idx_conversations_user_id', 'conversations', ['user_id'])

    # Create chat_messages table
    op.create_table(
        'chat_messages',
        sa.Column('message_id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('conversation_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('conversations.conversation_id')),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('users.user_id')),
        sa.Column('role', sa.String(20), nullable=False),
        sa.Column('content', sa.Text, nullable=False),
        sa.Column('timestamp', sa.DateTime(timezone=True), server_default=sa.func.now()),
        sa.Column('page_context', sa.String(500)),
        sa.Column('sources', postgresql.JSONB)
    )
    op.create_index('idx_chat_messages_conversation_id', 'chat_messages', ['conversation_id'])

def downgrade():
    op.drop_table('chat_messages')
    op.drop_table('conversations')
    op.drop_table('sessions')
    op.drop_table('users')
```

---

## Performance Considerations

### Query Optimization

1. **User Login Lookup**:
   ```sql
   SELECT * FROM users WHERE email = ? AND is_active = TRUE;
   ```
   - Uses `idx_users_email` index
   - Expected: <10ms for single user lookup

2. **Load Conversation Messages**:
   ```sql
   SELECT * FROM chat_messages
   WHERE conversation_id = ?
   ORDER BY timestamp ASC
   LIMIT 50;
   ```
   - Uses `idx_chat_messages_conversation_id` index
   - Pagination with LIMIT/OFFSET for >50 messages
   - Expected: <100ms for 50 messages

3. **User Conversation History**:
   ```sql
   SELECT * FROM conversations
   WHERE user_id = ?
   ORDER BY last_message_at DESC
   LIMIT 20;
   ```
   - Uses `idx_conversations_user_id` and `idx_conversations_last_message_at` indexes
   - Expected: <50ms for 20 conversations

### Connection Pooling

```python
# SQLAlchemy async engine with connection pool
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession

engine = create_async_engine(
    "postgresql+asyncpg://user:pass@host/db",
    pool_size=20,          # Max 20 connections
    max_overflow=10,       # Allow 10 overflow connections
    pool_pre_ping=True,    # Verify connection health
    echo=False             # Disable SQL logging in production
)
```

### Cleanup Queries

**Expire Old Sessions** (scheduled job, run daily):
```sql
DELETE FROM sessions WHERE expires_at < NOW();
```

**Inactive User Cleanup** (future enhancement):
```sql
UPDATE users SET is_active = FALSE WHERE last_login < NOW() - INTERVAL '1 year';
```

---

## Data Retention

**User Data**:
- Users retained indefinitely unless manually deleted
- Soft deletion via `is_active=FALSE` (hard delete is future enhancement)

**Chat History**:
- Messages retained indefinitely (spec Assumption #12: unlimited storage)
- Future enhancement: archive messages older than 1 year to cold storage

**Sessions**:
- Expired sessions deleted after 30 days (cleanup job)
- Active sessions persist until expiration (7 days from issuance)

---

## Security Considerations

### Password Storage
- Passwords NEVER stored in plaintext
- Argon2id hashing with parameters: `time_cost=2, memory_cost=65536, parallelism=4`
- Password hashes are 255 characters max (Argon2id output)

### JWT Token Storage
- JWT tokens stored in `sessions` table for revocation tracking
- Tokens signed with HS256 algorithm and 256-bit secret key
- Token payload includes: `user_id`, `email`, `issued_at`, `expires_at`

### SQL Injection Prevention
- All queries use SQLAlchemy ORM with parameterized queries
- No raw SQL concatenation
- Input validation via Pydantic models before database operations

### Data Encryption
- Database connections use SSL/TLS in production
- Passwords transmitted only over HTTPS
- JWT secret key stored in environment variables, never in code

---

## Testing Strategy

### Unit Tests
- Test each model's validation rules (email format, password length, role constraint)
- Test relationship cascades (deleting user deletes conversations)
- Test state transitions (user creation, login, session expiration)

### Integration Tests
- Test database CRUD operations (create user, create conversation, add message)
- Test query performance (load 50 messages in <300ms)
- Test concurrent user creation (unique email constraint prevents duplicates)

### Migration Tests
- Test upgrade and downgrade migrations
- Test data integrity after migration (no data loss)

---

## Compliance Mapping

**Functional Requirements**:
- FR-001: User model with `email`, `password_hash` ✓
- FR-010: ChatMessage model with `user_id` for history tracking ✓
- FR-012: ChatMessage metadata (`role`, `timestamp`, `session_id` via conversation) ✓

**Non-Functional Requirements**:
- NFR-001: Password hashing with Argon2id (bcrypt equivalent, cost factor 12+) ✓
- NFR-006: Session expiration after 7 days (`expires_at` column) ✓
- NFR-010: Chat history retrieval <300ms (indexed queries with LIMIT 50) ✓

**Success Criteria**:
- SC-006: Session tokens valid for 7 days (`expires_at = issued_at + 7 days`) ✓
- SC-009: Password hashing with Argon2id (bcrypt cost 12 equivalent) ✓
- SC-012: Chat history persists across devices (user_id foreign key) ✓

---

## Future Enhancements

1. **Message Soft Delete**: Add `deleted_at` column for soft deletion
2. **Conversation Search**: Full-text search index on `chat_messages.content`
3. **Multi-Device Session Management**: Track device names, allow session revocation
4. **Message Reactions**: Add `reactions` JSONB column for emoji reactions
5. **Conversation Sharing**: Add `shared_with` JSONB column for collaboration
6. **Message Edit History**: Add `edited_at` and `edit_history` JSONB columns
7. **User Preferences**: Add `preferences` JSONB column for theme, language settings

---

**Status**: Design Complete | Ready for API Contract Generation
