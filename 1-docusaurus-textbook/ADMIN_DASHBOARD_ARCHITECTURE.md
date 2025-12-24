# Admin Dashboard Architecture - AI-Humanoid-Robotics

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Vercel Deployment                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────────┐         ┌──────────────────┐         │
│  │  Docusaurus      │         │  Admin Dashboard │         │
│  │  Frontend        │         │  (React SPA)     │         │
│  │  /docs           │         │  /admin          │         │
│  └────────┬─────────┘         └────────┬─────────┘         │
│           │                            │                    │
│           └────────────┬───────────────┘                    │
│                        │                                    │
│           ┌────────────▼─────────────┐                     │
│           │   Next.js API Routes     │                     │
│           │   /api/*                 │                     │
│           └────────────┬─────────────┘                     │
│                        │                                    │
└────────────────────────┼────────────────────────────────────┘
                         │
        ┌────────────────┼────────────────┐
        │                │                │
   ┌────▼─────┐   ┌─────▼──────┐  ┌─────▼──────┐
   │ PostgreSQL│   │  Qdrant    │  │  Redis     │
   │ (Users/   │   │  (RAG      │  │  (Sessions)│
   │  Logs)    │   │   Vector)  │  │            │
   └───────────┘   └────────────┘  └────────────┘
```

## 📁 Project Structure

```
AI-Humanoid-Robotics/
├── 1-docusaurus-textbook/
│   ├── frontend/                    # Docusaurus site
│   │   ├── docs/                    # Documentation
│   │   ├── src/
│   │   │   ├── components/
│   │   │   ├── pages/
│   │   │   └── admin/              # NEW: Admin Dashboard
│   │   │       ├── components/     # Admin UI components
│   │   │       ├── pages/          # Admin pages
│   │   │       ├── hooks/          # Admin hooks
│   │   │       ├── services/       # API client
│   │   │       └── utils/          # Utilities
│   │   └── docusaurus.config.js
│   │
│   └── backend/                     # API Backend
│       ├── app/
│       │   ├── admin/              # NEW: Admin module
│       │   │   ├── routes/         # Admin API routes
│       │   │   ├── controllers/    # Business logic
│       │   │   ├── middleware/     # Auth & RBAC
│       │   │   └── services/       # Admin services
│       │   ├── routers/
│       │   ├── schemas/
│       │   └── services/
│       ├── database/               # NEW: Database layer
│       │   ├── models/             # Database models
│       │   ├── migrations/         # DB migrations
│       │   └── seeders/            # Seed data
│       └── requirements.txt
│
└── docs/
    └── admin/                       # Admin documentation
        ├── API.md
        ├── SETUP.md
        └── SECURITY.md
```

## 🔐 Authentication & Authorization Flow

### 1. User Authentication
```
User Login → Better Auth JWT → Redis Session → Role Check → Access Grant/Deny
```

### 2. Role-Based Access Control (RBAC)

**Roles:**
- `SUPER_ADMIN`: Full system access
- `ADMIN`: User & content management
- `MODERATOR`: Chat monitoring only
- `USER`: No admin access

**Permissions Matrix:**

| Feature              | SUPER_ADMIN | ADMIN | MODERATOR | USER |
|---------------------|-------------|-------|-----------|------|
| View Dashboard      | ✅          | ✅    | ✅        | ❌   |
| Manage Users        | ✅          | ✅    | ❌        | ❌   |
| View Chats          | ✅          | ✅    | ✅        | ❌   |
| Delete Chats        | ✅          | ✅    | ✅        | ❌   |
| System Settings     | ✅          | ❌    | ❌        | ❌   |
| Re-index RAG        | ✅          | ✅    | ❌        | ❌   |
| View Logs           | ✅          | ✅    | ❌        | ❌   |

## 🗄️ Database Schema

### PostgreSQL Tables

#### 1. Users Table (Extended)
```sql
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255) NOT NULL,
  full_name VARCHAR(255),
  role VARCHAR(50) DEFAULT 'USER', -- USER, MODERATOR, ADMIN, SUPER_ADMIN
  is_active BOOLEAN DEFAULT true,
  is_verified BOOLEAN DEFAULT false,
  last_login_at TIMESTAMP,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_role ON users(role);
```

#### 2. Chat Sessions Table
```sql
CREATE TABLE chat_sessions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id) ON DELETE CASCADE,
  session_token VARCHAR(255) UNIQUE,
  started_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  ended_at TIMESTAMP,
  messages_count INTEGER DEFAULT 0
);

CREATE INDEX idx_chat_sessions_user ON chat_sessions(user_id);
CREATE INDEX idx_chat_sessions_started ON chat_sessions(started_at DESC);
```

#### 3. Chat Messages Table
```sql
CREATE TABLE chat_messages (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
  user_id UUID REFERENCES users(id) ON DELETE SET NULL,
  role VARCHAR(20) NOT NULL, -- 'user' or 'assistant'
  content TEXT NOT NULL,
  chapter_id VARCHAR(100), -- Which doc chapter was active
  is_flagged BOOLEAN DEFAULT false,
  flag_reason TEXT,
  flagged_by UUID REFERENCES users(id),
  flagged_at TIMESTAMP,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_chat_messages_session ON chat_messages(session_id);
CREATE INDEX idx_chat_messages_flagged ON chat_messages(is_flagged);
CREATE INDEX idx_chat_messages_created ON chat_messages(created_at DESC);
```

#### 4. System Logs Table
```sql
CREATE TABLE system_logs (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  level VARCHAR(20) NOT NULL, -- INFO, WARNING, ERROR, CRITICAL
  service VARCHAR(100) NOT NULL, -- api, chatbot, rag, auth
  message TEXT NOT NULL,
  metadata JSONB,
  user_id UUID REFERENCES users(id),
  ip_address INET,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_system_logs_level ON system_logs(level);
CREATE INDEX idx_system_logs_service ON system_logs(service);
CREATE INDEX idx_system_logs_created ON system_logs(created_at DESC);
```

#### 5. Admin Actions Audit Table
```sql
CREATE TABLE admin_actions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  admin_id UUID REFERENCES users(id) NOT NULL,
  action_type VARCHAR(100) NOT NULL, -- USER_DEACTIVATED, CHAT_DELETED, RAG_REINDEXED
  target_type VARCHAR(50), -- user, chat_message, system
  target_id UUID,
  details JSONB,
  ip_address INET,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_admin_actions_admin ON admin_actions(admin_id);
CREATE INDEX idx_admin_actions_created ON admin_actions(created_at DESC);
```

#### 6. Chapter Settings Table
```sql
CREATE TABLE chapter_settings (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  chapter_id VARCHAR(100) UNIQUE NOT NULL,
  chatbot_enabled BOOLEAN DEFAULT true,
  max_queries_per_hour INTEGER DEFAULT 100,
  updated_by UUID REFERENCES users(id),
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

## 📊 Key Metrics & Analytics

### Dashboard Stats (Real-time)

1. **User Metrics:**
   - Total registered users
   - Active users (last 24h, 7d, 30d)
   - New user registrations (daily trend)

2. **Chatbot Metrics:**
   - Total conversations
   - Total messages sent
   - Average messages per session
   - Most active chapters
   - Response time (avg, p50, p95, p99)

3. **System Health:**
   - API uptime percentage
   - Error rate (last 1h)
   - RAG query latency
   - Database connection pool status

## 🔒 Security Measures

### 1. Authentication Security
- JWT tokens with short expiry (15 min)
- Refresh tokens with rotation
- HTTP-only secure cookies
- CSRF protection
- Rate limiting on auth endpoints

### 2. Admin Access Security
- IP whitelist for admin access (optional)
- 2FA for admin accounts (recommended)
- Session timeout (30 min inactivity)
- Audit logging for all admin actions
- Password strength requirements

### 3. API Security
- Role-based middleware on all admin routes
- Input validation (Pydantic/Zod)
- SQL injection prevention (parameterized queries)
- XSS protection
- CORS configuration

### 4. Data Privacy
- PII encryption at rest
- Sensitive data masking in logs
- GDPR compliance (data export/deletion)
- Chat message retention policies

## 🚀 Technology Stack

### Frontend (Admin Dashboard)
```json
{
  "framework": "React 18",
  "routing": "React Router v6",
  "state": "Zustand / React Query",
  "ui": "shadcn/ui + Tailwind CSS",
  "charts": "Recharts / Apache ECharts",
  "tables": "TanStack Table",
  "forms": "React Hook Form + Zod"
}
```

### Backend (Admin API)
```json
{
  "runtime": "Node.js 20 / Python 3.11",
  "framework": "Express.js / FastAPI",
  "orm": "Prisma / SQLAlchemy",
  "cache": "Redis",
  "queue": "BullMQ (for async tasks)",
  "validation": "Zod / Pydantic"
}
```

### Infrastructure
```json
{
  "hosting": "Vercel (Frontend + API)",
  "database": "Vercel Postgres / Supabase",
  "cache": "Upstash Redis",
  "vector": "Qdrant Cloud",
  "monitoring": "Vercel Analytics + Sentry",
  "logging": "Axiom / Logtail"
}
```

## 📈 Performance Targets

| Metric                    | Target      |
|---------------------------|-------------|
| Admin page load time      | < 1.5s      |
| API response time (p95)   | < 500ms     |
| Dashboard data refresh    | Real-time   |
| Database query time       | < 100ms     |
| Concurrent admin users    | 10+         |

## 🔄 Implementation Phases

### Phase 1: Foundation (Week 1)
- Database schema setup
- Authentication middleware
- Basic admin routes
- User management UI

### Phase 2: Monitoring (Week 2)
- Chat message viewing
- System logs dashboard
- Real-time stats

### Phase 3: Advanced Features (Week 3)
- Content management
- RAG re-indexing
- Analytics & reporting
- Audit logs

### Phase 4: Polish & Deploy (Week 4)
- Security hardening
- Performance optimization
- Documentation
- Production deployment

## 📝 Next Steps

1. Review this architecture
2. Set up database (Vercel Postgres recommended)
3. Implement authentication layer
4. Build admin API endpoints
5. Create admin UI components
6. Deploy to Vercel
7. Set up monitoring

---

**Status:** Architecture Complete ✅
**Next:** Database Schema Implementation
