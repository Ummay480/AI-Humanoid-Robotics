# Admin Dashboard Implementation Guide
## AI-Humanoid-Robotics - Production-Ready Admin Panel

**Status:** Architecture Complete | Implementation Ready
**Estimated Time:** 2-3 weeks for full implementation
**Complexity:** Advanced (Production-grade)

---

## 📋 What Has Been Created

### ✅ Completed Files

1. **`ADMIN_DASHBOARD_ARCHITECTURE.md`**
   - Complete system architecture
   - Database schema design
   - Technology stack decisions
   - Performance targets
   - Implementation phases

2. **`backend/database/schema.sql`**
   - PostgreSQL schema with 8 core tables
   - Materialized views for performance
   - Triggers and functions
   - Indexes for optimization
   - Initial admin seed data

3. **`backend/app/admin/middleware/auth.py`**
   - JWT authentication system
   - Role-Based Access Control (RBAC)
   - Permission checks
   - Rate limiting
   - Audit logging functions
   - Security headers

---

## 🚀 Step-by-Step Implementation

### Phase 1: Database Setup (Day 1-2)

#### 1.1 Set Up PostgreSQL Database

**Option A: Vercel Postgres (Recommended)**
```bash
# Install Vercel Postgres
npm install -g vercel
vercel link

# Create Postgres database
vercel postgres create ai-humanoid-robotics-db

# Get connection string
vercel env pull .env
```

**Option B: Supabase**
```bash
#Go to https://supabase.com
# Create new project
# Copy connection string to .env
```

**Environment Variables (`.env`):**
```env
# Database
DATABASE_URL="postgresql://user:password@host:5432/dbname"
POSTGRES_PRISMA_URL="postgresql://user:password@host:5432/dbname?pgbouncer=true"
POSTGRES_URL_NON_POOLING="postgresql://user:password@host:5432/dbname"

# JWT
JWT_SECRET_KEY="your-super-secret-jwt-key-min-32-chars"

# Redis (for sessions)
REDIS_URL="redis://default:password@host:6379"

# Admin
ADMIN_EMAIL="admin@yourdomain.com"
ADMIN_PASSWORD="ChangeMe123!"
```

#### 1.2 Run Database Migration

```bash
cd 1-docusaurus-textbook/backend

# Connect to your database
psql $DATABASE_URL -f database/schema.sql

# Verify tables created
psql $DATABASE_URL -c "\dt"
```

**Expected Output:**
```
 users
 chat_sessions
 chat_messages
 system_logs
 admin_actions
 chapter_settings
 rate_limits
 analytics_snapshots
```

#### 1.3 Verify Initial Admin User

```sql
SELECT id, email, role, is_active
FROM users
WHERE role = 'SUPER_ADMIN';
```

**Default Credentials:**
- Email: `admin@ai-humanoid-robotics.com`
- Password: `ChangeMe123!`
- **⚠️ CHANGE THIS IMMEDIATELY IN PRODUCTION**

---

### Phase 2: Backend API Implementation (Day 3-7)

#### 2.1 Install Dependencies

**For Python/FastAPI Backend:**
```bash
cd 1-docusaurus-textbook/backend

# Add to requirements.txt
cat >> requirements.txt << EOF
fastapi==0.109.0
uvicorn[standard]==0.27.0
python-jose[cryptography]==3.3.0
passlib[bcrypt]==1.7.4
python-multipart==0.0.6
asyncpg==0.29.0
redis==5.0.1
pydantic==2.5.3
pydantic-settings==2.1.0
EOF

# Install
pip install -r requirements.txt
```

**For Node.js/Express Backend:**
```bash
cd 1-docusaurus-textbook/backend

npm install express jsonwebtoken bcrypt pg redis zod
npm install --save-dev @types/express @types/jsonwebtoken
```

#### 2.2 Create Admin API Routes

**File:** `backend/app/admin/routes/admin_routes.py`

```python
from fastapi import APIRouter, Depends, HTTPException, Query
from typing import List, Optional
from ..middleware.auth import require_admin, require_super_admin, get_current_active_user
from ..controllers import admin_controller
from ..schemas import UserResponse, ChatMessageResponse, SystemLogResponse

router = APIRouter(prefix="/api/admin", tags=["admin"])

# ============================================================================
# DASHBOARD STATS
# ============================================================================

@router.get("/dashboard/stats")
async def get_dashboard_stats(
    current_user: dict = Depends(require_admin)
):
    """
    Get real-time dashboard statistics

    Returns:
        - total_users
        - active_users_24h
        - total_chats
        - daily_chat_usage
        - system_health
    """
    return await admin_controller.get_dashboard_stats()

# ============================================================================
# USER MANAGEMENT
# ============================================================================

@router.get("/users", response_model=List[UserResponse])
async def get_all_users(
    page: int = Query(1, ge=1),
    limit: int = Query(50, ge=1, le=100),
    role: Optional[str] = None,
    is_active: Optional[bool] = None,
    search: Optional[str] = None,
    current_user: dict = Depends(require_admin)
):
    """Get paginated list of users with filters"""
    return await admin_controller.get_users(
        page=page,
        limit=limit,
        role=role,
        is_active=is_active,
        search=search
    )


@router.get("/users/{user_id}", response_model=UserResponse)
async def get_user_by_id(
    user_id: str,
    current_user: dict = Depends(require_admin)
):
    """Get detailed user information"""
    return await admin_controller.get_user_by_id(user_id)


@router.patch("/users/{user_id}/activate")
async def activate_user(
    user_id: str,
    current_user: dict = Depends(require_admin)
):
    """Activate a user account"""
    result = await admin_controller.activate_user(
        user_id=user_id,
        admin_id=current_user["user_id"]
    )
    return {"message": "User activated successfully", "user": result}


@router.patch("/users/{user_id}/deactivate")
async def deactivate_user(
    user_id: str,
    reason: Optional[str] = None,
    current_user: dict = Depends(require_admin)
):
    """Deactivate a user account"""
    result = await admin_controller.deactivate_user(
        user_id=user_id,
        admin_id=current_user["user_id"],
        reason=reason
    )
    return {"message": "User deactivated successfully", "user": result}


@router.patch("/users/{user_id}/role")
async def change_user_role(
    user_id: str,
    new_role: str,
    current_user: dict = Depends(require_super_admin)  # Only super admin can change roles
):
    """Change user role (SUPER_ADMIN only)"""
    result = await admin_controller.change_user_role(
        user_id=user_id,
        new_role=new_role,
        admin_id=current_user["user_id"]
    )
    return {"message": f"User role changed to {new_role}", "user": result}


@router.delete("/users/{user_id}")
async def delete_user(
    user_id: str,
    permanent: bool = False,
    current_user: dict = Depends(require_super_admin)
):
    """Delete user (soft delete by default, permanent if specified)"""
    await admin_controller.delete_user(
        user_id=user_id,
        permanent=permanent,
        admin_id=current_user["user_id"]
    )
    delete_type = "permanently deleted" if permanent else "deactivated"
    return {"message": f"User {delete_type} successfully"}

# ============================================================================
# CHAT MONITORING
# ============================================================================

@router.get("/chats/recent", response_model=List[ChatMessageResponse])
async def get_recent_chats(
    page: int = Query(1, ge=1),
    limit: int = Query(100, ge=1, le=200),
    chapter_id: Optional[str] = None,
    flagged_only: bool = False,
    current_user: dict = Depends(require_admin)
):
    """Get recent chat messages with optional filters"""
    return await admin_controller.get_recent_chats(
        page=page,
        limit=limit,
        chapter_id=chapter_id,
        flagged_only=flagged_only
    )


@router.get("/chats/session/{session_id}")
async def get_chat_session(
    session_id: str,
    current_user: dict = Depends(require_admin)
):
    """Get full chat session with all messages"""
    return await admin_controller.get_chat_session(session_id)


@router.patch("/chats/messages/{message_id}/flag")
async def flag_chat_message(
    message_id: str,
    reason: str,
    current_user: dict = Depends(require_admin)
):
    """Flag a chat message as inappropriate"""
    await admin_controller.flag_message(
        message_id=message_id,
        reason=reason,
        flagged_by=current_user["user_id"]
    )
    return {"message": "Chat message flagged successfully"}


@router.delete("/chats/messages/{message_id}")
async def delete_chat_message(
    message_id: str,
    current_user: dict = Depends(require_admin)
):
    """Delete a chat message"""
    await admin_controller.delete_message(
        message_id=message_id,
        admin_id=current_user["user_id"]
    )
    return {"message": "Chat message deleted successfully"}


@router.get("/chats/analytics")
async def get_chat_analytics(
    days: int = Query(30, ge=1, le=365),
    current_user: dict = Depends(require_admin)
):
    """Get chat usage analytics"""
    return await admin_controller.get_chat_analytics(days=days)

# ============================================================================
# CONTENT MANAGEMENT
# ============================================================================

@router.get("/chapters")
async def get_all_chapters(
    current_user: dict = Depends(require_admin)
):
    """Get all chapters with settings"""
    return await admin_controller.get_chapters()


@router.patch("/chapters/{chapter_id}/chatbot")
async def toggle_chapter_chatbot(
    chapter_id: str,
    enabled: bool,
    current_user: dict = Depends(require_admin)
):
    """Enable/disable chatbot for a specific chapter"""
    await admin_controller.update_chapter_setting(
        chapter_id=chapter_id,
        setting="chatbot_enabled",
        value=enabled,
        updated_by=current_user["user_id"]
    )
    status = "enabled" if enabled else "disabled"
    return {"message": f"Chatbot {status} for chapter {chapter_id}"}


@router.post("/rag/reindex")
async def reindex_rag_database(
    chapter_ids: Optional[List[str]] = None,
    current_user: dict = Depends(require_admin)
):
    """
    Trigger RAG knowledge base re-indexing

    Args:
        chapter_ids: Optional list of specific chapters to reindex (None = all)
    """
    result = await admin_controller.reindex_rag(
        chapter_ids=chapter_ids,
        admin_id=current_user["user_id"]
    )
    return {
        "message": "RAG reindexing started",
        "job_id": result["job_id"],
        "estimated_time": result["estimated_time"]
    }


@router.get("/rag/status")
async def get_rag_index_status(
    current_user: dict = Depends(require_admin)
):
    """Get RAG indexing status and health"""
    return await admin_controller.get_rag_status()

# ============================================================================
# SYSTEM LOGS & MONITORING
# ============================================================================

@router.get("/logs", response_model=List[SystemLogResponse])
async def get_system_logs(
    page: int = Query(1, ge=1),
    limit: int = Query(100, ge=1, le=500),
    level: Optional[str] = None,  # DEBUG, INFO, WARNING, ERROR, CRITICAL
    service: Optional[str] = None,  # api, chatbot, rag, auth
    start_date: Optional[str] = None,
    end_date: Optional[str] = None,
    current_user: dict = Depends(require_admin)
):
    """Get system logs with filters"""
    return await admin_controller.get_logs(
        page=page,
        limit=limit,
        level=level,
        service=service,
        start_date=start_date,
        end_date=end_date
    )


@router.get("/system/health")
async def get_system_health(
    current_user: dict = Depends(require_admin)
):
    """Get system health status"""
    return await admin_controller.get_system_health()


@router.get("/audit/actions")
async def get_audit_logs(
    page: int = Query(1, ge=1),
    limit: int = Query(50, ge=1, le=100),
    admin_id: Optional[str] = None,
    action_type: Optional[str] = None,
    current_user: dict = Depends(require_super_admin)
):
    """Get admin action audit logs (SUPER_ADMIN only)"""
    return await admin_controller.get_audit_logs(
        page=page,
        limit=limit,
        admin_id=admin_id,
        action_type=action_type
    )

# ============================================================================
# ANALYTICS & REPORTS
# ============================================================================

@router.get("/analytics/users")
async def get_user_analytics(
    period: str = "30d",  # 7d, 30d, 90d, 1y
    current_user: dict = Depends(require_admin)
):
    """Get user growth and activity analytics"""
    return await admin_controller.get_user_analytics(period=period)


@router.get("/analytics/chats")
async def get_chat_analytics(
    period: str = "30d",
    current_user: dict = Depends(require_admin)
):
    """Get chat usage analytics"""
    return await admin_controller.get_chat_usage_analytics(period=period)


@router.get("/analytics/export")
async def export_analytics(
    format: str = "csv",  # csv, json, excel
    period: str = "30d",
    current_user: dict = Depends(require_admin)
):
    """Export analytics data"""
    # Return file download
    pass

```

---

### Phase 3: Frontend Admin Dashboard (Day 8-14)

#### 3.1 Install Frontend Dependencies

```bash
cd 1-docusaurus-textbook/frontend

# Install admin dashboard dependencies
npm install @tanstack/react-query zustand recharts @radix-ui/react-dialog
npm install @radix-ui/react-dropdown-menu @radix-ui/react-select
npm install clsx tailwind-merge date-fns axios react-router-dom
npm install lucide-react # For icons
```

#### 3.2 Create Admin Dashboard Structure

```bash
mkdir -p src/admin/{components,pages,hooks,services,utils,types}
```

**File Structure:**
```
src/admin/
├── components/
│   ├── layout/
│   │   ├── AdminLayout.tsx
│   │   ├── Sidebar.tsx
│   │   └── Header.tsx
│   ├── dashboard/
│   │   ├── StatsCard.tsx
│   │   ├── UserActivityChart.tsx
│   │   └── ChatUsageChart.tsx
│   ├── users/
│   │   ├── UserTable.tsx
│   │   ├── UserDetailsModal.tsx
│   │   └── UserRoleSelect.tsx
│   └── chats/
│       ├── ChatMessageList.tsx
│       ├── ChatSessionView.tsx
│       └── FlagMessageDialog.tsx
├── pages/
│   ├── Dashboard.tsx
│   ├── Users.tsx
│   ├── ChatMonitoring.tsx
│   ├── ContentManagement.tsx
│   └── SystemLogs.tsx
├── hooks/
│   ├── useAdminStats.ts
│   ├── useUsers.ts
│   └── useChatMonitoring.ts
├── services/
│   └── adminApi.ts
└── types/
    └── admin.ts
```

---

## 🔐 Security Implementation Checklist

### Critical Security Measures

- [ ] **Environment Variables**
  - [ ] Generate strong JWT secret (min 32 characters)
  - [ ] Never commit `.env` files to git
  - [ ] Use Vercel Environment Variables in production

- [ ] **Password Security**
  - [ ] Use bcrypt with minimum 12 rounds
  - [ ] Enforce password strength requirements
  - [ ] Implement password reset flow

- [ ] **Authentication**
  - [ ] Short-lived access tokens (15 min)
  - [ ] Refresh token rotation
  - [ ] HTTP-only secure cookies
  - [ ] CSRF protection

- [ ] **Authorization**
  - [ ] Role checks on all admin routes
  - [ ] Permission checks for sensitive actions
  - [ ] Audit logging for all admin actions

- [ ] **API Security**
  - [ ] Input validation on all endpoints
  - [ ] SQL injection prevention (parameterized queries)
  - [ ] XSS protection
  - [ ] CORS configuration
  - [ ] Rate limiting

- [ ] **Data Privacy**
  - [ ] PII encryption at rest
  - [ ] Sensitive data masking in logs
  - [ ] GDPR compliance features

---

## 🚀 Deployment to Vercel

### 1. Configure Vercel Environment Variables

```bash
# Set environment variables in Vercel dashboard
vercel env add DATABASE_URL production
vercel env add JWT_SECRET_KEY production
vercel env add REDIS_URL production
```

### 2. Update `vercel.json`

```json
{
  "buildCommand": "npm run build",
  "outputDirectory": "build",
  "framework": "docusaurus",
  "installCommand": "npm install",
  "devCommand": "npm start",
  "rewrites": [
    {
      "source": "/api/:path*",
      "destination": "/api/:path*"
    },
    {
      "source": "/admin/:path*",
      "destination": "/admin/:path*"
    }
  ],
  "headers": [
    {
      "source": "/admin/(.*)",
      "headers": [
        {
          "key": "X-Frame-Options",
          "value": "DENY"
        },
        {
          "key": "X-Content-Type-Options",
          "value": "nosniff"
        }
      ]
    }
  ]
}
```

### 3. Deploy

```bash
vercel --prod
```

---

## 📊 Monitoring & Maintenance

### Set Up Monitoring

1. **Vercel Analytics**
   - Enable in Vercel dashboard
   - Monitor page views, API calls

2. **Sentry (Error Tracking)**
   ```bash
   npm install @sentry/react @sentry/node
   ```

3. **Database Monitoring**
   - Enable slow query logging
   - Set up connection pool monitoring

### Automated Tasks

**Cron Jobs (via Vercel Cron or external):**

1. **Refresh Materialized Views** (Every hour)
   ```sql
   SELECT refresh_dashboard_views();
   ```

2. **Clean Old Logs** (Daily)
   ```sql
   DELETE FROM system_logs
   WHERE created_at < NOW() - INTERVAL '90 days';
   ```

3. **Analytics Snapshots** (Daily)
   ```sql
   INSERT INTO analytics_snapshots (snapshot_type, snapshot_date, metrics)
   VALUES ('daily', CURRENT_DATE, '{...computed metrics...}');
   ```

---

## 📝 Testing Guide

### 1. Test Admin Authentication

```bash
# Login as admin
curl -X POST http://localhost:3000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email": "admin@ai-humanoid-robotics.com", "password": "ChangeMe123!"}'

# Save the token
export TOKEN="eyJhbGc..."

# Test admin endpoint
curl -X GET http://localhost:3000/api/admin/dashboard/stats \
  -H "Authorization: Bearer $TOKEN"
```

### 2. Test Role-Based Access

```bash
# Try accessing as regular user (should fail)
curl -X GET http://localhost:3000/api/admin/users \
  -H "Authorization: Bearer $USER_TOKEN"
# Expected: 403 Forbidden
```

### 3. Test Rate Limiting

```bash
# Make 101 requests in 1 minute (should hit rate limit)
for i in {1..101}; do
  curl -X GET http://localhost:3000/api/admin/logs -H "Authorization: Bearer $TOKEN"
done
# Request 101 should return: 429 Too Many Requests
```

---

## 🎯 Next Steps

1. **Review Architecture** ✅ (Done)
2. **Set Up Database** (1-2 days)
3. **Implement Backend API** (3-5 days)
4. **Build Frontend Dashboard** (5-7 days)
5. **Security Hardening** (2-3 days)
6. **Testing & QA** (3-4 days)
7. **Deploy to Production** (1 day)
8. **Documentation & Training** (2 days)

**Total Estimated Time:** 2-3 weeks

---

## 📞 Support & Resources

- **Architecture Doc:** `ADMIN_DASHBOARD_ARCHITECTURE.md`
- **Database Schema:** `backend/database/schema.sql`
- **Auth Middleware:** `backend/app/admin/middleware/auth.py`
- **API Routes:** See above implementation

**Status:** Ready for Implementation 🚀

---

**Built with production standards by Claude Code**
