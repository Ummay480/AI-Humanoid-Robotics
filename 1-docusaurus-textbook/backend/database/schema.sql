-- Admin Dashboard Database Schema for AI-Humanoid-Robotics
-- PostgreSQL 14+

-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
CREATE EXTENSION IF NOT EXISTS "pg_trgm"; -- For full-text search

-- ============================================================================
-- 1. USERS TABLE (Extended for Admin Dashboard)
-- ============================================================================
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    full_name VARCHAR(255),

    -- Role-based access control
    role VARCHAR(50) DEFAULT 'USER' CHECK (role IN ('USER', 'MODERATOR', 'ADMIN', 'SUPER_ADMIN')),

    -- Account status
    is_active BOOLEAN DEFAULT true,
    is_verified BOOLEAN DEFAULT false,
    is_deleted BOOLEAN DEFAULT false, -- Soft delete

    -- Activity tracking
    last_login_at TIMESTAMP,
    login_count INTEGER DEFAULT 0,

    -- Profile
    avatar_url TEXT,
    bio TEXT,

    -- Timestamps
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    deleted_at TIMESTAMP
);

-- Indexes for users table
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_role ON users(role);
CREATE INDEX idx_users_is_active ON users(is_active) WHERE is_active = true;
CREATE INDEX idx_users_created_at ON users(created_at DESC);

-- ============================================================================
-- 2. CHAT SESSIONS TABLE
-- ============================================================================
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    session_token VARCHAR(255) UNIQUE,

    -- Session metadata
    chapter_id VARCHAR(100), -- Which chapter user was reading
    device_type VARCHAR(50), -- mobile, desktop, tablet
    user_agent TEXT,
    ip_address INET,

    -- Session stats
    started_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    ended_at TIMESTAMP,
    last_activity_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    messages_count INTEGER DEFAULT 0,
    duration_seconds INTEGER GENERATED ALWAYS AS (
        EXTRACT(EPOCH FROM (COALESCE(ended_at, CURRENT_TIMESTAMP) - started_at))::INTEGER
    ) STORED
);

CREATE INDEX idx_chat_sessions_user ON chat_sessions(user_id);
CREATE INDEX idx_chat_sessions_started ON chat_sessions(started_at DESC);
CREATE INDEX idx_chat_sessions_chapter ON chat_sessions(chapter_id);

-- ============================================================================
-- 3. CHAT MESSAGES TABLE
-- ============================================================================
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,

    -- Message content
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,

    -- Context
    chapter_id VARCHAR(100), -- Which doc chapter was referenced
    sources JSONB, -- RAG sources used for response

    -- Moderation
    is_flagged BOOLEAN DEFAULT false,
    flag_reason TEXT,
    flagged_by UUID REFERENCES users(id),
    flagged_at TIMESTAMP,
    is_deleted BOOLEAN DEFAULT false,

    -- Metadata
    metadata JSONB, -- Store additional context
    response_time_ms INTEGER, -- For assistant messages
    tokens_used INTEGER, -- LLM tokens used

    -- Timestamps
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_chat_messages_session ON chat_messages(session_id);
CREATE INDEX idx_chat_messages_user ON chat_messages(user_id);
CREATE INDEX idx_chat_messages_flagged ON chat_messages(is_flagged) WHERE is_flagged = true;
CREATE INDEX idx_chat_messages_created ON chat_messages(created_at DESC);
CREATE INDEX idx_chat_messages_chapter ON chat_messages(chapter_id);

-- Full-text search on message content
CREATE INDEX idx_chat_messages_content_search ON chat_messages USING gin(to_tsvector('english', content));

-- ============================================================================
-- 4. SYSTEM LOGS TABLE
-- ============================================================================
CREATE TABLE system_logs (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),

    -- Log classification
    level VARCHAR(20) NOT NULL CHECK (level IN ('DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL')),
    service VARCHAR(100) NOT NULL, -- api, chatbot, rag, auth, admin

    -- Log details
    message TEXT NOT NULL,
    error_code VARCHAR(50),
    stack_trace TEXT,
    metadata JSONB,

    -- Context
    user_id UUID REFERENCES users(id),
    session_id UUID REFERENCES chat_sessions(id),
    request_id VARCHAR(255),
    ip_address INET,

    -- Timestamp
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_system_logs_level ON system_logs(level);
CREATE INDEX idx_system_logs_service ON system_logs(service);
CREATE INDEX idx_system_logs_created ON system_logs(created_at DESC);
CREATE INDEX idx_system_logs_user ON system_logs(user_id) WHERE user_id IS NOT NULL;

-- Partition system_logs by month for better performance
-- (Optional: Implement if logs grow beyond 1M records)

-- ============================================================================
-- 5. ADMIN ACTIONS AUDIT TABLE
-- ============================================================================
CREATE TABLE admin_actions (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),

    -- Admin who performed action
    admin_id UUID REFERENCES users(id) NOT NULL,

    -- Action details
    action_type VARCHAR(100) NOT NULL,
    -- Examples: USER_ACTIVATED, USER_DEACTIVATED, USER_ROLE_CHANGED,
    --           CHAT_DELETED, CHAT_FLAGGED, RAG_REINDEXED, SETTINGS_UPDATED

    -- Target of action
    target_type VARCHAR(50), -- user, chat_message, chat_session, system, chapter
    target_id UUID,

    -- Action details
    details JSONB, -- Store before/after values
    reason TEXT, -- Admin's reason for action

    -- Context
    ip_address INET,
    user_agent TEXT,

    -- Timestamp
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_admin_actions_admin ON admin_actions(admin_id);
CREATE INDEX idx_admin_actions_type ON admin_actions(action_type);
CREATE INDEX idx_admin_actions_target ON admin_actions(target_type, target_id);
CREATE INDEX idx_admin_actions_created ON admin_actions(created_at DESC);

-- ============================================================================
-- 6. CHAPTER SETTINGS TABLE
-- ============================================================================
CREATE TABLE chapter_settings (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),

    -- Chapter identification
    chapter_id VARCHAR(100) UNIQUE NOT NULL,
    chapter_title VARCHAR(255),
    chapter_path VARCHAR(500),

    -- Chatbot settings
    chatbot_enabled BOOLEAN DEFAULT true,
    max_queries_per_hour INTEGER DEFAULT 100,
    custom_system_prompt TEXT,

    -- RAG settings
    rag_enabled BOOLEAN DEFAULT true,
    context_window_size INTEGER DEFAULT 5,
    similarity_threshold DECIMAL(3,2) DEFAULT 0.70,

    -- Metadata
    last_indexed_at TIMESTAMP,
    document_count INTEGER DEFAULT 0,

    -- Audit
    updated_by UUID REFERENCES users(id),
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_chapter_settings_chapter_id ON chapter_settings(chapter_id);

-- ============================================================================
-- 7. RATE LIMITING TABLE
-- ============================================================================
CREATE TABLE rate_limits (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),

    -- Rate limit key (user_id:endpoint or ip:endpoint)
    key VARCHAR(255) UNIQUE NOT NULL,

    -- Counters
    request_count INTEGER DEFAULT 1,
    window_start TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    window_duration_seconds INTEGER DEFAULT 3600, -- 1 hour

    -- Limits
    max_requests INTEGER DEFAULT 100,

    -- Timestamps
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_rate_limits_key ON rate_limits(key);
CREATE INDEX idx_rate_limits_window ON rate_limits(window_start);

-- ============================================================================
-- 8. ANALYTICS SNAPSHOTS TABLE (for dashboard performance)
-- ============================================================================
CREATE TABLE analytics_snapshots (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),

    -- Snapshot metadata
    snapshot_type VARCHAR(50) NOT NULL, -- daily, hourly, realtime
    snapshot_date DATE NOT NULL,
    snapshot_hour INTEGER, -- 0-23 for hourly snapshots

    -- Metrics (JSONB for flexibility)
    metrics JSONB NOT NULL,
    -- Example structure:
    -- {
    --   "total_users": 1000,
    --   "active_users_24h": 150,
    --   "total_chats": 5000,
    --   "total_messages": 25000,
    --   "avg_session_duration": 180,
    --   "top_chapters": [...]
    -- }

    -- Timestamp
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE UNIQUE INDEX idx_analytics_snapshots_unique
ON analytics_snapshots(snapshot_type, snapshot_date, COALESCE(snapshot_hour, -1));

-- ============================================================================
-- MATERIALIZED VIEWS FOR DASHBOARD PERFORMANCE
-- ============================================================================

-- Active users (last 24 hours)
CREATE MATERIALIZED VIEW mv_active_users_24h AS
SELECT
    COUNT(DISTINCT user_id) as active_users,
    DATE_TRUNC('hour', last_activity_at) as hour
FROM chat_sessions
WHERE last_activity_at > NOW() - INTERVAL '24 hours'
GROUP BY DATE_TRUNC('hour', last_activity_at)
ORDER BY hour DESC;

CREATE INDEX ON mv_active_users_24h(hour DESC);

-- Popular chapters
CREATE MATERIALIZED VIEW mv_popular_chapters AS
SELECT
    chapter_id,
    COUNT(*) as chat_count,
    COUNT(DISTINCT user_id) as unique_users,
    AVG(messages_count) as avg_messages_per_session,
    MAX(started_at) as last_activity
FROM chat_sessions
WHERE chapter_id IS NOT NULL
  AND started_at > NOW() - INTERVAL '30 days'
GROUP BY chapter_id
ORDER BY chat_count DESC;

CREATE INDEX ON mv_popular_chapters(chapter_id);

-- ============================================================================
-- FUNCTIONS & TRIGGERS
-- ============================================================================

-- Auto-update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ language 'plpgsql';

-- Apply to relevant tables
CREATE TRIGGER update_users_updated_at BEFORE UPDATE ON users
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_chat_messages_updated_at BEFORE UPDATE ON chat_messages
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_chapter_settings_updated_at BEFORE UPDATE ON chapter_settings
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- Increment login count on user login
CREATE OR REPLACE FUNCTION increment_login_count()
RETURNS TRIGGER AS $$
BEGIN
    IF NEW.last_login_at > OLD.last_login_at OR OLD.last_login_at IS NULL THEN
        NEW.login_count = COALESCE(OLD.login_count, 0) + 1;
    END IF;
    RETURN NEW;
END;
$$ language 'plpgsql';

CREATE TRIGGER user_login_counter BEFORE UPDATE ON users
    FOR EACH ROW EXECUTE FUNCTION increment_login_count();

-- Refresh materialized views (call via cron job or after significant updates)
CREATE OR REPLACE FUNCTION refresh_dashboard_views()
RETURNS void AS $$
BEGIN
    REFRESH MATERIALIZED VIEW CONCURRENTLY mv_active_users_24h;
    REFRESH MATERIALIZED VIEW CONCURRENTLY mv_popular_chapters;
END;
$$ language 'plpgsql';

-- ============================================================================
-- INITIAL DATA / SEED
-- ============================================================================

-- Create initial super admin (password: 'ChangeMe123!')
-- Hash generated with bcrypt rounds=12
INSERT INTO users (email, password_hash, full_name, role, is_active, is_verified)
VALUES (
    'admin@ai-humanoid-robotics.com',
    '$2b$12$LQv3c1yqBWVHxkd0LHAkCOYz6TtxMQJqhN8/LewY5GyYIeWZx2DEi',
    'System Administrator',
    'SUPER_ADMIN',
    true,
    true
) ON CONFLICT (email) DO NOTHING;

-- ============================================================================
-- COMMENTS FOR DOCUMENTATION
-- ============================================================================

COMMENT ON TABLE users IS 'User accounts with role-based access control';
COMMENT ON TABLE chat_sessions IS 'User chat sessions with context and metadata';
COMMENT ON TABLE chat_messages IS 'Individual messages within chat sessions';
COMMENT ON TABLE system_logs IS 'System-wide logging for monitoring and debugging';
COMMENT ON TABLE admin_actions IS 'Audit trail for all administrative actions';
COMMENT ON TABLE chapter_settings IS 'Per-chapter configuration for chatbot and RAG';
COMMENT ON TABLE rate_limits IS 'Rate limiting counters for API endpoints';
COMMENT ON TABLE analytics_snapshots IS 'Pre-computed analytics for dashboard performance';

-- ============================================================================
-- GRANT PERMISSIONS (Adjust based on your setup)
-- ============================================================================

-- Example for application user
-- CREATE USER app_user WITH PASSWORD 'secure_password_here';
-- GRANT SELECT, INSERT, UPDATE, DELETE ON ALL TABLES IN SCHEMA public TO app_user;
-- GRANT USAGE, SELECT ON ALL SEQUENCES IN SCHEMA public TO app_user;

-- ============================================================================
-- MAINTENANCE & OPTIMIZATION
-- ============================================================================

-- Vacuum and analyze regularly (set up pg_cron or external scheduler)
-- VACUUM ANALYZE users;
-- VACUUM ANALYZE chat_messages;
-- VACUUM ANALYZE system_logs;

-- Archive old logs (example: delete logs older than 90 days)
-- DELETE FROM system_logs WHERE created_at < NOW() - INTERVAL '90 days';

-- ============================================================================
-- SCHEMA VERSION
-- ============================================================================

CREATE TABLE schema_version (
    version VARCHAR(20) PRIMARY KEY,
    applied_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

INSERT INTO schema_version (version) VALUES ('1.0.0');
