"""
Admin Authentication & Authorization Middleware
Implements JWT verification and Role-Based Access Control (RBAC)
"""

from functools import wraps
from typing import List, Optional, Callable
from fastapi import HTTPException, status, Depends, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import jwt
from datetime import datetime, timedelta
import os
from enum import Enum

# Configuration
JWT_SECRET = os.getenv("JWT_SECRET_KEY", "your-secret-key-change-in-production")
JWT_ALGORITHM = "HS256"
JWT_EXPIRATION_MINUTES = 15  # Short-lived access tokens
REFRESH_TOKEN_EXPIRATION_DAYS = 30

# Security
security = HTTPBearer()

# ============================================================================
# USER ROLES ENUM
# ============================================================================

class UserRole(str, Enum):
    """User role definitions with hierarchy"""
    USER = "USER"
    MODERATOR = "MODERATOR"
    ADMIN = "ADMIN"
    SUPER_ADMIN = "SUPER_ADMIN"

    @classmethod
    def get_hierarchy_level(cls, role: str) -> int:
        """Return numeric level for role comparison"""
        hierarchy = {
            cls.USER: 1,
            cls.MODERATOR: 2,
            cls.ADMIN: 3,
            cls.SUPER_ADMIN: 4
        }
        return hierarchy.get(role, 0)

# ============================================================================
# TOKEN UTILITIES
# ============================================================================

def create_access_token(user_id: str, email: str, role: str) -> str:
    """
    Create JWT access token

    Args:
        user_id: User UUID
        email: User email
        role: User role (USER, MODERATOR, ADMIN, SUPER_ADMIN)

    Returns:
        Encoded JWT token
    """
    payload = {
        "user_id": user_id,
        "email": email,
        "role": role,
        "type": "access",
        "exp": datetime.utcnow() + timedelta(minutes=JWT_EXPIRATION_MINUTES),
        "iat": datetime.utcnow()
    }
    return jwt.encode(payload, JWT_SECRET, algorithm=JWT_ALGORITHM)


def create_refresh_token(user_id: str) -> str:
    """Create long-lived refresh token"""
    payload = {
        "user_id": user_id,
        "type": "refresh",
        "exp": datetime.utcnow() + timedelta(days=REFRESH_TOKEN_EXPIRATION_DAYS),
        "iat": datetime.utcnow()
    }
    return jwt.encode(payload, JWT_SECRET, algorithm=JWT_ALGORITHM)


def decode_token(token: str) -> dict:
    """
    Decode and verify JWT token

    Raises:
        HTTPException: If token is invalid, expired, or malformed
    """
    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[JWT_ALGORITHM])
        return payload
    except jwt.ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired"
        )
    except jwt.InvalidTokenError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token"
        )


# ============================================================================
# AUTHENTICATION DEPENDENCY
# ============================================================================

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> dict:
    """
    Extract and validate current user from JWT token

    Returns:
        dict: User payload from token
    """
    token = credentials.credentials
    payload = decode_token(token)

    # Verify token type
    if payload.get("type") != "access":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token type"
        )

    return payload


async def get_current_active_user(
    current_user: dict = Depends(get_current_user),
    request: Request = None
) -> dict:
    """
    Get current user and verify account is active
    TODO: Add database query to check is_active status
    """
    # In production, query database to verify user is_active
    # For now, assume token presence means active user

    # Log access for security audit
    if request:
        await log_user_access(
            user_id=current_user["user_id"],
            endpoint=request.url.path,
            method=request.method,
            ip_address=request.client.host if request.client else None
        )

    return current_user


# ============================================================================
# ROLE-BASED ACCESS CONTROL (RBAC)
# ============================================================================

def require_role(allowed_roles: List[UserRole]) -> Callable:
    """
    Decorator to enforce role-based access control

    Args:
        allowed_roles: List of roles permitted to access the endpoint

    Example:
        @require_role([UserRole.ADMIN, UserRole.SUPER_ADMIN])
        async def delete_user(user_id: str):
            ...
    """
    async def role_checker(
        current_user: dict = Depends(get_current_active_user)
    ) -> dict:
        user_role = current_user.get("role")

        if user_role not in [role.value for role in allowed_roles]:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail=f"Insufficient permissions. Required roles: {[r.value for r in allowed_roles]}"
            )

        return current_user

    return role_checker


# Convenience decorators
require_admin = require_role([UserRole.ADMIN, UserRole.SUPER_ADMIN])
require_super_admin = require_role([UserRole.SUPER_ADMIN])
require_moderator = require_role([UserRole.MODERATOR, UserRole.ADMIN, UserRole.SUPER_ADMIN])


# ============================================================================
# PERMISSION CHECKS
# ============================================================================

class Permissions:
    """Granular permission checks for specific actions"""

    @staticmethod
    def can_manage_users(user_role: str) -> bool:
        """Check if user can manage other users"""
        return UserRole.get_hierarchy_level(user_role) >= UserRole.get_hierarchy_level(UserRole.ADMIN)

    @staticmethod
    def can_view_system_logs(user_role: str) -> bool:
        """Check if user can view system logs"""
        return UserRole.get_hierarchy_level(user_role) >= UserRole.get_hierarchy_level(UserRole.ADMIN)

    @staticmethod
    def can_manage_content(user_role: str) -> bool:
        """Check if user can manage content/chapters"""
        return UserRole.get_hierarchy_level(user_role) >= UserRole.get_hierarchy_level(UserRole.ADMIN)

    @staticmethod
    def can_view_chats(user_role: str) -> bool:
        """Check if user can view chat messages"""
        return UserRole.get_hierarchy_level(user_role) >= UserRole.get_hierarchy_level(UserRole.MODERATOR)

    @staticmethod
    def can_delete_chats(user_role: str) -> bool:
        """Check if user can delete chat messages"""
        return UserRole.get_hierarchy_level(user_role) >= UserRole.get_hierarchy_level(UserRole.MODERATOR)

    @staticmethod
    def can_reindex_rag(user_role: str) -> bool:
        """Check if user can trigger RAG re-indexing"""
        return UserRole.get_hierarchy_level(user_role) >= UserRole.get_hierarchy_level(UserRole.ADMIN)

    @staticmethod
    def can_modify_user_role(admin_role: str, target_role: str) -> bool:
        """
        Check if admin can modify a user with target_role
        Rule: Admin can only modify users with lower or equal role
        """
        admin_level = UserRole.get_hierarchy_level(admin_role)
        target_level = UserRole.get_hierarchy_level(target_role)

        # Super admin can modify anyone
        if admin_role == UserRole.SUPER_ADMIN:
            return True

        # Admin can modify users and moderators, but not other admins
        return admin_level > target_level


# ============================================================================
# RATE LIMITING
# ============================================================================

class RateLimiter:
    """Rate limiting for admin endpoints"""

    # In production, use Redis for distributed rate limiting
    # This is a simple in-memory implementation

    _requests = {}  # {user_id: [(timestamp, endpoint), ...]}

    @classmethod
    def check_rate_limit(
        cls,
        user_id: str,
        endpoint: str,
        max_requests: int = 100,
        window_seconds: int = 3600
    ) -> bool:
        """
        Check if user has exceeded rate limit

        Returns:
            True if request is allowed, False if rate limit exceeded
        """
        now = datetime.utcnow()

        # Initialize user entry
        if user_id not in cls._requests:
            cls._requests[user_id] = []

        # Clean old requests outside the window
        cls._requests[user_id] = [
            (ts, ep) for ts, ep in cls._requests[user_id]
            if (now - ts).total_seconds() < window_seconds
        ]

        # Count requests for this endpoint
        endpoint_requests = [
            (ts, ep) for ts, ep in cls._requests[user_id]
            if ep == endpoint
        ]

        if len(endpoint_requests) >= max_requests:
            return False

        # Add current request
        cls._requests[user_id].append((now, endpoint))
        return True


def rate_limit(max_requests: int = 100, window_seconds: int = 3600):
    """
    Rate limit decorator for admin endpoints

    Args:
        max_requests: Maximum requests allowed in window
        window_seconds: Time window in seconds
    """
    async def rate_limit_dependency(
        current_user: dict = Depends(get_current_active_user),
        request: Request = None
    ):
        user_id = current_user["user_id"]
        endpoint = request.url.path if request else "unknown"

        if not RateLimiter.check_rate_limit(user_id, endpoint, max_requests, window_seconds):
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail=f"Rate limit exceeded. Max {max_requests} requests per {window_seconds}s"
            )

        return current_user

    return rate_limit_dependency


# ============================================================================
# AUDIT LOGGING
# ============================================================================

async def log_admin_action(
    admin_id: str,
    action_type: str,
    target_type: Optional[str] = None,
    target_id: Optional[str] = None,
    details: Optional[dict] = None,
    ip_address: Optional[str] = None
):
    """
    Log administrative action to database

    Args:
        admin_id: UUID of admin performing action
        action_type: Type of action (e.g., USER_DEACTIVATED)
        target_type: Type of target (e.g., user, chat_message)
        target_id: ID of target
        details: Additional details as JSON
        ip_address: IP address of admin
    """
    # TODO: Implement database insert
    # This is a critical security feature - all admin actions must be logged

    print(f"[AUDIT] Admin {admin_id} performed {action_type} on {target_type}:{target_id}")

    # In production:
    # await db.execute(
    #     "INSERT INTO admin_actions (admin_id, action_type, target_type, target_id, details, ip_address) VALUES (...)"
    # )


async def log_user_access(
    user_id: str,
    endpoint: str,
    method: str,
    ip_address: Optional[str] = None
):
    """Log user access for security monitoring"""
    # TODO: Implement access logging
    # Can be used for intrusion detection, unusual access patterns, etc.
    pass


# ============================================================================
# SECURITY HEADERS MIDDLEWARE
# ============================================================================

async def add_security_headers(request: Request, call_next):
    """Add security headers to all responses"""
    response = await call_next(request)

    # Security headers
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-Frame-Options"] = "DENY"
    response.headers["X-XSS-Protection"] = "1; mode=block"
    response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
    response.headers["Content-Security-Policy"] = "default-src 'self'"

    return response


# ============================================================================
# IP WHITELIST (Optional for production)
# ============================================================================

ADMIN_IP_WHITELIST = os.getenv("ADMIN_IP_WHITELIST", "").split(",")

async def check_ip_whitelist(request: Request):
    """
    Check if request IP is in admin whitelist
    Only enable this in production if you have static admin IPs
    """
    if not ADMIN_IP_WHITELIST or ADMIN_IP_WHITELIST == [""]:
        return  # Whitelist disabled

    client_ip = request.client.host if request.client else None

    if client_ip not in ADMIN_IP_WHITELIST:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: IP not whitelisted"
        )


# ============================================================================
# USAGE EXAMPLES
# ============================================================================

"""
Example 1: Protect admin endpoint with role check

@router.get("/admin/users")
async def get_all_users(
    current_user: dict = Depends(require_admin)
):
    # Only ADMIN and SUPER_ADMIN can access
    return {"users": [...]}


Example 2: Custom permission check

@router.delete("/admin/users/{user_id}")
async def delete_user(
    user_id: str,
    current_user: dict = Depends(require_admin)
):
    if not Permissions.can_manage_users(current_user["role"]):
        raise HTTPException(403, "Cannot manage users")

    # Delete user logic
    await log_admin_action(
        admin_id=current_user["user_id"],
        action_type="USER_DELETED",
        target_type="user",
        target_id=user_id
    )


Example 3: Rate limited endpoint

@router.post("/admin/rag/reindex")
async def reindex_rag(
    current_user: dict = Depends(rate_limit(max_requests=5, window_seconds=3600)),
    _: dict = Depends(require_admin)
):
    # Only 5 reindex requests per hour
    # Trigger RAG reindexing
    pass
"""
