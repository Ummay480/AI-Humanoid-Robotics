"""Authentication router for Better Auth integration."""
import logging
from datetime import datetime, timedelta
from typing import Optional

from fastapi import APIRouter, HTTPException, Depends, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import jwt
from pydantic import BaseModel

from app.config import settings
from app.schemas.user import UserCreate, UserLogin, UserResponse

logger = logging.getLogger(__name__)

router = APIRouter()
security = HTTPBearer()

# In-memory user store (replace with database in production)
users_db = {}

class TokenData(BaseModel):
    user_id: str
    email: str
    exp: int

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """Create JWT access token."""
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(hours=settings.jwt_expiration_hours)

    to_encode.update({"exp": expire.timestamp()})
    encoded_jwt = jwt.encode(to_encode, settings.jwt_secret_key, algorithm=settings.jwt_algorithm)
    return encoded_jwt

def verify_token(token: str) -> TokenData:
    """Verify and decode JWT token."""
    try:
        payload = jwt.decode(token, settings.jwt_secret_key, algorithms=[settings.jwt_algorithm])
        user_id: str = payload.get("user_id")
        email: str = payload.get("email")

        if user_id is None or email is None:
            raise HTTPException(status_code=401, detail="Invalid token")

        return TokenData(user_id=user_id, email=email, exp=payload.get("exp"))
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token has expired")
    except jwt.JWTError:
        raise HTTPException(status_code=401, detail="Invalid token")

def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """Get current user from JWT token."""
    token_data = verify_token(credentials.credentials)
    # In a real implementation, you'd fetch user from database
    user = users_db.get(token_data.user_id)
    if user is None:
        raise HTTPException(status_code=401, detail="User not found")
    return user

@router.post("/auth/register", response_model=UserResponse)
async def register(user_data: UserCreate):
    """Register a new user."""
    logger.info(f"Registering user: {user_data.email}")

    # Check if user already exists
    for user_id, user in users_db.items():
        if user["email"] == user_data.email:
            raise HTTPException(status_code=400, detail="Email already registered")

    # Create new user (in memory for now)
    user_id = f"user_{len(users_db) + 1}"
    user = {
        "id": user_id,
        "email": user_data.email,
        "name": user_data.name,
        "password": user_data.password,  # In production, hash this!
        "created_at": datetime.utcnow().isoformat(),
        "role": "user"  # Default role
    }

    users_db[user_id] = user
    logger.info(f"User registered successfully: {user_id}")

    return UserResponse(
        id=user["id"],
        email=user["email"],
        name=user["name"],
        role=user["role"]
    )

@router.post("/auth/login")
async def login(user_data: UserLogin):
    """Login user and return JWT token."""
    logger.info(f"Login attempt for: {user_data.email}")

    # Find user by email
    user = None
    user_id = None
    for uid, u in users_db.items():
        if u["email"] == user_data.email:
            user = u
            user_id = uid
            break

    if user is None or user["password"] != user_data.password:  # In production, verify hashed password
        raise HTTPException(status_code=401, detail="Invalid credentials")

    # Create access token
    access_token_expires = timedelta(hours=settings.jwt_expiration_hours)
    access_token = create_access_token(
        data={"user_id": user_id, "email": user["email"]},
        expires_delta=access_token_expires
    )

    logger.info(f"User logged in successfully: {user_id}")

    return {
        "access_token": access_token,
        "token_type": "bearer",
        "user": UserResponse(
            id=user["id"],
            email=user["email"],
            name=user["name"],
            role=user["role"]
        )
    }

@router.post("/auth/logout")
async def logout(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """Logout user (client-side token invalidation)."""
    # In a real implementation, you might add tokens to a blacklist
    logger.info("User logged out")
    return {"message": "Successfully logged out"}

@router.get("/auth/me", response_model=UserResponse)
async def get_user(current_user: dict = Depends(get_current_user)):
    """Get current user info."""
    return UserResponse(
        id=current_user["id"],
        email=current_user["email"],
        name=current_user["name"],
        role=current_user["role"]
    )