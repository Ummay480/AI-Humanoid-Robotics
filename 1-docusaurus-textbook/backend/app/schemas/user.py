"""User-related Pydantic schemas."""
from pydantic import BaseModel, EmailStr
from typing import Optional
from datetime import datetime

class UserCreate(BaseModel):
    """Schema for user creation."""
    email: EmailStr
    password: str
    name: Optional[str] = None

class UserLogin(BaseModel):
    """Schema for user login."""
    email: EmailStr
    password: str

class UserResponse(BaseModel):
    """Schema for user response."""
    id: str
    email: EmailStr
    name: Optional[str] = None
    role: Optional[str] = "user"
    created_at: Optional[datetime] = None