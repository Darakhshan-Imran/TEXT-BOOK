from pydantic import BaseModel, EmailStr
from typing import Optional
from datetime import datetime


class UserBase(BaseModel):
    """Base model for user data"""
    email: EmailStr
    first_name: Optional[str] = None
    last_name: Optional[str] = None


class UserCreate(UserBase):
    """Model for user creation with optional password"""
    password: Optional[str] = None
    auth_provider: str = "email"  # Default to email provider


class UserLogin(BaseModel):
    """Model for user login"""
    email: EmailStr
    password: str


class Token(BaseModel):
    """Model for JWT token response"""
    access_token: str
    token_type: str = "bearer"


class TokenData(BaseModel):
    """Model for JWT token data"""
    user_uuid: Optional[str] = None
    email: Optional[str] = None


class UserResponse(UserBase):
    """Model for user response without sensitive data"""
    id: int
    uuid: str
    is_active: bool
    is_verified: bool
    auth_provider: str
    created_at: datetime

    class Config:
        from_attributes = True