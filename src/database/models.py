from sqlalchemy import Boolean, Column, Integer, String, DateTime, Text, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID
import uuid

Base = declarative_base()


class User(Base):
    """User model for storing user information"""
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    uuid = Column(UUID(as_uuid=True), unique=True, index=True, default=uuid.uuid4)
    email = Column(String, unique=True, index=True, nullable=False)
    hashed_password = Column(String, nullable=True)  # Null for OAuth users
    first_name = Column(String, nullable=True)
    last_name = Column(String, nullable=True)
    is_active = Column(Boolean, default=True)
    is_verified = Column(Boolean, default=False)
    auth_provider = Column(String, nullable=False)  # 'email', 'google', 'github'
    provider_id = Column(String, nullable=True)  # For OAuth users
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class Session(Base):
    """Session model for storing active user sessions"""
    __tablename__ = "sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False)
    token = Column(String, unique=True, nullable=False)  # Hashed token
    expires_at = Column(DateTime(timezone=True), nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    last_accessed = Column(DateTime(timezone=True), onupdate=func.now())
    device_info = Column(Text, nullable=True)
    ip_address = Column(String, nullable=True)


class OAuthAccount(Base):
    """OAuth account model for storing OAuth provider information"""
    __tablename__ = "oauth_accounts"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False)
    provider = Column(String, nullable=False)  # 'google', 'github'
    provider_account_id = Column(String, nullable=False)
    provider_account_email = Column(String, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    __table_args__ = (
        # Ensure combination of provider and provider_account_id is unique
        # This prevents duplicate OAuth accounts for the same provider
        # Example: Same Google account can't be linked to multiple users
        # But the same email from different providers (Google vs GitHub) is allowed
    )