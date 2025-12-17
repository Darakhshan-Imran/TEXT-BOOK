from sqlalchemy.orm import Session
from sqlalchemy.exc import IntegrityError
from fastapi import HTTPException, status
from typing import Optional
import uuid

from src.database.models import User
from src.auth.password import get_password_hash, verify_password
from src.auth.jwt import create_access_token
from src.schemas.auth import UserCreate, UserLogin
from src.utils.validators import validate_email_format, validate_password_strength


def register_user(db: Session, user_data: UserCreate):
    """
    Register a new user. Handles both email/password registration and OAuth registration.

    Args:
        db (Session): Database session
        user_data (UserCreate): User registration data

    Returns:
        User: The created user object

    Raises:
        HTTPException: If email is invalid, password is weak, or email already exists
    """
    # Validate email format
    if not validate_email_format(user_data.email):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid email format"
        )

    # Check if user already exists
    existing_user = db.query(User).filter(User.email == user_data.email).first()
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="A user with this email already exists"
        )

    # Handle different auth providers
    if user_data.auth_provider in ["google", "github"]:
        # OAuth registration - no password required
        db_user = User(
            email=user_data.email,
            hashed_password=None,  # OAuth users don't have local passwords
            first_name=user_data.first_name,
            last_name=user_data.last_name,
            auth_provider=user_data.auth_provider,
            is_verified=True,  # OAuth users are considered verified
            provider_id=None  # Will be set when OAuth flow is completed
        )
    else:
        # Email/password registration - validate and hash password
        is_valid, error_msg = validate_password_strength(user_data.password)
        if not is_valid:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=error_msg
            )

        hashed_password = get_password_hash(user_data.password)
        db_user = User(
            email=user_data.email,
            hashed_password=hashed_password,
            first_name=user_data.first_name,
            last_name=user_data.last_name,
            auth_provider=user_data.auth_provider,
            is_verified=False  # Email verification would happen later
        )

    try:
        db.add(db_user)
        db.commit()
        db.refresh(db_user)
        return db_user
    except IntegrityError:
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="A user with this email already exists"
        )


def authenticate_user(db: Session, email: str, password: str):
    """
    Authenticate a user with email and password.
    Note: This is only for email/password users, not OAuth users.

    Args:
        db (Session): Database session
        email (str): User's email
        password (str): User's password

    Returns:
        User: The authenticated user object, or None if authentication fails
    """
    # Find user by email
    user = db.query(User).filter(User.email == email).first()

    # Check if user exists
    if not user:
        return None

    # Check if user has a password (OAuth users don't have local passwords)
    if not user.hashed_password:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="This account uses OAuth. Please log in with your OAuth provider."
        )

    # Check if password is correct
    if not verify_password(password, user.hashed_password):
        return None

    # Check if user is active
    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Account is deactivated"
        )

    return user


def generate_token_for_user(user: User) -> str:
    """
    Generate a JWT token for an authenticated user.

    Args:
        user (User): The authenticated user

    Returns:
        str: The JWT access token
    """
    # Create data to encode in the token
    data = {
        "sub": str(user.uuid),  # Using UUID as the subject
        "user_id": user.id,
        "email": user.email,
        "auth_provider": user.auth_provider
    }

    # Create and return the access token
    token = create_access_token(data=data)
    return token


def create_or_get_oauth_user(db: Session, email: str, provider: str, provider_id: str,
                           first_name: Optional[str] = None, last_name: Optional[str] = None):
    """
    Create a new user from OAuth provider or return existing user if already exists.

    Args:
        db (Session): Database session
        email (str): User's email from OAuth provider
        provider (str): OAuth provider ('google', 'github')
        provider_id (str): User's ID from OAuth provider
        first_name (Optional[str]): User's first name
        last_name (Optional[str]): User's last name

    Returns:
        User: The user object (newly created or existing)
    """
    # First, try to find if user already exists with this email
    existing_user = db.query(User).filter(User.email == email).first()

    if existing_user:
        # If user exists, check if they authenticated with the same provider
        # If not, we might want to link the accounts, but for now we'll just return the existing user
        return existing_user

    # Create new user
    db_user = User(
        email=email,
        first_name=first_name,
        last_name=last_name,
        auth_provider=provider,
        provider_id=provider_id,
        is_verified=True  # OAuth users are considered verified
    )

    try:
        db.add(db_user)
        db.commit()
        db.refresh(db_user)
        return db_user
    except IntegrityError:
        db.rollback()
        # If there's an integrity error, the user might have been created in the meantime
        # Try to fetch the user again
        existing_user = db.query(User).filter(User.email == email).first()
        if existing_user:
            return existing_user
        else:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to create user from OAuth provider"
            )