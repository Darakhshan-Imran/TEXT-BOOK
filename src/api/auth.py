from fastapi import APIRouter, Depends, HTTPException, status, Request
from sqlalchemy.orm import Session
from typing import Optional

from src.database.connection import get_db
from src.schemas.auth import UserCreate, UserLogin, Token
from src.auth.services import register_user, authenticate_user, generate_token_for_user
from src.auth.dependencies import get_current_user
from src.database.models import User

router = APIRouter()


@router.post("/register", response_model=Token, status_code=status.HTTP_201_CREATED)
async def register(user_data: UserCreate, db: Session = Depends(get_db)):
    """
    Register a new user with email and password.

    Args:
        user_data (UserCreate): User registration data
        db (Session): Database session

    Returns:
        Token: JWT access token for the newly created user
    """
    # Register the user
    db_user = register_user(db, user_data)

    # Generate JWT token for the new user
    access_token = generate_token_for_user(db_user)

    # Return the token
    return Token(access_token=access_token, token_type="bearer")


@router.post("/login", response_model=Token)
async def login(user_credentials: UserLogin, db: Session = Depends(get_db)):
    """
    Login a user with email and password.

    Args:
        user_credentials (UserLogin): User login credentials
        db (Session): Database session

    Returns:
        Token: JWT access token for the authenticated user
    """
    # Authenticate the user
    user = authenticate_user(db, user_credentials.email, user_credentials.password)

    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Generate JWT token for the authenticated user
    access_token = generate_token_for_user(user)

    # Return the token
    return Token(access_token=access_token, token_type="bearer")


@router.get("/profile")
async def get_profile(current_user: User = Depends(get_current_user)):
    """
    Get the profile of the currently authenticated user.

    Args:
        current_user (User): The currently authenticated user

    Returns:
        User: The user profile information
    """
    return current_user


@router.post("/logout")
async def logout():
    """
    Logout the current user.
    In a stateless JWT system, the client is responsible for removing the token.
    This endpoint can be extended to add the token to a blacklist in future implementations.
    """
    return {"message": "Successfully logged out"}