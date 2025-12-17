from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
from typing import Optional

from src.database.connection import get_db
from src.auth.jwt import verify_token
from src.database.models import User

# Initialize the HTTP Bearer security scheme
security = HTTPBearer()


def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db: Session = Depends(get_db)
) -> User:
    """
    Dependency to get the current authenticated user from the JWT token.

    Args:
        credentials (HTTPAuthorizationCredentials): The authorization credentials from the request
        db (Session): Database session

    Returns:
        User: The authenticated user object

    Raises:
        HTTPException: If the token is invalid or the user doesn't exist
    """
    # Verify the token and get the payload
    token_payload = verify_token(credentials.credentials)

    # Extract user UUID from the token
    user_uuid = token_payload.get("sub")

    if user_uuid is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Query the database for the user
    user = db.query(User).filter(User.uuid == user_uuid).first()

    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return user