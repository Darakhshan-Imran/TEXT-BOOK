from fastapi import APIRouter, Depends
from src.auth.dependencies import get_current_user
from src.database.models import User
from src.schemas.auth import UserResponse

router = APIRouter()


@router.get("/profile", response_model=UserResponse)
async def get_user_profile(current_user: User = Depends(get_current_user)):
    """
    Get the profile of the currently authenticated user.

    Args:
        current_user (User): The currently authenticated user

    Returns:
        UserResponse: The user profile information
    """
    # Convert the User model to a dictionary to allow Pydantic to handle UUID conversion
    user_dict = {
        "email": current_user.email,
        "first_name": current_user.first_name,
        "last_name": current_user.last_name,
        "id": current_user.id,
        "uuid": str(current_user.uuid),  # Convert UUID to string
        "is_active": current_user.is_active,
        "is_verified": current_user.is_verified,
        "auth_provider": current_user.auth_provider,
        "created_at": current_user.created_at
    }
    return user_dict