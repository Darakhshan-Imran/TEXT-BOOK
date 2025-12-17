from fastapi import APIRouter, Depends
from src.auth.dependencies import get_current_user
from src.database.models import User

router = APIRouter()


@router.get("/resource")
async def get_protected_resource(current_user: User = Depends(get_current_user)):
    """
    Example of a protected resource that requires authentication.

    Args:
        current_user (User): The currently authenticated user

    Returns:
        dict: Protected resource data
    """
    return {
        "message": "This is a protected resource",
        "user_email": current_user.email,
        "user_id": current_user.id
    }