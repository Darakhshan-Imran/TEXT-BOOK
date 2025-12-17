import re
from typing import Optional


def validate_email_format(email: str) -> bool:
    """
    Validate the format of an email address using a regular expression.

    Args:
        email (str): The email address to validate

    Returns:
        bool: True if the email format is valid, False otherwise
    """
    # Regular expression pattern for email validation
    # This pattern checks for:
    # - At least one character before @
    # - At least one dot after @
    # - Valid characters in the email address
    pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'

    return re.match(pattern, email) is not None


def validate_password_strength(password: str) -> tuple[bool, Optional[str]]:
    """
    Validate the strength of a password based on security requirements.

    Args:
        password (str): The password to validate

    Returns:
        tuple[bool, Optional[str]]: (is_valid, error_message)
    """
    # Check minimum length
    if len(password) < 8:
        return False, "Password must be at least 8 characters long"

    # Check for at least one uppercase letter
    if not re.search(r'[A-Z]', password):
        return False, "Password must contain at least one uppercase letter"

    # Check for at least one lowercase letter
    if not re.search(r'[a-z]', password):
        return False, "Password must contain at least one lowercase letter"

    # Check for at least one digit
    if not re.search(r'\d', password):
        return False, "Password must contain at least one digit"

    # Check for at least one special character
    if not re.search(r'[!@#$%^&*(),.?":{}|<>]', password):
        return False, "Password must contain at least one special character"

    return True, None