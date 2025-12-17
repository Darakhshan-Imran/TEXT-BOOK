import logging
from typing import Optional
from passlib.context import CryptContext

# Create a password context for hashing passwords with fallback schemes
# Using multiple schemes to handle potential backend issues
try:
    pwd_context = CryptContext(
        schemes=["bcrypt", "argon2", "pbkdf2_sha256"],
        deprecated="auto",
        bcrypt__rounds=12,
    )
    # Test the bcrypt backend during initialization
    _test_hash = pwd_context.hash("test_password_for_init")
except Exception as e:
    logging.warning(f"Primary password hashing context failed to initialize: {e}")
    # Fallback to pbkdf2 only if bcrypt fails
    pwd_context = CryptContext(
        schemes=["pbkdf2_sha256"],
        deprecated="auto"
    )


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a plain password against a hashed password.

    Args:
        plain_password (str): The plain text password to verify
        hashed_password (str): The hashed password to compare against

    Returns:
        bool: True if the password matches, False otherwise
    """
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    """
    Generate a hash for a plain text password.

    Args:
        password (str): The plain text password to hash

    Returns:
        str: The hashed password
    """
    # Truncate password to 72 bytes to comply with bcrypt limit
    truncated_password = password[:72] if len(password) > 72 else password
    return pwd_context.hash(truncated_password)