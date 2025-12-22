"""Authentication service for JWT token management and password hashing."""

from datetime import datetime, timedelta, timezone
from typing import Optional
import uuid

import jwt
from passlib.context import CryptContext

from config import settings

# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


def _truncate_password(password: str) -> str:
    """Truncate password to 72 bytes for bcrypt compatibility.

    bcrypt only uses the first 72 bytes of a password.
    Truncating explicitly avoids warnings and ensures consistent behavior.
    """
    return password.encode('utf-8')[:72].decode('utf-8', errors='ignore')


def hash_password(password: str) -> str:
    """Hash a plain text password.

    Args:
        password: Plain text password.

    Returns:
        Hashed password string.
    """
    return pwd_context.hash(_truncate_password(password))


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a password against its hash.

    Args:
        plain_password: Plain text password to verify.
        hashed_password: Stored password hash.

    Returns:
        True if password matches, False otherwise.
    """
    return pwd_context.verify(_truncate_password(plain_password), hashed_password)


def create_access_token(user_id: str, email: str) -> tuple[str, datetime]:
    """Create a JWT access token.

    Args:
        user_id: User's unique identifier.
        email: User's email address.

    Returns:
        Tuple of (token string, expiration datetime).
    """
    expires_at = datetime.now(timezone.utc) + timedelta(days=settings.JWT_EXPIRATION_DAYS)

    payload = {
        "sub": user_id,
        "email": email,
        "exp": expires_at,
        "iat": datetime.now(timezone.utc),
        "jti": str(uuid.uuid4()),  # Unique token ID
    }

    token = jwt.encode(
        payload,
        settings.JWT_SECRET_KEY,
        algorithm=settings.JWT_ALGORITHM
    )

    return token, expires_at


def verify_access_token(token: str) -> Optional[dict]:
    """Verify and decode a JWT access token.

    Args:
        token: JWT token string.

    Returns:
        Decoded token payload if valid, None otherwise.
    """
    try:
        payload = jwt.decode(
            token,
            settings.JWT_SECRET_KEY,
            algorithms=[settings.JWT_ALGORITHM]
        )
        return payload
    except jwt.ExpiredSignatureError:
        return None
    except jwt.InvalidTokenError:
        return None


def create_password_reset_token(user_id: str, email: str) -> str:
    """Create a password reset token.

    Args:
        user_id: User's unique identifier.
        email: User's email address.

    Returns:
        Password reset token string.
    """
    expires_at = datetime.now(timezone.utc) + timedelta(hours=settings.PASSWORD_RESET_EXPIRATION_HOURS)

    payload = {
        "sub": user_id,
        "email": email,
        "exp": expires_at,
        "type": "password_reset",
    }

    return jwt.encode(
        payload,
        settings.JWT_SECRET_KEY,
        algorithm=settings.JWT_ALGORITHM
    )


def verify_password_reset_token(token: str) -> Optional[dict]:
    """Verify a password reset token.

    Args:
        token: Password reset token string.

    Returns:
        Decoded token payload if valid, None otherwise.
    """
    try:
        payload = jwt.decode(
            token,
            settings.JWT_SECRET_KEY,
            algorithms=[settings.JWT_ALGORITHM]
        )
        if payload.get("type") != "password_reset":
            return None
        return payload
    except jwt.ExpiredSignatureError:
        return None
    except jwt.InvalidTokenError:
        return None
