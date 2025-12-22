"""Authentication middleware for FastAPI."""

from typing import Optional

from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials

from services.auth_service import verify_access_token

# Bearer token security scheme
security = HTTPBearer(auto_error=False)


async def get_current_user(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security)
) -> dict:
    """Dependency to get the current authenticated user.

    Args:
        credentials: Bearer token from Authorization header.

    Returns:
        Decoded user data from token.

    Raises:
        HTTPException: If token is missing or invalid.
    """
    if not credentials:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error": "missing_token", "message": "Authentication required"},
            headers={"WWW-Authenticate": "Bearer"},
        )

    token = credentials.credentials
    payload = verify_access_token(token)

    if not payload:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error": "invalid_token", "message": "Token is invalid or expired"},
            headers={"WWW-Authenticate": "Bearer"},
        )

    return {
        "user_id": payload.get("sub"),
        "email": payload.get("email"),
    }


async def get_optional_user(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security)
) -> Optional[dict]:
    """Dependency to get the current user if authenticated, None otherwise.

    Args:
        credentials: Bearer token from Authorization header.

    Returns:
        Decoded user data from token, or None if not authenticated.
    """
    if not credentials:
        return None

    token = credentials.credentials
    payload = verify_access_token(token)

    if not payload:
        return None

    return {
        "user_id": payload.get("sub"),
        "email": payload.get("email"),
    }
