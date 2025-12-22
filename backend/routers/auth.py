"""Authentication router for user signup, login, and password management."""

from fastapi import APIRouter, HTTPException, status, Depends

from models.user import (
    UserCreate,
    UserLogin,
    AuthResponse,
    ForgotPasswordRequest,
    ResetPasswordRequest,
    MessageResponse,
)
from services.user_service import user_service
from services.auth_service import (
    create_access_token,
    create_password_reset_token,
    verify_password_reset_token,
)
from middleware.auth import get_current_user

router = APIRouter(prefix="/api/auth", tags=["Authentication"])


@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def signup(request: UserCreate):
    """Register a new user with email, password, and optional profile.

    Args:
        request: User creation data including email, password, and questionnaire.

    Returns:
        AuthResponse with token and user data.

    Raises:
        HTTPException: If email already exists or validation fails.
    """
    # Validate password strength
    password = request.password
    if len(password) < 8:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"error": "weak_password", "message": "Password must be at least 8 characters"},
        )
    if not any(c.isupper() for c in password) or not any(c.islower() for c in password):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"error": "weak_password", "message": "Password must contain uppercase and lowercase letters"},
        )
    if not any(c.isdigit() for c in password):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"error": "weak_password", "message": "Password must contain at least one number"},
        )

    # Create user
    user, error = user_service.create_user(
        email=request.email,
        password=request.password,
        profile=request.profile,
    )

    if error:
        if "already registered" in error.lower():
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail={"error": "email_exists", "message": error},
            )
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "registration_failed", "message": error},
        )

    # Create access token
    token, expires_at = create_access_token(user.id, user.email)

    return AuthResponse(
        token=token,
        user=user,
        expires_at=expires_at,
    )


@router.post("/login", response_model=AuthResponse)
async def login(request: UserLogin):
    """Authenticate user with email and password.

    Args:
        request: Login credentials.

    Returns:
        AuthResponse with token and user data.

    Raises:
        HTTPException: If credentials are invalid.
    """
    user, error = user_service.authenticate_user(
        email=request.email,
        password=request.password,
    )

    if error:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error": "invalid_credentials", "message": error},
        )

    # Create access token
    token, expires_at = create_access_token(user.id, user.email)

    return AuthResponse(
        token=token,
        user=user,
        expires_at=expires_at,
    )


@router.post("/logout", response_model=MessageResponse)
async def logout(current_user: dict = Depends(get_current_user)):
    """Log out the current user.

    Note: Since we use stateless JWT tokens, logout is handled client-side
    by removing the token. This endpoint is for API completeness.

    Returns:
        Success message.
    """
    # In a stateless JWT system, logout is client-side.
    # For stateful sessions, we would invalidate the token here.
    return MessageResponse(message="Logged out successfully")


@router.post("/forgot-password", response_model=MessageResponse)
async def forgot_password(request: ForgotPasswordRequest):
    """Request a password reset email.

    Args:
        request: Email address for password reset.

    Returns:
        Success message (always returns success to prevent email enumeration).
    """
    # Find user by email
    user = user_service.get_user_by_email(request.email)

    if user:
        # Create reset token
        reset_token = create_password_reset_token(user.id, user.email)
        # TODO: Send email with reset link
        # For now, we just log the token (in production, send email)
        print(f"Password reset token for {user.email}: {reset_token}")

    # Always return success to prevent email enumeration
    return MessageResponse(
        message="If an account exists with this email, a password reset link has been sent."
    )


@router.post("/reset-password", response_model=MessageResponse)
async def reset_password(request: ResetPasswordRequest):
    """Reset password using a reset token.

    Args:
        request: Reset token and new password.

    Returns:
        Success message.

    Raises:
        HTTPException: If token is invalid or expired.
    """
    # Verify reset token
    payload = verify_password_reset_token(request.token)
    if not payload:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"error": "invalid_token", "message": "Reset token is invalid or expired"},
        )

    # Validate new password
    password = request.new_password
    if len(password) < 8:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"error": "weak_password", "message": "Password must be at least 8 characters"},
        )

    # Update password
    user_id = payload.get("sub")
    success, error = user_service.update_password(user_id, password)

    if not success:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "reset_failed", "message": error or "Failed to reset password"},
        )

    return MessageResponse(message="Password reset successfully")
