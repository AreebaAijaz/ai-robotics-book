"""User router for profile management."""

from fastapi import APIRouter, HTTPException, status, Depends

from models.user import (
    UserResponse,
    UserProfileResponse,
    UserProfileUpdate,
    UserProfileCreate,
    MessageResponse,
)
from services.user_service import user_service
from middleware.auth import get_current_user

router = APIRouter(prefix="/api/user", tags=["User"])


@router.get("/me", response_model=UserResponse)
async def get_current_user_info(current_user: dict = Depends(get_current_user)):
    """Get current user's information.

    Returns:
        UserResponse with user data.

    Raises:
        HTTPException: If user not found.
    """
    user = user_service.get_user_by_id(current_user["user_id"])

    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={"error": "user_not_found", "message": "User not found"},
        )

    return user


@router.get("/profile", response_model=UserProfileResponse)
async def get_profile(current_user: dict = Depends(get_current_user)):
    """Get current user's profile (questionnaire responses).

    Returns:
        UserProfileResponse with profile data.

    Raises:
        HTTPException: If profile not found.
    """
    profile = user_service.get_profile(current_user["user_id"])

    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={"error": "profile_not_found", "message": "Profile not found. Please complete the questionnaire."},
        )

    return profile


@router.post("/profile", response_model=UserProfileResponse, status_code=status.HTTP_201_CREATED)
async def create_profile(
    profile: UserProfileCreate,
    current_user: dict = Depends(get_current_user)
):
    """Create user profile (questionnaire responses).

    Args:
        profile: Profile data from questionnaire.

    Returns:
        UserProfileResponse with created profile data.

    Raises:
        HTTPException: If profile already exists or creation fails.
    """
    # Check if profile already exists
    existing = user_service.get_profile(current_user["user_id"])
    if existing:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail={"error": "profile_exists", "message": "Profile already exists. Use PUT to update."},
        )

    # Create profile using the user service
    from services.postgres import postgres_service
    from datetime import datetime, timezone

    conn = postgres_service.get_raw_connection()
    if not conn:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "database_error", "message": "Database connection failed"},
        )

    try:
        now = datetime.now(timezone.utc)
        with conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO user_profiles (
                    user_id, expertise_level, interests, learning_goals,
                    preferred_format, time_availability, challenges,
                    device_preference, created_at, updated_at
                )
                VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
                RETURNING user_id, expertise_level, interests, learning_goals,
                          preferred_format, time_availability, challenges,
                          device_preference, created_at, updated_at
                """,
                (
                    current_user["user_id"],
                    profile.expertise_level,
                    profile.interests,
                    profile.learning_goals,
                    profile.preferred_format,
                    profile.time_availability,
                    profile.challenges,
                    profile.device_preference,
                    now,
                    now,
                )
            )
            row = cur.fetchone()
            conn.commit()

            return UserProfileResponse(
                user_id=row[0],
                expertise_level=row[1],
                interests=row[2] or [],
                learning_goals=row[3],
                preferred_format=row[4],
                time_availability=row[5],
                challenges=row[6],
                device_preference=row[7],
                created_at=row[8],
                updated_at=row[9],
            )

    except Exception as e:
        conn.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "creation_failed", "message": str(e)},
        )
    finally:
        postgres_service.return_connection(conn)


@router.put("/profile", response_model=UserProfileResponse)
async def update_profile(
    updates: UserProfileUpdate,
    current_user: dict = Depends(get_current_user)
):
    """Update current user's profile.

    Args:
        updates: Profile fields to update.

    Returns:
        UserProfileResponse with updated profile data.

    Raises:
        HTTPException: If update fails.
    """
    profile, error = user_service.update_profile(
        user_id=current_user["user_id"],
        updates=updates,
    )

    if error:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "update_failed", "message": error},
        )

    return profile


@router.delete("/me", response_model=MessageResponse)
async def delete_account(current_user: dict = Depends(get_current_user)):
    """Delete current user's account and all associated data.

    Returns:
        Success message.

    Raises:
        HTTPException: If deletion fails.
    """
    success, error = user_service.delete_user(current_user["user_id"])

    if not success:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "deletion_failed", "message": error or "Failed to delete account"},
        )

    return MessageResponse(message="Account deleted successfully")
