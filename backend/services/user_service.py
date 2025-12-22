"""User service for database operations."""

from datetime import datetime, timezone
from typing import Optional
import uuid

from services.postgres import postgres_service
from services.auth_service import hash_password, verify_password
from models.user import (
    UserCreate,
    UserResponse,
    UserProfileCreate,
    UserProfileResponse,
    UserProfileUpdate,
)


class UserService:
    """Service for user and profile database operations."""

    def create_user(
        self,
        email: str,
        password: str,
        profile: Optional[UserProfileCreate] = None
    ) -> tuple[Optional[UserResponse], Optional[str]]:
        """Create a new user with optional profile.

        Args:
            email: User's email address.
            password: Plain text password (will be hashed).
            profile: Optional profile data from questionnaire.

        Returns:
            Tuple of (UserResponse or None, error message or None).
        """
        conn = postgres_service.get_raw_connection()
        if not conn:
            return None, "Database connection failed"

        try:
            with conn.cursor() as cur:
                # Check if email already exists
                cur.execute("SELECT id FROM users WHERE email = %s", (email,))
                if cur.fetchone():
                    return None, "Email already registered"

                # Create user
                user_id = str(uuid.uuid4())
                password_hash = hash_password(password)
                now = datetime.now(timezone.utc)

                cur.execute(
                    """
                    INSERT INTO users (id, email, password_hash, created_at, updated_at)
                    VALUES (%s, %s, %s, %s, %s)
                    RETURNING id, email, email_verified, created_at
                    """,
                    (user_id, email, password_hash, now, now)
                )
                row = cur.fetchone()

                # Create profile if provided
                if profile:
                    cur.execute(
                        """
                        INSERT INTO user_profiles (
                            user_id, expertise_level, interests, learning_goals,
                            preferred_format, time_availability, challenges,
                            device_preference, created_at, updated_at
                        )
                        VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
                        """,
                        (
                            user_id,
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

                conn.commit()

                return UserResponse(
                    id=row[0],
                    email=row[1],
                    email_verified=row[2],
                    created_at=row[3],
                ), None

        except Exception as e:
            conn.rollback()
            return None, str(e)
        finally:
            postgres_service.return_connection(conn)

    def authenticate_user(self, email: str, password: str) -> tuple[Optional[UserResponse], Optional[str]]:
        """Authenticate a user by email and password.

        Args:
            email: User's email address.
            password: Plain text password.

        Returns:
            Tuple of (UserResponse or None, error message or None).
        """
        conn = postgres_service.get_raw_connection()
        if not conn:
            return None, "Database connection failed"

        try:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    SELECT id, email, password_hash, email_verified, created_at
                    FROM users WHERE email = %s
                    """,
                    (email,)
                )
                row = cur.fetchone()

                if not row:
                    return None, "Invalid email or password"

                if not verify_password(password, row[2]):
                    return None, "Invalid email or password"

                return UserResponse(
                    id=row[0],
                    email=row[1],
                    email_verified=row[3],
                    created_at=row[4],
                ), None

        except Exception as e:
            return None, str(e)
        finally:
            postgres_service.return_connection(conn)

    def get_user_by_id(self, user_id: str) -> Optional[UserResponse]:
        """Get user by ID.

        Args:
            user_id: User's unique identifier.

        Returns:
            UserResponse or None if not found.
        """
        conn = postgres_service.get_raw_connection()
        if not conn:
            return None

        try:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    SELECT id, email, email_verified, created_at
                    FROM users WHERE id = %s
                    """,
                    (user_id,)
                )
                row = cur.fetchone()

                if not row:
                    return None

                return UserResponse(
                    id=row[0],
                    email=row[1],
                    email_verified=row[2],
                    created_at=row[3],
                )

        except Exception:
            return None
        finally:
            postgres_service.return_connection(conn)

    def get_user_by_email(self, email: str) -> Optional[UserResponse]:
        """Get user by email.

        Args:
            email: User's email address.

        Returns:
            UserResponse or None if not found.
        """
        conn = postgres_service.get_raw_connection()
        if not conn:
            return None

        try:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    SELECT id, email, email_verified, created_at
                    FROM users WHERE email = %s
                    """,
                    (email,)
                )
                row = cur.fetchone()

                if not row:
                    return None

                return UserResponse(
                    id=row[0],
                    email=row[1],
                    email_verified=row[2],
                    created_at=row[3],
                )

        except Exception:
            return None
        finally:
            postgres_service.return_connection(conn)

    def get_profile(self, user_id: str) -> Optional[UserProfileResponse]:
        """Get user profile by user ID.

        Args:
            user_id: User's unique identifier.

        Returns:
            UserProfileResponse or None if not found.
        """
        conn = postgres_service.get_raw_connection()
        if not conn:
            return None

        try:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    SELECT user_id, expertise_level, interests, learning_goals,
                           preferred_format, time_availability, challenges,
                           device_preference, created_at, updated_at
                    FROM user_profiles WHERE user_id = %s
                    """,
                    (user_id,)
                )
                row = cur.fetchone()

                if not row:
                    return None

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

        except Exception:
            return None
        finally:
            postgres_service.return_connection(conn)

    def update_profile(
        self,
        user_id: str,
        updates: UserProfileUpdate
    ) -> tuple[Optional[UserProfileResponse], Optional[str]]:
        """Update user profile.

        Args:
            user_id: User's unique identifier.
            updates: Profile fields to update.

        Returns:
            Tuple of (UserProfileResponse or None, error message or None).
        """
        conn = postgres_service.get_raw_connection()
        if not conn:
            return None, "Database connection failed"

        try:
            with conn.cursor() as cur:
                # Build dynamic update query
                update_fields = []
                values = []

                update_dict = updates.model_dump(exclude_unset=True)
                for field, value in update_dict.items():
                    if value is not None:
                        update_fields.append(f"{field} = %s")
                        values.append(value)

                if not update_fields:
                    # No fields to update, return current profile
                    return self.get_profile(user_id), None

                values.append(user_id)
                query = f"""
                    UPDATE user_profiles
                    SET {', '.join(update_fields)}, updated_at = NOW()
                    WHERE user_id = %s
                    RETURNING user_id, expertise_level, interests, learning_goals,
                              preferred_format, time_availability, challenges,
                              device_preference, created_at, updated_at
                """

                cur.execute(query, values)
                row = cur.fetchone()
                conn.commit()

                if not row:
                    return None, "Profile not found"

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
                ), None

        except Exception as e:
            conn.rollback()
            return None, str(e)
        finally:
            postgres_service.return_connection(conn)

    def update_password(self, user_id: str, new_password: str) -> tuple[bool, Optional[str]]:
        """Update user's password.

        Args:
            user_id: User's unique identifier.
            new_password: New plain text password.

        Returns:
            Tuple of (success boolean, error message or None).
        """
        conn = postgres_service.get_raw_connection()
        if not conn:
            return False, "Database connection failed"

        try:
            with conn.cursor() as cur:
                password_hash = hash_password(new_password)
                cur.execute(
                    """
                    UPDATE users SET password_hash = %s, updated_at = NOW()
                    WHERE id = %s
                    """,
                    (password_hash, user_id)
                )
                conn.commit()

                if cur.rowcount == 0:
                    return False, "User not found"

                return True, None

        except Exception as e:
            conn.rollback()
            return False, str(e)
        finally:
            postgres_service.return_connection(conn)

    def delete_user(self, user_id: str) -> tuple[bool, Optional[str]]:
        """Delete user and all associated data.

        Args:
            user_id: User's unique identifier.

        Returns:
            Tuple of (success boolean, error message or None).
        """
        conn = postgres_service.get_raw_connection()
        if not conn:
            return False, "Database connection failed"

        try:
            with conn.cursor() as cur:
                # Profile is deleted via CASCADE
                cur.execute("DELETE FROM users WHERE id = %s", (user_id,))
                conn.commit()

                if cur.rowcount == 0:
                    return False, "User not found"

                return True, None

        except Exception as e:
            conn.rollback()
            return False, str(e)
        finally:
            postgres_service.return_connection(conn)


# Singleton instance
user_service = UserService()
