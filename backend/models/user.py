"""User and UserProfile models for authentication and personalization."""

from datetime import datetime
from typing import Optional
from pydantic import BaseModel, EmailStr, Field


class UserCreate(BaseModel):
    """Request model for user registration."""
    email: EmailStr
    password: str = Field(..., min_length=8)
    profile: Optional["UserProfileCreate"] = None


class UserLogin(BaseModel):
    """Request model for user login."""
    email: EmailStr
    password: str


class UserProfileCreate(BaseModel):
    """Profile data from the 7-question questionnaire."""
    expertise_level: str = Field(..., pattern="^(beginner|intermediate|advanced|expert)$")
    interests: list[str] = Field(default_factory=list)
    learning_goals: str = Field(..., pattern="^(casual|deep_understanding|practical|research)$")
    preferred_format: str = Field(..., pattern="^(text|diagrams|videos|code|interactive)$")
    time_availability: str = Field(..., pattern="^(quick_5min|medium_15min|deep_30min)$")
    challenges: Optional[str] = None
    device_preference: str = Field(..., pattern="^(desktop|tablet|mobile)$")


class UserProfileUpdate(BaseModel):
    """Request model for updating user profile."""
    expertise_level: Optional[str] = Field(None, pattern="^(beginner|intermediate|advanced|expert)$")
    interests: Optional[list[str]] = None
    learning_goals: Optional[str] = Field(None, pattern="^(casual|deep_understanding|practical|research)$")
    preferred_format: Optional[str] = Field(None, pattern="^(text|diagrams|videos|code|interactive)$")
    time_availability: Optional[str] = Field(None, pattern="^(quick_5min|medium_15min|deep_30min)$")
    challenges: Optional[str] = None
    device_preference: Optional[str] = Field(None, pattern="^(desktop|tablet|mobile)$")


class UserResponse(BaseModel):
    """Response model for user data."""
    id: str
    email: str
    email_verified: bool = False
    created_at: datetime


class UserProfileResponse(BaseModel):
    """Response model for user profile data."""
    user_id: str
    expertise_level: str
    interests: list[str]
    learning_goals: str
    preferred_format: str
    time_availability: str
    challenges: Optional[str]
    device_preference: str
    created_at: datetime
    updated_at: datetime


class AuthResponse(BaseModel):
    """Response model for authentication (login/signup)."""
    token: str
    user: UserResponse
    expires_at: datetime


class ForgotPasswordRequest(BaseModel):
    """Request model for password reset."""
    email: EmailStr


class ResetPasswordRequest(BaseModel):
    """Request model for setting new password."""
    token: str
    new_password: str = Field(..., min_length=8)


class MessageResponse(BaseModel):
    """Generic message response."""
    message: str


# Update forward reference
UserCreate.model_rebuild()
