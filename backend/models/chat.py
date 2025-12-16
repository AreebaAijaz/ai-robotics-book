"""Pydantic models for chat endpoints."""

from typing import Optional
from pydantic import BaseModel, Field


class ChatRequest(BaseModel):
    """Request model for general chat messages."""

    session_id: str = Field(
        ...,
        description="Unique identifier for the chat session",
    )
    message: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="The user's question about the book content",
    )


class SelectionChatRequest(BaseModel):
    """Request model for text selection queries."""

    session_id: str = Field(
        ...,
        description="Unique identifier for the chat session",
    )
    selected_text: str = Field(
        ...,
        min_length=3,
        max_length=2000,
        description="The text selected by the user on the page",
    )
    question: Optional[str] = Field(
        None,
        max_length=500,
        description="Optional follow-up question about the selected text",
    )


class Citation(BaseModel):
    """Citation model representing a source reference from the book."""

    module: str = Field(
        ...,
        description="Module name",
    )
    chapter: str = Field(
        ...,
        description="Chapter name",
    )
    section: Optional[str] = Field(
        None,
        description="Section heading (if available)",
    )
    relevance_score: Optional[float] = Field(
        None,
        ge=0,
        le=1,
        description="Semantic similarity score (0-1)",
    )


class ChatResponse(BaseModel):
    """Response model for chat endpoints."""

    message_id: str = Field(
        ...,
        description="Unique identifier for this response message",
    )
    content: str = Field(
        ...,
        description="The assistant's response (markdown formatted)",
    )
    citations: list[Citation] = Field(
        default_factory=list,
        description="Source references from the book",
    )
    response_time_ms: int = Field(
        ...,
        ge=0,
        description="Time taken to generate the response in milliseconds",
    )


class ErrorResponse(BaseModel):
    """Error response model."""

    error: str = Field(
        ...,
        description="Error code",
    )
    message: str = Field(
        ...,
        description="Human-readable error message",
    )
    details: Optional[dict] = Field(
        None,
        description="Additional error details",
    )
