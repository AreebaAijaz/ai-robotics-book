"""Health check models for RAG Chatbot API."""

from typing import Optional, Literal
from pydantic import BaseModel


class DependencyStatus(BaseModel):
    """Individual dependency health status."""

    status: Literal["connected", "disconnected", "error"]
    latency_ms: Optional[int] = None
    error: Optional[str] = None


class DependenciesStatus(BaseModel):
    """Status of all dependencies."""

    qdrant: Optional[DependencyStatus] = None
    postgres: Optional[DependencyStatus] = None
    openai: Optional[DependencyStatus] = None


class HealthResponse(BaseModel):
    """Response from GET /api/health endpoint."""

    status: Literal["healthy", "degraded", "unhealthy"]
    timestamp: str
    dependencies: Optional[DependenciesStatus] = None
