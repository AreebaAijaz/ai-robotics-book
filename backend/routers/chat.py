"""Chat router for RAG Chatbot API."""

import logging
import time
import traceback
from datetime import datetime, timezone
from typing import Optional

logger = logging.getLogger(__name__)

from fastapi import APIRouter, HTTPException, Depends
from openai import OpenAI

from config import settings
from models.health import HealthResponse, DependenciesStatus, DependencyStatus
from models.chat import ChatRequest, ChatResponse, SelectionChatRequest, Citation
from services.qdrant import qdrant_service
from services.postgres import postgres_service
from services.agent import rag_agent
from middleware.auth import get_current_user, get_optional_user
from personalization.rag_personalizer import rag_personalizer

router = APIRouter(prefix="/api", tags=["Chat"])


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """Check service health and dependency connectivity.

    Returns:
        HealthResponse: Service health status and dependency statuses.
    """
    # Check Qdrant
    qdrant_status = qdrant_service.health_check()
    qdrant_dep = DependencyStatus(
        status=qdrant_status.get("status", "error"),
        latency_ms=qdrant_status.get("latency_ms"),
        error=qdrant_status.get("error"),
    )

    # Check Postgres
    postgres_status = postgres_service.health_check()
    postgres_dep = DependencyStatus(
        status=postgres_status.get("status", "error"),
        latency_ms=postgres_status.get("latency_ms"),
        error=postgres_status.get("error"),
    )

    # Check OpenAI
    openai_dep = _check_openai()

    # Determine overall status
    all_connected = all(
        dep.status == "connected"
        for dep in [qdrant_dep, postgres_dep, openai_dep]
    )
    any_error = any(
        dep.status == "error"
        for dep in [qdrant_dep, postgres_dep, openai_dep]
    )

    if all_connected:
        overall_status = "healthy"
    elif any_error:
        overall_status = "unhealthy"
    else:
        overall_status = "degraded"

    return HealthResponse(
        status=overall_status,
        timestamp=datetime.now(timezone.utc).isoformat(),
        dependencies=DependenciesStatus(
            qdrant=qdrant_dep,
            postgres=postgres_dep,
            openai=openai_dep,
        ),
    )


def _check_openai() -> DependencyStatus:
    """Check OpenAI API connectivity.

    Returns:
        DependencyStatus: OpenAI connection status.
    """
    try:
        start = time.time()
        client = OpenAI(api_key=settings.OPENAI_API_KEY)
        # Simple API call to verify connectivity
        client.models.list()
        latency_ms = int((time.time() - start) * 1000)
        return DependencyStatus(status="connected", latency_ms=latency_ms)
    except Exception as e:
        return DependencyStatus(status="error", error=str(e))


@router.post("/chat", response_model=ChatResponse)
async def send_chat_message(
    request: ChatRequest,
    current_user: Optional[dict] = Depends(get_optional_user)
):
    """Send a general chat message about the book content.

    Args:
        request: ChatRequest with session_id and message.
        current_user: Optional authenticated user for personalization.

    Returns:
        ChatResponse: Assistant response with citations.
    """
    start_time = time.time()

    try:
        # Get personalized prompt if user is authenticated
        personalized_prompt = None
        user_id = None
        if current_user:
            user_id = current_user.get("user_id")
            personalized_prompt = rag_personalizer.get_personalized_system_prompt(user_id)

        # Generate response using RAG agent
        result = rag_agent.generate_response(
            query=request.message,
            personalized_prompt=personalized_prompt,
        )

        # Calculate response time
        response_time_ms = int((time.time() - start_time) * 1000)

        # Build citations
        citations = [
            Citation(
                module=c.get("module", ""),
                chapter=c.get("chapter", ""),
                section=c.get("section"),
                relevance_score=c.get("relevance_score"),
            )
            for c in result.get("citations", [])
        ]

        # Log query (non-blocking, failure won't break response)
        postgres_service.log_query(
            session_id=request.session_id,
            query_text=request.message,
            response_text=result.get("content", ""),
            citations=result.get("citations", []),
            response_time_ms=response_time_ms,
            query_type="general",
            user_id=user_id,
        )

        return ChatResponse(
            message_id=result.get("message_id", ""),
            content=result.get("content", ""),
            citations=citations,
            response_time_ms=response_time_ms,
        )

    except Exception as e:
        logger.error("Chat error: %s\n%s", str(e), traceback.format_exc())
        raise HTTPException(
            status_code=500,
            detail={"error": "internal_error", "message": str(e)},
        )


@router.post("/chat/selection", response_model=ChatResponse)
async def send_selection_message(
    request: SelectionChatRequest,
    current_user: Optional[dict] = Depends(get_optional_user)
):
    """Send a message about selected text from the book.

    Args:
        request: SelectionChatRequest with session_id, selected_text, and optional question.
        current_user: Optional authenticated user for personalization.

    Returns:
        ChatResponse: Assistant response with citations.
    """
    start_time = time.time()

    try:
        # Build the query - use question if provided, otherwise ask to explain selection
        query = request.question if request.question else f"Please explain the following text: {request.selected_text}"

        # Get personalized prompt if user is authenticated
        personalized_prompt = None
        user_id = None
        if current_user:
            user_id = current_user.get("user_id")
            personalized_prompt = rag_personalizer.get_personalized_system_prompt(user_id)

        # Generate response using RAG agent with selected text context
        result = rag_agent.generate_response(
            query=query,
            selected_text=request.selected_text,
            personalized_prompt=personalized_prompt,
        )

        # Calculate response time
        response_time_ms = int((time.time() - start_time) * 1000)

        # Build citations
        citations = [
            Citation(
                module=c.get("module", ""),
                chapter=c.get("chapter", ""),
                section=c.get("section"),
                relevance_score=c.get("relevance_score"),
            )
            for c in result.get("citations", [])
        ]

        # Log query (non-blocking, failure won't break response)
        postgres_service.log_query(
            session_id=request.session_id,
            query_text=query,
            response_text=result.get("content", ""),
            citations=result.get("citations", []),
            response_time_ms=response_time_ms,
            query_type="selection",
            selected_text=request.selected_text,
            user_id=user_id,
        )

        return ChatResponse(
            message_id=result.get("message_id", ""),
            content=result.get("content", ""),
            citations=citations,
            response_time_ms=response_time_ms,
        )

    except Exception as e:
        logger.error("Selection chat error: %s\n%s", str(e), traceback.format_exc())
        raise HTTPException(
            status_code=500,
            detail={"error": "internal_error", "message": str(e)},
        )
