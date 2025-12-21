"""Chat router for RAG-powered conversational endpoints."""

import json
import logging
from typing import AsyncGenerator

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse

from app.schemas.chat import ChatRequest, ChatResponse, ChatChunk
from app.services.rag_service import generate_response, generate_response_stream

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest) -> ChatResponse:
    """
    Non-streaming chat endpoint for testing.

    Args:
        request: ChatRequest with user message

    Returns:
        ChatResponse with AI-generated answer and sources
    """
    logger.info(f"Received chat request: '{request.message[:100]}...'")

    try:
        response_text, sources = await generate_response(
            user_query=request.message,
            current_page=request.page,
        )

        return ChatResponse(
            response=response_text,
            sources=sources,
            session_id=request.session_id,
        )

    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        raise HTTPException(status_code=500, detail="Server configuration error")

    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.post("/chat/stream")
async def chat_stream(request: ChatRequest):
    """
    Streaming chat endpoint using Server-Sent Events (SSE).

    Args:
        request: ChatRequest with user message

    Returns:
        StreamingResponse with SSE events
    """
    logger.info(f"Received streaming chat request: '{request.message[:100]}...'")

    async def event_generator() -> AsyncGenerator[str, None]:
        """Generate SSE events for streaming response."""
        try:
            sources_sent = False

            async for text_chunk, sources in generate_response_stream(
                user_query=request.message,
                current_page=request.page,
            ):
                if sources is not None and not sources_sent:
                    # Send sources at the end
                    chunk_data = ChatChunk(chunk=None, done=True, sources=sources)
                    sources_sent = True
                elif text_chunk:
                    # Send text chunk
                    chunk_data = ChatChunk(chunk=text_chunk, done=False)
                else:
                    continue

                # Format as SSE
                yield f"data: {chunk_data.model_dump_json()}\n\n"

            # Ensure final "done" event is sent
            if not sources_sent:
                chunk_data = ChatChunk(chunk=None, done=True)
                yield f"data: {chunk_data.model_dump_json()}\n\n"

        except ValueError as e:
            logger.error(f"Configuration error: {e}")
            error_chunk = ChatChunk(
                chunk="Error: Server configuration issue. Please contact administrator.",
                done=True,
            )
            yield f"data: {error_chunk.model_dump_json()}\n\n"

        except Exception as e:
            logger.error(f"Error in streaming endpoint: {e}")
            error_chunk = ChatChunk(
                chunk="Error: Unable to generate response. Please try again.",
                done=True,
            )
            yield f"data: {error_chunk.model_dump_json()}\n\n"

    return StreamingResponse(
        event_generator(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "X-Accel-Buffering": "no",  # Disable buffering in nginx
        },
    )


@router.get("/chat/health")
async def chat_health():
    """
    Chat service health check.

    Returns:
        Health status of chat service components
    """
    from app.config import get_settings
    from app.services.vector_store import get_qdrant_client

    settings = get_settings()

    return {
        "status": "operational",
        "components": {
            "gemini": "configured" if settings.gemini_api_key else "not_configured",
            "qdrant": "connected" if get_qdrant_client() else "not_connected",
            "embedding_model": settings.gemini_embedding_model,
            "chat_model": settings.gemini_model,
        },
    }
