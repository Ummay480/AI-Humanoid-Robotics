"""Chat-related Pydantic schemas for request/response models."""

from typing import Optional
from pydantic import BaseModel, Field


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""

    message: str = Field(..., min_length=1, max_length=4000, description="User message")
    page: Optional[str] = Field(
        None, description="Current page/chapter for context-aware responses"
    )
    session_id: Optional[str] = Field(None, description="Session ID for conversation tracking")

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "message": "What is ROS 2?",
                    "page": "/docs/module1/ros2-nodes",
                    "session_id": "session-12345",
                }
            ]
        }
    }


class ChatResponse(BaseModel):
    """Response model for non-streaming chat endpoint."""

    response: str = Field(..., description="AI-generated response")
    sources: list[dict] = Field(
        default_factory=list,
        description="Source documents used for context",
    )
    session_id: Optional[str] = Field(None, description="Session ID")

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "response": "ROS 2 is the Robot Operating System...",
                    "sources": [
                        {
                            "file_path": "docs/module1/ros2-nodes.md",
                            "chunk_id": "chunk-001",
                            "score": 0.95,
                        }
                    ],
                    "session_id": "session-12345",
                }
            ]
        }
    }


class ChatChunk(BaseModel):
    """Streaming response chunk for SSE."""

    chunk: Optional[str] = Field(None, description="Text chunk")
    done: bool = Field(False, description="Whether streaming is complete")
    sources: Optional[list[dict]] = Field(None, description="Source documents (sent on completion)")

    model_config = {
        "json_schema_extra": {
            "examples": [
                {"chunk": "ROS 2 is", "done": False},
                {"chunk": None, "done": True, "sources": [{"file_path": "docs/module1/ros2-nodes.md"}]},
            ]
        }
    }


class DocumentChunk(BaseModel):
    """Model for document chunks stored in vector database."""

    id: str = Field(..., description="Unique chunk identifier")
    content: str = Field(..., description="Chunk text content")
    file_path: str = Field(..., description="Source file path")
    module: Optional[str] = Field(None, description="Module/chapter name")
    title: Optional[str] = Field(None, description="Document title")
    heading: Optional[str] = Field(None, description="Section heading")
    chunk_index: int = Field(..., description="Index of chunk within document")

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "id": "doc-001-chunk-000",
                    "content": "ROS 2 is the Robot Operating System...",
                    "file_path": "docs/module1/ros2-nodes.md",
                    "module": "Module 1: ROS 2 Fundamentals",
                    "title": "ROS 2 Nodes",
                    "heading": "Introduction to Nodes",
                    "chunk_index": 0,
                }
            ]
        }
    }
