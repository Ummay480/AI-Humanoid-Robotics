"""
FastAPI Application Entry Point
"""

import logging
from contextlib import asynccontextmanager
from datetime import datetime
from typing import AsyncGenerator

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from app.config import settings

# Configure logging
logging.basicConfig(
    level=getattr(logging, settings.log_level.upper()),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """
    Application lifespan manager
    Handles startup and shutdown events
    """
    # Startup
    logger.info("🚀 Starting Humanoid Robotics Backend...")
    logger.info(f"Environment: {settings.environment}")
    logger.info(f"Debug Mode: {settings.debug}")
    logger.info(f"Server: {settings.host}:{settings.port}")

    # TODO: Initialize database connections (Phase 2)
    # TODO: Initialize vector database (Phase 5)

    yield

    # Shutdown
    logger.info("🛑 Shutting down Humanoid Robotics Backend...")
    # TODO: Close database connections


# Create FastAPI application
app = FastAPI(
    title="Humanoid Robotics Book API",
    description="Production AI Backend for Interactive Robotics Learning",
    version="0.1.0",
    docs_url="/docs" if settings.debug else None,
    redoc_url="/redoc" if settings.debug else None,
    lifespan=lifespan,
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ============================================================
# HEALTH CHECK ENDPOINTS
# ============================================================


@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "service": "Humanoid Robotics Book API",
        "version": "0.1.0",
        "status": "running",
        "docs": "/docs" if settings.debug else "disabled",
    }


@app.get("/health")
async def health_check():
    """
    Health check endpoint
    Returns system status and component availability
    """
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "environment": settings.environment,
        "components": {
            "api": "operational",
            "database": "not_configured",  # Will update in Phase 2
            "vector_db": "not_configured",  # Will update in Phase 5
            "auth": "not_configured",  # Will update in Phase 3
        },
    }


@app.get("/ping")
async def ping():
    """Simple ping endpoint"""
    return {"ping": "pong", "timestamp": datetime.utcnow().isoformat()}


# ============================================================
# ERROR HANDLERS
# ============================================================


@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """Global exception handler"""
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": "internal_server_error",
            "message": "An unexpected error occurred",
            "detail": str(exc) if settings.debug else None,
        },
    )


# ============================================================
# FUTURE ROUTERS (To be added in later phases)
# ============================================================

# Phase 3: app.include_router(auth_router, prefix="/api/auth", tags=["auth"])
# Phase 4: app.include_router(chapters_router, prefix="/api/chapters", tags=["chapters"])
# Phase 5: app.include_router(chat_router, prefix="/api/chat", tags=["chat"])
# Phase 6: app.include_router(agents_router, prefix="/api/agents", tags=["agents"])
# Phase 7: app.include_router(personalization_router, prefix="/api/personalization", tags=["personalization"])
# Phase 8: app.include_router(translation_router, prefix="/api/translation", tags=["translation"])
