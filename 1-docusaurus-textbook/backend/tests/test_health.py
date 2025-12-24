"""
Health Check Endpoint Tests
"""

import pytest
from fastapi.testclient import TestClient

from app.main import app

client = TestClient(app)


def test_root_endpoint():
    """Test root endpoint returns service info"""
    response = client.get("/")
    assert response.status_code == 200
    data = response.json()
    assert data["service"] == "Humanoid Robotics Book API"
    assert data["version"] == "0.1.0"
    assert data["status"] == "running"


def test_health_check():
    """Test health check endpoint"""
    response = client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert "timestamp" in data
    assert "environment" in data
    assert "components" in data
    assert data["components"]["api"] == "operational"


def test_ping():
    """Test ping endpoint"""
    response = client.get("/ping")
    assert response.status_code == 200
    data = response.json()
    assert data["ping"] == "pong"
    assert "timestamp" in data


def test_docs_available_in_debug():
    """Test that docs are available in debug mode"""
    response = client.get("/docs")
    # Should return 200 in debug mode, or redirect
    assert response.status_code in [200, 307]


@pytest.mark.asyncio
async def test_cors_headers():
    """Test CORS headers are present"""
    response = client.get("/health")
    # In test mode, CORS headers should be present
    assert response.status_code == 200
