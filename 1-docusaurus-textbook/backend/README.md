# Humanoid Robotics Book - Production AI Backend

**Status**: ✅ Phase 1 Complete - Project Bootstrap

**Tech Stack**:
- **Framework**: FastAPI (Python 3.12)
- **Database**: Neon PostgreSQL + Qdrant Vector DB
- **Auth**: BetterAuth (JWT)
- **AI**: OpenAI + Claude (Anthropic)
- **Deployment**: Railway

---

## 🚀 Quick Start (Local Development)

### Prerequisites
- Python 3.12+
- pip or uv package manager

### 1. Create Virtual Environment

```bash
cd 1-docusaurus-textbook/backend

# Using venv
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# OR using uv (faster)
uv venv
source .venv/bin/activate
```

### 2. Install Dependencies

```bash
# Using pip
pip install -r requirements.txt

# OR using uv (faster)
uv pip install -r requirements.txt
```

### 3. Configure Environment

The `.env` file is already created with development defaults. No changes needed for Phase 1.

```bash
# Verify .env exists
cat .env
```

### 4. Run the Server

```bash
# Method 1: Using run.py
python run.py

# Method 2: Using uvicorn directly
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### 5. Verify It Works

**Open your browser or use curl:**

```bash
# Root endpoint
curl http://localhost:8000/

# Health check
curl http://localhost:8000/health

# Ping endpoint
curl http://localhost:8000/ping

# API Documentation (Swagger UI)
# Visit: http://localhost:8000/docs
```

**Expected Response (Health Check)**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-17T...",
  "environment": "development",
  "components": {
    "api": "operational",
    "database": "not_configured",
    "vector_db": "not_configured",
    "auth": "not_configured"
  }
}
```

---

## 📁 Project Structure

```
backend/
├── app/
│   ├── __init__.py          # Package initialization
│   ├── main.py              # FastAPI app entry point
│   ├── config.py            # Configuration management
│   ├── models/              # (Phase 2) Database models
│   ├── schemas/             # (Phase 3+) Pydantic schemas
│   ├── routers/             # (Phase 3+) API endpoints
│   ├── services/            # (Phase 5+) Business logic
│   ├── agents/              # (Phase 6) Agent orchestration
│   └── utils/               # Utility functions
├── tests/                   # Test suite
├── migrations/              # (Phase 2) Alembic migrations
├── .env                     # Environment variables
├── .env.example             # Example env file
├── requirements.txt         # Dependencies
├── pyproject.toml           # Project metadata
├── run.py                   # Development server runner
└── README.md                # This file
```

---

## ✅ Phase 1 Checklist

- [x] FastAPI project initialized
- [x] Folder structure created
- [x] Dependency management (requirements.txt, pyproject.toml)
- [x] Environment configuration (.env, config.py)
- [x] Health check endpoint implemented
- [x] Root and ping endpoints working
- [x] CORS middleware configured
- [x] Logging configured
- [x] Error handling implemented
- [x] Development server runnable

---

## 🔜 Next Phases

### Phase 2: Database (Neon Postgres)
- [ ] Connect to Neon PostgreSQL
- [ ] Create schema (users, chapters, chats, agent_logs)
- [ ] Set up Alembic migrations
- [ ] Verify CRUD operations

### Phase 3: Authentication (BetterAuth)
- [ ] Implement signup/signin endpoints
- [ ] JWT token generation/validation
- [ ] Auth middleware
- [ ] Session management

### Phase 4: Book Content Backend
- [ ] Chapter & lesson APIs
- [ ] Docusaurus content ingestion
- [ ] Metadata storage

### Phase 5: RAG Chatbot (Qdrant + OpenAI)
- [ ] Qdrant setup and connection
- [ ] Embedding pipeline
- [ ] Chat endpoint with RAG
- [ ] Chapter-aware retrieval

### Phase 6: Agents & Skills (Claude Code)
- [ ] Agent orchestration system
- [ ] Sub-agents (Chapter Tutor, ROS2, etc.)
- [ ] Skill execution engine

### Phase 7: Personalization Engine
- [ ] User learning profiles
- [ ] Adaptive responses

### Phase 8: Urdu Translation
- [ ] Translation pipeline
- [ ] Cached translations

### Phase 9: Frontend Integration
- [ ] API contracts validation
- [ ] Docusaurus integration

### Phase 10: Railway Deployment
- [ ] Railway configuration
- [ ] Production deployment
- [ ] Environment variables setup

---

## 🧪 Testing

```bash
# Run tests (Phase 1 - basic smoke test)
pytest

# Run with coverage
pytest --cov=app tests/

# Run specific test file
pytest tests/test_health.py -v
```

---

## 📝 Development Commands

```bash
# Format code
black app/ tests/

# Lint code
ruff check app/ tests/

# Type checking (optional - add mypy later)
# mypy app/

# Run development server with auto-reload
python run.py
```

---

## 🌐 API Endpoints (Phase 1)

| Endpoint | Method | Description | Auth Required |
|----------|--------|-------------|---------------|
| `/` | GET | Root endpoint | No |
| `/health` | GET | Health check with component status | No |
| `/ping` | GET | Simple ping/pong | No |
| `/docs` | GET | Swagger UI (dev only) | No |
| `/redoc` | GET | ReDoc UI (dev only) | No |

---

## 🔧 Configuration

All configuration is managed via environment variables in `.env` file:

- **ENVIRONMENT**: `development` | `staging` | `production`
- **HOST**: Server host (default: `0.0.0.0`)
- **PORT**: Server port (default: `8000`)
- **DEBUG**: Enable debug mode (default: `true`)
- **LOG_LEVEL**: Logging level (default: `INFO`)
- **CORS_ORIGINS**: Comma-separated allowed origins

See `.env.example` for full configuration reference.

---

## 🐛 Troubleshooting

### Port Already in Use
```bash
# Find process using port 8000
lsof -i :8000

# Kill process
kill -9 <PID>
```

### Module Not Found
```bash
# Reinstall dependencies
pip install -r requirements.txt --force-reinstall
```

### Import Errors
```bash
# Ensure you're in the backend directory
cd 1-docusaurus-textbook/backend

# Ensure virtual environment is activated
source venv/bin/activate  # or .venv/bin/activate
```

---

## 📚 Resources

- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [Pydantic Settings](https://docs.pydantic.dev/latest/concepts/pydantic_settings/)
- [Uvicorn](https://www.uvicorn.org/)

---

**Phase 1 Status**: ✅ **COMPLETE**

**Ready for Phase 2**: Database Integration (Neon PostgreSQL)
