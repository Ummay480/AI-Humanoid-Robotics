# RAG Chat Integration - Testing Setup Guide

**Date**: 2025-12-18
**Status**: Environment Setup Required
**Feature**: 002-docusaurus-chatkit-frontend

---

## Current Environment Status

### ✅ Ready Components
- Node.js v20.19.5 and npm 10.8.2 installed
- Frontend dependencies installed (node_modules exists)
- Gemini API key configured in backend/.env
- Docker Desktop installed on Windows
- Python 3.12.3 available

### ⚠️ Setup Required
- Python virtual environment (venv) needs repair/recreation
- Python packages need installation
- Qdrant vector database needs to be started
- Docker Desktop needs to be running

---

## Setup Instructions (Run These Manually)

### Step 1: Fix Python Environment

The backend virtual environment is broken and needs to be recreated. Run these commands:

```bash
# Navigate to backend directory
cd /mnt/d/aidd/hackathon/1-docosaurus-textbook/backend

# Install python3-venv (requires sudo password)
sudo apt update
sudo apt install -y python3.12-venv

# Remove broken venv and recreate
rm -rf venv
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install all requirements
pip install -r requirements.txt

# Verify installation
pip list | grep -E "(fastapi|uvicorn|qdrant|openai|pydantic)"
```

**Expected output**: All packages from requirements.txt should be installed.

---

### Step 2: Start Qdrant Vector Database

You have two options for running Qdrant:

#### Option A: Docker Desktop (Recommended)

1. **Start Docker Desktop** on Windows
   - Open Docker Desktop application
   - Wait for it to fully start (Docker icon in system tray turns green)

2. **Start Qdrant container** (from WSL terminal):
   ```bash
   docker run -d -p 6333:6333 -v $(pwd)/qdrant_storage:/qdrant/storage qdrant/qdrant
   ```

3. **Verify Qdrant is running**:
   ```bash
   curl http://localhost:6333/healthz
   # Expected: Should return OK
   ```

#### Option B: Skip Qdrant (Limited Functionality)

You can test the chat without Qdrant, but:
- ❌ No document context from the textbook
- ❌ No RAG (Retrieval-Augmented Generation)
- ✅ General chat with Gemini works
- ✅ All other functionality works

To skip Qdrant, just proceed to Step 3. The backend will log a warning but continue running.

---

### Step 3: Ingest Documents (Optional, requires Qdrant)

If you started Qdrant in Step 2, run document ingestion:

```bash
cd /mnt/d/aidd/hackathon/1-docosaurus-textbook/backend

# Activate venv
source venv/bin/activate

# Run ingestion script
python scripts/ingest_documents.py --docs-dir ../frontend/docs

# Expected output:
# ✅ SUCCESS! Ingested X document chunks
```

This will:
- Process all markdown files from `frontend/docs/`
- Generate embeddings using Gemini
- Store them in Qdrant for context retrieval

---

### Step 4: Start Backend Server

```bash
cd /mnt/d/aidd/hackathon/1-docosaurus-textbook/backend

# Activate venv
source venv/bin/activate

# Start the server
python run.py

# Expected output:
# 🚀 Starting Humanoid Robotics Backend...
# ✅ Vector database (Qdrant) initialized successfully  [if Qdrant running]
# ⚠️ Vector database initialization failed...         [if Qdrant not running]
# INFO: Uvicorn running on http://0.0.0.0:8000
```

**Keep this terminal open** - the server needs to stay running.

---

### Step 5: Test Backend API

Open a **new terminal** and run these tests:

#### Test 1: Health Check
```bash
curl http://localhost:8000/health
```

**Expected response**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-18T...",
  "environment": "development",
  "components": {
    "api": "operational",
    "database": "not_configured",
    "vector_db": "operational",  // or "not_configured" without Qdrant
    "auth": "not_configured"
  }
}
```

#### Test 2: Chat Health
```bash
curl http://localhost:8000/api/chat/health
```

**Expected response**:
```json
{
  "status": "operational",
  "components": {
    "gemini": "configured",
    "qdrant": "connected",  // or "not_connected" without Qdrant
    "openai": "not_configured"
  }
}
```

#### Test 3: Chat Streaming
```bash
curl -X POST http://localhost:8000/api/chat/stream \
  -H "Content-Type: application/json" \
  -d '{"message":"What is ROS 2?","page":null}'
```

**Expected output**: SSE stream with chunks:
```
data: {"chunk":"ROS","done":false,"sources":[]}
data: {"chunk":" 2","done":false,"sources":[]}
data: {"chunk":" is","done":false,"sources":[]}
...
data: {"chunk":null,"done":true,"sources":[...]}
```

---

### Step 6: Start Frontend Development Server

Open a **new terminal**:

```bash
cd /mnt/d/aidd/hackathon/1-docosaurus-textbook/frontend

# Install dependencies (if not already done)
npm install

# Start development server
npm start

# Expected output:
# [SUCCESS] Docusaurus website is running at:
#   Local:            http://localhost:3000/
```

**Browser should auto-open** to http://localhost:3000

---

### Step 7: Test End-to-End Chat in Browser

1. **Open browser** to http://localhost:3000 (should open automatically)

2. **Look for chat button** in the bottom-right corner

3. **Click chat button** - panel should slide in from right

4. **Send test messages**:
   - "Hello, can you help me?"
   - "What is ROS 2?"
   - "Explain humanoid robotics"

5. **Verify streaming**:
   - ✅ Message appears instantly
   - ✅ Loading indicator shows
   - ✅ Response streams character-by-character
   - ✅ No console errors

6. **Check browser console** (F12):
   - Should see API calls to http://localhost:8000
   - Should see SSE connection established
   - Should see chunks being received

---

## Troubleshooting

### Backend Won't Start

**Error: "ModuleNotFoundError: No module named 'fastapi'"**
- Solution: Virtual environment not activated or packages not installed
- Run: `source venv/bin/activate && pip install -r requirements.txt`

**Error: "GEMINI_API_KEY not configured"**
- Solution: Check backend/.env file
- Verify: `GEMINI_API_KEY = 'AIzaSyCpk4r2BSFC6D8MbOsOQWxC0A5NBAmzsNw'`

**Error: "Address already in use"**
- Solution: Port 8000 is occupied
- Find process: `lsof -i :8000`
- Kill it: `kill -9 <PID>`

### Frontend Won't Start

**Error: "Port 3000 is already in use"**
- Solution: Another process is using port 3000
- Run: `npx kill-port 3000`
- Or change port: `PORT=3001 npm start`

**Error: "Module not found"**
- Solution: Dependencies not installed
- Run: `npm install`

### Chat Not Working

**Error: "Failed to fetch" in browser console**
- Check: Backend is running on http://localhost:8000
- Check: `.env.local` exists in frontend directory
- Check: CORS_ORIGINS includes http://localhost:3000

**Chat doesn't stream / appears all at once**
- This is normal in some browsers
- Try Chrome or Edge for best experience

**No context from textbook**
- Verify: Qdrant is running and accessible
- Verify: Documents were ingested successfully
- Run: `curl http://localhost:6333/collections`

---

## Success Criteria Checklist

After completing all steps, verify:

- [ ] Backend starts without errors
- [ ] Health endpoint returns "healthy"
- [ ] Chat health shows Gemini "configured"
- [ ] Streaming endpoint returns SSE chunks
- [ ] Frontend starts and loads at localhost:3000
- [ ] Chat button visible in bottom-right
- [ ] Chat panel opens/closes smoothly
- [ ] Can send messages via input field
- [ ] Responses stream in real-time
- [ ] No errors in browser console
- [ ] (Optional) Qdrant is connected
- [ ] (Optional) Documents are ingested

---

## Quick Start Summary

If you just want to test quickly (without Qdrant):

```bash
# Terminal 1: Backend
cd /mnt/d/aidd/hackathon/1-docosaurus-textbook/backend
sudo apt install -y python3.12-venv  # One-time setup
python3 -m venv venv --clear
source venv/bin/activate
pip install -r requirements.txt
python run.py

# Terminal 2: Frontend
cd /mnt/d/aidd/hackathon/1-docosaurus-textbook/frontend
npm start

# Browser: http://localhost:3000
# Test chat in bottom-right corner
```

---

## Next Steps After Testing

Once everything works:

1. **Commit the integration**:
   ```bash
   git add -A
   git commit -m "feat: RAG chat integration with Gemini API"
   git push origin 002-docusaurus-chatkit-frontend
   ```

2. **Create PHR (Prompt History Record)** - automatically done

3. **Deploy to production**:
   - Backend → Railway/Render
   - Frontend → Vercel
   - Qdrant → Qdrant Cloud

4. **Enhance features**:
   - Add more tutorial content
   - Improve chat UI
   - Add analytics

---

**Status**: Ready for manual setup and testing
**Estimated Time**: 15-20 minutes for full setup
**Documentation**: Complete
