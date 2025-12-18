# RAG Backend Testing Guide with Gemini

## Prerequisites
✅ Gemini API key configured in backend/.env
✅ Qdrant running (Docker or local)
✅ Backend code updated to use Gemini

## Quick Test Commands

### 1. Test Backend Health
```bash
curl http://localhost:8000/health
```

**Expected:**
```json
{
  "status": "healthy",
  "components": {
    "api": "operational",
    "vector_db": "operational"
  }
}
```

### 2. Test Chat Health
```bash
curl http://localhost:8000/api/chat/health
```

**Expected:**
```json
{
  "status": "operational",
  "components": {
    "gemini": "configured",
    "qdrant": "connected",
    "embedding_model": "text-embedding-004",
    "chat_model": "gemini-2.0-flash"
  }
}
```

### 3. Test Streaming Chat (without document ingestion)
```bash
curl -X POST http://localhost:8000/api/chat/stream \
  -H "Content-Type: application/json" \
  -d '{"message":"What is ROS 2?","page":null}'
```

**Expected:** SSE stream with Gemini response
```
data: {"chunk":"ROS 2","done":false}
data: {"chunk":" is","done":false}
data: {"chunk":" the","done":false}
...
data: {"chunk":null,"done":true,"sources":[]}
```

### 4. Test Non-Streaming Chat
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message":"Explain robotics","page":null}'
```

**Expected:**
```json
{
  "response": "Robotics is...",
  "sources": [],
  "session_id": null
}
```

## Frontend Testing

### 1. Start Frontend
```bash
cd /mnt/d/aidd/hackathon/1-docosaurus-textbook/frontend
npm start
```

### 2. Open Browser
- Go to http://localhost:3000
- Click the chat button (bottom-right)
- Send message: "What is ROS 2?"
- Watch Gemini response stream in!

## With Document Ingestion (Optional)

If you want context-aware responses:

```bash
cd /mnt/d/aidd/hackathon/1-docosaurus-textbook/backend

# Install dependencies
pip3 install fastapi uvicorn qdrant-client openai tiktoken pydantic pydantic-settings

# Run ingestion
python3 scripts/ingest_documents.py --docs-dir ../frontend/docs

# Start server
python3 run.py
```

Then test again - responses will include document context from your textbook!

## Troubleshooting

### "Connection refused"
- Check backend is running on port 8000
- Check Qdrant is running on port 6333

### "GEMINI_API_KEY not configured"
- Verify backend/.env has: `GEMINI_API_KEY=your-key-here`

### "No relevant context found"
- This is normal without document ingestion
- Gemini will still respond with general knowledge

### CORS errors in frontend
- Check backend/.env has: `CORS_ORIGINS=http://localhost:3000`
- Restart backend after changing .env

## What's Working Without Ingestion

✅ Backend server starts
✅ Health checks pass
✅ Gemini API connection works
✅ Chat streaming works
✅ Frontend displays responses
❌ No document context (sources will be empty)

## Next Steps

1. **Now**: Test chat without document context
2. **Later**: Run document ingestion for RAG
3. **Production**: Deploy to Railway/Vercel
