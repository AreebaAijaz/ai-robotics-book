# Quickstart: RAG Chatbot with Text Selection

**Feature**: 002-rag-chatbot
**Date**: 2025-12-16

This guide provides step-by-step instructions to set up and run the RAG chatbot locally.

---

## Prerequisites

- **Node.js** >= 20.0 (for Docusaurus frontend)
- **Python** >= 3.11 (for FastAPI backend)
- **uv** (Python package manager) - [Install uv](https://docs.astral.sh/uv/getting-started/installation/)
- **Git** (for version control)

### External Accounts (Free Tier)

1. **OpenAI API** - Get API key from [platform.openai.com](https://platform.openai.com)
2. **Qdrant Cloud** - Create free cluster at [cloud.qdrant.io](https://cloud.qdrant.io)
3. **Neon Postgres** - Create free database at [neon.tech](https://neon.tech)

---

## Quick Setup (5 minutes)

### 1. Clone and Setup Environment

```bash
# Navigate to project root
cd ai-robotics

# Create backend directory
mkdir -p backend
cd backend

# Initialize Python project with uv
uv init --name rag-chatbot
```

### 2. Configure Environment Variables

Create `backend/.env`:

```env
# OpenAI API
OPENAI_API_KEY=sk-your-openai-api-key

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Neon Postgres
DATABASE_URL=postgresql://user:password@your-host.neon.tech/dbname?sslmode=require

# CORS (for local development)
CORS_ORIGINS=http://localhost:3000
```

### 3. Install Backend Dependencies

```bash
cd backend

# Install dependencies with uv
uv add fastapi uvicorn qdrant-client openai psycopg2-binary python-dotenv pydantic
```

### 4. Run Database Migrations

```bash
# Using psql or your preferred PostgreSQL client
psql $DATABASE_URL -f scripts/migrations/001_create_tables.sql
```

### 5. Ingest Book Content

```bash
cd backend

# Run the ingestion script
uv run python scripts/ingest_docs.py
```

Expected output:
```
Loading documents from ../ai-robotics-book/docs/
Found 20 markdown files
Processing module-1-ros2/01-architecture.md... 12 chunks
Processing module-1-ros2/02-nodes-topics-services.md... 15 chunks
...
Ingestion complete: 180 chunks stored in Qdrant
```

### 6. Start the Backend Server

```bash
cd backend

# Start FastAPI server
uv run uvicorn main:app --reload --port 8000
```

Verify at: http://localhost:8000/api/health

### 7. Start the Frontend

```bash
cd ai-robotics-book

# Install dependencies (if not already done)
npm install

# Start Docusaurus dev server
npm start
```

Frontend runs at: http://localhost:3000

---

## Verification Checklist

### Backend Health Check

```bash
curl http://localhost:8000/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-16T10:30:00Z",
  "dependencies": {
    "qdrant": {"status": "connected", "latency_ms": 45},
    "postgres": {"status": "connected", "latency_ms": 30},
    "openai": {"status": "connected"}
  }
}
```

### Test Chat Endpoint

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "test-session-001",
    "message": "What is ROS 2?"
  }'
```

Expected response (truncated):
```json
{
  "message_id": "...",
  "content": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software...",
  "citations": [
    {
      "module": "Module 1: ROS 2 Fundamentals",
      "chapter": "Chapter 1: Architecture",
      "section": "Introduction",
      "relevance_score": 0.92
    }
  ],
  "response_time_ms": 1250
}
```

### Test Selection Endpoint

```bash
curl -X POST http://localhost:8000/api/chat/selection \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "test-session-001",
    "selected_text": "DDS middleware",
    "question": "What does this stand for?"
  }'
```

### Frontend Verification

1. Open http://localhost:3000
2. Click the chat button (bottom-right corner)
3. Ask: "What is URDF?"
4. Verify response includes citations

---

## Project Structure

After setup, your project should look like:

```
ai-robotics/
├── ai-robotics-book/           # Docusaurus frontend
│   ├── docs/                   # Book content (markdown)
│   ├── src/
│   │   ├── components/
│   │   │   ├── ChatWidget/     # Chat UI components
│   │   │   └── SelectionHandler/
│   │   ├── services/
│   │   │   └── apiClient.js    # Backend API client
│   │   └── theme/
│   │       └── Root.js         # Swizzled root with ChatWidget
│   └── package.json
│
├── backend/                    # FastAPI backend
│   ├── main.py                 # App entry point
│   ├── config.py               # Environment config
│   ├── models/                 # Pydantic models
│   ├── services/               # Business logic
│   │   ├── embeddings.py
│   │   ├── qdrant.py
│   │   ├── agent.py
│   │   └── postgres.py
│   ├── routers/
│   │   └── chat.py             # API endpoints
│   ├── scripts/
│   │   ├── ingest_docs.py      # Data ingestion
│   │   └── migrations/         # SQL migrations
│   ├── .env                    # Environment variables
│   └── pyproject.toml          # uv dependencies
│
└── specs/
    └── 002-rag-chatbot/        # This feature's specs
```

---

## Common Issues

### CORS Errors

If you see CORS errors in browser console:
1. Verify `CORS_ORIGINS` in `.env` matches your frontend URL
2. Restart the backend server after changing `.env`

### Qdrant Connection Failed

1. Verify `QDRANT_URL` includes `https://`
2. Check API key is correct
3. Ensure cluster is not paused (free tier auto-pauses)

### Empty Search Results

1. Verify ingestion completed successfully
2. Check Qdrant dashboard for collection and point count
3. Try a broader query

### Slow Responses

1. First request may be slow (model warmup)
2. Check OpenAI API status at [status.openai.com](https://status.openai.com)
3. Verify network connectivity to cloud services

---

## Development Commands Reference

```bash
# Backend
cd backend
uv run uvicorn main:app --reload --port 8000  # Start server
uv run python scripts/ingest_docs.py          # Re-ingest content
uv run pytest                                  # Run tests

# Frontend
cd ai-robotics-book
npm start                                      # Start dev server
npm run build                                  # Production build
npm run serve                                  # Serve production build
```

---

## Next Steps

After completing this quickstart:

1. Test all user stories from the specification
2. Verify analytics logging in Neon Postgres
3. Test mobile responsiveness
4. Review error handling for edge cases

For detailed architecture and decisions, see:
- `specs/002-rag-chatbot/research.md` - Technology decisions
- `specs/002-rag-chatbot/data-model.md` - Data structures
- `specs/002-rag-chatbot/contracts/api-spec.yaml` - API contract
