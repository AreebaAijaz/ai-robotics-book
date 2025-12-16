---
id: "0002"
title: "Create RAG Chatbot Implementation Plan"
stage: plan
date: 2025-12-16
surface: agent
model: claude-opus-4-5-20251101
feature: 002-rag-chatbot
branch: 002-rag-chatbot
user: user
command: /sp.plan
labels: ["rag", "chatbot", "architecture", "fastapi", "docusaurus", "planning"]
links:
  spec: specs/002-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-rag-chatbot/plan.md
  - specs/002-rag-chatbot/research.md
  - specs/002-rag-chatbot/data-model.md
  - specs/002-rag-chatbot/quickstart.md
  - specs/002-rag-chatbot/contracts/api-spec.yaml
  - specs/002-rag-chatbot/contracts/frontend-types.ts
tests:
  - N/A (planning phase)
---

## Prompt

/sp.plan

Architecture:
Frontend (Docusaurus + React) → Backend (FastAPI) → Qdrant (vectors) + Neon (metadata) + OpenAI (embeddings + agent)

Phases:

Phase 1: Backend Setup & Database Connections (60 min)
- Create backend/ folder with uv project
- Install dependencies (fastapi, qdrant-client, openai, psycopg2, python-dotenv)
- Set up .env with API keys
- Create FastAPI app with CORS for localhost:3000
- Test Qdrant connection (create collection)
- Test Neon Postgres connection (create tables: chat_sessions, queries)

Phase 2: Data Ingestion (45 min)
- Build ingest_docs.py: read docs/ markdown files
- Chunk content (800 tokens, 100 overlap)
- Generate embeddings with OpenAI
- Store in Qdrant with metadata (module, chapter, file)
- Run ingestion script, verify data in Qdrant

Phase 3: RAG Agent & API Endpoints (90 min)
- Build qdrant_service.py (semantic search function)
- Build embeddings.py (OpenAI embedding wrapper)
- Build rag_agent.py (OpenAI Agents SDK(MCP Server attached) with retrieval)
- Create POST /api/chat endpoint (general questions)
- Create POST /api/chat/selection endpoint (selected text questions)
- Build postgres_service.py (log queries to Neon)
- Test endpoints with curl/Postman

Phase 4: Frontend Chat Widget (90 min)
- Create ChatWidget component (floating button)
- Build ChatPanel (message UI, input field)
- Build apiClient.js (API calls to localhost:8000)
- Add loading states, error handling
- Implement sessionStorage for chat history

Phase 5: Text Selection Feature (60 min)
- Build SelectionHandler.js (detect text selection)
- Show "Ask about this" button on selection
- Send selection as context to /api/chat/selection
- Display responses with selection context highlighted

Phase 6: Integration & Testing (45 min)
- Swizzle Docusaurus Root, inject ChatWidget
- Test general questions
- Test text selection questions
- Verify Neon logs chat history
- Test mobile responsiveness
- Polish UI (styling, animations)

Dependencies:
Phase 1 → Phase 2 → Phase 3 → Phase 4 → Phase 5 → Phase 6 (sequential)

## Response snapshot

Created comprehensive implementation plan for RAG Chatbot with Text Selection:

**Phase 0 Output - research.md**:
- 10 technology decisions documented with rationale
- Alternatives evaluated: Qdrant vs Pinecone/ChromaDB, text-embedding-3-small vs alternatives, gpt-4o-mini vs other LLMs
- Chunking strategy: 800 tokens with 100 overlap
- All NEEDS CLARIFICATION items resolved

**Phase 1 Outputs**:
- **data-model.md**: 6 entities defined (ContentChunk, ChatSession, QueryLog, ChatSessionState, ChatMessage, Citation)
- **contracts/api-spec.yaml**: OpenAPI 3.1 spec with 3 endpoints (health, chat, chat/selection)
- **contracts/frontend-types.ts**: 20+ TypeScript interfaces for frontend
- **quickstart.md**: Step-by-step local setup guide

**plan.md Updates**:
- Technical context filled (Python 3.11+, React 19, FastAPI, Qdrant, Neon)
- Constitution check passed all 6 principles
- Architecture diagram with data flow
- 6 implementation phases defined
- Project structure for backend/ and frontend additions

**Agent Context**: CLAUDE.md updated with active technologies

## Outcome

- ✅ Impact: Complete architecture and design artifacts for RAG chatbot feature
- 🧪 Tests: N/A (planning phase - no code tests)
- 📁 Files: 6 files created/updated
- 🔁 Next prompts: Run `/sp.tasks` to generate implementation task breakdown
- 🧠 Reflection: User-provided phase breakdown was incorporated; tech stack decisions documented with clear rationale

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully
- Graders run and results (PASS/FAIL): Constitution check PASS (6/6 principles)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
