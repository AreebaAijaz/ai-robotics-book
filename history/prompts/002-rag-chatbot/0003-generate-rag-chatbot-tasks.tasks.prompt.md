---
id: "0003"
title: "Generate RAG Chatbot Tasks"
stage: tasks
date: 2025-12-16
surface: agent
model: claude-opus-4-5-20251101
feature: 002-rag-chatbot
branch: 002-rag-chatbot
user: user
command: /sp.tasks
labels: ["rag", "chatbot", "tasks", "implementation", "breakdown"]
links:
  spec: specs/002-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-rag-chatbot/tasks.md
tests:
  - N/A (task generation phase)
---

## Prompt

/sp.tasks

Phase 1: Backend Setup (4 tasks)
1.1 Create backend/ with uv, install dependencies → requirements.txt, .env template
1.2 Build main.py with FastAPI + CORS → GET /api/health working
1.3 Connect Qdrant Cloud, create collection → qdrant_service.py
1.4 Connect Neon Postgres, create tables → postgres_service.py
CHECKPOINT 1: Backend connects to both databases

Phase 2: Data Ingestion (3 tasks)
2.1 Build ingest_docs.py (read markdown, chunk text)→ chunking works
2.2 Generate embeddings, store in Qdrant → ingest_docs.py complete
2.3 Run ingestion script  → verify Qdrant has vectors
CHECKPOINT 2: Book content loaded into Qdrant

Phase 3: RAG API (5 tasks)
3.1 Build embeddings.py wrapper→ OpenAI embedding function
3.2 Build qdrant_service.py search → semantic search returns chunks
3.3 Build rag_agent.py with OpenAI Agents SDK → agent responds with context
3.4 Create POST /api/chat endpoint → general questions work
3.5 Create POST /api/chat/selection endpoint → selected text questions work
CHECKPOINT 3: API endpoints functional, tested with curl

Phase 4: Frontend Widget (4 tasks)
4.1 Create ChatWidget component structure → ChatWidget.js, ChatButton.js
4.2 Build ChatPanel UI (messages, input) → UI displays messages
4.3 Build apiClient.js, connect to backend → API calls work
4.4 Add sessionStorage, loading states  → chat persists
CHECKPOINT 4: Chat widget functional

Phase 5: Text Selection (3 tasks)
5.1 Build SelectionHandler.js (detect selection)→ selection detected
5.2 Show "Ask about this" button → button appears on selection
5.3 Send selection to /api/chat/selection → selection questions work
CHECKPOINT 5: Text selection feature complete

Phase 6: Integration (3 tasks)
6.1 Swizzle Root, inject ChatWidget → widget on all pages
6.2 Test all features locally  → general + selection working
6.3 Polish UI (styling, animations) → professional look
CHECKPOINT 6: Complete system working locally

## Response snapshot

Generated comprehensive tasks.md with 65 tasks organized by user story:

**Task Distribution:**
- Phase 1 (Setup): 5 tasks - Project initialization
- Phase 2 (Foundational): 14 tasks - Database connections, health check, data ingestion
- Phase 3 (US1 - General Q&A): 17 tasks - Backend API + Frontend ChatWidget
- Phase 4 (US2 - Text Selection): 8 tasks - Selection handler + API extension
- Phase 5 (US3 - Persistence): 4 tasks - sessionStorage implementation
- Phase 6 (US4 - Mobile): 4 tasks - Responsive styling
- Phase 7 (Polish): 13 tasks - Error handling, animations, validation

**Key Features:**
- All tasks follow strict format: `- [ ] T### [P?] [US?] Description with file path`
- Parallel opportunities marked with [P]
- User story labels [US1], [US2], etc. for traceability
- Clear checkpoints after each phase
- MVP strategy: Setup + Foundational + US1 = working Q&A chatbot

**File Paths Specified:**
- Backend: backend/main.py, services/, models/, routers/, scripts/
- Frontend: ai-robotics-book/src/components/, hooks/, services/, theme/

## Outcome

- ✅ Impact: Complete implementation task breakdown ready for execution
- 🧪 Tests: Manual testing specified (no automated tests for local dev phase)
- 📁 Files: 1 file created (tasks.md)
- 🔁 Next prompts: Run `/sp.implement` to begin task execution or start manually with T001
- 🧠 Reflection: User-provided phase breakdown was expanded with specific file paths and proper task formatting; tasks organized by user story for independent delivery

## Evaluation notes (flywheel)

- Failure modes observed: None - task generation completed successfully
- Graders run and results (PASS/FAIL): Format validation PASS (all 65 tasks have checkbox, ID, file paths)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
