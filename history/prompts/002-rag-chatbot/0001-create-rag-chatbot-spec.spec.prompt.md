---
id: "0001"
title: "Create RAG Chatbot Spec"
stage: spec
date: 2025-12-16
surface: agent
model: claude-opus-4-5-20251101
feature: 002-rag-chatbot
branch: 002-rag-chatbot
user: user
command: /sp.specify
labels: ["rag", "chatbot", "text-selection", "docusaurus", "specification"]
links:
  spec: specs/002-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-rag-chatbot/spec.md
  - specs/002-rag-chatbot/checklists/requirements.md
tests:
  - N/A (specification phase)
---

## Prompt

Complete RAG Chatbot System with Text Selection

Goal: Build RAG chatbot that answers questions about Physical AI book content, including selected text queries

System Components:

Backend (FastAPI in /backend folder):
- POST /api/chat - general questions
- POST /api/chat/selection - questions about selected text
- GET /api/health - health check
- Data ingestion script for book content
- Local development only (no deployment yet)

Tech Stack:
- FastAPI (API server)
- Qdrant Cloud Free Tier (vector DB for embeddings)
- OpenAI API (embeddings: text-embedding-3-small, chat: gpt-4o-mini)
- OpenAI Agents SDK (conversational RAG)
- Neon Postgres Free Tier (store chat history, analytics)
- Python uv for dependency management

Data Pipeline:
- Ingest: Read docs/ markdown → chunk (800 tokens) → embed → store in Qdrant
- Retrieval: User query → embed → search Qdrant → get top 5 chunks
- Agent: Query + context → OpenAI Agent → response with citations
- Storage: Log query + response in Neon Postgres

Frontend (React in Docusaurus):
- ChatWidget component (floating button, bottom-right)
- Text selection handler (user highlights text → "Ask about this" button)
- Chat panel with message history
- API calls to localhost:8000
- sessionStorage for chat persistence

Features:
1. General chat: "What is ROS 2?" → searches entire book
2. Selection chat: Highlight text → ask specific questions about it
3. Source citations in responses (Module X, Chapter Y)
4. Typing indicators, error handling
5. Mobile responsive

Success Criteria:
- Backend runs locally: uvicorn main:app --reload --port 8000
- Ingest script loads book content into Qdrant successfully
- API returns accurate answers to test queries (<5s response time)
- Frontend chat widget works on all pages
- Text selection feature functional
- All interactions logged in Neon Postgres
- Works on localhost only (deployment later)

Environment Variables (.env):
- OPENAI_API_KEY
- QDRANT_URL
- QDRANT_API_KEY
- DATABASE_URL (Neon Postgres)
- CORS_ORIGINS=http://localhost

## Response snapshot

Created feature specification for RAG Chatbot with Text Selection:

**Branch**: `002-rag-chatbot`

**User Stories (4)**:
- P1: General Book Questions - core Q&A functionality with citations
- P2: Text Selection Queries - contextual "Ask about this" feature
- P3: Chat Persistence Within Session - session-scoped history
- P4: Mobile Responsive Chat - adaptive layout for mobile devices

**Functional Requirements (14)**:
- FR-001 to FR-014 covering chat widget, Q&A, citations, text selection, persistence, error handling, analytics logging, health check, content ingestion, and local development

**Success Criteria (8)**:
- SC-001 to SC-008 with measurable outcomes for response time, citation coverage, selection activation, responsive design, logging, indexing, persistence, and error handling

**Key Entities**: ChatMessage, ChatSession, ContentChunk, QueryLog

**Checklist**: All items passed validation - ready for planning phase

## Outcome

- ✅ Impact: Created comprehensive specification for RAG chatbot feature with text selection capability
- 🧪 Tests: N/A (specification phase - no code tests)
- 📁 Files: 2 files created (spec.md, checklists/requirements.md)
- 🔁 Next prompts: Run `/sp.clarify` for additional requirements or `/sp.plan` to begin architecture planning
- 🧠 Reflection: Spec intentionally excludes implementation details (tech stack) to keep focus on user value; tech decisions will be documented in plan.md

## Evaluation notes (flywheel)

- Failure modes observed: None - spec completed successfully
- Graders run and results (PASS/FAIL): Quality checklist PASS (all 12 items)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
