# Implementation Plan: RAG Chatbot with Text Selection

**Branch**: `002-rag-chatbot` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-rag-chatbot/spec.md`

## Summary

Build a RAG (Retrieval-Augmented Generation) chatbot for the Physical AI & Humanoid Robotics book that enables readers to ask questions about book content and get contextual answers. Includes a text selection feature allowing users to highlight text and ask specific questions about it. Backend uses FastAPI with Qdrant for vector search and OpenAI for embeddings/chat. Frontend integrates as a React component in the existing Docusaurus site.

## Technical Context

**Language/Version**: Python 3.11+ (backend), JavaScript/React 19 (frontend)
**Primary Dependencies**: FastAPI, Qdrant Client, OpenAI SDK, Pydantic (backend); React 19, Docusaurus 3.9.2 (frontend)
**Storage**: Qdrant Cloud (vectors), Neon Postgres (analytics), sessionStorage (client chat history)
**Testing**: pytest (backend), manual testing (frontend - local dev phase)
**Target Platform**: Local development (localhost:8000 backend, localhost:3000 frontend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <5s response time for queries
**Constraints**: Free tier limits for Qdrant/Neon/OpenAI; local development only
**Scale/Scope**: ~20 markdown files, ~180 content chunks, single user (local)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Technical Accuracy | ✅ PASS | Uses authoritative sources (OpenAI API, Qdrant docs) |
| II. Educational Clarity | ✅ PASS | Chatbot enhances learning experience with citations |
| III. Practical Applicability | ✅ PASS | Provides hands-on Q&A for book content |
| IV. Progressive Learning | ✅ PASS | Citations guide readers to relevant modules |
| V. Industry Standards | ✅ PASS | REST API, React patterns, Python best practices |
| VI. Quality Assurance | ✅ PASS | Health checks, error handling, responsive design |

**Post-Design Re-Check**: All gates pass. No constitution violations.

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-chatbot/
├── plan.md              # This file
├── spec.md              # Feature requirements
├── research.md          # Phase 0: Technology decisions
├── data-model.md        # Phase 1: Entity definitions
├── quickstart.md        # Phase 1: Setup guide
├── contracts/           # Phase 1: API contracts
│   ├── api-spec.yaml    # OpenAPI specification
│   └── frontend-types.ts # TypeScript interfaces
└── tasks.md             # Phase 2: Implementation tasks (created by /sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── main.py                    # FastAPI app entry point
├── config.py                  # Environment configuration
├── pyproject.toml             # uv dependencies
├── .env                       # Environment variables (not committed)
├── models/
│   ├── __init__.py
│   ├── chat.py                # ChatRequest, ChatResponse, etc.
│   └── health.py              # HealthResponse, DependencyStatus
├── services/
│   ├── __init__.py
│   ├── embeddings.py          # OpenAI embedding wrapper
│   ├── qdrant.py              # Vector search service
│   ├── agent.py               # RAG agent with OpenAI Agents SDK
│   └── postgres.py            # Query logging to Neon
├── routers/
│   ├── __init__.py
│   └── chat.py                # /api/chat, /api/chat/selection, /api/health
└── scripts/
    ├── ingest_docs.py         # Content ingestion script
    └── migrations/
        └── 001_create_tables.sql

ai-robotics-book/
├── src/
│   ├── components/
│   │   ├── ChatWidget/
│   │   │   ├── index.js       # Main ChatWidget component
│   │   │   ├── ChatPanel.js   # Chat message panel
│   │   │   ├── ChatInput.js   # Message input field
│   │   │   ├── Message.js     # Individual message component
│   │   │   └── styles.module.css
│   │   └── SelectionHandler/
│   │       ├── index.js       # Selection detection HOC
│   │       └── AskButton.js   # "Ask about this" floating button
│   ├── services/
│   │   └── apiClient.js       # Backend API client
│   ├── hooks/
│   │   ├── useChat.js         # Chat state management hook
│   │   └── useTextSelection.js # Selection detection hook
│   └── theme/
│       └── Root.js            # Swizzled root component
└── [existing Docusaurus files]
```

**Structure Decision**: Web application pattern selected. Backend is a standalone FastAPI service in `/backend`. Frontend components integrate into existing Docusaurus app at `/ai-robotics-book`.

## Complexity Tracking

> No constitution violations requiring justification.

| Decision | Rationale | Simpler Alternative Rejected Because |
|----------|-----------|-------------------------------------|
| Separate backend service | Clear separation of concerns; Python ecosystem for AI/ML | Serverless functions would require more complex deployment |
| OpenAI Agents SDK | Structured conversation management with tool calling | Raw API calls would require manual context/tool handling |
| Qdrant Cloud | Managed vector DB with free tier | Self-hosted would add infrastructure complexity |

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              USER BROWSER                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────┐    ┌─────────────────────────────────────────┐    │
│  │   Docusaurus Site   │    │           ChatWidget                     │    │
│  │   (localhost:3000)  │    │  ┌─────────────┐  ┌──────────────────┐  │    │
│  │                     │    │  │ ChatPanel   │  │ SelectionHandler │  │    │
│  │  ┌───────────────┐  │    │  │  - Messages │  │  - Text detect   │  │    │
│  │  │  Book Content │◀─┼────┼──│  - Input    │  │  - Ask button    │  │    │
│  │  │  (Markdown)   │  │    │  │  - Loading  │  │                  │  │    │
│  │  └───────────────┘  │    │  └─────────────┘  └──────────────────┘  │    │
│  └─────────────────────┘    └──────────────┬──────────────────────────┘    │
│                                            │                                │
│                              sessionStorage (chat history)                  │
└────────────────────────────────────────────┼────────────────────────────────┘
                                             │ HTTP (localhost)
                                             ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         BACKEND (FastAPI - localhost:8000)                   │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                           API Router                                 │    │
│  │  POST /api/chat          POST /api/chat/selection    GET /api/health│    │
│  └───────────┬─────────────────────────┬─────────────────────┬─────────┘    │
│              │                         │                     │              │
│  ┌───────────▼─────────────────────────▼─────────────────────▼─────────┐    │
│  │                         Services Layer                               │    │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │    │
│  │  │ embeddings  │  │   qdrant    │  │    agent    │  │  postgres   │ │    │
│  │  │  (OpenAI)   │  │  (search)   │  │ (RAG+Tools) │  │  (logging)  │ │    │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘ │    │
│  └─────────┼────────────────┼────────────────┼────────────────┼────────┘    │
└────────────┼────────────────┼────────────────┼────────────────┼─────────────┘
             │                │                │                │
             ▼                ▼                ▼                ▼
      ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
      │   OpenAI     │ │    Qdrant    │ │   OpenAI     │ │    Neon      │
      │   Embed API  │ │    Cloud     │ │   Chat API   │ │   Postgres   │
      │              │ │ (Free Tier)  │ │ (gpt-4o-mini)│ │ (Free Tier)  │
      └──────────────┘ └──────────────┘ └──────────────┘ └──────────────┘
```

## Data Flow

### Query Flow

```
1. User types question in ChatPanel
2. useChat hook calls apiClient.sendMessage()
3. POST /api/chat → FastAPI router
4. embeddings.embed(query) → OpenAI Embedding API
5. qdrant.search(vector, top_k=5) → Qdrant Cloud
6. agent.generate(query, context) → OpenAI Chat API with retrieved chunks
7. postgres.log_query(session_id, query, response)
8. Return ChatResponse with citations
9. Frontend stores in sessionStorage, renders message
```

### Selection Flow

```
1. User selects text on page
2. useTextSelection detects selection via mouseup/touchend
3. AskButton appears near selection
4. User clicks "Ask about this"
5. ChatPanel opens with selectedText context
6. POST /api/chat/selection with { selected_text, question? }
7. Agent receives selectedText as additional context
8. (Same steps 7-9 as Query Flow)
```

## Implementation Phases

### Phase 1: Backend Setup & Connections
- Initialize backend/ with uv
- Configure FastAPI app with CORS
- Implement health check endpoint
- Test Qdrant connection (create collection)
- Test Neon Postgres connection (run migrations)

### Phase 2: Data Ingestion
- Build markdown parser for docs/ content
- Implement chunking (800 tokens, 100 overlap)
- Generate embeddings via OpenAI
- Store chunks in Qdrant with metadata

### Phase 3: RAG Agent & API
- Build embeddings service wrapper
- Build Qdrant search service
- Implement RAG agent with OpenAI Agents SDK
- Create /api/chat endpoint
- Create /api/chat/selection endpoint
- Implement Postgres logging service

### Phase 4: Frontend Chat Widget
- Swizzle Docusaurus Root component
- Build ChatWidget (floating button)
- Build ChatPanel (messages, input, loading)
- Build apiClient service
- Implement useChat hook with sessionStorage

### Phase 5: Text Selection Feature
- Build useTextSelection hook
- Build SelectionHandler component
- Build AskButton component
- Integrate with ChatWidget

### Phase 6: Integration & Polish
- End-to-end testing all user stories
- Mobile responsive styling
- Error handling polish
- Verify analytics logging

## Key Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Vector DB | Qdrant Cloud | Free tier, excellent performance, Python client |
| Embedding Model | text-embedding-3-small | Cost-effective, same API as chat |
| Chat Model | gpt-4o-mini | Best cost/quality ratio for Q&A |
| Backend Framework | FastAPI | Native async, auto-docs, Pydantic |
| Package Manager | uv | Fast, lockfile support |
| Analytics DB | Neon Postgres | Free tier, serverless, standard SQL |
| Client Storage | sessionStorage | Session-scoped, privacy-friendly |
| Chunking | 800 tokens / 100 overlap | Balances context and precision |

## Related Artifacts

- **Research**: [research.md](./research.md) - Technology evaluation
- **Data Model**: [data-model.md](./data-model.md) - Entity definitions
- **API Contract**: [contracts/api-spec.yaml](./contracts/api-spec.yaml) - OpenAPI spec
- **Types**: [contracts/frontend-types.ts](./contracts/frontend-types.ts) - TypeScript interfaces
- **Quickstart**: [quickstart.md](./quickstart.md) - Setup guide

## Next Steps

Run `/sp.tasks` to generate the implementation task breakdown based on this plan.
