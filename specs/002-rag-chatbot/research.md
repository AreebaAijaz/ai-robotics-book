# Research: RAG Chatbot with Text Selection

**Feature**: 002-rag-chatbot
**Date**: 2025-12-16
**Status**: Complete

## Executive Summary

This research document consolidates technology decisions for the RAG chatbot feature. All key decisions have been evaluated against alternatives and validated for compatibility with the existing Docusaurus (React 19) frontend.

---

## Research Areas

### 1. Vector Database: Qdrant Cloud

**Decision**: Use Qdrant Cloud Free Tier

**Rationale**:
- Free tier offers 1GB storage, sufficient for book content (~20 markdown files)
- Native Python client with async support
- Excellent performance for semantic search (HNSW algorithm)
- Built-in payload filtering for metadata (module, chapter)
- No infrastructure management required

**Alternatives Considered**:

| Alternative | Rejected Because |
|-------------|------------------|
| Pinecone | Free tier has stricter limits; more complex authentication |
| ChromaDB | Local-only unless self-hosted; no managed cloud option |
| Weaviate | More complex setup; overkill for this use case |
| pgvector | Requires Postgres setup; less optimized for vector ops |

**Configuration**:
- Collection name: `physical_ai_book`
- Vector dimensions: 1536 (OpenAI text-embedding-3-small)
- Distance metric: Cosine similarity
- Payload fields: `module`, `chapter`, `file_path`, `section`

---

### 2. Embedding Model: OpenAI text-embedding-3-small

**Decision**: Use OpenAI text-embedding-3-small

**Rationale**:
- Cost-effective ($0.02 per 1M tokens)
- 1536 dimensions provides good quality/performance balance
- Same API ecosystem as chat completion (single API key)
- Excellent semantic understanding for technical content

**Alternatives Considered**:

| Alternative | Rejected Because |
|-------------|------------------|
| text-embedding-3-large | Higher cost, marginal quality gain for this use case |
| Cohere Embed | Additional API key/account required |
| Local models (sentence-transformers) | Requires GPU/compute; adds deployment complexity |
| Azure OpenAI | More complex setup; overkill for local dev |

**Usage Pattern**:
- Ingestion: Batch embed all chunks once
- Query: Single embedding per user query
- Estimated cost: <$0.01 total for full book ingestion

---

### 3. LLM for Chat: gpt-4o-mini with OpenAI Agents SDK

**Decision**: Use gpt-4o-mini via OpenAI Agents SDK

**Rationale**:
- gpt-4o-mini offers excellent cost/quality ratio ($0.15/1M input, $0.60/1M output)
- OpenAI Agents SDK provides structured conversation management
- Built-in tool calling for retrieval augmentation
- Supports streaming responses for better UX

**Alternatives Considered**:

| Alternative | Rejected Because |
|-------------|------------------|
| gpt-4o | 10x more expensive; not needed for Q&A |
| Claude (Anthropic) | Additional API integration; good but not necessary |
| Llama 3 (local) | Requires GPU; deployment complexity |
| LangChain | More abstraction than needed; Agents SDK is simpler |

**Agent Configuration**:
- System prompt: Scoped to Physical AI book content
- Temperature: 0.3 (factual, consistent responses)
- Max tokens: 1024 (sufficient for detailed answers)
- Tools: `search_book_content` (Qdrant retrieval)

---

### 4. Analytics Database: Neon Postgres

**Decision**: Use Neon Postgres Free Tier

**Rationale**:
- Free tier offers 0.5GB storage, 3GB transfer/month
- Serverless with auto-suspend (cost efficient)
- Standard PostgreSQL compatibility
- Simple connection string (no special driver)

**Alternatives Considered**:

| Alternative | Rejected Because |
|-------------|------------------|
| Supabase | More features than needed; adds complexity |
| PlanetScale | MySQL-based; less familiar |
| Local SQLite | Doesn't persist across dev sessions easily |
| MongoDB Atlas | Document DB not ideal for relational query logs |

**Schema Design**:
```sql
-- Chat sessions
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMPTZ DEFAULT NOW(),
    updated_at TIMESTAMPTZ DEFAULT NOW()
);

-- Query logs
CREATE TABLE query_logs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID REFERENCES chat_sessions(id),
    query_text TEXT NOT NULL,
    response_text TEXT NOT NULL,
    response_time_ms INTEGER NOT NULL,
    sources JSONB,
    created_at TIMESTAMPTZ DEFAULT NOW()
);

-- Index for analytics queries
CREATE INDEX idx_query_logs_created_at ON query_logs(created_at);
CREATE INDEX idx_query_logs_session_id ON query_logs(session_id);
```

---

### 5. Backend Framework: FastAPI

**Decision**: Use FastAPI with uvicorn

**Rationale**:
- Native async support for concurrent API calls
- Automatic OpenAPI documentation
- Pydantic validation for request/response models
- Lightweight, fast startup for local development

**Alternatives Considered**:

| Alternative | Rejected Because |
|-------------|------------------|
| Flask | No native async; requires additional setup |
| Django | Heavyweight for simple API; unnecessary ORM |
| Starlette | Lower-level; FastAPI adds convenience |
| Express.js | Different language; Python preferred for AI/ML |

**Structure**:
```
backend/
├── main.py           # FastAPI app entry
├── config.py         # Environment config
├── models/           # Pydantic models
│   ├── chat.py
│   └── query.py
├── services/
│   ├── embeddings.py    # OpenAI embedding wrapper
│   ├── qdrant.py        # Vector search service
│   ├── agent.py         # RAG agent with Agents SDK
│   └── postgres.py      # Query logging service
├── routers/
│   └── chat.py          # /api/chat endpoints
└── scripts/
    └── ingest_docs.py   # Data ingestion script
```

---

### 6. Python Package Management: uv

**Decision**: Use uv for dependency management

**Rationale**:
- 10-100x faster than pip
- Lockfile support (uv.lock)
- Built-in virtual environment management
- Drop-in pip replacement

**Alternatives Considered**:

| Alternative | Rejected Because |
|-------------|------------------|
| pip + venv | Slower; no lockfile by default |
| Poetry | Heavier; more complex for simple projects |
| Conda | Overkill; not needed for pure Python |
| PDM | Less mainstream adoption |

**Setup Commands**:
```bash
# Initialize project
uv init backend
cd backend

# Add dependencies
uv add fastapi uvicorn qdrant-client openai psycopg2-binary python-dotenv

# Run server
uv run uvicorn main:app --reload --port 8000
```

---

### 7. Content Chunking Strategy

**Decision**: 800 tokens with 100 token overlap

**Rationale**:
- 800 tokens (~3200 chars) captures full concepts without fragmentation
- 100 token overlap preserves context at chunk boundaries
- Fits comfortably in context window with multiple retrieved chunks
- Balances precision (smaller) vs context (larger)

**Alternatives Considered**:

| Alternative | Rejected Because |
|-------------|------------------|
| 500 tokens | Too fragmented; loses context |
| 1500 tokens | Fewer chunks; less precise retrieval |
| Semantic chunking | More complex; marginal benefit for structured docs |
| Fixed character count | Doesn't respect token boundaries |

**Metadata Extraction**:
- Module: Parsed from directory path (e.g., `module-1-ros2`)
- Chapter: Parsed from filename (e.g., `01-architecture`)
- Section: Parsed from markdown headers (h1, h2)
- File path: Full path for source citation

---

### 8. Frontend Integration: Docusaurus Swizzle

**Decision**: Swizzle Root component to inject ChatWidget

**Rationale**:
- Non-destructive customization
- ChatWidget available on all pages
- Follows Docusaurus best practices
- Preserves upgrade path

**Alternatives Considered**:

| Alternative | Rejected Because |
|-------------|------------------|
| Custom plugin | More complex; harder to maintain |
| Edit theme directly | Breaks on Docusaurus updates |
| iframe embed | Poor UX; isolated from page context |
| MDX injection | Would need per-page changes |

**Implementation**:
```bash
# Swizzle Root component
npx docusaurus swizzle @docusaurus/theme-classic Root --wrap

# Creates: src/theme/Root.js
```

**Component Structure**:
```
src/
├── components/
│   ├── ChatWidget/
│   │   ├── index.js       # Main component
│   │   ├── ChatPanel.js   # Message UI
│   │   ├── ChatInput.js   # Input field
│   │   └── styles.module.css
│   └── SelectionHandler/
│       ├── index.js       # Selection detection
│       └── AskButton.js   # Contextual button
├── services/
│   └── apiClient.js       # Backend API calls
└── theme/
    └── Root.js            # Swizzled root with ChatWidget
```

---

### 9. Text Selection Detection

**Decision**: Use window.getSelection() with event delegation

**Rationale**:
- Native browser API, no dependencies
- Works across all content areas
- Can extract selected text and position for button placement
- Event delegation avoids memory leaks

**Implementation Approach**:
1. Listen for `mouseup` and `touchend` events on document
2. Check if selection is within docs content area
3. If selection exists (length > 0), show "Ask about this" button
4. Position button near selection end point
5. On button click, open chat with selected text as context

**Constraints**:
- Maximum selection: 2000 characters (truncate with warning)
- Minimum selection: 3 characters (ignore single characters)
- Exclude code blocks from auto-selection prompt (user can still manually ask)

---

### 10. Session Persistence: sessionStorage

**Decision**: Use sessionStorage for chat history

**Rationale**:
- Persists across page navigation (spec requirement)
- Clears on browser close (privacy by design)
- No backend storage of user data required
- Simple synchronous API

**Alternatives Considered**:

| Alternative | Rejected Because |
|-------------|------------------|
| localStorage | Persists indefinitely; privacy concern |
| IndexedDB | Overkill for simple message list |
| Backend sessions | Adds complexity; requires auth |
| React state only | Lost on navigation |

**Storage Schema**:
```typescript
// Key: 'chatbot_session'
interface StoredSession {
  sessionId: string;
  messages: Array<{
    id: string;
    role: 'user' | 'assistant';
    content: string;
    timestamp: number;
    citations?: Array<{module: string; chapter: string}>;
  }>;
  createdAt: number;
}
```

---

## Compatibility Matrix

| Component | Version | Compatibility Notes |
|-----------|---------|---------------------|
| Docusaurus | 3.9.2 | React 19 compatible |
| React | 19.0.0 | Hooks-based components |
| Node.js | >=20.0 | Required by Docusaurus |
| Python | 3.11+ | Required for latest OpenAI SDK |
| FastAPI | 0.115+ | Async support |
| Qdrant Client | 1.12+ | Async operations |
| OpenAI SDK | 1.50+ | Agents SDK support |

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Qdrant free tier limits exceeded | Low | Medium | Monitor usage; upgrade if needed |
| OpenAI API rate limits | Low | High | Implement retry with backoff |
| CORS issues during dev | Medium | Low | Configure FastAPI CORS middleware |
| Slow response times | Medium | Medium | Add loading states; optimize prompts |
| Selection detection edge cases | Medium | Low | Test across browsers; graceful fallbacks |

---

## Conclusion

All technology decisions have been researched and validated. No outstanding clarifications required. Ready to proceed to Phase 1 (Design & Contracts).
