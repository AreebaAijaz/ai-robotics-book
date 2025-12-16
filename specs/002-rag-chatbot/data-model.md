# Data Model: RAG Chatbot with Text Selection

**Feature**: 002-rag-chatbot
**Date**: 2025-12-16
**Status**: Complete

## Overview

This document defines the data entities, their relationships, and storage locations for the RAG chatbot feature. Data is distributed across three storage systems based on purpose:

1. **Qdrant** - Vector embeddings for semantic search
2. **Neon Postgres** - Query logs and analytics
3. **sessionStorage** - Client-side chat history

---

## Entity Relationship Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        QDRANT CLOUD                              │
├─────────────────────────────────────────────────────────────────┤
│  ContentChunk                                                    │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │ id: string (UUID)                                          │  │
│  │ vector: float[1536]                                        │  │
│  │ payload: {                                                 │  │
│  │   module: string                                           │  │
│  │   chapter: string                                          │  │
│  │   section: string                                          │  │
│  │   file_path: string                                        │  │
│  │   text: string                                             │  │
│  │   chunk_index: int                                         │  │
│  │ }                                                          │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                       NEON POSTGRES                              │
├─────────────────────────────────────────────────────────────────┤
│  ChatSession                    QueryLog                         │
│  ┌─────────────────────┐       ┌─────────────────────────────┐  │
│  │ id: UUID (PK)       │──┐    │ id: UUID (PK)               │  │
│  │ created_at: timestamp│  │    │ session_id: UUID (FK)       │──┤
│  │ updated_at: timestamp│  └───▶│ query_text: text            │  │
│  └─────────────────────┘       │ response_text: text          │  │
│                                │ response_time_ms: int        │  │
│                                │ sources: jsonb               │  │
│                                │ query_type: enum             │  │
│                                │ selected_text: text (null)   │  │
│                                │ created_at: timestamp        │  │
│                                └─────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                    CLIENT (sessionStorage)                       │
├─────────────────────────────────────────────────────────────────┤
│  ChatSessionState                                                │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │ sessionId: string                                          │  │
│  │ messages: ChatMessage[]                                    │  │
│  │ createdAt: number                                          │  │
│  └───────────────────────────────────────────────────────────┘  │
│                                                                  │
│  ChatMessage                                                     │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │ id: string                                                 │  │
│  │ role: 'user' | 'assistant'                                 │  │
│  │ content: string                                            │  │
│  │ timestamp: number                                          │  │
│  │ citations?: Citation[]                                     │  │
│  │ selectedText?: string                                      │  │
│  └───────────────────────────────────────────────────────────┘  │
│                                                                  │
│  Citation                                                        │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │ module: string                                             │  │
│  │ chapter: string                                            │  │
│  │ section: string                                            │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Entity Definitions

### 1. ContentChunk (Qdrant)

Represents a segment of book content indexed for semantic search.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | string | Unique identifier | UUID v4 |
| vector | float[1536] | Embedding from text-embedding-3-small | Required |
| payload.module | string | Module name (e.g., "module-1-ros2") | Required |
| payload.chapter | string | Chapter name (e.g., "01-architecture") | Required |
| payload.section | string | Section heading (e.g., "ROS 2 Nodes") | Optional |
| payload.file_path | string | Relative path to source file | Required |
| payload.text | string | Original text content | Max 4000 chars |
| payload.chunk_index | int | Position within source file | >= 0 |

**Indexing**:
- HNSW index on vector field for fast approximate nearest neighbor search
- Payload indexes on `module` and `chapter` for filtered queries

---

### 2. ChatSession (Postgres)

Represents a conversation session for analytics tracking.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | UUID | Primary key | Auto-generated |
| created_at | TIMESTAMPTZ | Session start time | Default NOW() |
| updated_at | TIMESTAMPTZ | Last activity time | Default NOW() |

**SQL Definition**:
```sql
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);
```

---

### 3. QueryLog (Postgres)

Records each user query and system response for analytics.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | UUID | Primary key | Auto-generated |
| session_id | UUID | Foreign key to ChatSession | Required |
| query_text | TEXT | User's question | Required, max 2000 chars |
| response_text | TEXT | Assistant's response | Required |
| response_time_ms | INTEGER | Processing time | Required, >= 0 |
| sources | JSONB | Retrieved chunks metadata | Optional |
| query_type | TEXT | 'general' or 'selection' | Required |
| selected_text | TEXT | User's selected text (if selection query) | Optional |
| created_at | TIMESTAMPTZ | Query timestamp | Default NOW() |

**SQL Definition**:
```sql
CREATE TYPE query_type AS ENUM ('general', 'selection');

CREATE TABLE query_logs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES chat_sessions(id) ON DELETE CASCADE,
    query_text TEXT NOT NULL CHECK (char_length(query_text) <= 2000),
    response_text TEXT NOT NULL,
    response_time_ms INTEGER NOT NULL CHECK (response_time_ms >= 0),
    sources JSONB,
    query_type query_type NOT NULL DEFAULT 'general',
    selected_text TEXT,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_query_logs_session_id ON query_logs(session_id);
CREATE INDEX idx_query_logs_created_at ON query_logs(created_at);
CREATE INDEX idx_query_logs_query_type ON query_logs(query_type);
```

**Sources JSONB Structure**:
```json
[
  {
    "module": "module-1-ros2",
    "chapter": "01-architecture",
    "section": "ROS 2 Nodes",
    "score": 0.89
  }
]
```

---

### 4. ChatSessionState (Client)

Client-side representation of current chat session.

```typescript
interface ChatSessionState {
  sessionId: string;        // UUID, matches Postgres session
  messages: ChatMessage[];  // Ordered list of messages
  createdAt: number;        // Unix timestamp (ms)
}
```

**Storage Key**: `chatbot_session`
**Storage Location**: `sessionStorage`

---

### 5. ChatMessage (Client)

Individual message in the chat history.

```typescript
interface ChatMessage {
  id: string;                      // UUID
  role: 'user' | 'assistant';      // Message sender
  content: string;                 // Message text (markdown supported)
  timestamp: number;               // Unix timestamp (ms)
  citations?: Citation[];          // Source references (assistant only)
  selectedText?: string;           // Selected text context (user only)
  isLoading?: boolean;             // Typing indicator state
  error?: string;                  // Error message if failed
}
```

---

### 6. Citation (Client)

Reference to source material in assistant responses.

```typescript
interface Citation {
  module: string;    // e.g., "Module 1: ROS 2 Fundamentals"
  chapter: string;   // e.g., "Chapter 1: Architecture"
  section: string;   // e.g., "ROS 2 Nodes"
}
```

---

## Data Flow

### Ingestion Flow (One-time)

```
1. Read markdown files from ai-robotics-book/docs/
2. Parse metadata (module, chapter, section from headers)
3. Chunk text (800 tokens, 100 overlap)
4. Generate embeddings via OpenAI API
5. Store in Qdrant collection "physical_ai_book"
```

### Query Flow (Runtime)

```
1. User submits query via ChatWidget
2. Frontend sends POST /api/chat with query + sessionId
3. Backend embeds query text
4. Backend searches Qdrant for top 5 similar chunks
5. Backend invokes RAG agent with query + retrieved context
6. Backend logs query to Neon Postgres
7. Backend returns response with citations
8. Frontend stores message in sessionStorage
9. Frontend renders response with citations
```

### Selection Query Flow

```
1. User selects text on page
2. SelectionHandler shows "Ask about this" button
3. User clicks button, optionally adds question
4. Frontend sends POST /api/chat/selection with:
   - selectedText: the highlighted content
   - question: optional follow-up question
   - sessionId: current session
5. Backend includes selectedText in agent context
6. (Same steps 5-9 as Query Flow)
```

---

## Validation Rules

### ContentChunk
- `text` must not exceed 4000 characters
- `vector` must have exactly 1536 dimensions
- `module` must match pattern `module-[1-4]-*`
- `file_path` must be valid relative path

### ChatMessage
- `content` must not be empty
- `content` max length: 10,000 characters
- `selectedText` max length: 2000 characters

### QueryLog
- `query_text` max length: 2000 characters
- `response_time_ms` must be >= 0
- `session_id` must reference existing session

---

## Migration Strategy

### Initial Setup (Qdrant)
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

client.create_collection(
    collection_name="physical_ai_book",
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
)
```

### Initial Setup (Postgres)
```sql
-- Run migrations in order
-- 001_create_sessions.sql
-- 002_create_query_logs.sql
-- 003_add_indexes.sql
```

---

## Retention Policy

| Data Type | Location | Retention |
|-----------|----------|-----------|
| ContentChunk | Qdrant | Permanent (re-ingest on content update) |
| ChatSession | Postgres | 90 days (local dev, can be shorter) |
| QueryLog | Postgres | 90 days |
| ChatSessionState | sessionStorage | Browser session only |
