# Tasks: RAG Chatbot with Text Selection

**Input**: Design documents from `/specs/002-rag-chatbot/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Manual testing for local development phase (no automated tests required)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/` at repository root (Python/FastAPI)
- **Frontend**: `ai-robotics-book/src/` (React/Docusaurus)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, dependencies, and environment configuration

- [x] T001 Create backend/ directory and initialize uv project with `uv init --name rag-chatbot`
- [x] T002 [P] Install backend dependencies: `uv add fastapi uvicorn qdrant-client openai psycopg2-binary python-dotenv pydantic`
- [x] T003 [P] Create backend/.env.template with placeholder environment variables (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, CORS_ORIGINS)
- [x] T004 [P] Create backend/config.py with environment configuration loader using python-dotenv
- [x] T005 Create backend/main.py with FastAPI app skeleton and CORS middleware for localhost:3000

**Checkpoint**: Backend project initialized, dependencies installed, environment config ready

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Database connections, health checks, and data ingestion that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database Connections

- [x] T006 Create backend/services/__init__.py (empty module init)
- [x] T007 Create backend/services/qdrant.py with Qdrant client initialization and collection creation (physical_ai_book, 1536 dimensions, cosine distance)
- [x] T008 Create backend/scripts/migrations/001_create_tables.sql with chat_sessions and query_logs tables per data-model.md
- [x] T009 Create backend/services/postgres.py with Neon connection pool and basic query functions

### Health Check Endpoint

- [x] T010 Create backend/models/__init__.py (empty module init)
- [x] T011 [P] Create backend/models/health.py with HealthResponse, DependencyStatus Pydantic models per contracts/api-spec.yaml
- [x] T012 Create backend/routers/__init__.py (empty module init)
- [x] T013 Create backend/routers/chat.py with GET /api/health endpoint that checks Qdrant, Postgres, and OpenAI connectivity
- [x] T014 Register chat router in backend/main.py and verify health endpoint returns status

### Data Ingestion Pipeline

- [x] T015 Create backend/services/embeddings.py with OpenAI text-embedding-3-small wrapper function
- [x] T016 Create backend/scripts/ingest_docs.py - Part 1: Read markdown files from ai-robotics-book/docs/ directory
- [x] T017 Update backend/scripts/ingest_docs.py - Part 2: Implement text chunking (800 tokens, 100 overlap) with metadata extraction (module, chapter, section, file_path)
- [x] T018 Update backend/scripts/ingest_docs.py - Part 3: Generate embeddings and store chunks in Qdrant with payload metadata
- [x] T019 Run ingestion script and verify Qdrant collection has ~180 vectors with correct metadata (338 chunks ingested)

**Checkpoint**: Foundation ready - Backend connects to Qdrant and Postgres, health check works, book content indexed in Qdrant

---

## Phase 3: User Story 1 - General Book Questions (Priority: P1) 🎯 MVP

**Goal**: Readers can ask questions about book content and receive accurate answers with source citations

**Independent Test**: Type "What is ROS 2?" in chat widget, verify response includes relevant information and citations to Module 1

### Backend API for User Story 1

- [x] T020 [P] [US1] Create backend/models/chat.py with ChatRequest, ChatResponse, Citation, SelectionChatRequest Pydantic models per contracts/api-spec.yaml
- [x] T021 [US1] Update backend/services/qdrant.py - Add semantic_search() function that embeds query and returns top 5 chunks with scores
- [x] T022 [US1] Create backend/services/agent.py with RAG agent using OpenAI Agents SDK - system prompt scoped to Physical AI book, temperature 0.3
- [x] T023 [US1] Update backend/services/agent.py - Add search_book_content tool that calls qdrant.semantic_search()
- [x] T024 [US1] Update backend/services/agent.py - Implement generate_response() that returns answer with citations extracted from retrieved chunks
- [x] T025 [US1] Update backend/services/postgres.py - Add log_query() function to insert into query_logs table
- [x] T026 [US1] Add POST /api/chat endpoint in backend/routers/chat.py that orchestrates: embed → search → agent → log → respond
- [x] T027 [US1] Test /api/chat endpoint with curl: `curl -X POST localhost:8000/api/chat -H "Content-Type: application/json" -d '{"session_id":"test","message":"What is ROS 2?"}'`

### Frontend Chat Widget for User Story 1

- [x] T028 [P] [US1] Create ai-robotics-book/src/services/apiClient.js with sendMessage() and healthCheck() functions calling localhost:8000
- [x] T029 [P] [US1] Create ai-robotics-book/src/components/ChatWidget/styles.module.css with floating button and panel styles
- [x] T030 [US1] Create ai-robotics-book/src/components/ChatWidget/ChatInput.js - text input with submit button, disabled state during loading
- [x] T031 [US1] Create ai-robotics-book/src/components/ChatWidget/Message.js - renders user/assistant messages with markdown support and citation links
- [x] T032 [US1] Create ai-robotics-book/src/components/ChatWidget/ChatPanel.js - scrollable message list, ChatInput, loading indicator
- [x] T033 [US1] Create ai-robotics-book/src/components/ChatWidget/index.js - floating button that toggles ChatPanel visibility
- [x] T034 [US1] Create ai-robotics-book/src/hooks/useChat.js - manages messages state, sendMessage function, loading state, error state
- [x] T035 [US1] Create ai-robotics-book/src/theme/Root.js (custom Root component)
- [x] T036 [US1] Update ai-robotics-book/src/theme/Root.js to render ChatWidget on all pages

**Checkpoint**: User Story 1 complete - General Q&A works end-to-end, chat widget visible on all pages, citations displayed

---

## Phase 4: User Story 2 - Text Selection Queries (Priority: P2)

**Goal**: Readers can select text on any page and ask questions about the selection

**Independent Test**: Select text "DDS middleware", click "Ask about this", verify response explains DDS in context

### Backend API Extension for User Story 2

- [x] T037 [US2] Add POST /api/chat/selection endpoint in backend/routers/chat.py that accepts selected_text and optional question (done in Phase 3)
- [x] T038 [US2] Update backend/services/agent.py - Modify generate_response() to include selected_text as additional context when provided (done in Phase 3)

### Frontend Selection Handler for User Story 2

- [x] T039 [P] [US2] Create ai-robotics-book/src/hooks/useTextSelection.js - detects text selection via mouseup/touchend, returns selectedText and position
- [x] T040 [P] [US2] Create ai-robotics-book/src/components/SelectionHandler/AskButton.js - floating "Ask about this" button positioned near selection
- [x] T041 [US2] Create ai-robotics-book/src/components/SelectionHandler/index.js - HOC that shows AskButton on text selection, calls onAskAboutSelection callback
- [x] T042 [US2] Update ai-robotics-book/src/services/apiClient.js - Add sendSelectionMessage() function for /api/chat/selection endpoint (done in Phase 3)
- [x] T043 [US2] Update ai-robotics-book/src/hooks/useChat.js - Add sendSelectionMessage function that passes selectedText to API (done in Phase 3)
- [x] T044 [US2] Update ai-robotics-book/src/theme/Root.js - Wrap content with SelectionHandler, connect to ChatWidget

**Checkpoint**: User Story 2 complete - Text selection shows "Ask about this" button, selection queries work

---

## Phase 5: User Story 3 - Chat Persistence Within Session (Priority: P3)

**Goal**: Chat history persists across page navigation within the same browser session

**Independent Test**: Have a conversation, navigate to a different page, verify chat history is preserved

### Frontend Session Storage for User Story 3

- [x] T045 [US3] Update ai-robotics-book/src/hooks/useChat.js - Add sessionStorage save on message change (done in Phase 3)
- [x] T046 [US3] Update ai-robotics-book/src/hooks/useChat.js - Add sessionStorage load on hook initialization (done in Phase 3)
- [x] T047 [US3] Update ai-robotics-book/src/hooks/useChat.js - Add clearChat function that clears sessionStorage (done in Phase 3)
- [x] T048 [US3] Update ai-robotics-book/src/components/ChatWidget/ChatPanel.js - Add clear chat button in header

**Checkpoint**: User Story 3 complete - Chat history persists across page navigation

---

## Phase 6: User Story 4 - Mobile Responsive Chat (Priority: P4)

**Goal**: Chat widget works well on mobile devices from 320px to 1920px width

**Independent Test**: Open chat widget on mobile viewport (320px), verify usability

### Mobile Responsive Styles for User Story 4

- [x] T049 [US4] Update ai-robotics-book/src/components/ChatWidget/styles.module.css - Add mobile breakpoints (max-width: 768px) for ChatPanel
- [x] T050 [US4] Update ai-robotics-book/src/components/ChatWidget/styles.module.css - Full-screen panel on mobile, adjusted button size
- [x] T051 [US4] Update ai-robotics-book/src/components/ChatWidget/styles.module.css - Handle mobile keyboard appearance, prevent viewport shift (CSS)
- [x] T052 [US4] Update ai-robotics-book/src/components/SelectionHandler/styles.module.css - Larger touch target for mobile (min 44px)

**Checkpoint**: User Story 4 complete - Chat widget works on mobile devices

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Error handling, edge cases, and final polish

### Error Handling & Edge Cases

- [x] T053 Update ai-robotics-book/src/components/ChatWidget/ChatInput.js - Prevent empty message submission, show validation
- [x] T054 Update ai-robotics-book/src/components/SelectionHandler/index.js - Truncate selections >2000 characters
- [x] T055 Update ai-robotics-book/src/components/ChatWidget/ChatPanel.js - Show error state with retry button when backend unavailable
- [ ] T056 Update ai-robotics-book/src/hooks/useChat.js - Add request queue for rapid sequential messages (skipped - not critical for local dev)

### Visual Polish

- [x] T057 [P] Update ai-robotics-book/src/components/ChatWidget/styles.module.css - Add subtle animations (fade-in, slide-up)
- [x] T058 [P] Update ai-robotics-book/src/components/ChatWidget/ChatPanel.js - Loading indicator with animation (typing indicator)
- [x] T059 Update ai-robotics-book/src/components/ChatWidget/ChatPanel.js - Auto-scroll to latest message

### Final Validation

- [ ] T060 Run quickstart.md validation - verify all setup steps work
- [ ] T061 Test all acceptance scenarios from spec.md User Story 1 (3 scenarios)
- [ ] T062 Test all acceptance scenarios from spec.md User Story 2 (3 scenarios)
- [ ] T063 Test all acceptance scenarios from spec.md User Story 3 (3 scenarios)
- [ ] T064 Test all acceptance scenarios from spec.md User Story 4 (2 scenarios)
- [ ] T065 Verify analytics logging in Neon Postgres query_logs table

**Checkpoint**: Complete system working locally with all user stories and edge cases handled

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User stories can then proceed in priority order (P1 → P2 → P3 → P4)
  - Or in parallel if team capacity allows
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Depends on US1 frontend components (ChatWidget, useChat hook)
- **User Story 3 (P3)**: Depends on US1 useChat hook
- **User Story 4 (P4)**: Depends on US1 and US2 components for responsive updates

### Within Each Phase

- Backend tasks before frontend tasks that depend on API
- Services before routers
- Hooks before components that use them
- Core implementation before integration

### Parallel Opportunities

**Phase 1 (Setup):**
```
T002 (dependencies) || T003 (.env) || T004 (config)
```

**Phase 2 (Foundational):**
```
After T006: T007 (qdrant) || T008 (migrations) || T009 (postgres) || T011 (models)
After T015: T016-T019 (ingestion - sequential)
```

**Phase 3 (US1):**
```
Backend: T020 (models) || then T021-T026 (sequential services/routes)
Frontend: T028 (apiClient) || T029 (styles) || then T030-T036 (sequential components)
```

**Phase 4 (US2):**
```
T039 (useTextSelection) || T040 (AskButton) || then T041-T044 (sequential)
```

---

## Parallel Example: Phase 2 Foundational

```bash
# After T006 completes, launch these in parallel:
Task: "Create backend/services/qdrant.py with Qdrant client initialization"
Task: "Create backend/scripts/migrations/001_create_tables.sql"
Task: "Create backend/services/postgres.py with Neon connection"
Task: "Create backend/models/health.py with Pydantic models"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (5 tasks)
2. Complete Phase 2: Foundational (14 tasks)
3. Complete Phase 3: User Story 1 (17 tasks)
4. **STOP and VALIDATE**: Test general Q&A functionality end-to-end
5. Deploy/demo if ready - readers can ask questions about the book!

### Incremental Delivery

1. **MVP**: Setup + Foundational + US1 → General Q&A working
2. **+US2**: Text selection queries → Contextual learning feature
3. **+US3**: Session persistence → Better UX for extended study sessions
4. **+US4**: Mobile responsive → Accessible on all devices
5. **+Polish**: Error handling, animations → Production-ready quality

### Estimated Task Counts

| Phase | Tasks | Cumulative |
|-------|-------|------------|
| Phase 1: Setup | 5 | 5 |
| Phase 2: Foundational | 14 | 19 |
| Phase 3: User Story 1 | 17 | 36 |
| Phase 4: User Story 2 | 8 | 44 |
| Phase 5: User Story 3 | 4 | 48 |
| Phase 6: User Story 4 | 4 | 52 |
| Phase 7: Polish | 13 | 65 |

**Total**: 65 tasks

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story is independently completable and testable after Phase 2
- Manual testing is sufficient for local development phase
- Commit after each task or logical group
- Stop at any checkpoint to validate independently
- Backend must be running (`uv run uvicorn main:app --reload --port 8000`) before frontend testing
