# Implementation Plan: Personalized RAG with User Authentication

**Branch**: `003-personalized-rag-auth` | **Date**: 2025-12-19 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-personalized-rag-auth/spec.md`

## Summary

Add user authentication and personalization to the RAG chatbot:
- **Authentication**: Better Auth (frontend) + JWT verification (FastAPI backend)
- **User Profiling**: 7-question onboarding questionnaire
- **Personalization**: Profile-aware system prompts for tailored responses

## Technical Context

**Language/Version**: Python 3.11+ (backend), JavaScript/React (frontend)
**Primary Dependencies**: 
- Frontend: Better Auth ^1.4.x, React, Docusaurus 3.9.2
- Backend: FastAPI, PyJWT ^2.8.x, psycopg2-binary

**Storage**: Neon PostgreSQL (users, profiles), Qdrant Cloud (vectors)
**Testing**: pytest (backend), Jest (frontend)
**Target Platform**: Web (GitHub Pages frontend, Render backend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <10s login, <3min registration + questionnaire
**Constraints**: Must integrate with existing RAG pipeline
**Scale/Scope**: Single-tenant, 1000+ users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|------|
| Technical Accuracy | PASS | Using established libraries (Better Auth, PyJWT) |
| Educational Clarity | PASS | Personalization enhances learning experience |
| Practical Applicability | PASS | Direct value to users |
| Quality Assurance | PASS | CI/CD will be maintained |

## Project Structure

### Documentation (this feature)

```text
specs/003-personalized-rag-auth/
``` plan.md              # This file
``` research.md          # Technology decisions
``` data-model.md        # Database schema
``` quickstart.md        # Developer setup
``` contracts/           # OpenAPI spec
```

### Source Code

```text
backend/
``` main.py              # FastAPI app (existing)
``` config.py            # Settings (existing)
``` models/
    ``` chat.py          # Existing
    ``` user.py          # NEW: User/Profile models
``` routers/
    ``` chat.py          # Existing (modify for auth)
    ``` auth.py          # NEW: Auth endpoints
    ``` user.py          # NEW: Profile endpoints
``` services/
    ``` postgres.py      # Existing (extend)
    ``` user_service.py  # NEW: User CRUD
    ``` auth_service.py  # NEW: JWT verification
``` personalization/
    ``` rag_personalizer.py  # NEW: Profile-based prompts
``` scripts/
    ``` migrations/      # SQL migrations

ai-robotics-book/src/
``` components/
    ``` ChatWidget/      # Existing (modify)
    ``` AuthProvider/    # NEW: Better Auth context
    ``` LoginForm/       # NEW
    ``` SignupForm/      # NEW
    ``` ProfileSettings/ # NEW
``` hooks/
    ``` useAuth.js       # NEW
    ``` useProfile.js    # NEW
```

**Structure Decision**: Web application structure with separate frontend/backend. Extends existing codebase with new auth/user modules.

## Implementation Phases

### Phase 1: Better Auth Setup (Frontend)
- Install better-auth in frontend
- Create AuthProvider context
- Build signup page with background form (7 questions)
- Build login page
- Set up session storage
- Test auth flow locally

### Phase 2: Backend Auth Integration
- Install PyJWT in FastAPI - Create auth/better_auth.py (verify tokens)
- Create auth middleware
- Build POST /api/auth/signup (save profile to Neon)
- Build POST /api/auth/login (verify + return session)
- Create user_profiles table in Neon

### Phase 3: User Profile Management
- Build services/user_service.py (CRUD operations)
- Create GET /api/user/profile (fetch user background)
- Create PUT /api/user/profile (update profile)
- Test profile storage/retrieval

### Phase 4: Personalized RAG
- Build personalization/rag_personalizer.py
- Fetch user profile on chat request
- Generate personalized system prompt

## References

- [research.md](./research.md) - Technology decisions
- [data-model.md](./data-model.md) - Database schema
- [contracts/openapi.yaml](./contracts/openapi.yaml) - API spec
- [quickstart.md](./quickstart.md) - Setup guide