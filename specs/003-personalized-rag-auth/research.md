# Research: Personalized RAG with User Authentication

**Feature Branch**: **Date**: 2025-12-19
**Spec**: [spec.md](./spec.md)

## Research Questions Addressed

### 1. Better Auth Integration Architecture

**Decision**: Use Better Auth on frontend (JavaScript) with JWT token verification on FastAPI backend

**Rationale**:
- Better Auth is framework-agnostic and works with React/Docusaurus
- Built-in email/password authentication out of the box
- Session management with configurable duration (7 days as per spec)
- JWT plugin available for cross-origin token verification
- No per-user costs - data stays in our database

**Alternatives Considered**:
- **Auth.js/NextAuth**: Now merging with Better Auth, recommends new projects use Better Auth
- **Auth0**: Per-user costs, external dependency, overkill for this use case
- **Custom JWT**: More work, less tested, security risks

**Integration Pattern**:
1. Better Auth server runs within the Node.js frontend (or separate auth service)
2. On successful auth, JWT token issued
3. FastAPI backend verifies JWT via JWKS endpoint or shared secret
4. User profile stored in Neon PostgreSQL

### 2. Backend Token Verification Strategy

**Decision**: Use PyJWT library with JWKS endpoint verification

**Rationale**:
- FastAPI has built-in security utilities (HTTPBearer)
- PyJWT is well-maintained and focused API for JWT operations
- JWKS endpoint allows key rotation without backend changes
- Dependency injection pattern keeps code clean

**Implementation Pattern**:
### 3. Database Schema Design

**Decision**: Extend existing Neon PostgreSQL with users and user_profiles tables

**Rationale**:
- Already using Neon for chat_sessions and query_logs
- Keeps all data in one place
- psycopg2-binary already in requirements
- UUID primary keys for consistency

**Schema Extensions**:
- \ table: id, email, password_hash, created_at, updated_at
- \ table: user_id (FK), expertise_level, interests (array), learning_goals, preferred_format, time_availability, challenges, device_preference
- \ table: managed by Better Auth (can use database adapter)

### 4. Personalization Strategy

**Decision**: Build personalized system prompt from user profile

**Rationale**:
- Minimal changes to existing RAG pipeline
- Profile context injected into system prompt
- GPT model naturally adapts response style
- Easy to iterate and tune prompts

**Prompt Template Structure**:
### 5. Frontend Architecture

**Decision**: Add auth components to existing Docusaurus React structure

**Rationale**:
- Docusaurus uses React, Better Auth has React support
- Can wrap existing ChatWidget with AuthProvider
- Session state managed via React context
- sessionStorage already used for chat history

**Component Structure**:
## Technology Stack Summary

| Layer | Technology | Version | Purpose |
|-------|------------|---------|---------|
| Frontend Auth | Better Auth | ^1.4.x | Authentication framework |
| Frontend | React/Docusaurus | 3.9.2 | Existing UI framework |
| Backend | FastAPI | ^0.109.0 | Existing API server |
| Backend Auth | PyJWT | ^2.8.x | JWT verification |
| Database | Neon PostgreSQL | - | User data, profiles |
| Vectors | Qdrant Cloud | - | Existing RAG vectors |

## Security Considerations

1. **Password Storage**: Handled by Better Auth (bcrypt/argon2)
2. **Token Security**: 
   - Short-lived JWTs (15-60 min)
   - Refresh tokens for session continuity
   - HttpOnly cookies where possible
3. **CORS**: Already configured in FastAPI for frontend domains
4. **Rate Limiting**: Better Auth has built-in rate limiting

## Sources

- [Better Auth Official Site](https://www.better-auth.com/)
- [Better Auth API Documentation](https://www.better-auth.com/docs/concepts/api)
- [Better Auth Session Management](https://www.better-auth.com/docs/concepts/session-management)
- [FastAPI JWT Authentication Guide](https://testdriven.io/blog/fastapi-jwt-auth/)
- [Neon FastAPI JWT Guide](https://neon.com/guides/fastapi-jwt)
