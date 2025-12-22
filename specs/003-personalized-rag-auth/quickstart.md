# Quickstart: Personalized RAG with User Authentication

**Feature Branch**: `003-personalized-rag-auth`
**Date**: 2025-12-19

## Prerequisites

- Node.js 18+
- Python 3.11+
- Access to Neon PostgreSQL database
- Existing RAG chatbot backend running

## Quick Setup

### 1. Frontend Setup (Better Auth)

```bash
# In ai-robotics-book/ directory
npm install better-auth @better-auth/react
```

### 2. Backend Setup (PyJWT)

```bash
# In backend/ directory
pip install PyJWT>=2.8.0
```

### 3. Database Migrations

Run the SQL migrations from `data-model.md`:

```bash
# Connect to Neon and run:
psql $DATABASE_URL -f scripts/001_create_users_table.sql
psql $DATABASE_URL -f scripts/002_create_user_profiles_table.sql
psql $DATABASE_URL -f scripts/003_add_user_id_to_chat_sessions.sql
```

### 4. Environment Variables

Add to `backend/.env`:

```env
# Existing
DATABASE_URL=postgresql://...

# New for auth
BETTER_AUTH_SECRET=your-secret-key
JWT_PUBLIC_KEY=your-public-key
```

## Development Workflow

1. Start backend: `uvicorn main:app --reload`
2. Start frontend: `npm start`
3. Test signup at `http://localhost:3000`

## Testing

```bash
# Test signup
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"Test1234!","name":"Test"}'

# Test login
curl -X POST http://localhost:8000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"Test1234!"}'
```

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Implement Phase 1 (Auth) first
3. Test locally before deploying
