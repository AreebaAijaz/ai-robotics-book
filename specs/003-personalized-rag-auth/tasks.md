# Tasks: Personalized RAG with User Authentication

**Input**: Design documents from `/specs/003-personalized-rag-auth/`
**Prerequisites**: plan.md ✓, spec.md ✓, research.md ✓, data-model.md ✓, contracts/openapi.yaml ✓

**Organization**: Tasks are grouped by implementation phase as specified by user, with user story mappings.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US5 from spec.md)
- Include exact file paths in descriptions

---

## Phase 1: Better Auth Setup (Frontend)

**Purpose**: Install and configure Better Auth in the Docusaurus frontend

**Maps to**: US1 (Registration), US2 (Login)

### 1.1 Auth Provider Setup

- [x] T001 [US1] Install better-auth package in `ai-robotics-book/package.json`
- [x] T002 [US1] Create AuthProvider context in `ai-robotics-book/src/components/AuthProvider/index.js`
- [x] T003 [US1] Create useAuth hook in `ai-robotics-book/src/hooks/useAuth.js`
- [x] T004 [US1] Wrap app with AuthProvider in `ai-robotics-book/src/theme/Root.js`

### 1.2 Signup Page with Questionnaire

- [x] T005 [US1] Create SignupForm component in `ai-robotics-book/src/components/SignupForm/index.js`
- [x] T006 [P] [US1] Create SignupForm styles in `ai-robotics-book/src/components/SignupForm/styles.module.css`
- [x] T007 [US1] Build 7-question questionnaire form within SignupForm (expertise, interests, goals, format, time, challenges, device)
- [x] T008 [US1] Add form validation for email/password and questionnaire fields
- [x] T009 [US1] Connect SignupForm to Better Auth signup API

### 1.3 Login Page

- [x] T010 [US2] Create LoginForm component in `ai-robotics-book/src/components/LoginForm/index.js`
- [x] T011 [P] [US2] Create LoginForm styles in `ai-robotics-book/src/components/LoginForm/styles.module.css`
- [x] T012 [US2] Add email/password validation and error display
- [x] T013 [US2] Connect LoginForm to Better Auth login API
- [x] T014 [US2] Implement session persistence with Better Auth

### 1.4 Auth Flow Testing

- [ ] T015 [US1] Test signup flow: create account → questionnaire → access granted
- [ ] T016 [US2] Test login flow: enter credentials → session created → chatbot access
- [ ] T017 [US1] [US2] Verify session persists across browser refresh

**CHECKPOINT 1**: Auth working locally - signup creates account, login creates session, session persists ✓

---

## Phase 2: Backend Auth Integration

**Purpose**: Add JWT verification and auth endpoints to FastAPI backend

**Maps to**: US1 (Registration), US2 (Login), US5 (Password Recovery)

### 2.1 Auth Dependencies and Middleware

- [x] T018 Add PyJWT and python-jose to `backend/requirements.txt`
- [x] T019 Create auth service in `backend/services/auth_service.py` for JWT verification
- [x] T020 Create auth middleware in `backend/middleware/auth.py` for token extraction
- [x] T021 Add auth configuration (JWKS URL, secret) to `backend/config.py`

### 2.2 Database Setup

- [x] T022 [US1] Create users table migration in `backend/scripts/migrations/001_create_users_table.sql`
- [x] T023 [US1] Create user_profiles table migration in `backend/scripts/migrations/002_create_user_profiles_table.sql`
- [x] T024 Create migration for chat_sessions user_id column in `backend/scripts/migrations/003_add_user_id_to_chat_sessions.sql`
- [ ] T025 Run migrations against Neon PostgreSQL database
- [x] T026 [US1] Create User model in `backend/models/user.py`
- [x] T027 [P] [US1] Create UserProfile model in `backend/models/user.py`

### 2.3 Auth Endpoints

- [x] T028 [US1] Create auth router in `backend/routers/auth.py`
- [x] T029 [US1] Implement POST `/api/auth/signup` endpoint (creates user + profile)
- [x] T030 [US2] Implement POST `/api/auth/login` endpoint (verify credentials, return token)
- [x] T031 [US2] Implement POST `/api/auth/logout` endpoint (invalidate session)
- [x] T032 [US5] Implement POST `/api/auth/forgot-password` endpoint
- [x] T033 [US5] Implement POST `/api/auth/reset-password` endpoint
- [x] T034 Register auth router in `backend/main.py`

**CHECKPOINT 2**: Backend auth endpoints functional - signup saves user/profile, login returns token, token verification works ✓

---

## Phase 3: User Profile Management

**Purpose**: Build profile CRUD operations and endpoints

**Maps to**: US4 (Profile Management)

### 3.1 Profile Service

- [x] T035 [US4] Create user service in `backend/services/user_service.py`
- [x] T036 [US4] Implement get_user_by_id function
- [x] T037 [US4] Implement get_profile function
- [x] T038 [US4] Implement update_profile function
- [x] T039 [US4] Implement delete_user function (cascades to profile)

### 3.2 Profile Endpoints

- [x] T040 [US4] Create user router in `backend/routers/user.py`
- [x] T041 [US4] Implement GET `/api/user/me` endpoint (returns user info)
- [x] T042 [US4] Implement GET `/api/user/profile` endpoint (returns questionnaire answers)
- [x] T043 [US4] Implement PUT `/api/user/profile` endpoint (updates profile)
- [x] T044 [US4] Implement DELETE `/api/user/me` endpoint (account deletion)
- [x] T045 Register user router in `backend/main.py`

### 3.3 Frontend Profile Settings

- [x] T046 [US4] Create ProfileSettings component in `ai-robotics-book/src/components/ProfileSettings/index.js`
- [x] T047 [P] [US4] Create ProfileSettings styles in `ai-robotics-book/src/components/ProfileSettings/styles.module.css`
- [x] T048 [US4] Create useProfile hook in `ai-robotics-book/src/hooks/useProfile.js`
- [x] T049 [US4] Fetch and display current profile in ProfileSettings
- [x] T050 [US4] Enable profile editing with save functionality
- [ ] T051 [US4] Test profile update flow: edit → save → verify changes persist

**CHECKPOINT 3**: Profile management complete - users can view and update their questionnaire answers ✓

---

## Phase 4: Personalized RAG

**Purpose**: Inject user profile context into RAG system prompts

**Maps to**: US3 (Personalized Responses)

### 4.1 RAG Personalizer

- [x] T052 [US3] Create rag_personalizer module in `backend/personalization/rag_personalizer.py`
- [x] T053 [US3] Implement get_user_context function (fetches profile, builds context)
- [x] T054 [US3] Implement generate_personalized_prompt function (injects context into system prompt)
- [x] T055 [US3] Create persona templates for expertise levels (beginner, intermediate, advanced, expert)

### 4.2 Chat Integration

- [x] T056 [US3] Modify chat router in `backend/routers/chat.py` to support authentication
- [x] T057 [US3] Fetch user profile on chat request in `backend/routers/chat.py`
- [x] T058 [US3] Inject personalized system prompt into RAG pipeline
- [x] T059 [US3] Update ChatWidget to pass auth token in `ai-robotics-book/src/components/ChatWidget/index.js`
- [x] T060 [US3] Add login/signup prompt when unauthenticated user clicks chat

### 4.3 Personalization Testing

- [ ] T061 [US3] Test beginner user gets simplified explanations
- [ ] T062 [US3] Test expert user gets technical details
- [ ] T063 [US3] Test time preference affects response length
- [ ] T064 [US3] Test format preference prioritizes appropriate content types

**CHECKPOINT 4**: Personalized RAG complete - chatbot responses adapt to user profile ✓

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Final integration, error handling, and cleanup

- [ ] T065 [P] Add comprehensive error handling to all auth endpoints
- [ ] T066 [P] Add rate limiting to auth endpoints (prevent brute force)
- [ ] T067 Add CORS configuration for auth endpoints in `backend/main.py`
- [ ] T068 [P] Update README with auth setup instructions
- [ ] T069 Update `specs/003-personalized-rag-auth/quickstart.md` with environment variables
- [ ] T070 Validate full user journey: signup → questionnaire → personalized chat → profile update

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Frontend Auth)**: No dependencies - can start immediately
- **Phase 2 (Backend Auth)**: Can run in parallel with Phase 1 after T001-T004
- **Phase 3 (Profile Management)**: Depends on Phase 2 completion (needs auth endpoints + database)
- **Phase 4 (Personalized RAG)**: Depends on Phase 3 completion (needs profile data)
- **Phase 5 (Polish)**: Depends on all phases complete

### User Story Dependencies

| User Story | Priority | Phase | Can Start After |
|------------|----------|-------|-----------------|
| US1 (Registration) | P1 | 1, 2 | Immediately |
| US2 (Login) | P1 | 1, 2 | Immediately |
| US5 (Password Recovery) | P3 | 2 | T028 (auth router created) |
| US4 (Profile Management) | P3 | 3 | Phase 2 complete |
| US3 (Personalization) | P2 | 4 | Phase 3 complete |

### Parallel Opportunities

```text
# Phase 1 parallelism:
T006 (SignupForm styles) || T011 (LoginForm styles)
T005-T009 (Signup) || T010-T014 (Login) after T001-T004

# Phase 2 parallelism:
T022, T023, T024 (migrations) can run in parallel
T026 (User model) || T027 (UserProfile model)

# Phase 3 parallelism:
T046 (ProfileSettings) || T047 (styles)

# Phase 5 parallelism:
T065 || T066 || T068 (independent polish tasks)
```

---

## Implementation Strategy

### MVP First (US1 + US2)

1. Complete Phase 1: Frontend Auth → Users can signup/login
2. Complete Phase 2: Backend Auth → Full authentication working
3. **VALIDATE**: Test signup → login → session persistence
4. Deploy to staging for user testing

### Full Feature (Add US3, US4, US5)

5. Complete Phase 3: Profile Management → Users can update profiles
6. Complete Phase 4: Personalized RAG → Core value proposition delivered
7. Complete Phase 5: Polish → Production-ready
8. Deploy to production

---

## Notes

- All auth endpoints use JWT tokens verified via JWKS endpoint pattern
- User profiles stored in Neon PostgreSQL with users table
- Personalization uses profile data to customize system prompts
- Session duration: 7 days (as per spec.md assumptions)
- Password reset links expire after 1 hour
