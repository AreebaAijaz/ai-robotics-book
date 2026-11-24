---
id: 001
title: Fix auth navbar, email case, chatbot errors
stage: general
date: 2026-11-24
surface: agent
model: claude-opus-5-5
feature: none
branch: 001-physical-ai-book
user: AreebaAijaz
command: none
labels: ["auth", "login", "signup", "chatbot", "qdrant", "debugging"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - ai-robotics-book/src/components/NavbarLoginButton/index.js
 - ai-robotics-book/src/components/AuthModal/index.js
 - backend/services/user_service.py
 - backend/routers/chat.py
tests:
 - Manual E2E in Chrome against localhost: signup, logout, login (exact + mixed-case email), navbar persistence across navigation, modal reopens on login form
 - API: case-variant duplicate signup returns 409
 - Chat with Qdrant down returns 503 with readable message
---

## Prompt

I just found out that the login/signup isn't working as expected. try using it in chrome first, identify what's going wrong then fix it in codebase and push everything when's working. The chatbot is also not working, you can properly examine it once login/signup issue fix.

## Response snapshot

Reproduced in Chrome (local frontend + backend, throwaway test account). Found and fixed:
1. Navbar Login/Sign Out button disappeared after any client-side navigation (portal bound to a detached navbar node) -> MutationObserver re-locates the placeholder.
2. Auth modal reopened on whichever form was last shown and kept stale errors -> resets to login and clears error on open/switch.
3. Email lookup was case-sensitive ("User@x.com" could not log in) -> normalize to lowercase, match on LOWER(email).
Chatbot: Qdrant Cloud cluster resets every TLS connection (from Render and locally) - cluster appears suspended/deleted; infra action needed. Chat endpoints now return 503 "knowledge base temporarily unavailable" instead of raw socket errors.

## Outcome

- ✅ Impact: Auth flows work across all pages; chatbot failure is now clearly reported; root cause (Qdrant cluster) identified.
- 🧪 Tests: Manual E2E in Chrome + curl checks, all passing.
- 📁 Files: 4 files modified.
- 🔁 Next prompts: Resume/recreate Qdrant cluster, update QDRANT_URL/QDRANT_API_KEY on Render, re-run scripts/ingest_docs.py.
- 🧠 Reflection: Health endpoint already exposed the Qdrant error; checking it first shortened chatbot diagnosis.

## Evaluation notes (flywheel)

- Failure modes observed: Portal target replaced on route change; case-sensitive identity lookup; leaking low-level exceptions to UI.
- Graders run and results (PASS/FAIL): Manual E2E PASS.
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Add automated tests for auth email normalization.
