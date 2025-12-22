# Data Model: Personalized RAG with User Authentication

**Feature Branch**: `003-personalized-rag-auth`
**Date**: 2025-12-19
**Spec**: [spec.md](./spec.md)

## Entity Relationship Diagram

```
+------------------+       +-----------------------+
|      users       |       |     user_profiles     |
+------------------+       +-----------------------+
| id (PK, UUID)    |--+-->| user_id (PK, FK)      |
| email (UNIQUE)   |  |    | expertise_level       |
| password_hash    |  |    | interests             |
| email_verified   |  |    | learning_goals        |
| created_at       |  |    | preferred_format      |
| updated_at       |  |    | time_availability     |
+------------------+  |    | challenges            |
                      |    | device_preference     |
                      |    | created_at            |
                      |    | updated_at            |
                      |    +-----------------------+
                      |
                      |    +-----------------------+
                      |    |    chat_sessions      |
                      +--->|     (existing)        |
                           +-----------------------+
                           | id (PK, UUID)         |
                           | user_id (FK, nullable)|  <-- NEW COLUMN
                           | created_at            |
                           | updated_at            |
                           +-----------------------+
```

## Entity Definitions

### users

Core user identity and authentication credentials.

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PK, DEFAULT gen_random_uuid() | Unique user identifier |
| email | VARCHAR(255) | UNIQUE, NOT NULL | User email (login identifier) |
| password_hash | VARCHAR(255) | NOT NULL | Bcrypt/Argon2 hashed password |
| email_verified | BOOLEAN | DEFAULT false | Email verification status |
| created_at | TIMESTAMPTZ | DEFAULT NOW() | Account creation time |
| updated_at | TIMESTAMPTZ | DEFAULT NOW() | Last update time |

**Indexes**:
- `idx_users_email` on `email` (for login lookups)

**Validation Rules**:
- Email must be valid format (checked at application layer)
- Password minimum 8 characters, mixed case, number (checked before hashing)

### user_profiles

Background questionnaire responses for personalization.

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| user_id | UUID | PK, FK -> users.id | One-to-one with users |
| expertise_level | VARCHAR(20) | NOT NULL | beginner/intermediate/advanced/expert |
| interests | TEXT[] | NOT NULL, DEFAULT ARRAY[] | Array of book topic interests |
| learning_goals | VARCHAR(50) | NOT NULL | casual/deep_understanding/practical/research |
| preferred_format | VARCHAR(20) | NOT NULL | text/diagrams/videos/code/interactive |
| time_availability | VARCHAR(20) | NOT NULL | quick_5min/medium_15min/deep_30min |
| challenges | TEXT | NULL | Free-text current challenges |
| device_preference | VARCHAR(20) | NOT NULL | desktop/tablet/mobile |
| created_at | TIMESTAMPTZ | DEFAULT NOW() | Profile creation time |
| updated_at | TIMESTAMPTZ | DEFAULT NOW() | Last profile update |

**Validation Rules**:
- expertise_level: ENUM (beginner, intermediate, advanced, expert)
- learning_goals: ENUM (casual, deep_understanding, practical, research)
- preferred_format: ENUM (text, diagrams, videos, code, interactive)
- time_availability: ENUM (quick_5min, medium_15min, deep_30min)
- device_preference: ENUM (desktop, tablet, mobile)

### sessions (Better Auth Managed)

Better Auth manages session storage with database adapter.

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PK | Session identifier |
| user_id | UUID | FK -> users.id | Session owner |
| token | VARCHAR(255) | UNIQUE, NOT NULL | Session token |
| expires_at | TIMESTAMPTZ | NOT NULL | Session expiry (7 days default) |
| created_at | TIMESTAMPTZ | DEFAULT NOW() | Session creation time |

### chat_sessions (Existing - Modified)

Add optional user_id to link authenticated sessions.

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| user_id | UUID | FK -> users.id, NULL | Link to authenticated user (NEW) |

## SQL Migration Scripts

### 001_create_users_table.sql

```sql
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    email_verified BOOLEAN DEFAULT false,
    created_at TIMESTAMPTZ DEFAULT NOW(),
    updated_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_users_email ON users(email);
```

### 002_create_user_profiles_table.sql

```sql
CREATE TABLE IF NOT EXISTS user_profiles (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    expertise_level VARCHAR(20) NOT NULL,
    interests TEXT[] NOT NULL DEFAULT ARRAY[]::text[],
    learning_goals VARCHAR(50) NOT NULL,
    preferred_format VARCHAR(20) NOT NULL,
    time_availability VARCHAR(20) NOT NULL,
    challenges TEXT,
    device_preference VARCHAR(20) NOT NULL,
    created_at TIMESTAMPTZ DEFAULT NOW(),
    updated_at TIMESTAMPTZ DEFAULT NOW()
);
```

### 003_add_user_id_to_chat_sessions.sql

```sql
ALTER TABLE chat_sessions 
ADD COLUMN IF NOT EXISTS user_id UUID REFERENCES users(id) ON DELETE SET NULL;

CREATE INDEX idx_chat_sessions_user_id ON chat_sessions(user_id);
```

## State Transitions

### User Registration Flow

```
[Anonymous] --> [User Created] --> [Profile Created] --> [Active User]
```

### Session Lifecycle

```
[No Session] --> [Active Session] --> [Logout/Expire] --> [Re-login Required]
```
