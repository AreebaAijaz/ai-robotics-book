-- Migration: 003_add_user_id_to_chat_sessions
-- Description: Add user_id column to chat_sessions for linking authenticated sessions
-- Date: 2025-12-19

-- Add user_id column if it doesn't exist
DO $$
BEGIN
    IF NOT EXISTS (
        SELECT 1 FROM information_schema.columns
        WHERE table_name = 'chat_sessions' AND column_name = 'user_id'
    ) THEN
        ALTER TABLE chat_sessions
        ADD COLUMN user_id UUID REFERENCES users(id) ON DELETE SET NULL;
    END IF;
END $$;

-- Create index for user_id lookups
CREATE INDEX IF NOT EXISTS idx_chat_sessions_user_id ON chat_sessions(user_id);
