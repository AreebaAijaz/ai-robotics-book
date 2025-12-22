-- Migration: 002_create_user_profiles_table
-- Description: Create user_profiles table for questionnaire responses
-- Date: 2025-12-19

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

-- Trigger to update updated_at timestamp
DROP TRIGGER IF EXISTS update_user_profiles_updated_at ON user_profiles;
CREATE TRIGGER update_user_profiles_updated_at
    BEFORE UPDATE ON user_profiles
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();
