"""Environment configuration loader for RAG Chatbot backend."""

import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()


class Settings:
    """Application settings loaded from environment variables."""

    # OpenAI
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")

    # Qdrant
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_COLLECTION: str = "physical_ai_book"

    # Postgres (Neon)
    DATABASE_URL: str = os.getenv("DATABASE_URL", "")

    # CORS - supports multiple origins separated by comma
    CORS_ORIGINS: list[str] = [
        origin.strip() for origin in
        os.getenv("CORS_ORIGINS", "http://localhost:3000,https://areebaaijaz.github.io").split(",")
        if origin.strip()
    ]

    # Embedding model
    EMBEDDING_MODEL: str = "text-embedding-3-small"
    EMBEDDING_DIMENSIONS: int = 1536

    # Chat model
    CHAT_MODEL: str = "gpt-4o-mini"
    CHAT_TEMPERATURE: float = 0.3
    CHAT_MAX_TOKENS: int = 1024

    # RAG settings
    TOP_K_RESULTS: int = 5
    CHUNK_SIZE: int = 800
    CHUNK_OVERLAP: int = 100

    # Authentication
    JWT_SECRET_KEY: str = os.getenv("JWT_SECRET_KEY", "your-secret-key-change-in-production")
    JWT_ALGORITHM: str = "HS256"
    JWT_EXPIRATION_DAYS: int = 7
    PASSWORD_RESET_EXPIRATION_HOURS: int = 1


settings = Settings()
