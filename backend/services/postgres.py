"""PostgreSQL (Neon) database service for RAG Chatbot."""

import json
from contextlib import contextmanager
from typing import Optional

import psycopg2
from psycopg2.extras import RealDictCursor
from config import settings


class PostgresService:
    """Service for interacting with Neon PostgreSQL database."""

    def __init__(self):
        """Initialize PostgreSQL connection parameters."""
        self.connection_string = settings.DATABASE_URL

    @contextmanager
    def get_connection(self):
        """Context manager for database connections.

        Yields:
            psycopg2 connection object.
        """
        # Add connection timeout for serverless cold starts
        conn = psycopg2.connect(
            self.connection_string,
            connect_timeout=30,
        )
        try:
            yield conn
        finally:
            conn.close()

    def health_check(self) -> dict:
        """Check PostgreSQL connection health.

        Returns:
            dict: Health status with connected status and latency.
        """
        try:
            import time

            start = time.time()
            with self.get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("SELECT 1")
            latency_ms = int((time.time() - start) * 1000)
            return {"status": "connected", "latency_ms": latency_ms}
        except Exception as e:
            return {"status": "error", "error": str(e)}

    def create_session(self) -> str:
        """Create a new chat session.

        Returns:
            str: The UUID of the created session.
        """
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    "INSERT INTO chat_sessions DEFAULT VALUES RETURNING id"
                )
                session_id = cur.fetchone()[0]
                conn.commit()
                return str(session_id)

    def get_or_create_session(self, session_id: str) -> str:
        """Get existing session or create new one.

        Args:
            session_id: The session ID to look up or create.

        Returns:
            str: The session ID.
        """
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                # Check if session exists
                cur.execute(
                    "SELECT id FROM chat_sessions WHERE id = %s",
                    (session_id,)
                )
                result = cur.fetchone()

                if result:
                    # Update the session's updated_at timestamp
                    cur.execute(
                        "UPDATE chat_sessions SET updated_at = NOW() WHERE id = %s",
                        (session_id,)
                    )
                    conn.commit()
                    return session_id
                else:
                    # Create new session with specified ID
                    cur.execute(
                        "INSERT INTO chat_sessions (id) VALUES (%s) RETURNING id",
                        (session_id,)
                    )
                    new_id = cur.fetchone()[0]
                    conn.commit()
                    return str(new_id)

    def run_migration(self, migration_path: str) -> bool:
        """Run a SQL migration file.

        Args:
            migration_path: Path to the SQL migration file.

        Returns:
            bool: True if migration succeeded.
        """
        with open(migration_path, "r") as f:
            sql = f.read()

        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(sql)
                conn.commit()
        return True

    def log_query(
        self,
        session_id: str,
        query_text: str,
        response_text: str,
        citations: list[dict],
        response_time_ms: int,
        query_type: str = "general",
        selected_text: Optional[str] = None,
    ) -> Optional[str]:
        """Log a query to the query_logs table.

        Args:
            session_id: The chat session ID.
            query_text: The user's query.
            response_text: The assistant's response.
            citations: List of citations.
            response_time_ms: Response time in milliseconds.
            query_type: Type of query ('general' or 'selection').
            selected_text: Optional selected text for selection queries.

        Returns:
            str: The query log ID, or None if logging failed.
        """
        try:
            with self.get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        INSERT INTO query_logs
                        (session_id, query_text, response_text, citations,
                         response_time_ms, query_type, selected_text)
                        VALUES (%s, %s, %s, %s, %s, %s, %s)
                        RETURNING id
                        """,
                        (
                            session_id,
                            query_text,
                            response_text,
                            json.dumps(citations),
                            response_time_ms,
                            query_type,
                            selected_text,
                        ),
                    )
                    log_id = cur.fetchone()[0]
                    conn.commit()
                    return str(log_id)
        except Exception as e:
            # Log failure should not break the main flow
            print(f"Warning: Failed to log query: {e}")
            return None


# Singleton instance
postgres_service = PostgresService()
