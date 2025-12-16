"""Qdrant vector database service for RAG Chatbot."""

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from config import settings


class QdrantService:
    """Service for interacting with Qdrant vector database."""

    def __init__(self):
        """Initialize Qdrant client."""
        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=120,  # Longer timeout for cloud operations
        )
        self.collection_name = settings.QDRANT_COLLECTION

    def create_collection_if_not_exists(self) -> bool:
        """Create the collection if it doesn't exist.

        Returns:
            bool: True if collection was created, False if it already exists.
        """
        collections = self.client.get_collections().collections
        collection_names = [c.name for c in collections]

        if self.collection_name not in collection_names:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=settings.EMBEDDING_DIMENSIONS,
                    distance=Distance.COSINE,
                ),
            )
            return True
        return False

    def health_check(self) -> dict:
        """Check Qdrant connection health.

        Returns:
            dict: Health status with connected status and latency.
        """
        try:
            import time

            start = time.time()
            self.client.get_collections()
            latency_ms = int((time.time() - start) * 1000)
            return {"status": "connected", "latency_ms": latency_ms}
        except Exception as e:
            return {"status": "error", "error": str(e)}

    def upsert_chunks(self, chunks: list[dict]) -> int:
        """Insert or update content chunks in Qdrant.

        Args:
            chunks: List of dicts with 'id', 'vector', and 'payload' keys.

        Returns:
            int: Number of chunks upserted.
        """
        points = [
            PointStruct(
                id=chunk["id"],
                vector=chunk["vector"],
                payload=chunk["payload"],
            )
            for chunk in chunks
        ]

        self.client.upsert(
            collection_name=self.collection_name,
            points=points,
        )

        return len(points)

    def get_collection_info(self) -> dict:
        """Get information about the collection.

        Returns:
            dict: Collection info including point count.
        """
        try:
            info = self.client.get_collection(self.collection_name)
            return {
                "name": self.collection_name,
                "points_count": info.points_count,
                "vectors_count": info.vectors_count,
            }
        except Exception as e:
            return {"error": str(e)}

    def semantic_search(
        self,
        query_vector: list[float],
        top_k: int = 5,
    ) -> list[dict]:
        """Search for similar chunks using a query vector.

        Args:
            query_vector: The embedding vector for the query.
            top_k: Number of results to return.

        Returns:
            list[dict]: List of search results with id, score, and payload.
        """
        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_vector,
            limit=top_k,
            with_payload=True,
        )

        return [
            {
                "id": str(hit.id),
                "score": hit.score,
                "payload": hit.payload,
            }
            for hit in results.points
        ]


# Singleton instance
qdrant_service = QdrantService()
