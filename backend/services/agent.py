"""RAG Agent service using OpenAI for the Physical AI book chatbot."""

import uuid
from typing import Optional

from openai import OpenAI

from config import settings
from services.embeddings import embeddings_service
from services.qdrant import qdrant_service


# System prompt for the RAG agent
SYSTEM_PROMPT = """You are an expert assistant for the "Physical AI & Humanoid Robotics" educational book. Your role is to help readers understand robotics concepts, ROS 2, and related technologies.

Guidelines:
1. Base your answers ONLY on the provided context from the book. Do not make up information.
2. If the context doesn't contain enough information to fully answer the question, acknowledge this and provide what you can from the available context.
3. Use clear, educational language appropriate for technical learners.
4. When explaining code or technical concepts, be precise and accurate.
5. Reference specific sections when possible (e.g., "As explained in Module 1...").
6. Format your responses using markdown for better readability.
7. Keep responses focused and concise while being comprehensive.

Remember: You are helping students learn about robotics and ROS 2. Be encouraging and educational in your tone."""


class RAGAgent:
    """RAG Agent for answering questions about the Physical AI book."""

    def __init__(self):
        """Initialize the RAG agent with OpenAI client."""
        self.client = OpenAI(api_key=settings.OPENAI_API_KEY)
        self.model = settings.CHAT_MODEL
        self.temperature = settings.CHAT_TEMPERATURE
        self.top_k = settings.TOP_K_RESULTS

    def search_book_content(self, query: str) -> list[dict]:
        """Search the book content for relevant chunks.

        Args:
            query: The user's question or search query.

        Returns:
            list[dict]: List of relevant chunks with metadata.
        """
        # Embed the query
        query_vector = embeddings_service.embed_text(query)

        # Search Qdrant
        results = qdrant_service.semantic_search(
            query_vector=query_vector,
            top_k=self.top_k,
        )

        return results

    def _format_context(self, search_results: list[dict]) -> str:
        """Format search results into context for the LLM.

        Args:
            search_results: List of search results from Qdrant.

        Returns:
            str: Formatted context string.
        """
        if not search_results:
            return "No relevant content found in the book."

        context_parts = []
        for i, result in enumerate(search_results, 1):
            payload = result.get("payload", {})
            module = payload.get("module", "Unknown Module")
            chapter = payload.get("chapter", "Unknown Chapter")
            section = payload.get("section", "")
            text = payload.get("text", "")

            location = f"{module} > {chapter}"
            if section:
                location += f" > {section}"

            context_parts.append(
                f"[Source {i}] ({location})\n{text}"
            )

        return "\n\n---\n\n".join(context_parts)

    def _extract_citations(self, search_results: list[dict]) -> list[dict]:
        """Extract citations from search results.

        Args:
            search_results: List of search results from Qdrant.

        Returns:
            list[dict]: List of citation dictionaries.
        """
        citations = []
        seen = set()  # Avoid duplicate citations

        for result in search_results:
            payload = result.get("payload", {})
            module = payload.get("module", "")
            chapter = payload.get("chapter", "")
            section = payload.get("section", "")

            # Create a unique key for deduplication
            key = f"{module}|{chapter}|{section}"
            if key in seen:
                continue
            seen.add(key)

            citations.append({
                "module": module or "General",
                "chapter": chapter or "Introduction",
                "section": section,
                "relevance_score": result.get("score", 0),
            })

        return citations

    def generate_response(
        self,
        query: str,
        selected_text: Optional[str] = None,
    ) -> dict:
        """Generate a response to the user's query using RAG.

        Args:
            query: The user's question.
            selected_text: Optional selected text for context.

        Returns:
            dict: Response with content, citations, and message_id.
        """
        # Search for relevant content
        search_results = self.search_book_content(query)

        # Format context for the LLM
        context = self._format_context(search_results)

        # Build the user message
        if selected_text:
            user_message = f"""The user has selected the following text from the book:

<selected_text>
{selected_text}
</selected_text>

Their question: {query}

Here is relevant context from the book to help answer:

{context}

Please answer the question, taking into account both the selected text and the additional context."""
        else:
            user_message = f"""User question: {query}

Here is relevant context from the book:

{context}

Please provide a helpful answer based on the book content."""

        # Call OpenAI
        response = self.client.chat.completions.create(
            model=self.model,
            temperature=self.temperature,
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": user_message},
            ],
        )

        # Extract the response content
        content = response.choices[0].message.content

        # Extract citations
        citations = self._extract_citations(search_results)

        return {
            "message_id": str(uuid.uuid4()),
            "content": content,
            "citations": citations,
        }


# Singleton instance
rag_agent = RAGAgent()
