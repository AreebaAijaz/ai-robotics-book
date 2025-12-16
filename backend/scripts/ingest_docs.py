"""Data ingestion script for RAG Chatbot.

Reads markdown files from the book docs directory, chunks them,
generates embeddings, and stores them in Qdrant.
"""

import os
import re
import uuid
from pathlib import Path

# Add parent directory to path for imports
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))

from config import settings
from services.embeddings import embeddings_service
from services.qdrant import qdrant_service


# Path to book documentation
DOCS_PATH = Path(__file__).parent.parent.parent / "ai-robotics-book" / "docs"


def read_markdown_files(docs_path: Path) -> list[dict]:
    """Read all markdown files from the docs directory.

    Args:
        docs_path: Path to the docs directory.

    Returns:
        list[dict]: List of dicts with file_path, content, and metadata.
    """
    documents = []

    for md_file in docs_path.rglob("*.md"):
        relative_path = md_file.relative_to(docs_path)
        parts = relative_path.parts

        # Extract module from path (e.g., module-1-ros2)
        module = ""
        chapter = ""
        if len(parts) >= 1:
            if parts[0].startswith("module-"):
                module = parts[0]
                if len(parts) >= 2:
                    chapter = parts[1].replace(".md", "")
            else:
                # Top-level files like intro.md
                chapter = parts[0].replace(".md", "")

        with open(md_file, "r", encoding="utf-8") as f:
            content = f.read()

        documents.append({
            "file_path": str(relative_path),
            "content": content,
            "module": module,
            "chapter": chapter,
        })

    return documents


def extract_sections(content: str) -> list[dict]:
    """Extract sections from markdown content based on headers.

    Args:
        content: Markdown content.

    Returns:
        list[dict]: List of sections with title and content.
    """
    # Split by h1 and h2 headers
    sections = []
    current_section = {"title": "Introduction", "content": ""}

    lines = content.split("\n")
    for line in lines:
        # Check for h1 or h2 header
        if line.startswith("# ") or line.startswith("## "):
            # Save previous section if it has content
            if current_section["content"].strip():
                sections.append(current_section)

            # Start new section
            title = line.lstrip("#").strip()
            current_section = {"title": title, "content": ""}
        else:
            current_section["content"] += line + "\n"

    # Add last section
    if current_section["content"].strip():
        sections.append(current_section)

    return sections


def chunk_text(text: str, chunk_size: int = 800, overlap: int = 100) -> list[str]:
    """Split text into overlapping chunks by approximate token count.

    Args:
        text: Text to chunk.
        chunk_size: Target chunk size in tokens (approx 4 chars per token).
        overlap: Overlap between chunks in tokens.

    Returns:
        list[str]: List of text chunks.
    """
    # Approximate: 1 token ≈ 4 characters
    char_chunk_size = chunk_size * 4
    char_overlap = overlap * 4

    # Clean up the text
    text = text.strip()
    if not text:
        return []

    chunks = []
    start = 0

    while start < len(text):
        end = start + char_chunk_size

        # If not at the end, try to break at a sentence or paragraph
        if end < len(text):
            # Look for a good break point (paragraph or sentence end)
            break_point = text.rfind("\n\n", start, end)
            if break_point == -1 or break_point < start + char_chunk_size // 2:
                break_point = text.rfind(". ", start, end)
            if break_point == -1 or break_point < start + char_chunk_size // 2:
                break_point = text.rfind(" ", start, end)
            if break_point != -1 and break_point > start:
                end = break_point + 1

        chunk = text[start:end].strip()
        if chunk:
            chunks.append(chunk)

        # Move start with overlap
        start = end - char_overlap

    return chunks


def process_documents(documents: list[dict]) -> list[dict]:
    """Process documents into chunks with metadata.

    Args:
        documents: List of document dicts from read_markdown_files.

    Returns:
        list[dict]: List of chunk dicts ready for embedding.
    """
    all_chunks = []

    for doc in documents:
        sections = extract_sections(doc["content"])

        for section in sections:
            chunks = chunk_text(
                section["content"],
                chunk_size=settings.CHUNK_SIZE,
                overlap=settings.CHUNK_OVERLAP,
            )

            for i, chunk_text_content in enumerate(chunks):
                chunk_id = str(uuid.uuid4())

                # Create human-readable module/chapter names
                module_name = doc["module"].replace("-", " ").title() if doc["module"] else ""
                chapter_name = doc["chapter"].replace("-", " ").title() if doc["chapter"] else ""

                all_chunks.append({
                    "id": chunk_id,
                    "text": chunk_text_content,
                    "metadata": {
                        "module": module_name,
                        "chapter": chapter_name,
                        "section": section["title"],
                        "file_path": doc["file_path"],
                        "chunk_index": i,
                    },
                })

    return all_chunks


def ingest_to_qdrant(chunks: list[dict], batch_size: int = 10) -> int:
    """Generate embeddings and store chunks in Qdrant.

    Args:
        chunks: List of chunk dicts with id, text, and metadata.
        batch_size: Number of chunks to process at once.

    Returns:
        int: Total number of chunks ingested.
    """
    total = 0

    # Process in batches
    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i + batch_size]
        texts = [c["text"] for c in batch]

        print(f"  Embedding batch {i // batch_size + 1}/{(len(chunks) + batch_size - 1) // batch_size}...")

        # Generate embeddings
        embeddings = embeddings_service.embed_texts(texts)

        # Prepare points for Qdrant
        points = []
        for chunk, embedding in zip(batch, embeddings):
            points.append({
                "id": chunk["id"],
                "vector": embedding,
                "payload": {
                    **chunk["metadata"],
                    "text": chunk["text"],
                },
            })

        # Upsert to Qdrant
        qdrant_service.upsert_chunks(points)
        total += len(points)

    return total


def main():
    """Main ingestion function."""
    print(f"Loading documents from {DOCS_PATH}")

    # Check if docs path exists
    if not DOCS_PATH.exists():
        print(f"ERROR: Docs path does not exist: {DOCS_PATH}")
        return

    # Read documents
    documents = read_markdown_files(DOCS_PATH)
    print(f"Found {len(documents)} markdown files")

    if not documents:
        print("No documents found!")
        return

    # Process into chunks
    print("Processing documents into chunks...")
    chunks = process_documents(documents)
    print(f"Created {len(chunks)} chunks")

    # Create collection if needed
    print("Ensuring Qdrant collection exists...")
    created = qdrant_service.create_collection_if_not_exists()
    if created:
        print(f"Created collection: {settings.QDRANT_COLLECTION}")
    else:
        print(f"Collection already exists: {settings.QDRANT_COLLECTION}")

    # Ingest to Qdrant
    print("Generating embeddings and storing in Qdrant...")
    total = ingest_to_qdrant(chunks)
    print(f"Ingestion complete: {total} chunks stored in Qdrant")

    # Verify
    info = qdrant_service.get_collection_info()
    print(f"Collection info: {info}")


if __name__ == "__main__":
    main()
