"""
Data Ingestion Pipeline for RAG Chatbot.

This module handles:
1. Loading markdown files from the docs/ directory
2. Chunking content into smaller pieces
3. Generating embeddings using Cohere
4. Uploading vectors to Qdrant
"""

import os
import re
import uuid
from pathlib import Path
from typing import List, Dict, Any
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Load environment variables from root .env file
env_path = Path(__file__).resolve().parent.parent.parent / ".env"
load_dotenv(dotenv_path=env_path)

# Configuration
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "course_content"
EMBEDDING_MODEL = "embed-english-v3.0"
CHUNK_SIZE = 500  # tokens (approximate)
CHUNK_OVERLAP = 50  # tokens overlap between chunks


def load_markdown_files(docs_path: str) -> List[Dict[str, Any]]:
    """
    Load all markdown files from the docs directory.

    Args:
        docs_path: Path to the docs directory

    Returns:
        List of dictionaries containing file content and metadata
    """
    documents = []
    docs_dir = Path(docs_path)

    if not docs_dir.exists():
        print(f"Warning: Docs directory not found at {docs_path}")
        return documents

    for md_file in docs_dir.rglob("*.md"):
        try:
            with open(md_file, "r", encoding="utf-8") as f:
                content = f.read()

            # Extract metadata from frontmatter if present
            metadata = extract_frontmatter(content)

            # Remove frontmatter from content
            content = remove_frontmatter(content)

            # Get relative path for source reference
            relative_path = str(md_file.relative_to(docs_dir))

            documents.append({
                "content": content,
                "source": relative_path,
                "chapter": metadata.get("title", relative_path),
                "file_path": str(md_file)
            })
            print(f"Loaded: {relative_path}")

        except Exception as e:
            print(f"Error loading {md_file}: {e}")

    print(f"Total documents loaded: {len(documents)}")
    return documents


def extract_frontmatter(content: str) -> Dict[str, str]:
    """
    Extract YAML frontmatter from markdown content.

    Args:
        content: Markdown content with potential frontmatter

    Returns:
        Dictionary of frontmatter key-value pairs
    """
    metadata = {}
    frontmatter_match = re.match(r"^---\s*\n(.*?)\n---\s*\n", content, re.DOTALL)

    if frontmatter_match:
        frontmatter = frontmatter_match.group(1)
        for line in frontmatter.split("\n"):
            if ":" in line:
                key, value = line.split(":", 1)
                metadata[key.strip()] = value.strip().strip('"\'')

    return metadata


def remove_frontmatter(content: str) -> str:
    """
    Remove YAML frontmatter from markdown content.

    Args:
        content: Markdown content with potential frontmatter

    Returns:
        Content without frontmatter
    """
    return re.sub(r"^---\s*\n.*?\n---\s*\n", "", content, flags=re.DOTALL)


def chunk_content(text: str, chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> List[str]:
    """
    Split text into chunks of approximately chunk_size tokens.

    Uses simple word-based splitting as approximation for tokens.
    Each chunk overlaps with the previous one to maintain context.

    Args:
        text: Text to split
        chunk_size: Approximate number of tokens per chunk
        overlap: Number of tokens to overlap between chunks

    Returns:
        List of text chunks
    """
    # Clean up the text
    text = re.sub(r"\n{3,}", "\n\n", text)  # Remove excessive newlines
    text = re.sub(r"```[\s\S]*?```", "[CODE BLOCK]", text)  # Simplify code blocks

    # Split by words (approximate token count)
    words = text.split()

    if len(words) <= chunk_size:
        return [text] if text.strip() else []

    chunks = []
    start = 0

    while start < len(words):
        end = start + chunk_size
        chunk_words = words[start:end]
        chunk_text = " ".join(chunk_words)

        if chunk_text.strip():
            chunks.append(chunk_text)

        # Move start position with overlap
        start = end - overlap

        # Prevent infinite loop
        if start >= len(words) - overlap:
            break

    return chunks


def generate_embeddings(texts: List[str], cohere_client: cohere.Client) -> List[List[float]]:
    """
    Generate embeddings for a list of texts using Cohere.

    Args:
        texts: List of text strings to embed
        cohere_client: Initialized Cohere client

    Returns:
        List of embedding vectors
    """
    if not texts:
        return []

    # Cohere has a limit on batch size, process in batches
    batch_size = 96
    all_embeddings = []

    for i in range(0, len(texts), batch_size):
        batch = texts[i:i + batch_size]

        response = cohere_client.embed(
            texts=batch,
            model=EMBEDDING_MODEL,
            input_type="search_document"
        )

        all_embeddings.extend(response.embeddings)
        print(f"Generated embeddings for batch {i // batch_size + 1}")

    return all_embeddings


def upload_to_qdrant(
    chunks: List[Dict[str, Any]],
    embeddings: List[List[float]],
    qdrant_client: QdrantClient
) -> None:
    """
    Upload chunks and embeddings to Qdrant.

    Args:
        chunks: List of chunk dictionaries with content and metadata
        embeddings: List of embedding vectors
        qdrant_client: Initialized Qdrant client
    """
    # Create collection if it doesn't exist
    collections = qdrant_client.get_collections().collections
    collection_names = [c.name for c in collections]

    if COLLECTION_NAME not in collection_names:
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=1024,  # Cohere embed-english-v3.0 dimension
                distance=models.Distance.COSINE
            )
        )
        print(f"Created collection: {COLLECTION_NAME}")
    else:
        # Clear existing data for fresh ingestion
        qdrant_client.delete_collection(collection_name=COLLECTION_NAME)
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=1024,
                distance=models.Distance.COSINE
            )
        )
        print(f"Recreated collection: {COLLECTION_NAME}")

    # Prepare points for upload
    points = []
    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        point = models.PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={
                "content": chunk["content"],
                "source": chunk["source"],
                "chapter": chunk["chapter"],
                "chunk_index": chunk["chunk_index"]
            }
        )
        points.append(point)

    # Upload in batches
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=batch
        )
        print(f"Uploaded batch {i // batch_size + 1} ({len(batch)} points)")

    print(f"Total points uploaded: {len(points)}")


def run_ingestion(docs_path: str = None) -> None:
    """
    Run the complete ingestion pipeline.

    Args:
        docs_path: Path to docs directory (defaults to ../docs)
    """
    print("=" * 50)
    print("Starting Data Ingestion Pipeline")
    print("=" * 50)

    # Set default docs path
    if docs_path is None:
        docs_path = Path(__file__).resolve().parent.parent / "docs"

    # Validate environment variables
    if not COHERE_API_KEY:
        raise ValueError("COHERE_API_KEY not set in environment")
    if not QDRANT_URL:
        raise ValueError("QDRANT_URL not set in environment")
    if not QDRANT_API_KEY:
        raise ValueError("QDRANT_API_KEY not set in environment")

    # Initialize clients
    print("\n1. Initializing clients...")
    cohere_client = cohere.Client(COHERE_API_KEY)
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY
    )
    print("   Clients initialized successfully")

    # Load markdown files
    print("\n2. Loading markdown files...")
    documents = load_markdown_files(str(docs_path))

    if not documents:
        print("No documents found. Exiting.")
        return

    # Chunk documents
    print("\n3. Chunking documents...")
    all_chunks = []
    for doc in documents:
        chunks = chunk_content(doc["content"])
        for i, chunk_text in enumerate(chunks):
            all_chunks.append({
                "content": chunk_text,
                "source": doc["source"],
                "chapter": doc["chapter"],
                "chunk_index": i
            })
    print(f"   Total chunks created: {len(all_chunks)}")

    if not all_chunks:
        print("No chunks created. Exiting.")
        return

    # Generate embeddings
    print("\n4. Generating embeddings...")
    chunk_texts = [chunk["content"] for chunk in all_chunks]
    embeddings = generate_embeddings(chunk_texts, cohere_client)
    print(f"   Total embeddings generated: {len(embeddings)}")

    # Upload to Qdrant
    print("\n5. Uploading to Qdrant...")
    upload_to_qdrant(all_chunks, embeddings, qdrant_client)

    print("\n" + "=" * 50)
    print("Ingestion Complete!")
    print(f"Documents processed: {len(documents)}")
    print(f"Chunks created: {len(all_chunks)}")
    print(f"Vectors uploaded: {len(embeddings)}")
    print("=" * 50)


if __name__ == "__main__":
    run_ingestion()
