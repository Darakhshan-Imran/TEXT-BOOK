"""
RAG (Retrieval Augmented Generation) Pipeline.

This module handles:
1. Embedding user queries
2. Searching Qdrant for relevant content
3. Assembling context from search results
4. Generating responses using Cohere
"""

import os
from pathlib import Path
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient

# Load environment variables from root .env file
env_path = Path(__file__).resolve().parent.parent.parent / ".env"
load_dotenv(dotenv_path=env_path)

# Configuration
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "course_content"
EMBEDDING_MODEL = "embed-english-v3.0"
GENERATION_MODEL = "command-r"  # Updated: command-r-plus was deprecated
TOP_K = 5  # Number of results to retrieve


# Initialize clients (lazy loading)
_cohere_client: Optional[cohere.Client] = None
_qdrant_client: Optional[QdrantClient] = None


def get_cohere_client() -> cohere.Client:
    """Get or create Cohere client."""
    global _cohere_client
    if _cohere_client is None:
        if not COHERE_API_KEY:
            raise ValueError("COHERE_API_KEY not set in environment")
        _cohere_client = cohere.Client(COHERE_API_KEY)
    return _cohere_client


def get_qdrant_client() -> QdrantClient:
    """Get or create Qdrant client."""
    global _qdrant_client
    if _qdrant_client is None:
        if not QDRANT_URL or not QDRANT_API_KEY:
            raise ValueError("QDRANT_URL or QDRANT_API_KEY not set in environment")
        _qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    return _qdrant_client


def embed_query(question: str) -> List[float]:
    """
    Generate embedding for a user query.

    Args:
        question: User's question text

    Returns:
        Embedding vector (1024 dimensions for Cohere)
    """
    client = get_cohere_client()

    response = client.embed(
        texts=[question],
        model=EMBEDDING_MODEL,
        input_type="search_query"  # Different from document embedding
    )

    return response.embeddings[0]


def search_qdrant(query_vector: List[float], top_k: int = TOP_K) -> List[Dict[str, Any]]:
    """
    Search Qdrant for similar content using the query vector.

    Args:
        query_vector: Embedding vector of the query
        top_k: Number of results to return

    Returns:
        List of search results with content and metadata
    """
    from qdrant_client.models import ScoredPoint

    client = get_qdrant_client()

    # Use query_points for newer qdrant-client versions
    try:
        # Try the newer API first (qdrant-client >= 1.7.1)
        results = client.query_points(
            collection_name=COLLECTION_NAME,
            query=query_vector,
            limit=top_k,
            with_payload=True
        )
        # query_points returns a QueryResponse object with .points attribute
        points = results.points
    except AttributeError:
        # Fallback to older API
        results = client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_vector,
            limit=top_k,
            with_payload=True
        )
        points = results

    # Format results
    formatted_results = []
    for result in points:
        formatted_results.append({
            "content": result.payload.get("content", ""),
            "source": result.payload.get("source", ""),
            "chapter": result.payload.get("chapter", ""),
            "chunk_index": result.payload.get("chunk_index", 0),
            "score": result.score
        })

    return formatted_results


def assemble_context(search_results: List[Dict[str, Any]]) -> str:
    """
    Assemble context string from search results.

    Args:
        search_results: List of search results from Qdrant

    Returns:
        Formatted context string for the LLM
    """
    if not search_results:
        return "No relevant content found in the course materials."

    context_parts = []

    for i, result in enumerate(search_results, 1):
        chapter = result.get("chapter", "Unknown")
        source = result.get("source", "Unknown")
        content = result.get("content", "")
        score = result.get("score", 0)

        context_parts.append(
            f"[Source {i}: {chapter} ({source}), Relevance: {score:.2f}]\n{content}"
        )

    return "\n\n---\n\n".join(context_parts)


def generate_response(
    question: str,
    context: str,
    chat_history: Optional[List[Dict[str, str]]] = None
) -> Dict[str, Any]:
    """
    Generate a response using Cohere with the provided context.

    Args:
        question: User's question
        context: Assembled context from search results
        chat_history: Optional previous conversation history

    Returns:
        Dictionary with answer and metadata
    """
    client = get_cohere_client()

    # Build the system prompt
    system_prompt = """You are an AI teaching assistant for the "Physical AI & Humanoid Robotics" course.
Your role is to help students understand course concepts by answering their questions.

Guidelines:
1. Base your answers ONLY on the provided course content context.
2. If the context doesn't contain relevant information, say so politely.
3. Be educational and explain concepts clearly.
4. Use examples from the course material when helpful.
5. Keep responses concise but comprehensive.
6. If asked about topics outside the course, redirect to course-related topics."""

    # Build the prompt with context
    prompt = f"""Course Content Context:
{context}

---

Student Question: {question}

Please provide a helpful answer based on the course content above."""

    # Format chat history for Cohere
    formatted_history = []
    if chat_history:
        for msg in chat_history[-6:]:  # Keep last 6 messages for context
            role = "USER" if msg.get("role") == "user" else "CHATBOT"
            formatted_history.append({
                "role": role,
                "message": msg.get("content", "")
            })

    # Generate response
    response = client.chat(
        model=GENERATION_MODEL,
        message=prompt,
        preamble=system_prompt,
        chat_history=formatted_history if formatted_history else None,
        temperature=0.3,  # Lower temperature for more focused responses
        max_tokens=1024
    )

    return {
        "answer": response.text,
        "model": GENERATION_MODEL,
        "tokens_used": getattr(response, 'meta', {}).get('tokens', {})
    }


def query_rag(
    question: str,
    chat_history: Optional[List[Dict[str, str]]] = None
) -> Dict[str, Any]:
    """
    Complete RAG pipeline: embed query, search, assemble context, generate response.

    Args:
        question: User's question
        chat_history: Optional conversation history

    Returns:
        Dictionary with answer, sources, and metadata
    """
    # Step 1: Embed the query
    query_vector = embed_query(question)

    # Step 2: Search Qdrant
    search_results = search_qdrant(query_vector)

    # Step 3: Assemble context
    context = assemble_context(search_results)

    # Step 4: Generate response
    response = generate_response(question, context, chat_history)

    # Format sources for response
    sources = []
    for result in search_results:
        sources.append({
            "chapter": result.get("chapter", "Unknown"),
            "section": result.get("source", ""),
            "relevance": round(result.get("score", 0), 2)
        })

    return {
        "answer": response["answer"],
        "sources": sources,
        "model": response.get("model"),
        "context_chunks": len(search_results)
    }


# For testing
if __name__ == "__main__":
    # Test the RAG pipeline
    test_question = "What is ROS 2?"

    print(f"Question: {test_question}")
    print("-" * 50)

    try:
        result = query_rag(test_question)
        print(f"Answer: {result['answer']}")
        print(f"\nSources: {result['sources']}")
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure API keys are set and Qdrant collection is populated.")
