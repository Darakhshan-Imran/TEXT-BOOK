"""
RAG Chatbot API for Physical AI & Humanoid Robotics Course.

This FastAPI application provides a chatbot endpoint that uses RAG
(Retrieval Augmented Generation) to answer questions about course content.
"""

from fastapi import FastAPI, HTTPException, Depends, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from dotenv import load_dotenv
from pathlib import Path
import jwt
import os
import uuid
import logging

# Load environment variables from root .env file
env_path = Path(__file__).resolve().parent.parent.parent / ".env"
load_dotenv(dotenv_path=env_path)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import RAG and Agent modules
from rag import query_rag, get_qdrant_client, get_cohere_client
from agent import query_agent_sync

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="AI Chatbot for Physical AI & Humanoid Robotics Course",
    version="1.0.0"
)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://localhost:3001",
        "https://darakhshan-imran.github.io",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["*"]
)

# Security
security = HTTPBearer()
SECRET_KEY = os.getenv("SECRET_KEY", "")

# In-memory session storage (for demo - use Redis in production)
chat_sessions: Dict[str, List[Dict[str, str]]] = {}


# Pydantic models
class ChatRequest(BaseModel):
    """Request model for chat queries."""
    question: str = Field(..., min_length=3, max_length=500, description="User's question")
    session_id: Optional[str] = Field(None, description="Session ID for conversation continuity")


class Source(BaseModel):
    """Source reference for chat responses."""
    chapter: str
    section: str
    relevance: float


class ChatResponse(BaseModel):
    """Response model for chat queries."""
    answer: str
    sources: List[Source] = []
    session_id: str


class HealthResponse(BaseModel):
    """Response model for health check."""
    status: str
    qdrant: str
    cohere: str


class ErrorResponse(BaseModel):
    """Response model for errors."""
    detail: str


# JWT verification
def verify_token(credentials: HTTPAuthorizationCredentials = Depends(security)) -> dict:
    """
    Verify JWT token from Authorization header.

    Args:
        credentials: HTTP Bearer credentials

    Returns:
        Decoded token payload

    Raises:
        HTTPException: If token is invalid or expired
    """
    try:
        token = credentials.credentials
        payload = jwt.decode(token, SECRET_KEY, algorithms=["HS256"])
        return payload
    except jwt.ExpiredSignatureError:
        logger.warning("Token expired")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired"
        )
    except jwt.InvalidTokenError as e:
        logger.warning(f"Invalid token: {e}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token"
        )


# Health check endpoint
@app.get("/chat/health", response_model=HealthResponse)
async def health_check():
    """
    Check if the chatbot service is running and dependencies are connected.

    Returns:
        Health status of the service and its dependencies
    """
    qdrant_status = "disconnected"
    cohere_status = "disconnected"

    # Check Qdrant connection
    try:
        client = get_qdrant_client()
        collections = client.get_collections()
        qdrant_status = "connected"
    except Exception as e:
        logger.error(f"Qdrant health check failed: {e}")

    # Check Cohere connection
    try:
        client = get_cohere_client()
        cohere_status = "connected"
    except Exception as e:
        logger.error(f"Cohere health check failed: {e}")

    overall_status = "healthy" if qdrant_status == "connected" and cohere_status == "connected" else "degraded"

    return HealthResponse(
        status=overall_status,
        qdrant=qdrant_status,
        cohere=cohere_status
    )


# Chat query endpoint
@app.post("/chat/query", response_model=ChatResponse)
async def chat_query(
    request: ChatRequest,
    token_payload: dict = Depends(verify_token)
):
    """
    Process a chat query using RAG with OpenAI Agent SDK.

    Args:
        request: Chat request with question
        token_payload: Verified JWT payload

    Returns:
        AI-generated response with sources
    """
    # Get or create session
    session_id = request.session_id or str(uuid.uuid4())

    # Get chat history for this session
    chat_history = chat_sessions.get(session_id, [])

    logger.info(f"Processing query: {request.question[:50]}... (session: {session_id})")

    try:
        # Try using the agent first
        result = query_agent_sync(request.question, chat_history)

        if result.get("success"):
            answer = result["answer"]
            sources = result.get("sources", [])
        else:
            # Fallback to direct RAG
            logger.warning("Agent failed, using direct RAG")
            rag_result = query_rag(request.question, chat_history)
            answer = rag_result["answer"]
            sources = rag_result.get("sources", [])

    except Exception as e:
        logger.error(f"Error processing query: {e}")
        # Final fallback
        try:
            rag_result = query_rag(request.question, chat_history)
            answer = rag_result["answer"]
            sources = rag_result.get("sources", [])
        except Exception as rag_error:
            logger.error(f"RAG fallback also failed: {rag_error}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to process your question. Please try again."
            )

    # Update chat history
    chat_history.append({"role": "user", "content": request.question})
    chat_history.append({"role": "assistant", "content": answer})

    # Keep only last 10 messages
    chat_sessions[session_id] = chat_history[-10:]

    # Format sources
    formatted_sources = []
    for source in sources:
        formatted_sources.append(Source(
            chapter=source.get("chapter", "Unknown"),
            section=source.get("section", ""),
            relevance=source.get("relevance", 0.0)
        ))

    return ChatResponse(
        answer=answer,
        sources=formatted_sources,
        session_id=session_id
    )


# Root endpoint
@app.get("/")
async def root():
    """Root endpoint returning API info."""
    return {
        "message": "RAG Chatbot API for Physical AI & Humanoid Robotics Course",
        "version": "1.0.0",
        "endpoints": {
            "health": "/chat/health",
            "query": "/chat/query (POST, requires auth)",
            "docs": "/docs"
        }
    }


# Global exception handler
@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """Handle unexpected exceptions."""
    logger.error(f"Unexpected error: {exc}")
    return JSONResponse(
        status_code=500,
        content={"detail": "An unexpected error occurred. Please try again."}
    )


# Startup event
@app.on_event("startup")
async def startup_event():
    """Initialize connections on startup."""
    logger.info("Starting RAG Chatbot API...")
    logger.info(f"SECRET_KEY configured: {'Yes' if SECRET_KEY else 'No'}")
    logger.info(f"COHERE_API_KEY configured: {'Yes' if os.getenv('COHERE_API_KEY') else 'No'}")
    logger.info(f"QDRANT_URL configured: {'Yes' if os.getenv('QDRANT_URL') else 'No'}")


# Shutdown event
@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown."""
    logger.info("Shutting down RAG Chatbot API...")
