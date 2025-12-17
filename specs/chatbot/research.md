# Research: RAG AI Chatbot

## Phase 0: Research Findings

### 1. Qdrant Cloud Integration

**Decision**: Use Qdrant Cloud free tier with REST API
**Rationale**:
- Free tier offers 1GB storage (sufficient for course content)
- REST API is simple and well-documented
- Python client `qdrant-client` simplifies integration
**Alternatives Considered**:
- Pinecone (requires paid plan for production)
- ChromaDB (local only, not suitable for cloud deployment)
- Weaviate (more complex setup)

### 2. Cohere API Strategy

**Decision**: Use `embed-english-v3.0` for embeddings, `command-r-plus` for generation
**Rationale**:
- embed-english-v3.0: Best-in-class embedding quality, 1024 dimensions
- command-r-plus: Optimized for RAG with citation support
- Free trial provides sufficient API calls for development
**Alternatives Considered**:
- OpenAI embeddings (more expensive)
- Hugging Face models (requires self-hosting)

### 3. Chunking Strategy

**Decision**: 500-token chunks with 50-token overlap
**Rationale**:
- 500 tokens balances context completeness with retrieval precision
- Overlap ensures no context is lost at boundaries
- Matches Cohere embedding model optimal input size
**Alternatives Considered**:
- Sentence-based chunking (too granular)
- Paragraph-based chunking (too variable in size)

### 4. OpenAI Agent SDK Integration

**Decision**: Use OpenAI Agent SDK for orchestration with Cohere as LLM
**Rationale**:
- Agent SDK provides tool-calling infrastructure
- Can define RAG search as a tool
- Handles conversation context management
**Note**: Agent SDK can work with external LLMs like Cohere

### 5. Docker Deployment Strategy

**Decision**: Single Dockerfile with multi-stage build
**Rationale**:
- Render supports Docker deployments
- Single container simplifies deployment
- Python 3.11 slim base for smaller image size
**Dockerfile Structure**:
```dockerfile
FROM python:3.11-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY . .
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8080"]
```

### 6. Authentication Integration

**Decision**: Reuse existing JWT verification from auth backend
**Rationale**:
- Same SECRET_KEY for token verification
- Consistent auth experience
- No additional auth infrastructure needed
**Implementation**: Middleware that validates JWT from Authorization header

## Resolved Clarifications

| Item | Resolution |
|------|------------|
| Embedding model | Cohere embed-english-v3.0 (1024 dimensions) |
| LLM model | Cohere command-r-plus |
| Chunk size | 500 tokens with 50-token overlap |
| Vector search | Top-5 results |
| Docker base | python:3.11-slim |
| Port | 8080 (Render default) |
