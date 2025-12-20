# Implementation Plan: RAG AI Chatbot

## Technical Context

| Aspect | Details |
|--------|---------|
| **Feature** | RAG AI Chatbot for Physical AI Course |
| **Branch** | chatbot-implementation |
| **Time Budget** | 2 hours |
| **Deployment** | Docker on Render |

## Constitution Compliance

| Principle | Compliance |
|-----------|------------|
| Docusaurus Integration | ✅ React components in src/chatbot/ |
| No Hardware Required | ✅ Cloud-based RAG, no local setup |
| Educational Focus | ✅ Answers based on course content |
| Code Standards | ✅ PEP 8, type hints, docstrings |

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    DOCUSAURUS FRONTEND                      │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │ ChatButton  │  │ ChatPanel   │  │ ChatMessage         │ │
│  │ (floating)  │─▶│ (expanded)  │─▶│ (bubbles)           │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
│                          │                                  │
│                    useChatbot.ts                            │
└────────────────────────────┼────────────────────────────────┘
                             │ POST /chat/query
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                  CHATBOT BACKEND (Docker)                   │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │   main.py   │  │   rag.py    │  │     agent.py        │ │
│  │  (FastAPI)  │─▶│  (RAG pipe) │─▶│  (OpenAI Agent)     │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
│                          │                                  │
│                    ┌─────┴─────┐                            │
│                    ▼           ▼                            │
│              ┌──────────┐ ┌──────────┐                     │
│              │  Qdrant  │ │  Cohere  │                     │
│              │  Cloud   │ │   API    │                     │
│              └──────────┘ └──────────┘                     │
└─────────────────────────────────────────────────────────────┘
```

## Directory Structure

```
docusaurus-course-book/
├── chatbot-backend/           # NEW: Backend for chatbot
│   ├── Dockerfile
│   ├── requirements.txt
│   ├── .env.example
│   ├── main.py               # FastAPI endpoints
│   ├── ingestion.py          # Data ingestion pipeline
│   ├── rag.py                # RAG query logic
│   └── agent.py              # OpenAI Agent SDK
├── src/
│   └── chatbot/              # NEW: Frontend components
│       ├── ChatButton.tsx    # Floating button
│       ├── ChatPanel.tsx     # Chat interface
│       ├── ChatMessage.tsx   # Message bubbles
│       ├── useChatbot.ts     # API hook
│       └── chatbot.css       # Styles
└── specs/chatbot/            # Specifications
```

## Implementation Phases

### Phase 1: Backend Setup (15 min)

**Files to create**:
- `chatbot-backend/Dockerfile`
- `chatbot-backend/requirements.txt`
- `chatbot-backend/.env.example`
- `chatbot-backend/main.py` (skeleton)

**Dependencies**:
```
fastapi==0.104.1
uvicorn[standard]==0.24.0
cohere==5.0.0
qdrant-client==1.7.0
openai-agents>=0.1.0
pyjwt==2.8.0
python-dotenv==1.0.0
```

### Phase 2: Data Ingestion (25 min)

**File**: `chatbot-backend/ingestion.py`

**Functions**:
1. `load_markdown_files(docs_path)` - Read all .md files
2. `chunk_content(text, chunk_size=500)` - Split into chunks
3. `generate_embeddings(chunks)` - Cohere embed API
4. `upload_to_qdrant(chunks, embeddings)` - Store vectors
5. `run_ingestion()` - Main pipeline

### Phase 3: RAG Pipeline (20 min)

**File**: `chatbot-backend/rag.py`

**Functions**:
1. `embed_query(question)` - Embed user question
2. `search_qdrant(query_vector, top_k=5)` - Vector search
3. `assemble_context(results)` - Build context string
4. `generate_response(question, context)` - Cohere generate

### Phase 4: Agent + API (15 min)

**Files**:
- `chatbot-backend/agent.py` - OpenAI Agent with RAG tool
- `chatbot-backend/main.py` - Complete FastAPI app

**Endpoints**:
- `POST /chat/query` - Main chat endpoint
- `GET /chat/health` - Health check

### Phase 5: Frontend UI (25 min)

**Files**:
- `src/chatbot/ChatButton.tsx` - Floating button
- `src/chatbot/ChatPanel.tsx` - Chat interface
- `src/chatbot/ChatMessage.tsx` - Message bubbles
- `src/chatbot/useChatbot.ts` - API hook
- `src/chatbot/chatbot.css` - Styles

**Integration**:
- Add ChatButton to Docusaurus Layout
- Connect to AuthContext for user check

### Phase 6: Deploy (20 min)

1. Run ingestion script locally
2. Build and test Docker image
3. Deploy to Render as Docker service
4. Update frontend with production URL
5. Push to GitHub, deploy frontend
6. End-to-end test

## Environment Variables

| Variable | Description |
|----------|-------------|
| `SECRET_KEY` | JWT secret (same as auth backend) |
| `COHERE_API_KEY` | Cohere API key |
| `QDRANT_URL` | Qdrant Cloud URL |
| `QDRANT_API_KEY` | Qdrant API key |
| `OPENAI_API_KEY` | OpenAI API key |

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Slow Cohere API | Cache common queries |
| Qdrant rate limits | Batch ingestion requests |
| Large response time | Set timeout, show loading state |
| Auth token expiry | Handle 401 gracefully in UI |

## Success Metrics

| Metric | Target |
|--------|--------|
| Response time | < 5 seconds |
| Ingestion time | < 5 minutes for all docs |
| Docker build | < 2 minutes |
| UI responsiveness | < 300ms interactions |

## Artifacts Generated

- [x] research.md
- [x] data-model.md
- [x] contracts/chatbot-api.yaml
- [x] quickstart.md
- [x] plan.md (this file)
