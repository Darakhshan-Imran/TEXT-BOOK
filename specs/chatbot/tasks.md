# Tasks: RAG AI Chatbot

## Feature Overview

| Attribute | Value |
|-----------|-------|
| **Feature** | RAG AI Chatbot |
| **Branch** | chatbot-implementation |
| **Time Budget** | 2 hours |
| **Total Tasks** | 24 |

---

## Phase 1: Backend Setup (15 min)

**Goal**: Create project structure and dependencies for chatbot backend

### Tasks

- [x] T001 Create chatbot-backend directory at `docusaurus-course-book/chatbot-backend/`
- [x] T002 Create requirements.txt with dependencies at `chatbot-backend/requirements.txt`
- [x] T003 Create .env.example with required variables at `chatbot-backend/.env.example`
- [x] T004 Create Dockerfile for Python 3.11 at `chatbot-backend/Dockerfile`
- [x] T005 Create main.py FastAPI skeleton with health endpoint at `chatbot-backend/main.py`

**Verification**: `docker build -t chatbot-test .` succeeds

---

## Phase 2: Data Ingestion (25 min)

**Goal**: Ingest course markdown content into Qdrant vector database

### Tasks

- [x] T006 [P] Create ingestion.py with load_markdown_files() function at `chatbot-backend/ingestion.py`
- [x] T007 Implement chunk_content() function for 500-token chunks in `chatbot-backend/ingestion.py`
- [x] T008 Implement generate_embeddings() using Cohere API in `chatbot-backend/ingestion.py`
- [x] T009 Implement upload_to_qdrant() function in `chatbot-backend/ingestion.py`
- [x] T010 Implement run_ingestion() main pipeline in `chatbot-backend/ingestion.py`
- [ ] T011 Run ingestion script to populate Qdrant with course content

**Verification**: Qdrant collection contains vectors from all docs/*.md files

---

## Phase 3: RAG Pipeline (20 min)

**Goal**: Implement query processing and response generation

### Tasks

- [x] T012 [P] Create rag.py with embed_query() function at `chatbot-backend/rag.py`
- [x] T013 Implement search_qdrant() for top-5 vector search in `chatbot-backend/rag.py`
- [x] T014 Implement assemble_context() to build prompt context in `chatbot-backend/rag.py`
- [x] T015 Implement generate_response() using Cohere command-r-plus in `chatbot-backend/rag.py`

**Verification**: `rag.generate_response("What is ROS2?")` returns relevant answer

---

## Phase 4: Agent + API (15 min)

**Goal**: Create FastAPI endpoints with JWT authentication

### Tasks

- [x] T016 [P] Create agent.py with RAG tool definition at `chatbot-backend/agent.py`
- [x] T017 Implement JWT verification middleware in `chatbot-backend/main.py`
- [x] T018 Implement POST /chat/query endpoint in `chatbot-backend/main.py`
- [x] T019 Add error handling and CORS configuration in `chatbot-backend/main.py`

**Verification**: POST /chat/query with valid JWT returns AI response

---

## Phase 5: Frontend UI (25 min)

**Goal**: Create React chat components for Docusaurus

### Tasks

- [x] T020 [P] Create ChatButton.tsx floating button component at `src/chatbot/ChatButton.tsx`
- [x] T021 [P] Create ChatMessage.tsx message bubble component at `src/chatbot/ChatMessage.tsx`
- [x] T022 Create ChatPanel.tsx expandable chat interface at `src/chatbot/ChatPanel.tsx`
- [x] T023 Create useChatbot.ts API hook at `src/chatbot/useChatbot.ts`
- [x] T024 [P] Create chatbot.css styles at `src/chatbot/chatbot.css`
- [x] T025 Integrate ChatButton into Docusaurus Root component
- [x] T026 Connect ChatPanel to AuthContext for logged-in check

**Verification**: Chat button visible only when logged in, sends questions to API

---

## Phase 6: Deploy (20 min)

**Goal**: Deploy backend to Render and test end-to-end

### Tasks

- [ ] T027 Build and test Docker image locally
- [ ] T028 Create Render Web Service with Docker environment
- [ ] T029 Configure environment variables on Render
- [ ] T030 Update frontend chatbot config with Render URL at `src/chatbot/config.ts`
- [ ] T031 Push changes to GitHub and trigger frontend deploy
- [ ] T032 End-to-end test: login → open chat → ask question → verify response

**Verification**: Full flow works on https://darakhshan-imran.github.io/TEXT-BOOK/

---

## Dependencies

```
T001 → T002 → T003 → T004 → T005 (Sequential: Backend Setup)
T005 → T006-T011 (Ingestion depends on backend skeleton)
T011 → T012-T015 (RAG depends on ingested data)
T015 → T016-T019 (API depends on RAG pipeline)
T005 → T020-T026 (Frontend can start after backend skeleton)
T019 + T026 → T027-T032 (Deploy after both backend and frontend ready)
```

## Parallel Execution Opportunities

| Phase | Parallel Tasks | Reason |
|-------|---------------|--------|
| Phase 2 | T006 | Independent file, no dependencies |
| Phase 3 | T012 | Independent file creation |
| Phase 4 | T016 | Independent agent.py file |
| Phase 5 | T020, T021, T024 | Independent components, different files |

## Implementation Strategy

### MVP Scope (First 1 hour)
1. Complete Phase 1-3 (Backend + Ingestion + RAG)
2. Test RAG pipeline locally
3. Basic API endpoint working

### Full Implementation (Hour 2)
1. Complete Phase 4-5 (Agent + Frontend)
2. Phase 6 (Deploy)
3. End-to-end verification

## Task Summary

| Phase | Tasks | Time |
|-------|-------|------|
| Phase 1: Backend Setup | 5 | 15 min |
| Phase 2: Data Ingestion | 6 | 25 min |
| Phase 3: RAG Pipeline | 4 | 20 min |
| Phase 4: Agent + API | 4 | 15 min |
| Phase 5: Frontend UI | 7 | 25 min |
| Phase 6: Deploy | 6 | 20 min |
| **Total** | **32** | **120 min** |
