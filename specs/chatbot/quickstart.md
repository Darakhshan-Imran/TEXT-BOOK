# Quickstart: RAG AI Chatbot

## Prerequisites

- Python 3.11+
- Docker installed
- Accounts: Qdrant Cloud, Cohere, OpenAI

## Environment Setup

1. **Create `.env` file in `chatbot-backend/`**:

```env
# Existing (copy from main backend)
SECRET_KEY=your_jwt_secret_key

# Chatbot-specific
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
OPENAI_API_KEY=your_openai_api_key
```

## Local Development

### Backend

```bash
# Navigate to backend
cd docusaurus-course-book/chatbot-backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Run ingestion (one-time)
python ingestion.py

# Start server
uvicorn main:app --reload --port 8080
```

### Frontend

```bash
# Navigate to Docusaurus
cd docusaurus-course-book

# Start dev server
npm start
```

## Docker Deployment

```bash
# Build image
cd chatbot-backend
docker build -t textbook-chatbot .

# Run locally
docker run -p 8080:8080 --env-file .env textbook-chatbot
```

## Render Deployment

1. Create new Web Service on Render
2. Select "Docker" as environment
3. Connect GitHub repo
4. Set root directory: `docusaurus-course-book/chatbot-backend`
5. Add environment variables from `.env`
6. Deploy

## API Testing

```bash
# Health check
curl http://localhost:8080/chat/health

# Send question (with JWT token)
curl -X POST http://localhost:8080/chat/query \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

## Directory Structure

```
docusaurus-course-book/
├── chatbot-backend/
│   ├── Dockerfile
│   ├── requirements.txt
│   ├── main.py          # FastAPI app
│   ├── ingestion.py     # Data pipeline
│   ├── rag.py           # RAG logic
│   └── agent.py         # OpenAI Agent
├── src/
│   └── chatbot/
│       ├── ChatButton.tsx
│       ├── ChatPanel.tsx
│       ├── ChatMessage.tsx
│       └── useChatbot.ts
└── docs/                # Course content (source for RAG)
```

## Verification Checklist

- [ ] Qdrant collection created with course content
- [ ] Backend responds to health check
- [ ] JWT authentication works
- [ ] Chat query returns relevant response
- [ ] Frontend chat button appears for logged-in users
- [ ] Full conversation flow works end-to-end
