# Data Model: RAG AI Chatbot

## Entities

### 1. DocumentChunk (Qdrant Collection)

Represents a chunk of course content stored as a vector.

| Field | Type | Description |
|-------|------|-------------|
| id | string (UUID) | Unique identifier |
| vector | float[1024] | Cohere embedding vector |
| content | string | Original text content |
| metadata.source | string | Source file path (e.g., "docs/part-1/chapter-01.md") |
| metadata.chapter | string | Chapter title |
| metadata.section | string | Section heading |
| metadata.chunk_index | int | Position within document |

**Qdrant Collection Config**:
```json
{
  "name": "course_content",
  "vectors": {
    "size": 1024,
    "distance": "Cosine"
  }
}
```

### 2. ChatMessage (In-Memory Session)

Represents a message in the conversation.

| Field | Type | Description |
|-------|------|-------------|
| role | enum | "user" or "assistant" |
| content | string | Message text |
| timestamp | datetime | When message was sent |
| sources | string[] | Source references (assistant only) |

### 3. ChatSession (In-Memory)

Represents an active chat session.

| Field | Type | Description |
|-------|------|-------------|
| session_id | string | Unique session identifier |
| user_id | string | Authenticated user ID (from JWT) |
| messages | ChatMessage[] | Conversation history |
| created_at | datetime | Session start time |

## Data Flow

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Markdown  │────▶│   Chunker   │────▶│   Cohere    │
│    Files    │     │             │     │  Embeddings │
└─────────────┘     └─────────────┘     └─────────────┘
                                               │
                                               ▼
                                        ┌─────────────┐
                                        │   Qdrant    │
                                        │  Collection │
                                        └─────────────┘
                                               │
┌─────────────┐     ┌─────────────┐           │
│    User     │────▶│   Query     │───────────┘
│  Question   │     │  Embedding  │
└─────────────┘     └─────────────┘
                           │
                           ▼
                    ┌─────────────┐     ┌─────────────┐
                    │   Vector    │────▶│   Context   │
                    │   Search    │     │  Assembly   │
                    └─────────────┘     └─────────────┘
                                               │
                                               ▼
                                        ┌─────────────┐
                                        │   Cohere    │
                                        │  Generate   │
                                        └─────────────┘
```

## State Transitions

### Chat Session States

```
[New] ──▶ [Active] ──▶ [Closed]
              │
              └──▶ [Expired] (after 30 min inactivity)
```

## Validation Rules

| Entity | Rule |
|--------|------|
| DocumentChunk | Content must be non-empty, max 2000 chars |
| ChatMessage | Content must be non-empty, max 4000 chars |
| ChatSession | Max 50 messages per session |
| Query | Min 3 characters, max 500 characters |
