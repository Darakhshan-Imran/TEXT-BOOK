# Feature Specification: RAG AI Chatbot

## Overview

An AI-powered chatbot that answers student questions about the "Physical AI & Humanoid Robotics" course content using Retrieval Augmented Generation (RAG). The chatbot retrieves relevant content from course materials stored in a Qdrant vector database and generates contextual responses using Cohere LLM.

## Problem Statement

Students studying the Physical AI & Humanoid Robotics course need instant answers to their questions about course content without waiting for instructor responses. Currently, there is no automated way to get contextual help based on the actual course materials.

## Target Users

- **Primary**: Authenticated students enrolled in the course
- **Access Control**: Logged-in users only (requires valid JWT token)

## User Requirements

| ID | Requirement | Priority |
|----|-------------|----------|
| UR-01 | Users can access chatbot only when logged in | Must Have |
| UR-02 | Users can ask questions about course content | Must Have |
| UR-03 | Users receive contextual answers based on course materials | Must Have |
| UR-04 | Users can open/close chat panel via floating button | Must Have |
| UR-05 | Users see typing indicator while response generates | Should Have |

## Functional Requirements

### FR-01: Chat Interface
- Floating chat button positioned at bottom-right corner of screen
- Button expands to reveal chat panel when clicked
- Chat panel displays conversation history within session
- Text input field with send button for submitting questions
- Close button to minimize chat panel back to floating button

### FR-02: Authentication Integration
- Chat button only visible to authenticated users
- API requests include JWT token for verification
- Unauthenticated requests return 401 error
- Graceful handling when session expires

### FR-03: Question Processing
- User submits question via text input
- System embeds question using Cohere embeddings
- System searches Qdrant for relevant course content (top-5 matches)
- System assembles context from retrieved chunks
- System generates response using Cohere LLM with context

### FR-04: Response Display
- Display AI response in chat bubble format
- Show typing indicator during response generation
- Handle errors gracefully with user-friendly messages
- Response time target: under 5 seconds

## User Scenarios & Testing

### Scenario 1: Successful Question-Answer Flow
**Given** a logged-in user on any course page
**When** they click the chat button and ask "What is ROS2?"
**Then** the system retrieves relevant course content and displays a contextual answer

### Scenario 2: Unauthenticated Access Attempt
**Given** a user who is not logged in
**When** they view any course page
**Then** the chat button is not visible

### Scenario 3: Chat Panel Interaction
**Given** a logged-in user
**When** they click the floating chat button
**Then** the chat panel expands with input field ready for typing

### Scenario 4: Error Handling
**Given** a logged-in user with network issues
**When** they submit a question
**Then** a user-friendly error message is displayed

## Success Criteria

| Criteria | Metric | Target |
|----------|--------|--------|
| Response Time | Time from question to answer | < 5 seconds |
| Relevance | Answers based on actual course content | 90% relevance |
| Availability | Chat accessible when logged in | 99% uptime |
| User Experience | Chat panel opens/closes smoothly | < 300ms transition |

## Tech Stack

| Component | Technology | Purpose |
|-----------|------------|---------|
| Vector Database | Qdrant Cloud | Store and search course content embeddings |
| Embeddings | Cohere embed-english-v3.0 | Convert text to vectors |
| LLM | Cohere command-r-plus | Generate contextual responses |
| Agent Framework | OpenAI Agent SDK | Orchestrate RAG pipeline |
| Backend | FastAPI + Docker | API endpoints for chat |
| Frontend | React (Docusaurus) | Chat UI components |

## Data Flow

1. **Ingestion (One-time)**:
   - Parse markdown files from `docs/` folder
   - Chunk content into 500-token segments
   - Generate embeddings via Cohere
   - Store vectors in Qdrant collection

2. **Query (Per Question)**:
   - Embed user question via Cohere
   - Search Qdrant for top-5 similar chunks
   - Assemble context from retrieved content
   - Generate response via Cohere with context
   - Return response to frontend

## Constraints

- Backend deployed via Docker on Render
- Must integrate with existing JWT authentication
- Must work within Docusaurus static site architecture
- API keys stored securely in environment variables

## Assumptions

- Qdrant Cloud free tier provides sufficient storage and queries
- Cohere API rate limits are adequate for expected usage
- Course content in markdown format is suitable for RAG
- Users have stable internet connection

## Out of Scope (This Phase)

- Voice input/output
- Chat history persistence across sessions
- Admin analytics dashboard
- Multi-language support (planned for future phase)

## Future Considerations

- **Multi-language Support**: Translation layer for international students
- **Chat History**: Persist conversations for returning users
- **Analytics**: Track popular questions and response quality

## Dependencies

- Existing authentication system (JWT-based)
- Qdrant Cloud account (user has this)
- Cohere API account (user has this)
- OpenAI API account (for Agent SDK)

## Environment Variables Required

```
COHERE_API_KEY=<cohere-api-key>
QDRANT_URL=<qdrant-cloud-url>
QDRANT_API_KEY=<qdrant-api-key>
OPENAI_API_KEY=<openai-api-key>
```
