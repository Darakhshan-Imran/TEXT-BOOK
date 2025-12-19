"""
OpenAI Agent SDK Integration with LiteLLM for RAG Chatbot.

This module uses OpenAI Agent SDK with LiteLLM to route requests
to Cohere, enabling the use of the agent framework without OpenAI API.
"""

import os
from pathlib import Path
from typing import Optional, List, Dict, Any
from dotenv import load_dotenv

# Load environment variables from root .env file
env_path = Path(__file__).resolve().parent.parent.parent / ".env"
load_dotenv(dotenv_path=env_path)

# Try to import OpenAI Agents SDK with LiteLLM
AGENT_AVAILABLE = False
try:
    from agents import Agent, Runner, function_tool
    from agents.models.litellm import LiteLLMModel
    AGENT_AVAILABLE = True
except ImportError:
    try:
        from agents import Agent, Runner, function_tool
        from agents.extensions.models.litellm_model import LitellmModel as LiteLLMModel
        AGENT_AVAILABLE = True
    except ImportError:
        # Agent SDK not available, will use RAG fallback
        AGENT_AVAILABLE = False
        print("OpenAI Agents SDK with LiteLLM not available, using direct RAG")

# Import RAG functions
from rag import query_rag, embed_query, search_qdrant, assemble_context

# Configuration
COHERE_API_KEY = os.getenv("COHERE_API_KEY")


def search_course_content_func(query: str) -> str:
    """
    Search the Physical AI & Humanoid Robotics course content for relevant information.

    Args:
        query: The search query or question about course content

    Returns:
        Relevant course content that matches the query
    """
    try:
        # Embed the query
        query_vector = embed_query(query)

        # Search Qdrant
        results = search_qdrant(query_vector, top_k=5)

        # Assemble context
        context = assemble_context(results)

        return context

    except Exception as e:
        return f"Error searching course content: {str(e)}"


# Create decorated version only if agent SDK is available
if AGENT_AVAILABLE:
    search_course_content = function_tool(search_course_content_func)
else:
    search_course_content = search_course_content_func


def create_course_agent():
    """
    Create an AI agent for the Physical AI & Humanoid Robotics course.

    Returns:
        Configured Agent instance with RAG tool, or None if not available
    """
    if not AGENT_AVAILABLE:
        return None

    # Create LiteLLM model pointing to Cohere
    model = LiteLLMModel(
        model="cohere/command-r",  # Updated: command-r-plus was deprecated
        api_key=COHERE_API_KEY
    )

    # Define the agent
    agent = Agent(
        name="CourseAssistant",
        model=model,
        instructions="""You are an AI teaching assistant for the "Physical AI & Humanoid Robotics" course.

Your responsibilities:
1. Answer student questions about course content accurately
2. Use the search_course_content tool to find relevant information before answering
3. Explain concepts clearly with examples when helpful
4. If you cannot find relevant information, say so honestly
5. Stay focused on course-related topics

Guidelines:
- Always search the course content first before answering
- Cite sources when providing information
- Be encouraging and supportive to students
- Keep explanations appropriate for students learning robotics
- If asked about topics outside the course, politely redirect to course content""",
        tools=[search_course_content]
    )

    return agent


async def run_agent_query(
    question: str,
    chat_history: Optional[List[Dict[str, str]]] = None
) -> Dict[str, Any]:
    """
    Run a query through the course assistant agent.

    Args:
        question: User's question
        chat_history: Optional previous conversation context

    Returns:
        Dictionary with answer and metadata
    """
    # If agent SDK not available, use direct RAG
    if not AGENT_AVAILABLE:
        print("Agent SDK not available, using direct RAG")
        try:
            rag_result = query_rag(question, chat_history)
            return {
                "answer": rag_result["answer"],
                "sources": rag_result.get("sources", []),
                "success": True,
                "fallback": True
            }
        except Exception as rag_error:
            return {
                "answer": f"I apologize, but I encountered an error processing your question. Please try again.",
                "success": False,
                "error": str(rag_error)
            }

    try:
        # Create agent
        agent = create_course_agent()

        if agent is None:
            raise Exception("Agent creation failed")

        # Build context from chat history if available
        context_prompt = question
        if chat_history and len(chat_history) > 0:
            history_text = "\n".join([
                f"{msg['role'].title()}: {msg['content']}"
                for msg in chat_history[-4:]  # Last 4 messages
            ])
            context_prompt = f"Previous conversation:\n{history_text}\n\nCurrent question: {question}"

        # Run the agent
        result = await Runner.run(agent, context_prompt)

        return {
            "answer": result.final_output,
            "success": True,
            "agent": "CourseAssistant"
        }

    except Exception as e:
        # Fallback to direct RAG if agent fails
        print(f"Agent error, falling back to direct RAG: {e}")
        try:
            rag_result = query_rag(question, chat_history)
            return {
                "answer": rag_result["answer"],
                "sources": rag_result.get("sources", []),
                "success": True,
                "fallback": True
            }
        except Exception as rag_error:
            return {
                "answer": f"I apologize, but I encountered an error processing your question. Please try again.",
                "success": False,
                "error": str(rag_error)
            }


# Synchronous wrapper for FastAPI
def query_agent_sync(
    question: str,
    chat_history: Optional[List[Dict[str, str]]] = None
) -> Dict[str, Any]:
    """
    Synchronous wrapper for agent query (for use in FastAPI).

    Args:
        question: User's question
        chat_history: Optional conversation history

    Returns:
        Agent response dictionary
    """
    import asyncio

    # Check if there's a running event loop
    try:
        loop = asyncio.get_running_loop()
        # If we're in an async context, we need to run in a new thread
        import concurrent.futures
        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(
                asyncio.run,
                run_agent_query(question, chat_history)
            )
            return future.result()
    except RuntimeError:
        # No running event loop, we can use asyncio.run
        return asyncio.run(run_agent_query(question, chat_history))


# For testing
if __name__ == "__main__":
    import asyncio

    async def test():
        result = await run_agent_query("What is ROS 2?")
        print(f"Answer: {result['answer']}")
        print(f"Success: {result['success']}")

    asyncio.run(test())
