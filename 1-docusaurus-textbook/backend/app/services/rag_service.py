"""RAG (Retrieval-Augmented Generation) service for context-aware chat responses."""

import logging
from typing import AsyncGenerator, Optional
from openai import AsyncOpenAI

from app.config import get_settings
from app.services.vector_store import search_documents

logger = logging.getLogger(__name__)


async def create_query_embedding(query: str) -> list[float]:
    """
    Create embedding for user query using Gemini.

    Args:
        query: User query text

    Returns:
        Embedding vector
    """
    settings = get_settings()

    if not settings.gemini_api_key:
        raise ValueError("GEMINI_API_KEY not configured")

    client = AsyncOpenAI(
        api_key=settings.gemini_api_key,
        base_url=settings.gemini_base_url,
    )

    try:
        response = await client.embeddings.create(
            model=settings.gemini_embedding_model,
            input=query,
        )

        return response.data[0].embedding

    except Exception as e:
        logger.error(f"Error creating query embedding: {e}")
        raise


async def retrieve_context(
    user_query: str,
    current_page: Optional[str] = None,
    top_k: int = 5,
) -> tuple[list[dict], str]:
    """
    Retrieve relevant context from vector database.

    Args:
        user_query: User's question
        current_page: Current page for context boosting
        top_k: Number of results to retrieve

    Returns:
        Tuple of (source_documents, context_text)
    """
    logger.info(f"Retrieving context for query: '{user_query[:100]}...'")

    try:
        # Create query embedding
        query_embedding = await create_query_embedding(user_query)

        # Search vector database
        results = await search_documents(
            query_embedding=query_embedding,
            top_k=top_k,
            score_threshold=0.7,
        )

        if not results:
            logger.warning("No relevant context found in vector database")
            return [], ""

        # Extract source documents
        sources = []
        context_chunks = []

        for result in results:
            payload = result.payload
            sources.append({
                "file_path": payload.get("file_path"),
                "title": payload.get("title"),
                "module": payload.get("module"),
                "score": float(result.score),
                "chunk_id": payload.get("id"),
            })
            context_chunks.append(payload.get("content", ""))

        # Combine context chunks
        context_text = "\n\n---\n\n".join(context_chunks)

        logger.info(f"Retrieved {len(sources)} relevant chunks")
        return sources, context_text

    except Exception as e:
        logger.error(f"Error retrieving context: {e}")
        # Return empty context rather than failing
        return [], ""


def construct_prompt(
    user_query: str,
    context_text: str,
    current_page: Optional[str] = None,
) -> list[dict]:
    """
    Construct chat messages for OpenAI API.

    Args:
        user_query: User's question
        context_text: Retrieved context from vector database
        current_page: Current page user is viewing

    Returns:
        List of message dictionaries for OpenAI chat completion
    """
    system_prompt = """You are an expert AI tutor for the "Physical AI & Humanoid Robotics" textbook.
Your role is to help students understand concepts related to robotics, ROS 2, sensors, control systems, and embodied AI.

Guidelines:
- Provide clear, accurate, and educational responses
- Use the context from the textbook to ground your answers
- If the context doesn't contain the answer, say so and provide general knowledge
- Break down complex topics into understandable explanations
- Use examples and analogies when helpful
- Encourage further learning and exploration
- Be concise but comprehensive

When referencing content from the textbook, mention the relevant module or section if available."""

    if context_text:
        user_message = f"""Based on the following context from the textbook, please answer my question.

Context:
{context_text}

Question: {user_query}"""
    else:
        user_message = f"""I have a question about robotics and Physical AI.

Question: {user_query}

Note: I couldn't find specific context from the textbook for this question. Please provide a helpful answer based on your knowledge of robotics and AI."""

    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_message},
    ]

    return messages


async def generate_response_stream(
    user_query: str,
    current_page: Optional[str] = None,
    top_k: int = 5,
) -> AsyncGenerator[tuple[str, Optional[list[dict]]], None]:
    """
    Generate streaming response using RAG pipeline.

    Args:
        user_query: User's question
        current_page: Current page for context
        top_k: Number of context chunks to retrieve

    Yields:
        Tuples of (text_chunk, sources) where sources is None except for the final chunk
    """
    settings = get_settings()

    if not settings.gemini_api_key:
        raise ValueError("GEMINI_API_KEY not configured")

    logger.info(f"Starting RAG generation for query: '{user_query[:100]}...'")

    try:
        # Step 1: Retrieve context
        sources, context_text = await retrieve_context(user_query, current_page, top_k)

        # Step 2: Construct prompt
        messages = construct_prompt(user_query, context_text, current_page)

        # Step 3: Stream Gemini response
        client = AsyncOpenAI(
            api_key=settings.gemini_api_key,
            base_url=settings.gemini_base_url,
        )

        stream = await client.chat.completions.create(
            model=settings.gemini_model,
            messages=messages,
            temperature=0.7,
            max_tokens=1500,
            stream=True,
        )

        # Stream chunks
        async for chunk in stream:
            if chunk.choices and chunk.choices[0].delta.content:
                text_chunk = chunk.choices[0].delta.content
                yield text_chunk, None

        # Send sources at the end
        yield "", sources

        logger.info("RAG generation completed successfully")

    except Exception as e:
        logger.error(f"Error in RAG generation: {e}")
        error_message = "I apologize, but I encountered an error generating a response. Please try again."
        yield error_message, None


async def generate_response(
    user_query: str,
    current_page: Optional[str] = None,
    top_k: int = 5,
) -> tuple[str, list[dict]]:
    """
    Generate complete (non-streaming) response using RAG pipeline.

    Args:
        user_query: User's question
        current_page: Current page for context
        top_k: Number of context chunks to retrieve

    Returns:
        Tuple of (response_text, sources)
    """
    settings = get_settings()

    if not settings.gemini_api_key:
        raise ValueError("GEMINI_API_KEY not configured")

    logger.info(f"Starting RAG generation (non-streaming) for query: '{user_query[:100]}...'")

    try:
        # Step 1: Retrieve context
        sources, context_text = await retrieve_context(user_query, current_page, top_k)

        # Step 2: Construct prompt
        messages = construct_prompt(user_query, context_text, current_page)

        # Step 3: Get Gemini response
        client = AsyncOpenAI(
            api_key=settings.gemini_api_key,
            base_url=settings.gemini_base_url,
        )

        response = await client.chat.completions.create(
            model=settings.gemini_model,
            messages=messages,
            temperature=0.7,
            max_tokens=1500,
        )

        response_text = response.choices[0].message.content

        logger.info("RAG generation completed successfully")
        return response_text, sources

    except Exception as e:
        logger.error(f"Error in RAG generation: {e}")
        error_message = "I apologize, but I encountered an error generating a response. Please try again."
        return error_message, []
