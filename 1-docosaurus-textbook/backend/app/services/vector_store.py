"""Vector store service for Qdrant operations."""

import logging
from typing import Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, ScoredPoint
from qdrant_client.http.exceptions import UnexpectedResponse

from app.config import get_settings

logger = logging.getLogger(__name__)

# Global Qdrant client (initialized in lifespan)
qdrant_client: Optional[QdrantClient] = None


def get_qdrant_client() -> Optional[QdrantClient]:
    """Get the global Qdrant client instance."""
    return qdrant_client


async def init_qdrant() -> QdrantClient:
    """
    Initialize Qdrant client and create collection if not exists.

    Returns:
        QdrantClient: Initialized Qdrant client

    Raises:
        Exception: If connection fails
    """
    global qdrant_client

    settings = get_settings()

    try:
        logger.info(f"Connecting to Qdrant at {settings.qdrant_url}")

        # Initialize client
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key if settings.qdrant_api_key else None,
            timeout=10,
        )

        # Test connection
        collections = client.get_collections()
        logger.info(f"Successfully connected to Qdrant. Found {len(collections.collections)} collections.")

        # Create collection if it doesn't exist
        collection_name = "book_content"
        try:
            client.get_collection(collection_name)
            logger.info(f"Collection '{collection_name}' already exists.")
        except (UnexpectedResponse, Exception):
            logger.info(f"Creating collection '{collection_name}'...")
            client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(
                    size=768,  # Gemini text-embedding-004 dimension
                    distance=Distance.COSINE,
                ),
            )
            logger.info(f"Collection '{collection_name}' created successfully.")

        qdrant_client = client
        return client

    except Exception as e:
        logger.error(f"Failed to initialize Qdrant: {e}")
        raise


async def close_qdrant():
    """Close Qdrant client connection."""
    global qdrant_client
    if qdrant_client:
        try:
            qdrant_client.close()
            logger.info("Qdrant connection closed.")
        except Exception as e:
            logger.error(f"Error closing Qdrant connection: {e}")
        finally:
            qdrant_client = None


async def search_documents(
    query_embedding: list[float],
    collection_name: str = "book_content",
    top_k: int = 5,
    score_threshold: float = 0.7,
) -> list[ScoredPoint]:
    """
    Search for similar documents in Qdrant.

    Args:
        query_embedding: Query vector embedding
        collection_name: Name of the collection to search
        top_k: Number of results to return
        score_threshold: Minimum similarity score (0-1)

    Returns:
        List of scored points with document chunks

    Raises:
        Exception: If search fails or client not initialized
    """
    if not qdrant_client:
        raise Exception("Qdrant client not initialized. Call init_qdrant() first.")

    try:
        logger.info(f"Searching Qdrant collection '{collection_name}' for top {top_k} results...")

        results = qdrant_client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k,
            score_threshold=score_threshold,
            with_payload=True,
        )

        logger.info(f"Found {len(results)} results with score >= {score_threshold}")
        return results

    except Exception as e:
        logger.error(f"Error searching Qdrant: {e}")
        raise


async def upsert_documents(
    chunks: list[dict],
    embeddings: list[list[float]],
    collection_name: str = "book_content",
) -> int:
    """
    Insert or update document chunks in Qdrant.

    Args:
        chunks: List of document chunk dictionaries with metadata
        embeddings: List of vector embeddings corresponding to chunks
        collection_name: Name of the collection

    Returns:
        Number of chunks upserted

    Raises:
        Exception: If upsert fails or client not initialized
    """
    if not qdrant_client:
        raise Exception("Qdrant client not initialized. Call init_qdrant() first.")

    if len(chunks) != len(embeddings):
        raise ValueError(f"Mismatch: {len(chunks)} chunks but {len(embeddings)} embeddings")

    try:
        logger.info(f"Upserting {len(chunks)} chunks to Qdrant collection '{collection_name}'...")

        # Create PointStruct objects
        points = [
            PointStruct(
                id=i,  # Use sequential IDs (can use UUID for production)
                vector=embeddings[i],
                payload=chunks[i],  # Store metadata as payload
            )
            for i in range(len(chunks))
        ]

        # Upsert in batches of 100 for efficiency
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i : i + batch_size]
            qdrant_client.upsert(collection_name=collection_name, points=batch)
            logger.info(f"Upserted batch {i // batch_size + 1} ({len(batch)} points)")

        logger.info(f"Successfully upserted {len(chunks)} chunks to Qdrant.")
        return len(chunks)

    except Exception as e:
        logger.error(f"Error upserting to Qdrant: {e}")
        raise


async def get_collection_info(collection_name: str = "book_content") -> dict:
    """
    Get information about a Qdrant collection.

    Args:
        collection_name: Name of the collection

    Returns:
        Dictionary with collection stats

    Raises:
        Exception: If client not initialized
    """
    if not qdrant_client:
        raise Exception("Qdrant client not initialized.")

    try:
        collection_info = qdrant_client.get_collection(collection_name)
        return {
            "name": collection_name,
            "vectors_count": collection_info.vectors_count,
            "points_count": collection_info.points_count,
            "status": collection_info.status,
        }
    except Exception as e:
        logger.error(f"Error getting collection info: {e}")
        raise
