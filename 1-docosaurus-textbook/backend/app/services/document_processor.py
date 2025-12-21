"""Document processing service for markdown ingestion and embedding."""

import logging
import re
from pathlib import Path
from typing import Optional
import tiktoken
from openai import AsyncOpenAI

from app.config import get_settings
from app.schemas.chat import DocumentChunk

logger = logging.getLogger(__name__)


def count_tokens(text: str, model: str = "gpt-4") -> int:
    """Count tokens in text using tiktoken."""
    try:
        encoding = tiktoken.encoding_for_model(model)
        return len(encoding.encode(text))
    except Exception:
        # Fallback: rough estimation (1 token ≈ 4 characters)
        return len(text) // 4


def chunk_text(
    text: str,
    max_tokens: int = 500,
    overlap_tokens: int = 50,
) -> list[str]:
    """
    Split text into chunks with token-based splitting and overlap.

    Args:
        text: Text to split
        max_tokens: Maximum tokens per chunk
        overlap_tokens: Number of tokens to overlap between chunks

    Returns:
        List of text chunks
    """
    # Split by paragraphs first (double newline)
    paragraphs = re.split(r'\n\n+', text)

    chunks = []
    current_chunk = ""
    current_tokens = 0

    for para in paragraphs:
        para = para.strip()
        if not para:
            continue

        para_tokens = count_tokens(para)

        # If single paragraph exceeds max_tokens, split it by sentences
        if para_tokens > max_tokens:
            sentences = re.split(r'(?<=[.!?])\s+', para)
            for sentence in sentences:
                sentence_tokens = count_tokens(sentence)

                if current_tokens + sentence_tokens > max_tokens:
                    if current_chunk:
                        chunks.append(current_chunk.strip())
                        # Keep overlap
                        overlap_text = current_chunk.split()[-overlap_tokens:]
                        current_chunk = " ".join(overlap_text) + " " + sentence
                        current_tokens = count_tokens(current_chunk)
                    else:
                        # Sentence itself is too long, add it anyway
                        chunks.append(sentence.strip())
                        current_chunk = ""
                        current_tokens = 0
                else:
                    current_chunk += " " + sentence
                    current_tokens += sentence_tokens
        else:
            # Add paragraph to current chunk
            if current_tokens + para_tokens > max_tokens:
                if current_chunk:
                    chunks.append(current_chunk.strip())
                    # Keep overlap
                    overlap_text = current_chunk.split()[-overlap_tokens:]
                    current_chunk = " ".join(overlap_text) + "\n\n" + para
                    current_tokens = count_tokens(current_chunk)
                else:
                    current_chunk = para
                    current_tokens = para_tokens
            else:
                current_chunk += "\n\n" + para if current_chunk else para
                current_tokens += para_tokens

    # Add final chunk
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    logger.info(f"Split text into {len(chunks)} chunks (max {max_tokens} tokens, {overlap_tokens} overlap)")
    return chunks


def extract_metadata(file_path: Path) -> dict:
    """
    Extract metadata from markdown file path.

    Args:
        file_path: Path to markdown file

    Returns:
        Dictionary with metadata (module, title, etc.)
    """
    parts = file_path.parts
    metadata = {
        "file_path": str(file_path),
        "module": None,
        "title": None,
    }

    # Extract module from path (e.g., docs/module1/...)
    if "module1" in parts:
        metadata["module"] = "Module 1: ROS 2 Fundamentals"
    elif "module2" in parts:
        metadata["module"] = "Module 2: Simulation & Perception"
    elif len(parts) > 1 and parts[-2].startswith("module"):
        metadata["module"] = f"Module {parts[-2][-1]}"

    # Extract title from filename
    filename = file_path.stem.replace("-", " ").title()
    metadata["title"] = filename

    return metadata


async def process_markdown_file(
    file_path: Path,
    max_tokens: int = 500,
    overlap_tokens: int = 50,
) -> list[DocumentChunk]:
    """
    Process a single markdown file into document chunks.

    Args:
        file_path: Path to markdown file
        max_tokens: Maximum tokens per chunk
        overlap_tokens: Overlap between chunks

    Returns:
        List of DocumentChunk objects
    """
    logger.info(f"Processing file: {file_path}")

    try:
        # Read file content
        content = file_path.read_text(encoding="utf-8")

        # Extract metadata
        metadata = extract_metadata(file_path)

        # Remove frontmatter if present (Docusaurus uses YAML frontmatter)
        content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

        # Chunk the content
        chunks_text = chunk_text(content, max_tokens, overlap_tokens)

        # Create DocumentChunk objects
        document_chunks = []
        for idx, chunk_text in enumerate(chunks_text):
            chunk = DocumentChunk(
                id=f"{file_path.stem}-chunk-{idx:03d}",
                content=chunk_text,
                file_path=str(file_path),
                module=metadata.get("module"),
                title=metadata.get("title"),
                heading=None,  # Could extract section heading if needed
                chunk_index=idx,
            )
            document_chunks.append(chunk)

        logger.info(f"Created {len(document_chunks)} chunks from {file_path.name}")
        return document_chunks

    except Exception as e:
        logger.error(f"Error processing file {file_path}: {e}")
        return []


async def process_markdown_files(docs_dir: str | Path) -> list[DocumentChunk]:
    """
    Process all markdown files in a directory.

    Args:
        docs_dir: Path to documentation directory

    Returns:
        List of all DocumentChunk objects
    """
    docs_path = Path(docs_dir)

    if not docs_path.exists():
        raise FileNotFoundError(f"Documentation directory not found: {docs_dir}")

    logger.info(f"Processing markdown files in: {docs_dir}")

    # Find all .md files recursively
    md_files = list(docs_path.rglob("*.md"))
    logger.info(f"Found {len(md_files)} markdown files")

    all_chunks = []
    for md_file in md_files:
        chunks = await process_markdown_file(md_file)
        all_chunks.extend(chunks)

    logger.info(f"Total chunks created: {len(all_chunks)}")
    return all_chunks


async def create_embeddings(chunks: list[DocumentChunk]) -> list[list[float]]:
    """
    Create embeddings for document chunks using Gemini.

    Args:
        chunks: List of DocumentChunk objects

    Returns:
        List of embedding vectors
    """
    settings = get_settings()

    if not settings.gemini_api_key:
        raise ValueError("GEMINI_API_KEY not configured in environment")

    client = AsyncOpenAI(
        api_key=settings.gemini_api_key,
        base_url=settings.gemini_base_url,
    )

    logger.info(f"Creating embeddings for {len(chunks)} chunks using {settings.gemini_embedding_model}")

    try:
        # Extract text content
        texts = [chunk.content for chunk in chunks]

        # Create embeddings in batches (Gemini allows up to 2048 inputs)
        batch_size = 100
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i : i + batch_size]
            logger.info(f"Processing batch {i // batch_size + 1}/{(len(texts) + batch_size - 1) // batch_size}")

            response = await client.embeddings.create(
                model=settings.gemini_embedding_model,
                input=batch,
            )

            batch_embeddings = [item.embedding for item in response.data]
            all_embeddings.extend(batch_embeddings)

        logger.info(f"Successfully created {len(all_embeddings)} embeddings")
        return all_embeddings

    except Exception as e:
        logger.error(f"Error creating embeddings: {e}")
        raise


async def ingest_documents(
    docs_dir: str | Path,
    collection_name: str = "book_content",
) -> int:
    """
    Complete document ingestion pipeline.

    1. Process markdown files into chunks
    2. Create embeddings
    3. Upsert to Qdrant

    Args:
        docs_dir: Path to documentation directory
        collection_name: Qdrant collection name

    Returns:
        Number of chunks ingested
    """
    from app.services.vector_store import upsert_documents

    logger.info("Starting document ingestion pipeline...")

    # Process markdown files
    chunks = await process_markdown_files(docs_dir)

    if not chunks:
        logger.warning("No chunks created. Aborting ingestion.")
        return 0

    # Create embeddings
    embeddings = await create_embeddings(chunks)

    # Convert chunks to dictionaries for Qdrant payload
    chunks_dict = [chunk.model_dump() for chunk in chunks]

    # Upsert to Qdrant
    count = await upsert_documents(chunks_dict, embeddings, collection_name)

    logger.info(f"✅ Ingestion complete! {count} chunks ingested to '{collection_name}'")
    return count
