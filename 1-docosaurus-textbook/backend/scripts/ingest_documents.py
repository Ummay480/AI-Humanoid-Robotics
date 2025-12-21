#!/usr/bin/env python3
"""
Document Ingestion Script

Standalone script to ingest Docusaurus markdown files into Qdrant vector database.

Usage:
    python scripts/ingest_documents.py --docs-dir ../frontend/docs
    python scripts/ingest_documents.py --docs-dir /path/to/docs --collection book_content
"""

import asyncio
import argparse
import logging
import sys
from pathlib import Path

# Add parent directory to Python path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.vector_store import init_qdrant, close_qdrant
from app.services.document_processor import ingest_documents
from app.config import get_settings

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


async def main():
    """Main ingestion workflow."""
    parser = argparse.ArgumentParser(
        description="Ingest Docusaurus markdown files into Qdrant vector database"
    )
    parser.add_argument(
        "--docs-dir",
        type=str,
        required=True,
        help="Path to Docusaurus docs directory (e.g., ../frontend/docs)",
    )
    parser.add_argument(
        "--collection",
        type=str,
        default="book_content",
        help="Qdrant collection name (default: book_content)",
    )
    parser.add_argument(
        "--max-tokens",
        type=int,
        default=500,
        help="Maximum tokens per chunk (default: 500)",
    )
    parser.add_argument(
        "--overlap-tokens",
        type=int,
        default=50,
        help="Overlap tokens between chunks (default: 50)",
    )

    args = parser.parse_args()

    # Validate docs directory
    docs_path = Path(args.docs_dir)
    if not docs_path.exists():
        logger.error(f"❌ Documentation directory not found: {args.docs_dir}")
        return 1

    # Check configuration
    settings = get_settings()
    if not settings.openai_api_key:
        logger.error("❌ OPENAI_API_KEY not set in environment!")
        logger.error("Please set it in .env file or export it as an environment variable")
        return 1

    logger.info("=" * 80)
    logger.info("📚 Document Ingestion Pipeline")
    logger.info("=" * 80)
    logger.info(f"Docs Directory: {docs_path.absolute()}")
    logger.info(f"Collection: {args.collection}")
    logger.info(f"Qdrant URL: {settings.qdrant_url}")
    logger.info(f"OpenAI Model: {settings.openai_embedding_model}")
    logger.info(f"Max Tokens: {args.max_tokens}")
    logger.info(f"Overlap Tokens: {args.overlap_tokens}")
    logger.info("=" * 80)

    try:
        # Initialize Qdrant
        logger.info("\n🔌 Step 1: Connecting to Qdrant...")
        await init_qdrant()

        # Ingest documents
        logger.info("\n📥 Step 2: Ingesting documents...")
        count = await ingest_documents(
            docs_dir=docs_path,
            collection_name=args.collection,
        )

        # Success
        logger.info("\n" + "=" * 80)
        logger.info(f"✅ SUCCESS! Ingested {count} document chunks")
        logger.info("=" * 80)
        logger.info("\nYou can now:")
        logger.info("  1. Start the backend server: python run.py")
        logger.info("  2. Test the chat endpoint: curl http://localhost:8000/api/chat/stream")
        logger.info("  3. Use the frontend chat interface")

        return 0

    except Exception as e:
        logger.error(f"\n❌ Ingestion failed: {e}", exc_info=True)
        return 1

    finally:
        # Clean up
        await close_qdrant()


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
