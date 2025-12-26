#!/usr/bin/env python3
"""
Qdrant Vector Database Setup for Physical AI & Humanoid Robotics Book RAG System

This script creates the collection schema in Qdrant and handles vector operations.
"""
import os
import uuid
from typing import List, Dict, Any
from dataclasses import dataclass
import logging

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct
import cohere

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

@dataclass
class DocumentChunk:
    """Represents a chunk of documentation with metadata"""
    id: str
    content: str
    metadata: Dict[str, str]

class QdrantSetup:
    def __init__(self, qdrant_url: str = None, qdrant_api_key: str = None, cohere_api_key: str = None):
        # Initialize Qdrant client
        if qdrant_url and qdrant_api_key:
            self.client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        elif qdrant_url:
            self.client = QdrantClient(url=qdrant_url)
        else:
            # For local Qdrant
            self.client = QdrantClient(host="localhost", port=6333)

        # Initialize Cohere client for embeddings
        if cohere_api_key:
            self.cohere_client = cohere.Client(cohere_api_key)
        else:
            self.cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))

        # Collection name
        self.collection_name = "physical_ai_book_chunks"

    def create_collection(self) -> bool:
        """Create the collection in Qdrant with appropriate vector configuration"""
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            existing_collections = [c.name for c in collections.collections]

            if self.collection_name in existing_collections:
                logger.info(f"Collection '{self.collection_name}' already exists")
                return True

            # Get embedding dimensions from Cohere
            # Use a sample text to determine embedding size
            sample_embedding = self.cohere_client.embed(
                texts=["Sample text for embedding dimension detection"],
                model="embed-english-v3.0",
                input_type="search_document"
            ).embeddings[0]

            vector_size = len(sample_embedding)
            logger.info(f"Using vector size: {vector_size}")

            # Create collection with vector configuration
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=vector_size,
                    distance=Distance.COSINE  # Cosine similarity as requested
                )
            )

            logger.info(f"Collection '{self.collection_name}' created successfully")
            return True

        except Exception as e:
            logger.error(f"Error creating collection: {str(e)}")
            return False

    def embed_chunks(self, chunks: List[Dict[str, Any]]) -> List[List[float]]:
        """Generate embeddings for document chunks using Cohere"""
        try:
            # Extract content from chunks
            texts = [chunk['content'] for chunk in chunks]

            # Generate embeddings using Cohere
            response = self.cohere_client.embed(
                texts=texts,
                model="embed-english-v3.0",
                input_type="search_document"
            )

            embeddings = [embedding for embedding in response.embeddings]
            logger.info(f"Generated {len(embeddings)} embeddings")

            return embeddings

        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}")
            raise

    def index_chunks(self, chunks: List[Dict[str, Any]], batch_size: int = 100) -> bool:
        """Index document chunks in Qdrant with batching"""
        try:
            logger.info(f"Starting indexing of {len(chunks)} chunks...")

            # Generate embeddings for all chunks
            embeddings = self.embed_chunks(chunks)

            # Prepare points for insertion
            points = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                point = PointStruct(
                    id=chunk['id'],  # Use the existing ID from the chunk
                    vector=embedding,
                    payload={
                        "content": chunk['content'],
                        "part": chunk.get('part', ''),
                        "chapter": chunk.get('chapter', ''),
                        "section": chunk.get('section', ''),
                        "file_path": chunk.get('file_path', ''),
                        "heading_title": chunk.get('heading_title', ''),
                        "chunk_index": chunk.get('chunk_index', ''),
                        "total_chunks": chunk.get('total_chunks', '')
                    }
                )
                points.append(point)

                # Insert in batches
                if len(points) >= batch_size or i == len(chunks) - 1:
                    self.client.upsert(
                        collection_name=self.collection_name,
                        points=points
                    )
                    logger.info(f"Upserted batch of {len(points)} points")
                    points = []  # Reset for next batch

            logger.info(f"Successfully indexed {len(chunks)} chunks in Qdrant")
            return True

        except Exception as e:
            logger.error(f"Error indexing chunks: {str(e)}")
            return False

    def search(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """Search for relevant chunks based on the query"""
        try:
            # Generate embedding for the query
            query_embedding = self.cohere_client.embed(
                texts=[query],
                model="embed-english-v3.0",
                input_type="search_query"
            ).embeddings[0]

            # Perform search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True
            )

            # Format results
            results = []
            for result in search_results:
                formatted_result = {
                    'id': result.id,
                    'content': result.payload.get('content', ''),
                    'part': result.payload.get('part', ''),
                    'chapter': result.payload.get('chapter', ''),
                    'section': result.payload.get('section', ''),
                    'file_path': result.payload.get('file_path', ''),
                    'heading_title': result.payload.get('heading_title', ''),
                    'score': result.score
                }
                results.append(formatted_result)

            logger.info(f"Search completed, returned {len(results)} results")
            return results

        except Exception as e:
            logger.error(f"Error during search: {str(e)}")
            return []

    def count_points(self) -> int:
        """Get the total number of indexed points"""
        try:
            count_result = self.client.count(collection_name=self.collection_name)
            return count_result.count
        except Exception as e:
            logger.error(f"Error counting points: {str(e)}")
            return 0

def main():
    """Main function to set up Qdrant and index documents"""
    import argparse
    import json
    from pathlib import Path

    parser = argparse.ArgumentParser(description='Setup Qdrant for Physical AI Book RAG')
    parser.add_argument('--input-file', default='output/chunks.json', help='Input file with document chunks')
    parser.add_argument('--qdrant-url', help='Qdrant URL (optional, defaults to localhost)')
    parser.add_argument('--qdrant-api-key', help='Qdrant API key (optional)')
    parser.add_argument('--cohere-api-key', help='Cohere API key')
    parser.add_argument('--batch-size', type=int, default=100, help='Batch size for indexing')
    parser.add_argument('--recreate', action='store_true', help='Recreate collection if it exists')

    args = parser.parse_args()

    # Initialize Qdrant setup
    qdrant_setup = QdrantSetup(
        qdrant_url=args.qdrant_url,
        qdrant_api_key=args.qdrant_api_key,
        cohere_api_key=args.cohere_api_key
    )

    # Load document chunks from file
    input_path = Path(args.input_file)
    if not input_path.exists():
        logger.error(f"Input file {args.input_file} does not exist")
        return

    with open(input_path, 'r', encoding='utf-8') as f:
        chunks = json.load(f)

    logger.info(f"Loaded {len(chunks)} chunks from {args.input_file}")

    # Create or recreate collection
    if args.recreate:
        logger.info("Recreating collection...")
        # Delete collection if it exists
        try:
            qdrant_setup.client.delete_collection(qdrant_setup.collection_name)
            logger.info("Old collection deleted")
        except:
            logger.info("Collection didn't exist, proceeding with creation")

    # Create collection
    if not qdrant_setup.create_collection():
        logger.error("Failed to create collection")
        return

    # Index the chunks
    success = qdrant_setup.index_chunks(chunks, batch_size=args.batch_size)
    if success:
        total_points = qdrant_setup.count_points()
        logger.info(f"Indexing completed successfully. Total indexed points: {total_points}")
    else:
        logger.error("Indexing failed")

if __name__ == "__main__":
    main()