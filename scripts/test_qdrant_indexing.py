#!/usr/bin/env python3
"""
Test script to verify Qdrant indexing works with a small subset of chunks
"""
import json
import time
import os
from pathlib import Path

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct
import cohere

def test_qdrant_indexing():
    # Load a small subset of chunks
    chunks_path = Path("../output/chunks.json")
    if not chunks_path.exists():
        print(f"Input file {chunks_path} does not exist")
        return False

    with open(chunks_path, 'r', encoding='utf-8') as f:
        all_chunks = json.load(f)

    # Take only first 3 chunks to test
    test_chunks = all_chunks[:3]
    print(f"Testing with {len(test_chunks)} chunks out of {len(all_chunks)} total")

    # Initialize clients
    qdrant_url = os.getenv("QDRANT_URL", "https://bfe4b6c0-19f8-4de4-97c8-78492b3febbc.europe-west3-0.gcp.cloud.qdrant.io:6333")
    qdrant_api_key = os.getenv("QDRANT_API_KEY", "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.dXH5uYLXEqY_GZzASCdiy6fo_TIExtYpJugM8GR10V0")
    cohere_api_key = os.getenv("COHERE_API_KEY", "18NrUoBFLxwMLCYa1Ituo8EVMAixXKelYK8OfSSm")

    qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    cohere_client = cohere.Client(cohere_api_key)

    collection_name = "physical_ai_book_chunks"

    try:
        # Check if collection exists, create if it doesn't
        collections = qdrant_client.get_collections()
        existing_collections = [c.name for c in collections.collections]

        if collection_name not in existing_collections:
            # Get embedding dimensions from Cohere
            sample_embedding = cohere_client.embed(
                texts=["Sample text for embedding dimension detection"],
                model="embed-english-v3.0",
                input_type="search_document"
            ).embeddings[0]

            vector_size = len(sample_embedding)
            print(f"Using vector size: {vector_size}")

            # Create collection
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(
                    size=vector_size,
                    distance=Distance.COSINE
                )
            )
            print(f"Collection '{collection_name}' created successfully")
        else:
            print(f"Collection '{collection_name}' already exists")

        # Generate embeddings for test chunks one by one to avoid rate limits
        print("Generating embeddings for test chunks...")
        points = []
        for i, chunk in enumerate(test_chunks):
            print(f"Processing chunk {i+1}/{len(test_chunks)}")

            # Generate embedding for single chunk
            response = cohere_client.embed(
                texts=[chunk['content']],
                model="embed-english-v3.0",
                input_type="search_document"
            )
            embedding = response.embeddings[0]

            # Create point
            point = PointStruct(
                id=chunk['id'],
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

            # Add a small delay to avoid rate limits
            time.sleep(2)

        # Upsert the points
        qdrant_client.upsert(
            collection_name=collection_name,
            points=points
        )

        print(f"Successfully indexed {len(points)} test chunks to Qdrant")

        # Verify by counting points
        count_result = qdrant_client.count(collection_name=collection_name)
        print(f"Total points in collection after indexing: {count_result.count}")

        return True

    except Exception as e:
        print(f"Error during test indexing: {str(e)}")
        return False

if __name__ == "__main__":
    success = test_qdrant_indexing()
    if success:
        print("Test indexing completed successfully!")
    else:
        print("Test indexing failed.")