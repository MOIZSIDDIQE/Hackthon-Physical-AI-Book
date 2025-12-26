#!/usr/bin/env python3
"""
RAG Documentation Ingestion Script for Physical AI & Humanoid Robotics Book

This script processes all .md and .mdx files in the docs directory,
chunks them according to the specified strategy, and stores them in Qdrant.
"""
import os
import re
import json
import logging
from pathlib import Path
from typing import List, Dict, Tuple
from dataclasses import dataclass
import hashlib

import markdown
from bs4 import BeautifulSoup
import tiktoken

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

@dataclass
class DocumentChunk:
    """Represents a chunk of documentation with metadata"""
    id: str
    content: str
    metadata: Dict[str, str]
    part: str
    chapter: str
    section: str
    file_path: str
    heading_title: str

class DocIngestor:
    def __init__(self, docs_path: str, chunk_size: int = 400, overlap: int = 50):
        self.docs_path = Path(docs_path)
        self.chunk_size = chunk_size  # words
        self.overlap = overlap  # words
        self.enc = tiktoken.encoding_for_model("gpt-3.5-turbo")

        # Pattern to extract part, chapter, and section from file path
        self.path_pattern = re.compile(r'part-(\d+)-([^/\\]+)[/\\]chapter-(\d+)[/\\](\d+\.\d+)-(.+)\.md')

    def extract_metadata_from_path(self, file_path: str) -> Tuple[str, str, str, str, str]:
        """Extract metadata from file path"""
        path_obj = Path(file_path)
        relative_path = str(path_obj.relative_to(self.docs_path))

        # Try to match the path pattern
        match = self.path_pattern.search(relative_path.replace('\\', '/'))
        if match:
            part_num = match.group(1)
            part_name = match.group(2).replace('-', ' ').title()
            chapter_num = match.group(3)
            section_num = match.group(4)
            section_name = match.group(5).replace('-', ' ').title()

            part = f"Part {part_num}: {part_name}"
            chapter = f"Chapter {chapter_num}: {section_name.split()[0] if section_name.split() else 'General'}"  # Simplified
            section = f"{section_num} {section_name}"
        else:
            # Fallback if pattern doesn't match
            parts = relative_path.split('/')
            part = parts[0] if len(parts) > 0 else "Unknown Part"
            chapter = parts[1] if len(parts) > 1 else "Unknown Chapter"
            section = parts[2] if len(parts) > 2 else "Unknown Section"

        return part, chapter, section, relative_path, section_name

    def clean_markdown_content(self, content: str) -> str:
        """Clean and extract text content from markdown"""
        # Remove frontmatter if present
        if content.startswith('---'):
            try:
                end_frontmatter = content.find('---', 3)
                if end_frontmatter != -1:
                    content = content[end_frontmatter + 3:].strip()
            except:
                pass

        # Convert markdown to HTML then extract text
        html = markdown.markdown(content)
        soup = BeautifulSoup(html, 'html.parser')

        # Remove code blocks and other elements that shouldn't be indexed
        for element in soup(['code', 'pre', 'script', 'style']):
            element.decompose()

        # Get text content
        text = soup.get_text(separator=' ')

        # Clean up extra whitespace
        text = re.sub(r'\s+', ' ', text).strip()

        return text

    def chunk_text(self, text: str, heading_title: str = "") -> List[str]:
        """Chunk text into approximately chunk_size words with overlap"""
        # Split text into sentences to maintain semantic boundaries
        sentences = re.split(r'[.!?]+\s+', text)

        chunks = []
        current_chunk = []
        current_length = 0

        for sentence in sentences:
            sentence_words = sentence.split()
            sentence_length = len(sentence_words)

            # If adding this sentence would exceed chunk size
            if current_length + sentence_length > self.chunk_size and current_chunk:
                # Add the current chunk to chunks
                chunk_text = ' '.join(current_chunk)
                chunks.append(chunk_text)

                # Start a new chunk with overlap
                if self.overlap > 0:
                    # Calculate how many words from the end of the current chunk to include as overlap
                    overlap_words = []
                    temp_chunk = current_chunk.copy()

                    # Remove words from the beginning until we're within overlap size
                    while len(temp_chunk) > self.overlap:
                        temp_chunk.pop(0)

                    current_chunk = temp_chunk + sentence_words
                    current_length = len(current_chunk)
                else:
                    current_chunk = sentence_words
                    current_length = sentence_length
            else:
                current_chunk.extend(sentence_words)
                current_length += sentence_length

        # Add the final chunk if it has content
        if current_chunk:
            chunk_text = ' '.join(current_chunk)
            chunks.append(chunk_text)

        return chunks

    def process_file(self, file_path: str) -> List[DocumentChunk]:
        """Process a single documentation file"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract metadata from path
            part, chapter, section, relative_path, heading_title = self.extract_metadata_from_path(file_path)

            # Clean and extract text content
            clean_content = self.clean_markdown_content(content)

            # Extract potential heading from content if not provided
            if not heading_title:
                # Look for first heading in content
                lines = content.split('\n')
                for line in lines:
                    if line.strip().startswith('#'):
                        heading_title = line.strip('# ').strip()
                        break

            # Chunk the content
            chunks = self.chunk_text(clean_content, heading_title)

            # Create document chunks
            doc_chunks = []
            for i, chunk in enumerate(chunks):
                # Create a unique ID for this chunk
                chunk_id = hashlib.md5(f"{relative_path}_chunk_{i}_{chunk[:50]}".encode()).hexdigest()

                doc_chunk = DocumentChunk(
                    id=chunk_id,
                    content=chunk,
                    metadata={
                        "part": part,
                        "chapter": chapter,
                        "section": section,
                        "file_path": relative_path,
                        "heading_title": heading_title,
                        "chunk_index": str(i),
                        "total_chunks": str(len(chunks))
                    },
                    part=part,
                    chapter=chapter,
                    section=section,
                    file_path=relative_path,
                    heading_title=heading_title
                )

                doc_chunks.append(doc_chunk)

            logger.info(f"Processed {file_path}: {len(chunks)} chunks created")
            return doc_chunks

        except Exception as e:
            logger.error(f"Error processing file {file_path}: {str(e)}")
            return []

    def process_all_docs(self) -> List[DocumentChunk]:
        """Process all documentation files in the docs directory"""
        all_chunks = []

        # Find all .md and .mdx files
        md_files = list(self.docs_path.rglob("*.md"))
        mdx_files = list(self.docs_path.rglob("*.mdx"))

        all_files = md_files + mdx_files
        logger.info(f"Found {len(all_files)} documentation files to process")

        for file_path in all_files:
            logger.info(f"Processing: {file_path}")
            chunks = self.process_file(str(file_path))
            all_chunks.extend(chunks)

        logger.info(f"Total chunks created: {len(all_chunks)}")
        return all_chunks

def main():
    """Main function to run the ingestion process"""
    import argparse

    parser = argparse.ArgumentParser(description='Ingest documentation for RAG system')
    parser.add_argument('--docs-path', default='physical-ai-book/docs', help='Path to docs directory')
    parser.add_argument('--chunk-size', type=int, default=400, help='Chunk size in words')
    parser.add_argument('--overlap', type=int, default=50, help='Overlap size in words')
    parser.add_argument('--output', default='output/chunks.json', help='Output file for chunks')

    args = parser.parse_args()

    # Create output directory if it doesn't exist
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Initialize the ingestor
    ingestor = DocIngestor(args.docs_path, args.chunk_size, args.overlap)

    # Process all documentation
    logger.info("Starting documentation ingestion process...")
    all_chunks = ingestor.process_all_docs()

    # Save chunks to file
    logger.info(f"Saving {len(all_chunks)} chunks to {args.output}")

    # Convert DocumentChunk objects to dictionaries for JSON serialization
    chunks_data = []
    for chunk in all_chunks:
        chunk_dict = {
            'id': chunk.id,
            'content': chunk.content,
            'metadata': chunk.metadata,
            'part': chunk.part,
            'chapter': chunk.chapter,
            'section': chunk.section,
            'file_path': chunk.file_path,
            'heading_title': chunk.heading_title
        }
        chunks_data.append(chunk_dict)

    with open(args.output, 'w', encoding='utf-8') as f:
        json.dump(chunks_data, f, ensure_ascii=False, indent=2)

    logger.info(f"Ingestion completed. {len(all_chunks)} chunks saved to {args.output}")

if __name__ == "__main__":
    main()