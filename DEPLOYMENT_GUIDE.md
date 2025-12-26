# Physical AI & Humanoid Robotics Book - Complete RAG System

## Overview

This project implements a full-stack RAG (Retrieval-Augmented Generation) chatbot system for the Physical AI & Humanoid Robotics Book. The system allows users to ask questions about the book content and receive AI-generated responses based on the documentation.

## System Architecture

The RAG system consists of:

1. **Documentation Processing Pipeline** - Python scripts to process .md/.mdx files
2. **Vector Database** - Qdrant for storing document embeddings
3. **AI Integration** - Cohere for embeddings and text generation
4. **API Server** - Express.js server for handling chat requests
5. **Frontend UI** - Docusaurus-integrated chat component with floating button

## Setup & Deployment Instructions

### 1. Prerequisites

- Node.js (v20+)
- Python (v3.8+)
- Qdrant (local or cloud instance)
- Cohere API key

### 2. Installation

```bash
# Clone and navigate to the project
cd physical-ai-book

# Install main Docusaurus dependencies
npm install

# Install API server dependencies
cd api
npm install
cd ..

# Install Python dependencies for ingestion
cd ../scripts
pip install -r requirements.txt
```

### 3. Environment Configuration

Create `.env` file in `physical-ai-book/`:

```env
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key
PORT=3001
NODE_ENV=development
```

### 4. Process Documentation

```bash
cd scripts

# Process all documentation files
python ingest_docs.py --docs-path ../physical-ai-book/docs --output ../output/chunks.json

# Index in Qdrant
python qdrant_setup.py --input-file ../output/chunks.json --cohere-api-key YOUR_COHERE_KEY
```

### 5. Start the System

```bash
# Terminal 1: Start Docusaurus site with API
cd physical-ai-book
npm run dev

# The site runs on http://localhost:3000
# The API runs on http://localhost:3001
```

## Features

- **Floating AI Assistant**: Bottom-right corner chat button
- **Contextual Responses**: Answers based only on book content
- **Source Citations**: Shows which parts of the book were used
- **Suggested Questions**: Helpful prompts for users
- **Dark/Light Mode**: Adapts to Docusaurus theme
- **Professional UI**: MIT/Stanford-level academic design
- **Responsive Design**: Works on mobile and desktop

## Technical Specifications

- **Chunking**: 400-word chunks with 50-word overlap
- **Embeddings**: Cohere embed-english-v3.0 model
- **Vector Storage**: Qdrant with cosine similarity
- **Retrieval**: Top-5 results with relevance scoring
- **Generation**: Cohere Command-R+ for grounded responses
- **Frontend**: React component integrated with Docusaurus

## API Endpoints

- `POST /api/chat` - Main chat interface
- `GET /health` - Health check
- `GET /api/sources` - Source information

## Error Handling

- Graceful fallback for API failures
- "Not covered in this book" responses when no relevant content found
- Connection error handling
- Rate limiting ready for production

## Production Deployment

1. Deploy Docusaurus site to hosting platform (Vercel, Netlify, etc.)
2. Deploy API server to Node.js hosting (VPS, Heroku, etc.)
3. Set up cloud Qdrant instance
4. Configure environment variables for production
5. Update frontend to use production API URL

## Maintenance

- To update documentation: re-run ingestion and indexing scripts
- Monitor API usage and embedding quotas
- Regular backup of Qdrant collections
- Update dependencies periodically

## Security Considerations

- API keys stored in environment variables
- Input validation on all endpoints
- Rate limiting should be implemented in production
- Consider adding authentication for production use
- Validate and sanitize all user inputs

## Performance

- Embeddings cached for faster retrieval
- Efficient chunking strategy minimizes token usage
- Optimized React component with client-side rendering only
- Asynchronous API calls prevent UI blocking

This RAG system provides a state-of-the-art interface for exploring the Physical AI & Humanoid Robotics Book content with AI assistance while maintaining academic rigor and professional presentation.