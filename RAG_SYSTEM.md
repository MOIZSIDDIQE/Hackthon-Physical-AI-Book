# Physical AI & Humanoid Robotics Book - RAG System

This project implements a Retrieval-Augmented Generation (RAG) chatbot for the Physical AI & Humanoid Robotics Book, allowing users to ask questions about the content and receive AI-powered responses based on the book's documentation.

## Architecture Overview

The RAG system consists of three main components:

1. **Data Ingestion Pipeline**: Processes documentation files and prepares them for vector storage
2. **Vector Database**: Qdrant-based storage for document embeddings
3. **API & UI**: Express.js API server and Docusaurus UI component

## Setup Instructions

### 1. Install Dependencies

```bash
# Install main Docusaurus project dependencies
cd physical-ai-book
npm install

# Install API server dependencies
cd api
npm install
```

### 2. Environment Configuration

Create a `.env` file in the `physical-ai-book` directory based on `.env.example`:

```bash
cp .env.example .env
```

Update the `.env` file with your API keys:

```env
# Required API Keys
QDRANT_URL=http://localhost:6333  # Or your Qdrant cloud URL
QDRANT_API_KEY=your_qdrant_api_key_here
COHERE_API_KEY=your_cohere_api_key_here

# Optional Configuration
PORT=3001
NODE_ENV=development
```

### 3. Process Documentation

Run the ingestion script to process all documentation files:

```bash
cd ../scripts  # From api directory
pip install -r requirements.txt

# Run the ingestion script
python ingest_docs.py --docs-path ../physical-ai-book/docs --output ../output/chunks.json
```

### 4. Set up Vector Database

Index the processed documentation in Qdrant:

```bash
# Make sure Qdrant is running (default: http://localhost:6333)
# Then run:
python qdrant_setup.py --input-file ../output/chunks.json --cohere-api-key YOUR_COHERE_KEY
```

### 5. Start Services

Run both the Docusaurus site and API server:

```bash
# Terminal 1: Start Docusaurus site
cd physical-ai-book
npm run dev

# Terminal 2: Start API server
cd physical-ai-book/api
npm run dev
```

## Alternative: Run Both Services Simultaneously

If you installed `concurrently` (included in dev dependencies), you can run both services from the main directory:

```bash
cd physical-ai-book
npm run dev  # This runs both the Docusaurus site and API server
```

## Usage

1. Visit your Docusaurus site (typically http://localhost:3000)
2. Look for the floating "AI" chat button in the bottom-right corner
3. Click to open the chat interface
4. Ask questions about Physical AI & Humanoid Robotics topics
5. View source citations for retrieved information

## Technical Details

### Chunking Strategy
- Chunk size: 400 words
- Overlap: 50 words
- Maintains semantic boundaries by splitting on sentence boundaries

### Vector Database
- Qdrant collection: `physical_ai_book_chunks`
- Vector size: Determined by Cohere embedding model
- Distance metric: Cosine similarity
- Metadata stored: part, chapter, section, file_path, heading_title

### Embeddings
- Model: Cohere `embed-english-v3.0`
- Input type: `search_document` for content, `search_query` for queries

### API Endpoints

- `POST /api/chat`: Main chat endpoint
  - Request: `{ message: string, history?: [], sessionId?: string }`
  - Response: `{ response: string, sources: [], query: string }`

- `GET /health`: Health check
- `GET /api/sources`: Source information

## Development

### Adding New Documentation
When new documentation is added to the `docs/` directory:

1. Re-run the ingestion script
2. Re-index in Qdrant (use `--recreate` flag to replace existing collection)

### Customization

The chat interface can be customized by modifying:
- `src/components/ChatBot/ChatBot.js` - Main component logic
- `src/components/ChatBot/ChatBot.css` - Styling

## Troubleshooting

### Common Issues

1. **API Connection Issues**: Ensure both Docusaurus and API server are running
2. **CORS Issues**: The API server includes CORS middleware by default
3. **Qdrant Connection**: Verify Qdrant is running and credentials are correct
4. **Embedding API**: Ensure Cohere API key is valid and has sufficient quota

### Testing the API

You can test the API directly:

```bash
curl -X POST http://localhost:3001/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is Physical AI?"}'
```

## Security Considerations

- API keys are loaded from environment variables
- Input validation is performed on all requests
- Rate limiting should be implemented in production
- Consider implementing user authentication for production use