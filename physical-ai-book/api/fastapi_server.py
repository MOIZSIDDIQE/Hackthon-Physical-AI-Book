from fastapi import FastAPI, HTTPException, Request
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import os
import asyncio
import logging
from dotenv import load_dotenv
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Chat API",
    description="RAG Chat API for Physical AI & Humanoid Robotics Book",
    version="1.0.0"
)

# Initialize clients
gemini_client = None
cohere_client = None
qdrant_client = None

try:
    # Initialize Gemini client
    genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
    gemini_client = genai.GenerativeModel('gemini-2.5-flash')

    # Initialize Cohere client
    cohere_client = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))

    # Initialize Qdrant client
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    logger.info("Clients initialized successfully")
except Exception as e:
    logger.error(f"Error initializing clients: {e}")
    raise

# Pydantic models
class ChatRequest(BaseModel):
    message: str
    history: Optional[List[Dict[str, str]]] = []
    sessionId: Optional[str] = None

class SourceInfo(BaseModel):
    part: Optional[str] = ""
    chapter: Optional[str] = ""
    section: Optional[str] = ""
    file_path: Optional[str] = ""
    heading_title: Optional[str] = ""
    score: Optional[float] = 0.0

class ChatResponse(BaseModel):
    response: str
    sources: List[SourceInfo]
    query: str

def detect_intent(message: str) -> str:
    """Detect the intent of the user message"""
    trimmed_message = message.strip().lower()

    # GREETING intent
    greeting_patterns = ['hi', 'hello', 'hey', 'yo', 'greetings', 'good morning', 'good afternoon', 'good evening']
    for pattern in greeting_patterns:
        if pattern in trimmed_message:
            return 'GREETING'

    # META intent
    meta_patterns = ['what can you do', 'how does this work', 'what is this', 'who are you', 'help', 'what is this site', 'what is this chatbot', 'what is this about']
    for pattern in meta_patterns:
        if pattern in trimmed_message:
            return 'META'

    # UNCLEAR intent
    if len(trimmed_message.split()) < 3:
        return 'UNCLEAR'

    # BOOK-RELATED TECHNICAL QUESTION intent
    technical_terms = ['physical ai', 'humanoid', 'robot', 'perception', 'motion planning', 'control', 'simulation', 'kinematics', 'dynamics', 'locomotion', 'manipulation', 'navigation', 'sensors', 'actuators', 'ai', 'machine learning', 'neural networks', 'computer vision', 'robotics']
    for term in technical_terms:
        if term in trimmed_message:
            return 'BOOK_TECHNICAL'

    # If no clear technical terms but seems like a question
    question_indicators = ['?', 'what', 'how', 'why', 'when', 'where', 'who']
    if any(indicator in trimmed_message for indicator in question_indicators):
        return 'BOOK_TECHNICAL'

    return 'UNCLEAR'

async def perform_vector_search(query: str) -> List[Dict[str, Any]]:
    """Perform vector search in Qdrant"""
    try:
        # Generate embedding for the query using Cohere
        response = cohere_client.embed(
            texts=[query],
            model='embed-english-v3.0',
            input_type='search_query'
        )

        query_embedding = response.embeddings[0]

        # Search in Qdrant
        search_results = qdrant_client.search(
            collection_name='physical_ai_book_chunks',
            query_vector=query_embedding,
            limit=5,
            with_payload=True
        )

        return search_results
    except Exception as e:
        logger.error(f"Vector search error: {e}")
        return []

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "OK",
        "timestamp": "2025-12-31T00:00:00Z",
        "services": {
            "qdrant": "connected" if qdrant_client else "not configured",
            "cohere": "configured" if cohere_client else "not configured",
            "gemini": "configured" if gemini_client else "not configured"
        }
    }

@app.post("/api/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """Chat endpoint that handles user queries"""
    message = request.message

    if not message or not isinstance(message, str):
        raise HTTPException(status_code=400, detail="Message is required and must be a string")

    logger.info(f"Received chat request: {message[:50]}...")

    # Detect intent
    intent = detect_intent(message)
    logger.info(f"Detected intent: {intent}")

    # Handle non-technical intents without RAG
    if intent == 'GREETING':
        return ChatResponse(
            response="Hello 👋\n\nYou can ask me questions about perception, motion planning, robot control, simulation, or any chapter of the Physical AI book.",
            sources=[],
            query=message
        )

    if intent == 'META':
        return ChatResponse(
            response="I'm your Physical AI & Humanoid Robotics assistant. I can answer questions about the book content, including topics like perception, control systems, motion planning, simulation, and more. Ask me anything related to the Physical AI & Humanoid Robotics book!",
            sources=[],
            query=message
        )

    if intent == 'UNCLEAR':
        return ChatResponse(
            response="I couldn't find a clear question.\n\nPlease ask something specific related to Physical AI or humanoid robotics.",
            sources=[],
            query=message
        )

    # Only proceed with RAG for BOOK_TECHNICAL intent
    if intent == 'BOOK_TECHNICAL':
        # Perform vector search
        search_results = await perform_vector_search(message)

        # Apply similarity threshold
        filtered_results = [result for result in search_results if result.score >= 0.50]

        if not filtered_results:
            return ChatResponse(
                response="This topic is not clearly covered in the book.",
                sources=[],
                query=message
            )

        # Prepare context from search results
        context = "\n\n".join([result.payload.get('content', '') for result in filtered_results])
        sources = [
            SourceInfo(
                part=result.payload.get('part', ''),
                chapter=result.payload.get('chapter', ''),
                section=result.payload.get('section', ''),
                file_path=result.payload.get('file_path', ''),
                heading_title=result.payload.get('heading_title', ''),
                score=result.score
            )
            for result in filtered_results
        ]

        # Generate response using Gemini
        prompt = f"""You are an expert assistant for the "Physical AI & Humanoid Robotics Book".
        Answer the user's question based only on the provided context from the book.
        If the answer is not available in the context, respond with "Not covered in this book".

        Context from the book:
        {context}

        User question: {message}

        Answer and provide citations to the specific parts of the book where you found the information:"""

        try:
            # Generate response with retry logic
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    response = gemini_client.generate_content(prompt)
                    generated_text = response.text.strip()
                    break
                except Exception as e:
                    if attempt == max_retries - 1:
                        raise e
                    await asyncio.sleep(2 ** attempt)  # Exponential backoff

            # Check if the model says it's not in the book
            if ('not covered in this book' in generated_text.lower() or
                'not found in the context' in generated_text.lower() or
                'no relevant information' in generated_text.lower()):
                return ChatResponse(
                    response="Not covered in this book",
                    sources=[],
                    query=message
                )

            return ChatResponse(
                response=generated_text,
                sources=sources,
                query=message
            )
        except Exception as e:
            logger.error(f"Gemini API error: {e}")
            # Fallback response
            relevant_content = context[:800] if context else "No relevant content found"
            fallback_text = f"Based on the Physical AI & Humanoid Robotics book:\n\n{relevant_content}...\n\nFor more details about \"{message}\", please refer to the specific chapters in the book."

            return ChatResponse(
                response=fallback_text,
                sources=sources,
                query=message
            )

    # Safety fallback
    return ChatResponse(
        response="I couldn't find a clear question.\n\nPlease ask something specific related to Physical AI or humanoid robotics.",
        sources=[],
        query=message
    )

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)