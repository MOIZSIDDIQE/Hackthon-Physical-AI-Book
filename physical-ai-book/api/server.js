#!/usr/bin/env node
/**
 * RAG Chat API Server for Physical AI & Humanoid Robotics Book
 *
 * This API server handles chat requests, performs vector search,
 * and generates responses using Cohere and Qdrant.
 */

const express = require('express');
const cors = require('cors');
const path = require('path');
require('dotenv').config({ path: '../.env' }); // Load from parent directory

// Initialize Qdrant client
const { QdrantClient } = require('@qdrant/js-client-rest');
const cohere = require('cohere-ai');

// Initialize Google Generative AI for Gemini
const { GoogleGenerativeAI } = require('@google/generative-ai');

const app = express();
const PORT = process.env.PORT || 3001;

// Middleware
app.use(cors());
app.use(express.json({ limit: '10mb' }));
app.use(express.urlencoded({ extended: true }));

// Initialize clients
let qdrantClient;
let cohereClient;
let geminiClient;

try {
  // Initialize Qdrant client
  if (process.env.QDRANT_URL && process.env.QDRANT_API_KEY) {
    qdrantClient = new QdrantClient({
      url: process.env.QDRANT_URL,
      apiKey: process.env.QDRANT_API_KEY
    });
  } else {
    // Fallback to local Qdrant if environment variables are not set
    qdrantClient = new QdrantClient({
      host: process.env.QDRANT_HOST || 'localhost',
      port: parseInt(process.env.QDRANT_PORT || '6333'),
    });
  }

  // Initialize Cohere client for embeddings only (v7.x)
  if (!process.env.COHERE_API_KEY) {
    throw new Error('COHERE_API_KEY environment variable is required');
  }
  cohereClient = new cohere.CohereClient({
    token: process.env.COHERE_API_KEY,
  });

  // Initialize Gemini client for text generation
  if (!process.env.GEMINI_API_KEY) {
    throw new Error('GEMINI_API_KEY environment variable is required');
  }
  geminiClient = new GoogleGenerativeAI(process.env.GEMINI_API_KEY);

  console.log('Clients initialized successfully');
} catch (error) {
  console.error('Error initializing clients:', error.message);
  process.exit(1);
}

// Health check endpoint
app.get('/health', (req, res) => {
  res.status(200).json({
    status: 'OK',
    timestamp: new Date().toISOString(),
    services: {
      qdrant: 'connected',
      cohere: 'configured',
      gemini: 'configured'
    }
  });
});

// Chat endpoint
app.post('/api/chat', async (req, res) => {
  try {
    const { message, history = [], sessionId = null } = req.body;

    if (!message || typeof message !== 'string') {
      return res.status(400).json({
        error: 'Message is required and must be a string'
      });
    }

    console.log(`Received chat request: "${message.substring(0, 50)}..."`);

    // 1. Perform vector search in Qdrant
    const searchResults = await performVectorSearch(message);

    if (searchResults.length === 0) {
      return res.status(200).json({
        response: "I couldn't find relevant information in the Physical AI & Humanoid Robotics book to answer your question. Please try rephrasing or ask about a different topic covered in the book.",
        sources: [],
        query: message
      });
    }

    // 2. Prepare context from search results
    const context = searchResults.map(result => result.payload.content).join('\n\n');
    const sources = searchResults.map(result => ({
      part: result.payload.part || '',
      chapter: result.payload.chapter || '',
      section: result.payload.section || '',
      file_path: result.payload.file_path || '',
      heading_title: result.payload.heading_title || '',
      score: result.score
    }));

    // 3. Generate response using Gemini
    const prompt = `You are an expert assistant for the "Physical AI & Humanoid Robotics Book".
    Answer the user's question based only on the provided context from the book.
    If the answer is not available in the context, respond with "Not covered in this book".

    Context from the book:
    ${context}

    User question: ${message}

    Answer and provide citations to the specific parts of the book where you found the information:`;

    // Use Gemini for text generation with retry logic
    let generatedText = '';
    let hasError = false;

    try {
      const model = geminiClient.getGenerativeModel({ model: "gemini-2.5-flash" });

      let result;
      let attempt = 0;
      const maxRetries = 3;
      let lastError;

      while (attempt < maxRetries) {
        try {
          // Set up a timeout promise
          const timeoutPromise = new Promise((_, reject) => {
            setTimeout(() => reject(new Error('Gemini API call timeout after 30 seconds')), 30000);
          });

          // Race between the API call and timeout
          result = await Promise.race([
            model.generateContent(prompt),
            timeoutPromise
          ]);

          const response = await result.response;

          if (!response || !response.text) {
            throw new Error('Empty response from Gemini');
          }

          generatedText = response.text().trim();
          break; // Success, exit retry loop
        } catch (error) {
          lastError = error;
          attempt++;

          if (attempt >= maxRetries) {
            console.error('Gemini API error after retries:', error);
            hasError = true;
            break;
          }

          // Wait before retry with exponential backoff
          const waitTime = Math.pow(2, attempt) * 1000; // 2^attempt * 1000ms
          console.log(`Gemini API attempt ${attempt} failed, retrying in ${waitTime}ms...`);
          await new Promise(resolve => setTimeout(resolve, waitTime));
        }
      }
    } catch (error) {
      console.error('Gemini API initialization error:', error);
      hasError = true;
    }

    // If Gemini failed, try to use context directly as a fallback
    if (hasError) {
      console.log('Using fallback response generation');
      // Create a more structured response based on the context and query
      // Extract key information from the context that relates to the query
      const queryLower = message.toLowerCase();
      let relevantContent = context;

      // Try to find content that's most relevant to the query
      if (queryLower.includes('physical ai') || queryLower.includes('what is')) {
        // Look for definitions or explanations in the context
        const lines = context.split('\n');
        const relevantLines = lines.filter(line =>
          line.toLowerCase().includes('physical ai') ||
          line.toLowerCase().includes('definition') ||
          line.toLowerCase().includes('means') ||
          line.toLowerCase().includes('represents')
        );
        relevantContent = relevantLines.length > 0 ? relevantLines.join('\n') : context;
      }

      generatedText = `Based on the Physical AI & Humanoid Robotics book:\n\n${relevantContent.substring(0, 800)}...\n\nFor more details about "${message}", please refer to the specific chapters in the book.`;
    }

    // If the model says it's not in the book, use our standardized response
    if (generatedText.toLowerCase().includes('not covered in this book') ||
        generatedText.toLowerCase().includes('not found in the context') ||
        generatedText.toLowerCase().includes('no relevant information')) {
      return res.status(200).json({
        response: "Not covered in this book",
        sources: [],
        query: message
      });
    }

    res.status(200).json({
      response: generatedText,
      sources: sources,
      query: message
    });

  } catch (error) {
    console.error('Chat API error:', error);
    res.status(500).json({
      error: 'Internal server error',
      message: error.message
    });
  }
});

// Vector search function
async function performVectorSearch(query) {
  try {
    // Initialize Cohere client for embeddings
    const embeddings = await cohereClient.embed({
      model: 'embed-english-v3.0',
      texts: [query],
      inputType: 'search_query'
    });

    const queryEmbedding = embeddings.embeddings[0];

    // Search in Qdrant
    const searchResponse = await qdrantClient.search('physical_ai_book_chunks', {
      vector: queryEmbedding,
      limit: 5, // Top-k = 5 as specified
      with_payload: true,
    });

    return searchResponse;
  } catch (error) {
    console.error('Vector search error:', error);
    throw error;
  }
}

// Get sources endpoint - for testing and debugging
app.get('/api/sources', async (req, res) => {
  try {
    const countResponse = await qdrantClient.count('physical_ai_book_chunks');
    res.status(200).json({
      collection: 'physical_ai_book_chunks',
      total_documents: countResponse.count,
      status: 'active'
    });
  } catch (error) {
    console.error('Sources API error:', error);
    res.status(500).json({ error: 'Could not retrieve source information' });
  }
});

// Start server
app.listen(PORT, () => {
  console.log(`\n🤖 Physical AI Book RAG API Server`);
  console.log(`📍 Running on port ${PORT}`);
  console.log(`🔗 Health check: http://localhost:${PORT}/health`);
  console.log(`💬 Chat endpoint: http://localhost:${PORT}/api/chat`);
  console.log(`📚 Sources endpoint: http://localhost:${PORT}/api/sources`);
  console.log(`\nWaiting for requests...`);
});

module.exports = app;