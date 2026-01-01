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

// CORS configuration for Vercel deployment
const corsOptions = {
  origin: process.env.NODE_ENV === 'production'
    ? [
        'https://your-vercel-domain.vercel.app', // Replace with your actual Vercel domain
        'https://hackthon-physical-ai-book.vercel.app', // Common Vercel domain pattern
        'http://localhost:3000', // Local Docusaurus
        'http://localhost:3001', // Local development
        'http://localhost:3002',
        'http://localhost:3030'
      ]
    : [
        'http://localhost:3000',
        'http://localhost:3001',
        'http://localhost:3002',
        'http://localhost:3030'
      ],
  credentials: true,
  optionsSuccessStatus: 200
};

app.use(cors(corsOptions));
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

// Proxy for API requests to handle cross-origin issues in Vercel deployment
const { createProxyMiddleware } = require('http-proxy-middleware');

// Proxy for chat API
app.use('/api/chat', createProxyMiddleware({
  target: process.env.API_TARGET_URL || 'http://localhost:3001',
  changeOrigin: true,
  pathRewrite: {
    '^/api/chat': '/api/chat', // Remove /api/chat prefix when forwarding
  },
  onProxyReq: (proxyReq, req, res) => {
    console.log(`Proxying request: ${req.method} ${req.url}`);
  },
  onProxyRes: (proxyRes, req, res) => {
    console.log(`Proxy response: ${proxyRes.statusCode} for ${req.url}`);
  }
}));

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

// Handle preflight requests for all routes
app.options('*', (req, res) => {
  res.header('Access-Control-Allow-Origin', '*');
  res.header('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
  res.header('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept, Authorization');
  res.sendStatus(200);
});

// Intent detection function
function detectIntent(message) {
  const trimmedMessage = message.trim().toLowerCase();

  // GREETING intent: Check for greetings
  const greetingPatterns = ['hi', 'hello', 'hey', 'yo', 'greetings', 'good morning', 'good afternoon', 'good evening'];
  for (const pattern of greetingPatterns) {
    if (trimmedMessage === pattern || trimmedMessage.startsWith(pattern + ' ') || trimmedMessage.endsWith(' ' + pattern)) {
      return 'GREETING';
    }
  }

  // META intent: Check for meta questions about the site or chatbot
  const metaPatterns = ['what can you do', 'how does this work', 'what is this', 'who are you', 'help', 'what is this site', 'what is this chatbot', 'what is this about'];
  for (const pattern of metaPatterns) {
    if (trimmedMessage.includes(pattern)) {
      return 'META';
    }
  }

  // UNCLEAR intent: Less than 3 words or too short
  if (trimmedMessage.split(/\s+/).filter(word => word.length > 0).length < 3) {
    return 'UNCLEAR';
  }

  // BOOK-RELATED TECHNICAL QUESTION intent: Check for technical terms related to the book
  const technicalTerms = ['physical ai', 'humanoid', 'robot', 'perception', 'motion planning', 'control', 'simulation', 'kinematics', 'dynamics', 'locomotion', 'manipulation', 'navigation', 'sensors', 'actuators', 'ai', 'machine learning', 'neural networks', 'computer vision', 'robotics'];
  for (const term of technicalTerms) {
    if (trimmedMessage.includes(term)) {
      return 'BOOK_TECHNICAL';
    }
  }

  // If no clear technical terms but seems like a question, classify as unclear
  if (trimmedMessage.includes('?') || trimmedMessage.includes('what') || trimmedMessage.includes('how') || trimmedMessage.includes('why') || trimmedMessage.includes('when') || trimmedMessage.includes('where') || trimmedMessage.includes('who')) {
    return 'BOOK_TECHNICAL'; // Assume it's related to the book if it's a question
  }

  // Default to UNCLEAR if no patterns match
  return 'UNCLEAR';
}

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

    // 1. Detect intent before processing
    const intent = detectIntent(message);
    console.log(`Detected intent: ${intent}`);

    // Handle non-technical intents without RAG
    if (intent === 'GREETING') {
      return res.status(200).json({
        response: "Hello 👋\n\nYou can ask me questions about perception, motion planning, robot control, simulation, or any chapter of the Physical AI book.",
        sources: [],
        query: message
      });
    }

    if (intent === 'META') {
      return res.status(200).json({
        response: "I'm your Physical AI & Humanoid Robotics assistant. I can answer questions about the book content, including topics like perception, control systems, motion planning, simulation, and more. Ask me anything related to the Physical AI & Humanoid Robotics book!",
        sources: [],
        query: message
      });
    }

    if (intent === 'UNCLEAR') {
      return res.status(200).json({
        response: "I couldn't find a clear question.\n\nPlease ask something specific related to Physical AI or humanoid robotics.",
        sources: [],
        query: message
      });
    }

    // Only proceed with RAG for BOOK_TECHNICAL intent
    if (intent === 'BOOK_TECHNICAL') {
      // 2. Perform vector search in Qdrant
      const searchResults = await performVectorSearch(message);

      // Apply similarity threshold (e.g., >= 0.50) - adjusted for typical cosine similarity ranges
      const filteredResults = searchResults.filter(result => result.score >= 0.50);

      if (filteredResults.length === 0) {
        return res.status(200).json({
          response: "This topic is not clearly covered in the book.",
          sources: [],
          query: message
        });
      }

      // 3. Prepare context from search results
      const context = filteredResults.map(result => result.payload.content).join('\n\n');
      const sources = filteredResults.map(result => ({
        part: result.payload.part || '',
        chapter: result.payload.chapter || '',
        section: result.payload.section || '',
        file_path: result.payload.file_path || '',
        heading_title: result.payload.heading_title || '',
        score: result.score
      }));

      // 4. Generate response using Gemini
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
    } else {
      // This should not happen due to the checks above, but added for safety
      return res.status(200).json({
        response: "I couldn't find a clear question.\n\nPlease ask something specific related to Physical AI or humanoid robotics.",
        sources: [],
        query: message
      });
    }

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