const express = require('express');
const cors = require('cors');
const { GoogleGenerativeAI } = require('@google/generative-ai');
const { QdrantClient } = require('@qdrant/js-client-rest');
const userRoutes = require('./routes/userRoutes');
require('dotenv').config();

const app = express();
const PORT = process.env.PORT || 8000;

// Middleware
app.use(cors());
app.use(express.json());

// Routes
app.use('/api', userRoutes);

// Initialize clients
const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY);
const model = genAI.getGenerativeModel({ model: 'gemini-pro' }); // or gemini-1.5-pro-latest

const qdrant = new QdrantClient({
  url: process.env.QDRANT_URL,
  apiKey: process.env.QDRANT_API_KEY,
});

// RAG endpoint
app.post('/api/rag', async (req, res) => {
  const { query } = req.body;

  if (!query) {
    return res.status(400).json({ error: 'Query is required' });
  }

  try {
    // Step 1: Search in Qdrant vector database for relevant course content
    // Note: For Gemini, we'll need to create embeddings using another method
    // since we're not using OpenAI embeddings anymore
    // For this implementation, we'll use a simpler keyword-based search approach

    console.log(`Received query: ${query}`);

    // For actual vector search with Gemini embeddings, you would need:
    // 1. A service to generate embeddings from Gemini
    // 2. Those embeddings indexed in Qdrant
    const searchResult = await qdrant.scroll("course_content", {
      limit: 10, // Get top 10 results initially
      with_payload: true,
      offset: 0
    });

    // Simple keyword-based search since we don't have proper embeddings for Gemini
    // In a real implementation, you would replace this with actual vector search
    const queryLower = query.toLowerCase();
    const keywords = queryLower.split(/\s+/).filter(word => word.length > 2); // Remove short words

    let relevantPoints = searchResult.points.filter(point => {
      if (!point.payload || !point.payload.content) return false;

      const content = point.payload.content.toLowerCase();
      // Count how many keywords match in the content
      const matches = keywords.filter(keyword => content.includes(keyword)).length;

      // Only include if at least half the keywords match
      return matches >= Math.max(1, Math.floor(keywords.length / 2));
    });

    // Sort by number of keyword matches (descending)
    relevantPoints.sort((a, b) => {
      const aMatches = keywords.filter(kw => a.payload.content.toLowerCase().includes(kw)).length;
      const bMatches = keywords.filter(kw => b.payload.content.toLowerCase().includes(kw)).length;
      return bMatches - aMatches;
    });

    // Take top 5 most relevant
    const topResults = relevantPoints.slice(0, 5);

    // Step 2: Format retrieved documents for context
    const retrievedDocs = topResults.map((point) => ({
      content: point.payload.content,
      source: point.payload.source,
      module: point.payload.module
    }));

    // Step 3: Create a context for the LLM with retrieved documents
    const contextText = retrievedDocs.map(doc => doc.content).join('\n\n');
    const sources = retrievedDocs.map(doc => doc.source);

    // Step 4: Use Gemini to generate a response based on retrieved context
    const prompt = `Context: ${contextText}\n\nQuestion: ${query}\n\nPlease provide a helpful answer based on the context. If the context doesn't contain relevant information, please say so and suggest where the user might find the information in the course.`;

    const result = await model.generateContent(prompt);
    const response = await result.response;
    const text = response.text();

    // Step 5: Return the response along with source information
    res.status(200).json({
      response: text,
      sources: sources,
      retrieved_docs_count: retrievedDocs.length
    });

  } catch (error) {
    console.error('RAG API Error:', error);
    res.status(500).json({
      error: 'Internal server error during RAG processing',
      details: process.env.NODE_ENV === 'development' ? error.message : undefined
    });
  }
});

// Health check endpoint
app.get('/api/health', (req, res) => {
  res.status(200).json({ status: 'OK', timestamp: new Date().toISOString() });
});

// Start server
app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
  console.log(`Health check: http://localhost:${PORT}/api/health`);
});