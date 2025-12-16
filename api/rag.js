const { GoogleGenerativeAI } = require('@google/generative-ai');
const { QdrantClient } = require('@qdrant/js-client-rest');
require('dotenv').config();

// Initialize clients
const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY);
const model = genAI.getGenerativeModel({
  model: 'gemini-pro',
  generationConfig: {
    temperature: 0.3,
    maxOutputTokens: 800,
    topP: 0.95,
  }
});

const qdrant = new QdrantClient({
  url: process.env.QDRANT_URL,
  apiKey: process.env.QDRANT_API_KEY,
});

const ragHandler = async (req, res) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { query, context = null } = req.body;

  if (!query) {
    return res.status(400).json({ error: 'Query is required' });
  }

  try {
    // Step 1: Search in Qdrant vector database for relevant course content
    let searchResult;
    let retrievedDocs = [];
    let sources = [];

    try {
      // Attempt to search in Qdrant vector database
      searchResult = await qdrant.scroll("course_content", {
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

      // Format retrieved documents for context
      retrievedDocs = topResults.map((point) => ({
        content: point.payload.content,
        source: point.payload.source,
        module: point.payload.module
      }));

      sources = retrievedDocs.map(doc => doc.source);
    } catch (qdrantError) {
      console.warn('Qdrant connection failed, using fallback response:', qdrantError.message);
      // Use fallback response when Qdrant is not available
      retrievedDocs = [{
        content: "This is a simulated response as the Qdrant vector database is not accessible. In a working implementation, this would contain relevant course content retrieved from the knowledge base.",
        source: "simulation",
        module: "fallback"
      }];
      sources = ["simulation"];
    }

    // Step 3: Create a context for the LLM with retrieved documents
    const contextText = retrievedDocs.map(doc => doc.content).join('\n\n');

    // Step 4: Use Gemini to generate a response based on retrieved context
    // If using fallback, inform the model
    let prompt;
    if (retrievedDocs[0].source === "simulation") {
      prompt = `Question: ${query}\n\nThe knowledge base is currently unavailable. Please provide a general response about robotics and AI concepts based on your training. If the question is about ROS 2, Gazebo, Unity, NVIDIA Isaac, or Vision-Language-Action systems, acknowledge that these are topics covered in the course but that you cannot access the specific course materials right now.`;
    } else {
      prompt = `Context: ${contextText}\n\nQuestion: ${query}\n\nPlease provide a helpful answer based on the context. If the context doesn't contain relevant information, please say so and suggest where the user might find the information in the course.`;
    }

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
};

export default ragHandler;