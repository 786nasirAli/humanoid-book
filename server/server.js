const express = require('express');
const cors = require('cors');
const { GoogleGenerativeAI } = require('@google/generative-ai');
const { Pinecone } = require('@pinecone-database/pinecone');
const cohere = require('cohere-ai');
const userRoutes = require('./routes/userRoutes');
require('dotenv').config();

const app = express();
const PORT = process.env.PORT || 3001;  // Changed from 8000 to 3001 to avoid conflict with Docusaurus

// Middleware
app.use(cors({
  origin: ['http://localhost:3000', 'http://localhost:8000', 'http://localhost:3001'], // Allow common Docusaurus ports
  credentials: true
}));
app.use(express.json());

// Routes
app.use('/api', userRoutes);

// Initialize clients
const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY);
const model = genAI.getGenerativeModel({
  model: 'models/gemini-2.0-flash',
  generationConfig: {
    temperature: 0.3,
    maxOutputTokens: 800,
    topP: 0.95,
  }
}); // models/ prefix is required for Google's API format

// Initialize Pinecone client
const pinecone = new Pinecone({
  apiKey: process.env.PINECONE_API_KEY,
});

const index = pinecone.Index(process.env.PINECONE_INDEX_NAME);

// Initialize Cohere client
cohere.init(process.env.COHERE_API_KEY);

// Function to generate embedding using Cohere
async function generateEmbedding(text) {
  try {
    const response = await cohere.embed({
      texts: [text],
      model: 'embed-english-v3.0',
      inputType: 'search_query',
    });

    return response.body.embeddings[0]; // Return the embedding vector
  } catch (error) {
    console.error('Error generating embedding with Cohere:', error);
    // Fallback to random embedding if Cohere fails
    return Array(1536).fill(0).map(() => Math.random());
  }
}

// RAG endpoint
app.post('/api/rag', async (req, res) => {
  const { query } = req.body;

  if (!query) {
    return res.status(400).json({ error: 'Query is required' });
  }

  try {
    console.log(`Received query: ${query}`);

    // Attempt to search in Pinecone vector database
    let retrievedDocs = [];
    let sources = [];

    try {
      // Generate embedding for the query using a compatible method
      const queryEmbedding = await generateEmbedding(query);

      const queryResponse = await index.query({
        vector: queryEmbedding,
        topK: 5,
        includeMetadata: true,
      });

      // Check if matches exist and format retrieved documents for context
      if (queryResponse && queryResponse.matches && queryResponse.matches.length > 0) {
        retrievedDocs = queryResponse.matches.map((match) => ({
          content: match.metadata?.content || '',
          source: match.metadata?.source || '',
          module: match.metadata?.module || ''
        }));

        sources = retrievedDocs.map(doc => doc.source);
      } else {
        // If no matches found, use fallback
        retrievedDocs = [{
          content: "No relevant content found in the knowledge base.",
          source: "none",
          module: "fallback"
        }];
        sources = ["none"];
      }
    } catch (pineconeError) {
      console.warn('Pinecone connection failed, using fallback response:', pineconeError.message);
      // Use fallback response when Pinecone is not available
      retrievedDocs = [{
        content: "This is a simulated response as the Pinecone vector database is not accessible. In a working implementation, this would contain relevant course content retrieved from the knowledge base.",
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
});

// Analytics endpoint
app.post('/api/analytics', async (req, res) => {
  const { event, data } = req.body;

  if (!event) {
    return res.status(400).json({ error: 'Event type is required' });
  }

  try {
    // In a production environment, you would store this data in a database
    // For now, we'll just log it
    console.log(`Analytics Event: ${event}`, {
      data,
      timestamp: new Date().toISOString()
    });

    // Optional: Store analytics in MongoDB or other database
    // const analyticsRecord = { event, data, timestamp: new Date() };
    // await analyticsCollection.insertOne(analyticsRecord);

    res.status(200).json({ status: 'Analytics recorded' });
  } catch (error) {
    console.error('Analytics error:', error);
    res.status(500).json({ error: 'Failed to record analytics' });
  }
});

// Feedback endpoint
app.post('/api/feedback', async (req, res) => {
  const { messageId, feedback, messageText } = req.body;

  if (!messageId || !feedback) {
    return res.status(400).json({ error: 'Message ID and feedback are required' });
  }

  try {
    // In a production environment, you would store feedback in a database
    // For now, we'll just log it
    console.log(`Feedback received:`, {
      messageId,
      feedback,
      messageText: messageText ? messageText.substring(0, 100) + '...' : 'N/A',
      timestamp: new Date().toISOString()
    });

    // Optional: Store feedback in MongoDB or other database
    // const feedbackRecord = { messageId, feedback, messageText, timestamp: new Date() };
    // await feedbackCollection.insertOne(feedbackRecord);

    res.status(200).json({ status: 'Feedback recorded' });
  } catch (error) {
    console.error('Feedback error:', error);
    res.status(500).json({ error: 'Failed to record feedback' });
  }
});

// Health check endpoint
app.get('/api/health', (req, res) => {
  res.status(200).json({ status: 'OK', timestamp: new Date().toISOString() });
});

// Indexing endpoint
app.post('/api/index-content', async (req, res) => {
  try {
    // Import the indexing function
    const { indexContent } = require('./indexer');

    // Run indexing in background
    indexContent().then(() => {
      console.log('Indexing completed');
    }).catch((error) => {
      console.error('Indexing failed:', error);
    });

    res.status(200).json({ message: 'Indexing started' });
  } catch (error) {
    console.error('Error starting indexing:', error);
    res.status(500).json({ error: 'Failed to start indexing' });
  }
});

// Start server
app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
  console.log(`Health check: http://localhost:${PORT}/api/health`);
  console.log(`RAG endpoint: http://localhost:${PORT}/api/rag`);
});