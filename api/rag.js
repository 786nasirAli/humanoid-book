const { OpenAIClient, AzureKeyCredential } = require('@azure/openai');
const { QdrantClient } = require('@qdrant/js-client-rest');
require('dotenv').config();

// Initialize clients
const openai = new OpenAIClient({
  endpoint: process.env.AZURE_OPENAI_ENDPOINT,
  credential: new AzureKeyCredential(process.env.AZURE_OPENAI_API_KEY)
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
    const searchResult = await qdrant.search("course_content", {
      vector: query,
      limit: 5, // Get top 5 most relevant results
      with_payload: true,
    });

    // Step 2: Format retrieved documents for context
    const retrievedDocs = searchResult.map((hit) => ({
      content: hit.payload.content,
      source: hit.payload.source,
      module: hit.payload.module
    }));

    // Step 3: Create a context for the LLM with retrieved documents
    const contextText = retrievedDocs.map(doc => doc.content).join('\n\n');
    const sources = retrievedDocs.map(doc => doc.source);

    // Step 4: Use OpenAI to generate a response based on retrieved context
    const fullQuery = `Context: ${contextText}\n\nQuestion: ${query}\n\nPlease provide a helpful answer based on the context. If the context doesn't contain relevant information, please say so and suggest where the user might find the information in the course.`;

    const completion = await openai.getChatCompletions(
      process.env.AZURE_OPENAI_DEPLOYMENT_NAME, 
      [
        {
          role: "system",
          content: "You are a helpful assistant for the Physical AI & Humanoid Robotics course. Use the provided context to answer questions accurately and reference the relevant modules when possible. Be concise but comprehensive."
        },
        {
          role: "user",
          content: fullQuery
        }
      ],
      { maxTokens: 800 }
    );

    const response = completion.choices[0]?.message?.content?.trim() || "I couldn't generate a response. Please try rephrasing your question.";

    // Step 5: Return the response along with source information
    res.status(200).json({
      response: response,
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