const { Pinecone } = require('@pinecone-database/pinecone');
const { CohereClient } = require('cohere-ai');

// Only require dotenv in development, Vercel automatically provides environment variables
if (typeof process !== 'undefined' && process.env && process.env.NODE_ENV !== 'production') {
  require('dotenv').config();
}

// Initialize clients with safety checks
const cohere = new CohereClient({
  token: typeof process !== 'undefined' && process.env ? process.env.COHERE_API_KEY : undefined,
});

const pinecone = new Pinecone({
  apiKey: typeof process !== 'undefined' && process.env ? process.env.PINECONE_API_KEY : undefined,
});

const index = pinecone.Index(typeof process !== 'undefined' && process.env ? process.env.PINECONE_INDEX_NAME : '');

// Function to perform retrieval from Pinecone
async function retrieveFromPinecone(query, topK = 5) {
  try {
    console.log(`[DEBUG] Starting retrieval for query: ${query}`);
    console.log(`[DEBUG] Pinecone API key: ${typeof process !== 'undefined' && process.env ? (process.env.PINECONE_API_KEY ? 'Set' : 'Not set') : 'process undefined'}`);
    console.log(`[DEBUG] Pinecone index name: ${typeof process !== 'undefined' && process.env ? process.env.PINECONE_INDEX_NAME : 'process undefined'}`);
    console.log(`[DEBUG] Cohere API key: ${typeof process !== 'undefined' && process.env ? (process.env.COHERE_API_KEY ? 'Set' : 'Not set') : 'process undefined'}`);

    // Generate embedding for the query using Cohere
    console.log(`[DEBUG] Generating embedding for query: ${query}`);
    const response = await cohere.embed({
      texts: [query],
      model: 'embed-english-v3.0',
      inputType: 'search_query',
    });

    const queryEmbedding = response.embeddings[0];
    console.log(`[DEBUG] Generated ${queryEmbedding.length}-dimensional embedding`);

    // Query Pinecone for similar vectors
    console.log(`[DEBUG] Querying Pinecone index with embedding...`);
    const queryResponse = await index.query({
      vector: queryEmbedding,
      topK: topK,
      includeMetadata: true,
    });

    console.log(`[DEBUG] Pinecone query completed, returned ${queryResponse.matches.length} matches`);

    // Format the results
    const results = queryResponse.matches.map((match, index) => {
      console.log(`[DEBUG] Processing match ${index + 1}: ID=${match.id}, Score=${match.score}`);
      return {
        id: match.id,
        content: match.metadata?.content || '',
        source: match.metadata?.source || '',
        score: match.score,
        rank: index + 1
      };
    });

    console.log(`[DEBUG] Formatted ${results.length} results from Pinecone`);

    return {
      results: results,
      query: query,
      retrievedCount: results.length
    };
  } catch (error) {
    console.error('[ERROR] Full error in Pinecone retrieval:', error.message);
    console.error('[ERROR] Error details:', {
      name: error.name,
      message: error.message,
      stack: error.stack,
      code: error.code,
      status: error.status
    });

    // Check for specific error types
    if (error.message.includes('401') || error.message.includes('Unauthorized')) {
      console.error('[ERROR] Authentication error - check Pinecone API key');
    } else if (error.message.includes('404') || error.message.includes('index')) {
      console.error('[ERROR] Pinecone index not found - check index name');
    } else if (error.message.includes('429') || error.name.includes('RateLimit')) {
      console.error('[ERROR] Rate limit exceeded - check Cohere/Pinecone quotas');
    }

    // Return a structured error response
    return {
      results: [],
      query: query,
      retrievedCount: 0,
      error: {
        message: error.message,
        type: error.name,
        code: error.code
      }
    };
  }
}

// Export the function
module.exports = { retrieveFromPinecone };

// If this file is run directly, perform a test retrieval
if (require.main === module) {
  const testQuery = "What is ROS 2?";
  
  retrieveFromPinecone(testQuery, 5)
    .then(result => {
      console.log('Retrieval Results:', JSON.stringify(result, null, 2));
    })
    .catch(error => {
      console.error('Error in test retrieval:', error);
    });
}