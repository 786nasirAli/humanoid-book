import { Pinecone } from '@pinecone-database/pinecone';
import { CohereClient } from 'cohere-ai';

// Initialize clients
const cohere = new CohereClient({
  token: process.env.COHERE_API_KEY,
});

const pinecone = new Pinecone({
  apiKey: process.env.PINECONE_API_KEY,
});

const index = pinecone.Index(process.env.PINECONE_INDEX_NAME);

export default async function handler(req, res) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { query } = req.body;

  if (!query) {
    console.log('[API-DEBUG] Query is missing for retrieval');
    return res.status(400).json({ error: 'Query is required' });
  }

  try {
    console.log(`[API-DEBUG] Retrieving documents for query: ${query}`);

    // Generate embedding for the query using Cohere
    console.log(`[API-DEBUG] Generating embedding for query: ${query}`);
    const response = await cohere.embed({
      texts: [query],
      model: 'embed-english-v3.0',
      inputType: 'search_query',
    });

    const queryEmbedding = response.embeddings[0];
    console.log(`[API-DEBUG] Generated ${queryEmbedding.length}-dimensional embedding`);

    // Query Pinecone for similar vectors
    console.log(`[API-DEBUG] Querying Pinecone index with embedding...`);
    const queryResponse = await index.query({
      vector: queryEmbedding,
      topK: 5,
      includeMetadata: true,
    });

    console.log(`[API-DEBUG] Pinecone query completed, returned ${queryResponse.matches.length} matches`);

    // Format the results
    const results = queryResponse.matches.map((match, index) => {
      console.log(`[API-DEBUG] Processing match ${index + 1}: ID=${match.id}, Score=${match.score}`);
      return {
        id: match.id,
        content: match.metadata?.content || '',
        source: match.metadata?.source || '',
        score: match.score,
        rank: index + 1
      };
    });

    console.log(`[API-DEBUG] Sending ${results.length} formatted results to client`);

    // Return the retrieved documents
    res.status(200).json({
      results: results,
      query: query,
      retrievedCount: results.length
    });
    console.log('[API-DEBUG] Retrieval results sent to client');

  } catch (error) {
    console.error('[API-ERROR] Retrieval API Error:', error.message);
    console.error('[API-ERROR] Retrieval API Error Stack:', error.stack);

    res.status(500).json({
      error: 'Internal server error during document retrieval',
      details: process.env.NODE_ENV === 'development' ? error.message : undefined
    });
  }
}

export const config = {
  api: {
    bodyParser: {
      sizeLimit: '10mb',
    },
  },
};