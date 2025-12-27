import OpenAI from 'openai';
import { Pinecone } from '@pinecone-database/pinecone';
import { CohereClient } from 'cohere-ai';

// Initialize clients
const openai = new OpenAI({
  apiKey: process.env.OPENROUTER_API_KEY,
  baseURL: 'https://openrouter.ai/api/v1',
});

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
    console.log('[RAG-DEBUG] Query is missing');
    return res.status(400).json({ error: 'Query is required' });
  }

  try {
    console.log(`[RAG-DEBUG] Received query: ${query}`);

    // Generate embedding for the query using Cohere
    console.log(`[RAG-DEBUG] Generating embedding for query: ${query}`);
    const response = await cohere.embed({
      texts: [query],
      model: 'embed-english-v3.0',
      inputType: 'search_query',
    });

    const queryEmbedding = response.embeddings[0];
    console.log(`[RAG-DEBUG] Generated ${queryEmbedding.length}-dimensional embedding`);

    // Query Pinecone for similar vectors
    console.log(`[RAG-DEBUG] Querying Pinecone index with embedding...`);
    const queryResponse = await index.query({
      vector: queryEmbedding,
      topK: 5,
      includeMetadata: true,
    });

    console.log(`[RAG-DEBUG] Pinecone query completed, returned ${queryResponse.matches.length} matches`);

    // Format the results
    const results = queryResponse.matches.map((match, index) => {
      console.log(`[RAG-DEBUG] Processing match ${index + 1}: ID=${match.id}, Score=${match.score}`);
      return {
        id: match.id,
        content: match.metadata?.content || '',
        source: match.metadata?.source || '',
        score: match.score,
        rank: index + 1
      };
    });

    console.log(`[RAG-DEBUG] Context before formatting: ${results.length} documents`);

    // Format the context from retrieved documents
    const contextText = results.map(result =>
      `Source: ${result.source}\nContent: ${result.content}`
    ).join('\n\n---\n\n');

    console.log(`[RAG-DEBUG] Context text length: ${contextText.length}`);
    console.log(`[RAG-DEBUG] Retrieved ${results.length} documents`);

    // Create prompt using retrieved context
    const prompt = `Context: ${contextText}\n\nQuestion: ${query}\n\nPlease provide a helpful answer based on the context. If the context doesn't contain relevant information, please say so and suggest where the user might find the information in the course.`;

    try {
      console.log('[RAG-DEBUG] Sending prompt to OpenRouter...');
      // Use OpenRouter to generate a response based on retrieved context
      const response = await openai.chat.completions.create({
        model: 'meta-llama/llama-3.2-3b-instruct',  // Using Llama 3.2 3B Instruct model from OpenRouter
        messages: [
          {
            role: 'system',
            content: 'You are a helpful assistant for the Physical AI & Humanoid Robotics course. Use the provided context to answer questions accurately and reference the relevant modules when possible. Be concise but comprehensive.'
          },
          {
            role: 'user',
            content: prompt
          }
        ],
        max_tokens: 800,
        temperature: 0.3,
        top_p: 0.95,
      });

      const text = response.choices[0].message.content;
      console.log('[RAG-DEBUG] Response generated from OpenRouter:', text.substring(0, 100) + '...');

      // Extract sources from results
      const sources = results.map(r => r.source);

      // Return the response along with source information
      res.status(200).json({
        response: text,
        sources: sources,
        retrieved_docs_count: results.length
      });
      console.log('[RAG-DEBUG] Response sent to client');
    } catch (openaiError) {
      console.error('[RAG-ERROR] OpenRouter API Error:', openaiError.message);
      console.error('[RAG-ERROR] OpenRouter Error Details:', {
        type: openaiError.constructor.name,
        message: openaiError.message,
        code: openaiError.code,
        status: openaiError.status,
        url: openaiError.url,
        response_status: openaiError.status
      });

      // Log more details about the error for debugging
      if (openaiError.error) {
        console.error('[RAG-ERROR] OpenRouter specific error:', openaiError.error);
      }

      // In case of OpenRouter error, return the retrieved documents only
      res.status(200).json({
        response: "Could not generate a response due to an API error, but here are some relevant documents:",
        sources: results.map(r => r.source),
        retrieved_docs_count: results.length,
        fallback_context: results.map(r => r.content).join('\n\n'),
        error_details: process.env.NODE_ENV === 'development' ? openaiError.message : 'API error occurred'
      });
    }

  } catch (error) {
    console.error('[RAG-ERROR] RAG API Error:', error.message);
    console.error('[RAG-ERROR] RAG API Error Stack:', error.stack);

    // Send detailed error info for debugging
    res.status(500).json({
      error: 'Internal server error during RAG processing',
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