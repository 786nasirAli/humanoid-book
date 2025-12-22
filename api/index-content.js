import { Pinecone } from '@pinecone-database/pinecone';
import { CohereClient } from 'cohere-ai';
import fs from 'fs/promises';
import path from 'path';

// Initialize clients
const cohere = new CohereClient({
  token: process.env.COHERE_API_KEY,
});

const pinecone = new Pinecone({
  apiKey: process.env.PINECONE_API_KEY,
});

const index = pinecone.Index(process.env.PINECONE_INDEX_NAME);

// Function to read all course content
async function readCourseContent() {
  // In Vercel functions, we can't access the local file system the same way
  // For this implementation, we'll return mock data or would need to store content differently
  // For a production implementation, content would typically be stored in a database
  console.log('Reading course content...');
  
  // This is a simplified version - in practice you'd need to load your content differently in serverless
  return [
    {
      id: 'intro-to-ros',
      content: 'ROS 2 (Robot Operating System 2) functions as the nervous system of humanoid robots, enabling communication between different software components that control sensors, actuators, and decision-making processes.',
      source: '/docs/module-1/intro',
      module: 'module-1'
    },
    {
      id: 'ros-nodes',
      content: 'Nodes are the fundamental units of execution in ROS 2. Think of a Node as an individual brain region responsible for a specific function. For example, one Node might handle camera data processing, another might manage motor control, and a third might handle path planning.',
      source: '/docs/module-1/nodes',
      module: 'module-1'
    },
    {
      id: 'ros-topics',
      content: 'Topics are the communication channels that allow Nodes to exchange data. Data flow through Topics is unidirectional - one Node publishes data to a Topic, and other Nodes subscribe to that Topic to receive the data. This follows the publish-subscribe pattern.',
      source: '/docs/module-1/topics',
      module: 'module-1'
    }
  ];
}

// Function to generate embeddings using Cohere
async function generateEmbeddings(texts) {
  try {
    const response = await cohere.embed({
      texts: texts,
      model: 'embed-english-v3.0',
      inputType: 'search_document',
    });

    return response.embeddings;
  } catch (error) {
    console.error('Error generating embeddings with Cohere:', error);
    throw error;
  }
}

// Function to index content to Pinecone
async function indexContent() {
  console.log('Starting content indexing...');

  // Read course content
  const documents = await readCourseContent();
  console.log(`Found ${documents.length} documents`);

  // Prepare points for Pinecone
  const points = [];
  let pointId = 0;

  for (const doc of documents) {
    // For simplicity, we're not chunking in this example
    points.push({
      id: `doc_${pointId++}`,
      values: [], // Will be populated after embedding
      metadata: {
        content: doc.content,
        source: doc.source,
        module: doc.module,
        original_id: doc.id
      }
    });
  }

  console.log(`Prepared ${points.length} chunks for indexing`);

  // Process in batches
  const batchSize = 50; // Cohere's max batch size is 96, we use 50 for safety
  for (let i = 0; i < points.length; i += batchSize) {
    const batch = points.slice(i, i + batchSize);
    console.log(`Processing batch ${Math.floor(i/batchSize) + 1}/${Math.ceil(points.length/batchSize)}`);

    // Extract text content for embedding
    const texts = batch.map(point => point.metadata.content);

    // Generate embeddings
    const embeddings = await generateEmbeddings(texts);

    // Add embeddings to points
    for (let j = 0; j < batch.length; j++) {
      batch[j].values = embeddings[j];
    }

    // Upsert points to Pinecone
    try {
      await index.upsert(batch);
      console.log(`Completed batch ${Math.floor(i/batchSize) + 1}/${Math.ceil(points.length/batchSize)}`);
    } catch (error) {
      console.error(`Error upserting batch ${Math.floor(i/batchSize) + 1}:`, error);
    }
  }

  console.log('Indexing completed successfully!');
  console.log(`Total documents indexed: ${documents.length}`);
  console.log(`Total chunks indexed: ${points.length}`);
  
  return {
    totalDocuments: documents.length,
    totalChunks: points.length
  };
}

export default async function handler(req, res) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
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
}

export const config = {
  api: {
    bodyParser: {
      sizeLimit: '10mb',
    },
  },
};