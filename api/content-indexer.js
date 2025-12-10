const { QdrantClient } = require('@qdrant/js-client-rest');
const { OpenAIClient, AzureKeyCredential } = require('@azure/openai');
const fs = require('fs').promises;
const path = require('path');
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

// Function to read all course content
async function readCourseContent() {
  const contentDir = path.join(process.cwd(), 'docs');
  const modules = ['module-1', 'module-2', 'module-3', 'module-4'];
  const allContent = [];

  for (const module of modules) {
    const modulePath = path.join(contentDir, module);
    try {
      const files = await fs.readdir(modulePath);
      for (const file of files) {
        if (file.endsWith('.md')) {
          const filePath = path.join(modulePath, file);
          const content = await fs.readFile(filePath, 'utf8');
          
          // Extract content without frontmatter
          const contentWithoutFrontmatter = content.replace(/---[\s\S]*?---/, '');
          
          allContent.push({
            id: `${module}/${file}`,
            content: contentWithoutFrontmatter,
            source: `/docs/${module}/${file}`,
            module: module
          });
        }
      }
    } catch (error) {
      console.error(`Error reading ${module}:`, error);
    }
  }

  return allContent;
}

// Function to split content into chunks
function splitIntoChunks(content, maxLength = 1000) {
  const chunks = [];
  const paragraphs = content.split('\n\n');
  
  let currentChunk = '';
  
  for (const paragraph of paragraphs) {
    if (currentChunk.length + paragraph.length <= maxLength) {
      currentChunk += paragraph + '\n\n';
    } else {
      if (currentChunk.trim()) {
        chunks.push(currentChunk.trim());
      }
      currentChunk = paragraph + '\n\n';
      
      // If a single paragraph is too long, split it by sentences
      if (currentChunk.length > maxLength) {
        const sentences = currentChunk.split(/(?<=[.!?])\s+/);
        currentChunk = '';
        
        for (const sentence of sentences) {
          if (currentChunk.length + sentence.length <= maxLength) {
            currentChunk += sentence + ' ';
          } else {
            if (currentChunk.trim()) {
              chunks.push(currentChunk.trim());
            }
            currentChunk = sentence + ' ';
            
            // If a single sentence is too long, split by length
            if (currentChunk.length > maxLength) {
              const parts = currentChunk.match(new RegExp(`.{1,${maxLength}}`, 'g')) || [];
              for (let i = 0; i < parts.length - 1; i++) {
                chunks.push(parts[i]);
              }
              currentChunk = parts[parts.length - 1] || '';
            }
          }
        }
      }
    }
  }
  
  if (currentChunk.trim()) {
    chunks.push(currentChunk.trim());
  }
  
  return chunks;
}

// Function to generate embeddings
async function generateEmbeddings(texts) {
  try {
    const response = await openai.getEmbeddings(
      process.env.AZURE_OPENAI_EMBEDDINGS_DEPLOYMENT_NAME,
      texts
    );
    
    return response.data.map(item => item.embedding);
  } catch (error) {
    console.error('Error generating embeddings:', error);
    throw error;
  }
}

// Function to index content to Qdrant
async function indexContent() {
  console.log('Starting content indexing...');
  
  // Read course content
  const documents = await readCourseContent();
  console.log(`Found ${documents.length} documents`);
  
  // Prepare points for Qdrant
  const points = [];
  let pointId = 0;
  
  for (const doc of documents) {
    const chunks = splitIntoChunks(doc.content);
    
    for (const chunk of chunks) {
      points.push({
        id: pointId++,
        payload: {
          content: chunk,
          source: doc.source,
          module: doc.module,
          original_id: doc.id
        }
      });
    }
  }
  
  console.log(`Prepared ${points.length} chunks for indexing`);
  
  // Create collection in Qdrant if it doesn't exist
  try {
    await qdrant.getCollections();
    const collections = await qdrant.getCollections();
    const collectionExists = collections.collections.some(c => c.name === 'course_content');
    
    if (!collectionExists) {
      await qdrant.createCollection('course_content', {
        vector: 1536, // Assuming we're using text-embedding-ada-002 which has 1536 dimensions
        hnsw_config: {
          ef_construct: 100,
          m: 16,
        },
        optimizers_config: {
          full_scan_threshold: 10000,
        },
        quantization_config: {
          scalar: {
            type: 'int8',
            quantile: 0.99,
            always_ram: true,
          }
        }
      });
      console.log('Created new collection: course_content');
    }
  } catch (error) {
    console.error('Error creating collection:', error);
    throw error;
  }
  
  // Process in batches to avoid memory issues
  const batchSize = 100;
  for (let i = 0; i < points.length; i += batchSize) {
    const batch = points.slice(i, i + batchSize);
    console.log(`Processing batch ${i/batchSize + 1}/${Math.ceil(points.length/batchSize)}`);
    
    // Extract text content for embedding
    const texts = batch.map(point => point.payload.content);
    
    // Generate embeddings
    const embeddings = await generateEmbeddings(texts);
    
    // Prepare points with vectors for insertion
    const pointsWithVectors = batch.map((point, idx) => ({
      id: point.id,
      vector: embeddings[idx],
      payload: point.payload
    }));
    
    // Upsert points to Qdrant
    await qdrant.upsert('course_content', {
      points: pointsWithVectors
    });
  }
  
  console.log('Indexing completed successfully!');
}

// Run indexing if this file is executed directly
if (require.main === module) {
  indexContent()
    .then(() => {
      console.log('Content indexing complete');
      process.exit(0);
    })
    .catch((error) => {
      console.error('Indexing failed:', error);
      process.exit(1);
    });
}

module.exports = { indexContent, readCourseContent, splitIntoChunks };