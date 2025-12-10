# RAG Chatbot Setup for Physical AI & Humanoid Robotics Course

This document explains how to set up and run the RAG (Retrieval-Augmented Generation) chatbot for the course.

## Prerequisites

1. **Node.js** (v16 or higher)
2. **Access to OpenAI or Azure OpenAI** with appropriate API keys
3. **Qdrant Cloud** account for vector storage
4. **Neon Serverless Postgres** account for user data (optional, for personalization features)

## Installation

1. Clone the repository:
```bash
git clone https://github.com/panaversity/humanoid-book.git
cd humanoid-book
```

2. Install frontend dependencies:
```bash
npm install
```

3. Navigate to the server directory and install backend dependencies:
```bash
cd server
npm install
```

## Configuration

1. Create a `.env` file in the `server` directory (use `.env.example` as a template):
```bash
cp .env.example .env
```

2. Fill in the required environment variables:
   - `AZURE_OPENAI_ENDPOINT` - Your Azure OpenAI endpoint URL
   - `AZURE_OPENAI_API_KEY` - Your Azure OpenAI API key
   - `AZURE_OPENAI_DEPLOYMENT_NAME` - Name of your GPT model deployment
   - `AZURE_OPENAI_EMBEDDINGS_DEPLOYMENT_NAME` - Name of your embeddings model deployment
   - `QDRANT_URL` - Your Qdrant Cloud URL
   - `QDRANT_API_KEY` - Your Qdrant API key

## Setting up Vector Database

1. Run the content indexing script to populate Qdrant with course content:
```bash
cd server
npm run index-content
```

This script will:
- Read all course content from the `docs/` directory
- Split content into manageable chunks
- Generate embeddings using OpenAI
- Index the content in Qdrant vector database

## Running the Application

1. Start the backend server:
```bash
cd server
npm start
```
The server will run on `http://localhost:5000` by default.

2. In a separate terminal, start the Docusaurus frontend:
```bash
cd ..  # Navigate back to the project root
npm start
```

This will start the Docusaurus development server on `http://localhost:3000`.

## API Endpoints

- `POST /api/rag` - Query the RAG system with a question
- `GET /api/health` - Health check for the backend server

## How It Works

The RAG system works in three steps:

1. **Retrieve**: When a user asks a question, the system converts the query to embeddings and searches for the most relevant content chunks in the Qdrant vector database.

2. **Augment**: The retrieved content is combined with the original query to provide context.

3. **Generate**: The augmented query is sent to the LLM (GPT) which generates a response based on the retrieved context.

## Architecture

- **Frontend**: Docusaurus-based documentation site with React chatbot component
- **Backend**: Express.js server handling RAG logic
- **Vector Database**: Qdrant for storing course content embeddings
- **Language Model**: Azure OpenAI for generating responses

## Troubleshooting

- If content indexing fails, ensure all environment variables are correctly set
- If chatbot responses are generic, verify that the content was properly indexed
- Check CORS settings if running frontend and backend on different ports
- Monitor token usage as RAG queries can be expensive

## Customization

- Adjust chunk size in the indexer for different context windows
- Modify the system prompt in the backend for different response styles
- Add more metadata to indexed content for better source attribution

## Security

For production deployment:
- Use proper authentication for the API endpoints
- Implement rate limiting
- Securely manage environment variables
- Consider using Azure managed identities instead of API keys