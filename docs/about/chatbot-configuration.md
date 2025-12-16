---
title: RAG Chatbot Configuration
---

# RAG Chatbot Configuration

This section explains how the RAG (Retrieval-Augmented Generation) chatbot is configured for different modules in the course.

## Module-Specific Configuration

The RAG chatbot is designed to work across all modules of the course (Module 1-4). The system automatically indexes content from all modules, allowing users to ask questions about any topic covered in the course.

## Configuration Files

- **Frontend Component**: `src/components/RAGChatbot/RAGChatbot.js` - Implements the chat interface and user interaction logic
- **Backend Server**: `server/server.js` - Processes queries and connects to Qdrant and Gemini
- **Content Indexer**: `api/content-indexer.js` - Indexes course content into Qdrant vector database
- **Environment Variables**: `.env` - Contains API keys and service endpoints

## Available Features

- **Real-time Chat Interface**: Accessible from any page via the chat icon in the bottom-right corner
- **Source Citations**: Responses include links to the original course materials
- **Response Feedback**: Users can rate responses as helpful or not helpful
- **Usage Analytics**: Anonymous tracking of user queries to improve content

## API Endpoints

- `/api/rag` - Process user queries and retrieve relevant course content
- `/api/feedback` - Submit feedback on response quality
- `/api/analytics` - Track user interactions
- `/api/health` - Health check for the backend service

## Technologies Used

- **Vector Database**: Qdrant for storing and retrieving course content embeddings
- **LLM**: Google Gemini for generating contextual responses
- **Frontend Framework**: React component integrated into Docusaurus
- **Backend Framework**: Express.js server