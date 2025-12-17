import React, { useState, useEffect, useRef } from 'react';
import styles from './RAGChatbot.module.css';

const RAGChatbot = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false); // Chatbot panel visibility
  const [feedbackVisible, setFeedbackVisible] = useState(null); // Track which message feedback is visible
  const [retrievedDocuments, setRetrievedDocuments] = useState([]); // Track retrieved documents
  const messagesEndRef = useRef(null);

  // Function to scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to track analytics
  const trackAnalytics = async (event, data) => {
    console.log('Analytics event:', event, data);

    try {
      // API call to track analytics - using configurable backend URL
      const BACKEND_URL = process.env.REACT_APP_BACKEND_URL || 'http://localhost:3001';
      await fetch(`${BACKEND_URL}/api/analytics`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ event, data, timestamp: new Date() })
      });
    } catch (error) {
      console.error('Analytics tracking failed:', error);
    }
  };

  // Function to retrieve documents from Pinecone
  const retrieveDocuments = async (query) => {
    try {
      // Show retrieving status to user
      const retrievingMessage = {
        id: Date.now(),
        sender: 'system',
        text: 'Retrieving relevant documents...',
        timestamp: new Date(),
        status: 'retrieving'
      };

      setMessages(prev => [...prev, retrievingMessage]);

      // Call the retrieval API
      const BACKEND_URL = process.env.REACT_APP_BACKEND_URL || 'http://localhost:3001';
      const response = await fetch(`${BACKEND_URL}/api/retrieve`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query }),
      });

      if (!response.ok) {
        throw new Error(`Retrieve API request failed with status ${response.status}`);
      }

      const data = await response.json();

      // Update the retrieving message to show retrieved documents
      setMessages(prev => prev.map(msg =>
        msg.id === retrievingMessage.id
          ? {
              ...msg,
              text: `Retrieved ${data.retrievedCount} relevant documents. Generating response...`,
              status: 'retrieved',
              retrievedDocs: data.results
            }
          : msg
      ));

      // Store the retrieved documents for display
      setRetrievedDocuments(data.results);

      return data;
    } catch (error) {
      console.error('Error calling retrieval API:', error);

      // Update the retrieving message to show error
      setMessages(prev => prev.map(msg =>
        msg.id === Date.now()
          ? {
              ...msg,
              text: 'Error retrieving documents. Using general knowledge to answer.',
              status: 'error'
            }
          : msg
      ));

      return {
        results: [],
        retrievedCount: 0
      };
    }
  };

  // Function to generate response using Gemini with retrieved context
  const generateResponse = async (query, context = null) => {
    try {
      const BACKEND_URL = process.env.REACT_APP_BACKEND_URL || 'http://localhost:3001';
      const response = await fetch(`${BACKEND_URL}/api/rag`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query }),
      });

      if (!response.ok) {
        throw new Error(`RAG API request failed with status ${response.status}`);
      }

      const data = await response.json();
      return {
        response: data.response,
        sources: data.sources || []
      };
    } catch (error) {
      console.error('Error calling RAG API:', error);
      return {
        response: `Sorry, I encountered an error generating a response: ${error.message}.`,
        sources: []
      };
    }
  };

  // Function to handle sending messages to the RAG system
  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Track the user query
    await trackAnalytics('user_query', { query: inputValue, timestamp: new Date() });

    // Add user message to chat
    const userMessage = {
      id: Date.now(), // Add unique ID for feedback tracking
      sender: 'user',
      text: inputValue,
      timestamp: new Date()
    };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Step 1: Retrieve relevant documents using the retrieval tool
      const retrievalStartTime = Date.now();
      const retrievalResult = await retrieveDocuments(inputValue);
      const retrievalTime = Date.now() - retrievalStartTime;

      // Track the document retrieval
      await trackAnalytics('document_retrieval', {
        query: inputValue,
        retrievedCount: retrievalResult.retrievedCount || retrievalResult.results?.length || 0,
        retrievalTime: retrievalTime
      });

      // Step 2: Generate response using RAG with retrieved context
      const responseStartTime = Date.now();
      const responseResult = await generateResponse(inputValue);
      const responseTime = Date.now() - responseStartTime;

      // Track the response generation
      await trackAnalytics('response_generated', {
        responseTime,
        query: inputValue,
        responseLength: responseResult.response.length,
        sourcesCount: responseResult.sources.length
      });

      // Add bot response to chat
      const botMessage = {
        id: Date.now() + 1, // Add unique ID for feedback tracking
        sender: 'bot',
        text: responseResult.response,
        timestamp: new Date(),
        sources: responseResult.sources,
        responseTime, // Include response time for potential feedback
        retrievedDocs: retrievalResult.results // Attach retrieved documents to the response
      };
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      // Track error
      await trackAnalytics('error', { query: inputValue, error: error.message });

      const errorMessage = {
        id: Date.now() + 1, // Add unique ID for feedback tracking
        sender: 'bot',
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setRetrievedDocuments([]); // Reset retrieved documents after response
    }
  };

  // Function to handle feedback submission
  const handleFeedback = async (messageId, feedback) => {
    const message = messages.find(msg => msg.id === messageId);
    if (!message) return;

    // Track the feedback
    trackAnalytics('feedback', {
      messageId,
      feedback,
      messageText: message.text,
      timestamp: new Date()
    });

    try {
      // Send feedback to backend API - using configurable backend URL
      const BACKEND_URL = process.env.REACT_APP_BACKEND_URL || 'http://localhost:3001';
      await fetch(`${BACKEND_URL}/api/feedback`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ messageId, feedback, messageText: message.text })
      });
    } catch (error) {
      console.error('Feedback submission failed:', error);
    }

    // Temporarily show feedback received message
    setFeedbackVisible(null);

    // Optional: Add feedback indicator to the message
    setMessages(prev => prev.map(msg =>
      msg.id === messageId
        ? { ...msg, feedbackGiven: feedback }
        : msg
    ));
  };

  // Function to show feedback options for a message
  const toggleFeedback = (messageId) => {
    setFeedbackVisible(feedbackVisible === messageId ? null : messageId);
  };

  // Toggle chatbot panel visibility
  const toggleChatbot = () => {
    setIsOpen(!isOpen);
  };

  return (
    <div className={styles.chatbotContainer}>
      {/* Chatbot panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          <div className={styles.chatHeader}>
            <h3>Course Assistant</h3>
            <button
              className={styles.closeButton}
              onClick={toggleChatbot}
              aria-label="Close chatbot"
            >
              ×
            </button>
          </div>

          <div className={styles.chatMessages}>
            {messages.length === 0 ? (
              <div className={styles.welcomeMessage}>
                <p>Hello! I'm your Physical AI & Humanoid Robotics course assistant.</p>
                <p>Ask me anything about the course content, and I'll provide relevant information from the materials.</p>
              </div>
            ) : (
              messages.map((message, index) => (
                <div
                  key={`${message.id}-${index}`}
                  className={`${styles.message} ${styles[message.sender]}`}
                >
                  <div className={styles.messageText}>{message.text}</div>

                  {/* Show retrieved documents if this is a retrieval update */}
                  {message.retrievedDocs && message.retrievedDocs.length > 0 && (
                    <div className={styles.retrievedDocs}>
                      <details>
                        <summary>Retrieved {message.retrievedDocs.length} documents</summary>
                        <ul>
                          {message.retrievedDocs.map((doc, idx) => (
                            <li key={idx} className={styles.docItem}>
                              <a href={doc.source} target="_blank" rel="noopener noreferrer">
                                {doc.source.split('/').pop() || 'Document'}
                              </a>
                              <p className={styles.docPreview}>{doc.content.substring(0, 150)}...</p>
                            </li>
                          ))}
                        </ul>
                      </details>
                    </div>
                  )}

                  {/* Show sources for bot responses */}
                  {message.sender === 'bot' && message.sources && message.sources.length > 0 && (
                    <div className={styles.sources}>
                      <p>Sources:</p>
                      <ul>
                        {message.sources.map((source, idx) => (
                          <li key={idx}>
                            <a href={source} target="_blank" rel="noopener noreferrer">
                              {source.split('/').pop() || source}
                            </a>
                          </li>
                        ))}
                      </ul>
                    </div>
                  )}

                  {/* Feedback options for bot messages */}
                  {message.sender === 'bot' && (
                    <div className={styles.feedbackContainer}>
                      {!message.feedbackGiven ? (
                        <button
                          className={styles.feedbackButton}
                          onClick={() => toggleFeedback(message.id)}
                        >
                          Was this response helpful?
                        </button>
                      ) : (
                        <div className={styles.feedbackConfirmation}>
                          {message.feedbackGiven === 'positive' ? '👍 Thanks for the positive feedback!' : '👎 Thanks for the feedback!'}
                        </div>
                      )}

                      {feedbackVisible === message.id && !message.feedbackGiven && (
                        <div className={styles.feedbackOptions}>
                          <button
                            className={`${styles.feedbackOption} ${styles.positive}`}
                            onClick={() => handleFeedback(message.id, 'positive')}
                          >
                            👍 Yes
                          </button>
                          <button
                            className={`${styles.feedbackOption} ${styles.negative}`}
                            onClick={() => handleFeedback(message.id, 'negative')}
                          >
                            👎 No
                          </button>
                        </div>
                      )}
                    </div>
                  )}
                </div>
              ))
            )}
            {isLoading && (
              <div className={`${styles.message} ${styles.bot}`}>
                <div className={styles.typingIndicator}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form className={styles.chatInputForm} onSubmit={handleSendMessage}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question about the course..."
              disabled={isLoading}
              className={styles.chatInput}
            />
            <button
              type="submit"
              disabled={isLoading || !inputValue.trim()}
              className={styles.sendButton}
            >
              Send
            </button>
          </form>
        </div>
      )}

      {/* Chatbot toggle button */}
      <button
        className={`${styles.chatToggleButton} ${isOpen ? styles.open : ''}`}
        onClick={toggleChatbot}
        aria-label={isOpen ? "Close chatbot" : "Open chatbot"}
      >
        {isOpen ? '×' : '💬'}
      </button>
    </div>
  );
};

export default RAGChatbot;