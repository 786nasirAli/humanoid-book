import React, { useState, useEffect, useRef } from 'react';
import styles from './RAGChatbot.module.css';

const RAGChatbot = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false); // Chatbot panel visibility
  const [feedbackVisible, setFeedbackVisible] = useState(null); // Track which message feedback is visible
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
      // API call to track analytics - using absolute URL to backend server
      await fetch('http://localhost:3001/api/analytics', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ event, data, timestamp: new Date() })
      });
    } catch (error) {
      console.error('Analytics tracking failed:', error);
    }
  };

  // Function to handle sending messages to the RAG system
  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Track the user query
    trackAnalytics('user_query', { query: inputValue, timestamp: new Date() });

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
      // Call the actual RAG backend API
      const startTime = Date.now();
      const ragResult = await getRAGResponse(inputValue);
      const responseTime = Date.now() - startTime;

      // Track the response metrics
      trackAnalytics('response_received', {
        responseTime,
        query: inputValue,
        responseLength: ragResult.response.length,
        sourcesCount: ragResult.sources.length
      });

      // Add bot response to chat
      const botMessage = {
        id: Date.now() + 1, // Add unique ID for feedback tracking
        sender: 'bot',
        text: ragResult.response,
        timestamp: new Date(),
        sources: ragResult.sources,
        responseTime // Include response time for potential feedback
      };
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      // Track error
      trackAnalytics('error', { query: inputValue, error: error.message });

      const errorMessage = {
        id: Date.now() + 1, // Add unique ID for feedback tracking
        sender: 'bot',
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
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
      // Send feedback to backend API - using absolute URL to backend server
      await fetch('http://localhost:3001/api/feedback', {
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

  // Function to call the actual RAG backend API
  const getRAGResponse = async (query) => {
    try {
      // Using absolute URL to backend server to avoid CORS issues
      const response = await fetch('http://localhost:3001/api/rag', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query }),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();
      return {
        response: data.response,
        sources: data.sources || []
      };
    } catch (error) {
      console.error('Error calling RAG API:', error);
      return {
        response: `Sorry, I encountered an error processing your request: ${error.message}.
        Please make sure the backend server is running and properly configured.`,
        sources: []
      };
    }
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
                  {message.sources && message.sources.length > 0 && (
                    <div className={styles.sources}>
                      <p>Sources:</p>
                      <ul>
                        {message.sources.map((source, idx) => (
                          <li key={idx}>
                            <a href={source.url} target="_blank" rel="noopener noreferrer">
                              {source.title}
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