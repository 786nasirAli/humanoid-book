import React, { useState, useEffect, useRef } from 'react';
import styles from './RAGChatbot.module.css';

const RAGChatbot = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false); // Chatbot panel visibility
  const messagesEndRef = useRef(null);

  // Function to scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to handle sending messages to the RAG system
  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message to chat
    const userMessage = { sender: 'user', text: inputValue, timestamp: new Date() };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the actual RAG backend API
      const ragResult = await getRAGResponse(inputValue);

      // Add bot response to chat
      const botMessage = {
        sender: 'bot',
        text: ragResult.response,
        timestamp: new Date(),
        sources: ragResult.sources
      };
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        sender: 'bot',
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Function to call the actual RAG backend API
  const getRAGResponse = async (query) => {
    try {
      const response = await fetch('/api/rag', {
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
                  key={index} 
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