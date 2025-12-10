// src/theme/Root.js
import React from 'react';
import { AuthProvider as MainAuthProvider } from '../context/AuthContext';
import RAGChatbot from '../components/RAGChatbot/RAGChatbot';

// Default wrapper - can be customized to add more providers
const Root = ({ children }) => {
  return (
    <MainAuthProvider>
      {children}
      <RAGChatbot />
    </MainAuthProvider>
  );
};

export default Root;