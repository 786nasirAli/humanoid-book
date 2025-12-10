import React, { useEffect } from 'react';
import RAGChatbot from '@site/src/components/RAGChatbot/RAGChatbot';

// Custom theme wrapper to integrate RAG chatbot
const RAGLayout = (props) => {
  const { children } = props;

  return (
    <>
      {children}
      <RAGChatbot />
    </>
  );
};

export default RAGLayout;