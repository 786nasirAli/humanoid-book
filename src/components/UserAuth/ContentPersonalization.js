// src/components/UserAuth/ContentPersonalization.js
import React from 'react';
import PersonalizationControl from './PersonalizationControl';
import styles from './ContentPersonalization.module.css';

const ContentPersonalization = ({ title }) => {
  // Create a content ID based on the title
  const contentId = title ? title.toLowerCase().replace(/\s+/g, '-') : 'unknown-content';

  return (
    <div className={styles.personalizationWrapper}>
      <PersonalizationControl contentId={contentId} />
    </div>
  );
};

export default ContentPersonalization;