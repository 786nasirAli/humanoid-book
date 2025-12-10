// src/components/UserAuth/PersonalizationControl.js
import React, { useState, useEffect } from 'react';
import { useAuth } from '../../context/AuthContext'; // Use our new auth context (corrected path)
import styles from './PersonalizationControl.module.css';

const PersonalizationControl = ({ contentId }) => {
  const { user, session } = useAuth();
  const [userPreferences, setUserPreferences] = useState({
    difficultyLevel: 'medium', // Options: beginner, medium, advanced
    explanationStyle: 'detailed', // Options: concise, detailed, example-heavy
    contentFormat: 'text', // Options: text, diagram, video
    language: 'english' // Options: english, urdu
  });
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [isLoading, setIsLoading] = useState(false);

  useEffect(() => {
    if (user && contentId) {
      loadUserPreferences();
    }
  }, [user, contentId]);

  const loadUserPreferences = async () => {
    if (!user || !contentId) return;
    
    setIsLoading(true);
    try {
      const response = await fetch(`/api/user/preferences/${contentId}`, {
        headers: {
          'Authorization': `Bearer ${session?.access_token}`
        }
      });
      
      if (response.ok) {
        const prefs = await response.json();
        setUserPreferences(prefs);
      }
    } catch (error) {
      console.error('Error loading preferences:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const updatePreferences = async () => {
    if (!user || !contentId) return;
    
    setIsLoading(true);
    try {
      const response = await fetch('/api/user/preferences', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${session?.access_token}`
        },
        body: JSON.stringify({
          contentId,
          preferences: userPreferences
        })
      });
      
      if (response.ok) {
        setIsPersonalizing(false);
        window.location.reload(); // Refresh the page to apply personalization
      }
    } catch (error) {
      console.error('Error updating preferences:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const handlePreferenceChange = (prefKey, value) => {
    setUserPreferences(prev => ({
      ...prev,
      [prefKey]: value
    }));
  };

  if (!user) {
    return (
      <div className={styles.personalizationPrompt}>
        <p>Sign in to personalize this content to your background and preferences.</p>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      {!isPersonalizing ? (
        <button 
          className={styles.personalizeButton}
          onClick={() => setIsPersonalizing(true)}
        >
          🎯 Personalize Content
        </button>
      ) : (
        <div className={styles.personalizationPanel}>
          <h4>Personalize for Your Learning</h4>
          
          <div className={styles.preferenceRow}>
            <label>Difficulty Level:</label>
            <select 
              value={userPreferences.difficultyLevel}
              onChange={(e) => handlePreferenceChange('difficultyLevel', e.target.value)}
            >
              <option value="beginner">Beginner</option>
              <option value="medium">Medium</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>
          
          <div className={styles.preferenceRow}>
            <label>Explanation Style:</label>
            <select 
              value={userPreferences.explanationStyle}
              onChange={(e) => handlePreferenceChange('explanationStyle', e.target.value)}
            >
              <option value="concise">Concise</option>
              <option value="detailed">Detailed</option>
              <option value="example-heavy">Example Heavy</option>
            </select>
          </div>
          
          <div className={styles.preferenceRow}>
            <label>Content Format:</label>
            <select 
              value={userPreferences.contentFormat}
              onChange={(e) => handlePreferenceChange('contentFormat', e.target.value)}
            >
              <option value="text">Text</option>
              <option value="diagram">Diagram/Visual</option>
              <option value="video">Video</option>
            </select>
          </div>
          
          <div className={styles.preferenceRow}>
            <label>Language:</label>
            <select 
              value={userPreferences.language}
              onChange={(e) => handlePreferenceChange('language', e.target.value)}
            >
              <option value="english">English</option>
              <option value="urdu">Urdu</option>
            </select>
          </div>
          
          <div className={styles.buttonGroup}>
            <button 
              className={styles.saveButton}
              onClick={updatePreferences}
              disabled={isLoading}
            >
              {isLoading ? 'Saving...' : 'Save Preferences'}
            </button>
            <button 
              className={styles.cancelButton}
              onClick={() => setIsPersonalizing(false)}
              disabled={isLoading}
            >
              Cancel
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default PersonalizationControl;