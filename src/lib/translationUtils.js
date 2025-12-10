// src/lib/translationUtils.js
// Utility functions for handling translations across the site

// Function to check if current language preference is Urdu
export const isUrduPreferred = () => {
  if (typeof window !== 'undefined') {
    // Check user's language preference stored in localStorage or cookies
    return localStorage.getItem('preferredLanguage') === 'urdu' || 
           navigator.language.startsWith('ur');
  }
  return false;
};

// Function to set user's language preference
export const setLanguagePreference = (language) => {
  if (typeof window !== 'undefined') {
    localStorage.setItem('preferredLanguage', language);
  }
};

// Function to translate technical terms (mock implementation)
export const translateTechnicalTerms = (text, targetLanguage = 'urdu') => {
  // This is a mock implementation; in a real system, this would connect to 
  // a proper translation API with domain-specific dictionaries
  if (targetLanguage === 'urdu') {
    // For demo purposes, return the original text with a note
    return `TRANSLATED TEXT: ${text}`;
  }
  return text;
};

// Function to get translation status for a specific content
export const getTranslationStatus = (contentId) => {
  if (typeof window !== 'undefined') {
    const status = localStorage.getItem(`translation-${contentId}`);
    return status ? JSON.parse(status) : { isTranslated: false, lastUpdated: null };
  }
  return { isTranslated: false, lastUpdated: null };
};

// Function to save translation status for a specific content
export const saveTranslationStatus = (contentId, status) => {
  if (typeof window !== 'undefined') {
    localStorage.setItem(`translation-${contentId}`, JSON.stringify({
      ...status,
      lastUpdated: new Date().toISOString()
    }));
  }
};

// Mock dictionary for common terms
export const technicalTermsDictionary = {
  'robot': { urdu: 'روبوٹ', description: 'Automatic machine that can perform tasks' },
  'humanoid': { urdu: 'ہیومنوائڈ', description: 'Robot with human-like features' },
  'AI': { urdu: 'مصنوعی ذہانت', description: 'Artificial Intelligence' },
  'ROS': { urdu: 'آر او ایس', description: 'Robot Operating System' },
  'simulation': { urdu: 'سمولیشن', description: 'Virtual model of real system' },
  'perception': { urdu: 'ادراک', description: 'Sensing and interpretation of environment' },
  'navigation': { urdu: 'رہ نمائی', description: 'Finding and following a path' },
  'control': { urdu: 'کنٹرول', description: 'Managing robot behavior' },
  'learning': { urdu: 'سیکھ', description: 'Acquiring new skills or knowledge' },
  'sensor': { urdu: 'سینسر', description: 'Device that detects environmental changes' },
  'actuator': { urdu: 'اکچوایٹر', description: 'Device that creates motion' },
  'algorithm': { urdu: 'الگورتھم', description: 'Set of rules for solving problem' },
  'data': { urdu: 'ڈیٹا', description: 'Information used for processing' },
  'system': { urdu: 'سسٹم', description: 'Interconnected components' },
  'module': { urdu: 'ماڈیول', description: 'Stand-alone component' },
  'architecture': { urdu: 'تعمیرات', description: 'System structure' },
  'interface': { urdu: 'انٹرفیس', description: 'Connection between components' },
  'protocol': { urdu: 'پروٹوکول', description: 'Rules for communication' }
};