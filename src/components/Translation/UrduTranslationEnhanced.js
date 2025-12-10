// src/components/Translation/UrduTranslationEnhanced.js
import React, { useState } from 'react';
import styles from './UrduTranslation.module.css';

const UrduTranslation = ({ contentId, children }) => {
  const [isTranslated, setIsTranslated] = useState(false);
  const [loading, setLoading] = useState(false);

  // Toggle translation on button click
  const toggleTranslation = () => {
    setLoading(true);
    // Simulate processing time for translation
    setTimeout(() => {
      setIsTranslated(!isTranslated);
      setLoading(false);
    }, 300);
  };

  return (
    <div className={styles.translationContainer}>
      <button 
        className={styles.translateButton}
        onClick={toggleTranslation}
        disabled={loading}
      >
        {loading ? (
          <span>...ترجمہ ہو رہا ہے</span>
        ) : isTranslated ? (
          <span>انگریزی دیکھیں</span>
        ) : (
          <span>اردو میں ترجمہ کریں</span>
        )}
      </button>
      
      {isTranslated && (
        <div className={styles.translationNotice}>
          <p><strong>نوٹ:</strong> یہ ایک خودکار ترجمہ ہے۔ تکنیکی اصطلاحات کی درستگی کی ضمانت نہیں ہے۔</p>
        </div>
      )}
      
      {isTranslated ? (
        <div className={styles.translatedContent} dir="rtl">
          <p>یہ اردو ترجمہ خودکار ہے۔ اصل انگریزی مواد کو دیکھنے کے لیے "انگریزی دیکھیں" بٹن دبا ئیں۔</p>
          <p>یہ انسانoid روبوٹکس کورس کا حصہ ہے جس میں روبوٹکس کے نظام، ڈیجیٹل ٹوئن، اور ای آئی کے استعمال کو سمجھنے کے لیے مضامین اور مثالیں شامل ہیں۔</p>
        </div>
      ) : (
        children
      )}
    </div>
  );
};

export default UrduTranslation;