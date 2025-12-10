// src/components/UserAuth/AuthOverlay.js
import React, { useState } from 'react';
import { useAuth } from './useAuth'; // Use our temporary auth hook
import SignIn from './SignIn';
import SignUp from './SignUp';
import styles from './AuthOverlay.module.css';

const AuthOverlay = ({ isOpen, onClose, onAuthSuccess }) => {
  const [isLoginView, setIsLoginView] = useState(true);
  const { user } = useAuth(); // Use our temporary auth

  if (!isOpen || user) return null;

  const switchToLogin = () => setIsLoginView(true);
  const switchToSignup = () => setIsLoginView(false);

  const handleAuthComplete = () => {
    onAuthSuccess && onAuthSuccess();
    onClose();
  };

  return (
    <div className={styles.overlay}>
      <div className={styles.modal}>
        <div className={styles.header}>
          <h2>{isLoginView ? 'Welcome Back' : 'Join the Course'}</h2>
          <button className={styles.closeButton} onClick={onClose} aria-label="Close">
            ×
          </button>
        </div>
        
        {isLoginView ? (
          <SignIn 
            onSwitchToSignUp={switchToSignup} 
            onComplete={handleAuthComplete}
          />
        ) : (
          <SignUp 
            onSwitchToSignIn={switchToLogin} 
            onComplete={handleAuthComplete}
          />
        )}
      </div>
    </div>
  );
};

export default AuthOverlay;