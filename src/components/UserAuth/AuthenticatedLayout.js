// src/components/UserAuth/AuthenticatedLayout.js
import React, { useState, useEffect } from 'react';
import { useAuth } from './useAuth'; // Use our temporary auth hook
import AuthOverlay from './AuthOverlay';
import UserProfile from './UserProfile';
import styles from './AuthenticatedLayout.module.css';

const AuthenticatedLayout = ({ children }) => {
  const { user, isAuthenticating } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [authMode, setAuthMode] = useState('signin'); // 'signin' or 'signup'

  // Show auth modal if user is not authenticated
  useEffect(() => {
    if (!isAuthenticating && !user && !localStorage.getItem('dismissedAuth')) {
      setShowAuthModal(true);
    }
  }, [user, isAuthenticating]);

  const handleAuthSuccess = () => {
    setShowAuthModal(false);
    localStorage.setItem('dismissedAuth', 'true');
  };

  const handleCloseAuthModal = () => {
    setShowAuthModal(false);
    // Remember that user dismissed the modal
    localStorage.setItem('dismissedAuth', 'true');
  };

  return (
    <>
      <div className={styles.layout}>
        {user && (
          <header className={styles.authenticatedHeader}>
            <div className={styles.userInfo}>
              <div className={styles.avatar}>
                {user.image ? (
                  <img src={user.image} alt={user.name || 'User'} />
                ) : (
                  <div className={styles.initials}>
                    {(user.name || user.email)?.charAt(0)?.toUpperCase() || 'U'}
                  </div>
                )}
              </div>
              <div className={styles.userDetails}>
                <span>Welcome, {user.name || 'User'}!</span>
                <button 
                  className={styles.profileButton}
                  onClick={() => document.getElementById('profile-panel')?.scrollIntoView({ behavior: 'smooth' })}
                >
                  View Profile
                </button>
              </div>
            </div>
          </header>
        )}
        
        <main className={styles.mainContent}>
          {children}
          
          {user && (
            <section id="profile-panel" className={styles.profileSection}>
              <UserProfile />
            </section>
          )}
        </main>
      </div>
      
      <AuthOverlay
        isOpen={showAuthModal}
        onClose={handleCloseAuthModal}
        onAuthSuccess={handleAuthSuccess}
      />
    </>
  );
};

export default AuthenticatedLayout;