// src/components/UserAuth/AuthProvider.js
import React, { createContext, useContext, useState, useEffect } from 'react';
import { useAuth } from './useAuth';  // Import our temporary auth hook

const AuthContext = createContext();

export const AuthProvider = ({ children }) => {
  const { signIn, signOut, user, session, isPending } = useAuth();
  const [userProfile, setUserProfile] = useState(null);
  const [isProfileLoading, setIsProfileLoading] = useState(false);

  // Load user profile when user changes
  useEffect(() => {
    if (user) {
      loadUserProfile();
    } else {
      setUserProfile(null);
    }
  }, [user]);

  const loadUserProfile = async () => {
    if (!user) return;
    
    setIsProfileLoading(true);
    try {
      // In a real implementation, this would fetch from the backend
      const response = await fetch(`/api/user/${user.id}`);
      if (response.ok) {
        const profile = await response.json();
        setUserProfile(profile);
      }
    } catch (error) {
      console.error('Error loading user profile:', error);
    } finally {
      setIsProfileLoading(false);
    }
  };

  const value = {
    user,
    session,
    signIn,
    signOut,
    isPending,
    userProfile,
    isProfileLoading,
    refreshUserProfile: loadUserProfile
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuthContext = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuthContext must be used within an AuthProvider');
  }
  return context;
};