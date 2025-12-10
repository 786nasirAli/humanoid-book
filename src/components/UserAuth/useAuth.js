// src/components/UserAuth/useAuth.js
// Temporary auth hook for the application

import { useState, useEffect } from 'react';

export const useAuth = () => {
  const [user, setUser] = useState(null);
  const [session, setSession] = useState(null);
  const [isPending, setIsPending] = useState(false);

  const signIn = async (provider, credentials) => {
    setIsPending(true);
    // Simulate API call
    return new Promise((resolve) => {
      setTimeout(() => {
        const mockUser = {
          id: 'demo-user-' + Date.now(),
          name: credentials?.email?.split('@')[0] || 'Demo User',
          email: credentials?.email || 'demo@example.com',
          image: null
        };
        setUser(mockUser);
        setSession({ 
          id: 'session-' + Date.now(), 
          expires: new Date(Date.now() + 24 * 60 * 60 * 1000).toISOString() // 24 hours
        });
        setIsPending(false);
        resolve({ error: null });
      }, 500);
    });
  };

  const signOut = async () => {
    setUser(null);
    setSession(null);
  };

  // Check for stored session on component mount
  useEffect(() => {
    const storedUser = localStorage.getItem('user');
    const storedSession = localStorage.getItem('session');
    
    if (storedUser && storedSession) {
      setUser(JSON.parse(storedUser));
      setSession(JSON.parse(storedSession));
    }
  }, []);

  // Update localStorage when user/session changes
  useEffect(() => {
    if (user) {
      localStorage.setItem('user', JSON.stringify(user));
      localStorage.setItem('session', JSON.stringify(session));
    } else {
      localStorage.removeItem('user');
      localStorage.removeItem('session');
    }
  }, [user, session]);

  return { 
    user, 
    session, 
    signIn, 
    signOut, 
    isPending,
    isAuthenticating: false // Always false in this implementation
  };
};