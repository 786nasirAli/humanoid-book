// src/context/AuthContext.js
import React, { createContext, useContext, useState, useEffect } from 'react';

// Create Auth Context
const AuthContext = createContext();

// Auth Provider Component
export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  // Check for user in localStorage on load
  useEffect(() => {
    const storedUser = localStorage.getItem('demo_user');
    if (storedUser) {
      setUser(JSON.parse(storedUser));
    }
    setLoading(false);
  }, []);

  // Login function (creates a demo user)
  const login = (userData) => {
    const demoUser = {
      id: 'demo_' + Date.now(),
      name: userData.name || 'Demo User',
      email: userData.email || 'demo@example.com',
      joined: new Date().toISOString()
    };
    setUser(demoUser);
    localStorage.setItem('demo_user', JSON.stringify(demoUser));
  };

  // Logout function
  const logout = () => {
    setUser(null);
    localStorage.removeItem('demo_user');
  };

  // Value object to provide to consumers
  const value = {
    user,
    login,
    logout,
    isAuthenticated: !!user,
    loading
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

// Custom hook to use auth context
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};