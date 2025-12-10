// src/lib/auth.js
// Initialize better-auth.com client
import { createAuthClient } from 'better-auth/react';

const { useAuth, signIn, signOut } = createAuthClient({
  fetchOptions: {
    baseURL: process.env.BASE_URL || 'http://localhost:3000',
  },
});

export { useAuth, signIn, signOut };