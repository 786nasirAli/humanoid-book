// src/components/UserAuth/SignIn.js
import React, { useState } from 'react';
import { useAuth } from 'better-auth/react';
import styles from './Auth.module.css';

const SignIn = ({ onSwitchToSignUp, onComplete }) => {
  const {  signIn } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setIsLoading(true);
    setError('');

    try {
      const response = await signIn('credentials', {
        email,
        password,
        redirect: false,
      });

      if (response?.error) {
        setError(response.error.message || 'Sign in failed');
      } else {
        onComplete && onComplete();
      }
    } catch (err) {
      setError(err.message || 'An error occurred');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.authContainer}>
      <h2>Sign In</h2>
      {error && <div className={styles.error}>{error}</div>}
      
      <form onSubmit={handleSubmit} className={styles.authForm}>
        <div className={styles.inputGroup}>
          <label htmlFor="email">Email</label>
          <input
            id="email"
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            disabled={isLoading}
          />
        </div>
        
        <div className={styles.inputGroup}>
          <label htmlFor="password">Password</label>
          <input
            id="password"
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            disabled={isLoading}
          />
        </div>
        
        <button type="submit" disabled={isLoading} className={styles.submitButton}>
          {isLoading ? 'Signing in...' : 'Sign In'}
        </button>
      </form>
      
      <div className={styles.switchForm}>
        Don't have an account?{' '}
        <button 
          type="button" 
          onClick={onSwitchToSignUp}
          className={styles.switchButton}
        >
          Sign up
        </button>
      </div>
    </div>
  );
};

export default SignIn;