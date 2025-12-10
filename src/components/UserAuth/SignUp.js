// src/components/UserAuth/SignUp.js
import React, { useState } from 'react';
import { useAuth } from 'better-auth/react';
import styles from './Auth.module.css';

const SignUp = ({ onSwitchToSignIn, onComplete }) => {
  const {  signIn } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [showQuestionnaire, setShowQuestionnaire] = useState(false);
  
  // Background assessment questions
  const [background, setBackground] = useState({
    softwareExperience: '',
    hardwareExperience: '',
    roboticsBackground: '',
    learningGoals: '',
    preferredLanguage: 'english'
  });

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');

    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    setIsLoading(true);

    try {
      // In a real implementation, we would use better-auth's signUp function
      // For now, we'll simulate the signup process
      const response = await fetch('/api/auth/signup', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password }),
      });

      if (response.ok) {
        // Sign in the user after successful signup
        const signInResponse = await signIn('credentials', {
          email,
          password,
          redirect: false,
        });

        if (!signInResponse?.error) {
          // Show questionnaire after successful sign in
          setShowQuestionnaire(true);
        } else {
          setError(signInResponse.error.message || 'Sign in after signup failed');
        }
      } else {
        const errorData = await response.json();
        setError(errorData.message || 'Signup failed');
      }
    } catch (err) {
      setError(err.message || 'An error occurred');
    } finally {
      setIsLoading(false);
    }
  };

  const handleQuestionnaireSubmit = async (e) => {
    e.preventDefault();
    setIsLoading(true);
    
    try {
      // Save user background information
      const response = await fetch('/api/user/background', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(background),
      });

      if (response.ok) {
        onComplete && onComplete();
      } else {
        const errorData = await response.json();
        setError(errorData.message || 'Failed to save background information');
      }
    } catch (err) {
      setError(err.message || 'An error occurred while saving background');
    } finally {
      setIsLoading(false);
    }
  };

  if (showQuestionnaire) {
    return (
      <div className={styles.authContainer}>
        <h2>Tell Us About Your Background</h2>
        <p className={styles.subtitle}>This helps us personalize your learning experience</p>
        {error && <div className={styles.error}>{error}</div>}
        
        <form onSubmit={handleQuestionnaireSubmit} className={styles.authForm}>
          <div className={styles.inputGroup}>
            <label>Software Experience</label>
            <select
              value={background.softwareExperience}
              onChange={(e) => setBackground({...background, softwareExperience: e.target.value})}
              required
            >
              <option value="">Select your experience level</option>
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>
          
          <div className={styles.inputGroup}>
            <label>Hardware Experience</label>
            <select
              value={background.hardwareExperience}
              onChange={(e) => setBackground({...background, hardwareExperience: e.target.value})}
              required
            >
              <option value="">Select your experience level</option>
              <option value="none">None</option>
              <option value="basic">Basic</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>
          
          <div className={styles.inputGroup}>
            <label>Robotics Background</label>
            <select
              value={background.roboticsBackground}
              onChange={(e) => setBackground({...background, roboticsBackground: e.target.value})}
              required
            >
              <option value="">Select your background</option>
              <option value="none">No background</option>
              <option value="student">Student</option>
              <option value="researcher">Researcher</option>
              <option value="engineer">Engineer</option>
              <option value="hobbyist">Hobbyist</option>
            </select>
          </div>
          
          <div className={styles.inputGroup}>
            <label>Learning Goals</label>
            <textarea
              value={background.learningGoals}
              onChange={(e) => setBackground({...background, learningGoals: e.target.value})}
              placeholder="What do you hope to achieve in this course?"
              rows="4"
            />
          </div>
          
          <div className={styles.inputGroup}>
            <label>Preferred Language</label>
            <select
              value={background.preferredLanguage}
              onChange={(e) => setBackground({...background, preferredLanguage: e.target.value})}
            >
              <option value="english">English</option>
              <option value="urdu">Urdu</option>
            </select>
          </div>
          
          <button type="submit" disabled={isLoading} className={styles.submitButton}>
            {isLoading ? 'Saving...' : 'Complete Profile'}
          </button>
        </form>
      </div>
    );
  }

  return (
    <div className={styles.authContainer}>
      <h2>Sign Up</h2>
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
            minLength={8}
          />
        </div>
        
        <div className={styles.inputGroup}>
          <label htmlFor="confirmPassword">Confirm Password</label>
          <input
            id="confirmPassword"
            type="password"
            value={confirmPassword}
            onChange={(e) => setConfirmPassword(e.target.value)}
            required
            disabled={isLoading}
          />
        </div>
        
        <button type="submit" disabled={isLoading} className={styles.submitButton}>
          {isLoading ? 'Creating account...' : 'Sign Up'}
        </button>
      </form>
      
      <div className={styles.switchForm}>
        Already have an account?{' '}
        <button 
          type="button" 
          onClick={onSwitchToSignIn}
          className={styles.switchButton}
        >
          Sign in
        </button>
      </div>
    </div>
  );
};

export default SignUp;