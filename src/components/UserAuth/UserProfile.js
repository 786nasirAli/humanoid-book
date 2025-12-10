// src/components/UserAuth/UserProfile.js
import React, { useState, useEffect } from 'react';
import { useAuth } from 'better-auth/react';
import styles from './UserProfile.module.css';

const UserProfile = () => {
  const { user, signOut } = useAuth();
  const [userProfile, setUserProfile] = useState(null);
  const [isEditing, setIsEditing] = useState(false);

  useEffect(() => {
    if (user) {
      loadUserProfile();
    }
  }, [user]);

  const loadUserProfile = async () => {
    if (!user) return;

    try {
      const response = await fetch(`/api/user/${user.id}`);
      if (response.ok) {
        const profile = await response.json();
        setUserProfile(profile);
      }
    } catch (error) {
      console.error('Error loading user profile:', error);
    }
  };

  const updateUserProfile = async (updatedData) => {
    try {
      const response = await fetch('/api/user/profile', {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(updatedData)
      });

      if (response.ok) {
        setUserProfile(updatedData);
        setIsEditing(false);
      }
    } catch (error) {
      console.error('Error updating user profile:', error);
    }
  };

  if (!user) {
    return null;
  }

  return (
    <div className={styles.profileContainer}>
      <div className={styles.profileHeader}>
        <div className={styles.avatar}>
          {user.image ? (
            <img src={user.image} alt={user.name || 'User'} />
          ) : (
            <div className={styles.initials}>
              {(user.name || user.email)?.charAt(0)?.toUpperCase() || 'U'}
            </div>
          )}
        </div>
        <div className={styles.userInfo}>
          <h3>{user.name || 'User'}</h3>
          <p>{user.email}</p>
        </div>
        <button 
          className={styles.signOutButton} 
          onClick={() => signOut({ callbackUrl: '/' })}
        >
          Sign Out
        </button>
      </div>

      <div className={styles.profileContent}>
        {userProfile && (
          <>
            <div className={styles.backgroundInfo}>
              <h4>Background Information</h4>
              
              <div className={styles.infoItem}>
                <label>Software Experience:</label>
                <span>{userProfile.softwareExperience || 'Not specified'}</span>
              </div>
              
              <div className={styles.infoItem}>
                <label>Hardware Experience:</label>
                <span>{userProfile.hardwareExperience || 'Not specified'}</span>
              </div>
              
              <div className={styles.infoItem}>
                <label>Robotics Background:</label>
                <span>{userProfile.roboticsBackground || 'Not specified'}</span>
              </div>
              
              <div className={styles.infoItem}>
                <label>Learning Goals:</label>
                <p>{userProfile.learningGoals || 'No goals specified'}</p>
              </div>
              
              <div className={styles.infoItem}>
                <label>Preferred Language:</label>
                <span>{userProfile.preferredLanguage || 'English'}</span>
              </div>
            </div>

            <button 
              className={styles.editButton}
              onClick={() => setIsEditing(!isEditing)}
            >
              {isEditing ? 'Cancel' : 'Edit Profile'}
            </button>
          </>
        )}

        {isEditing && (
          <div className={styles.editForm}>
            <h4>Edit Profile</h4>
            {/* Form fields would go here in a real implementation */}
            <p>Profile editing functionality would be implemented here.</p>
          </div>
        )}
      </div>
    </div>
  );
};

export default UserProfile;